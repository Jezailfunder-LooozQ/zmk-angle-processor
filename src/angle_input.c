
/*
 * ZMK Trackball Angle Input Device
 *
 * PMW3610
 *   -> 10-degree rotation
 *   -> Vertical Angle Snapping
 *   -> Virtual Input Device
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_angle_input

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

#include <errno.h>
#include <stdint.h>

#include "angle_math.h"

LOG_MODULE_REGISTER(zmk_angle_input, LOG_LEVEL_INF);

#define ANGLE_INPUT_INIT_PRIORITY 95

/*
 * Vertical snapping settings.
 *
 * Strength: 0 to 10
 * 0 = disabled
 * 10 = maximum horizontal suppression
 */
#define VERTICAL_SNAP_STRENGTH 6

/*
 * The sensor coordinates are rotated first.
 *
 * The ZMK listener later applies:
 * XY_SWAP | X_INVERT
 *
 * Therefore:
 * rotated X -> screen vertical movement
 * rotated Y -> screen horizontal movement
 *
 * To straighten vertical movement,
 * suppress small rotated-Y deviations.
 */

/*
 * Direction tracking uses a short
 * exponential moving average.
 */
#define SNAP_HISTORY_WEIGHT 3
#define SNAP_HISTORY_DIVISOR 4

/*
 * Require dominant vertical motion.
 *
 * Vertical magnitude must exceed
 * horizontal magnitude by 3:1.
 */
#define SNAP_DIRECTION_RATIO 3

/*
 * Minimum accumulated activity
 * before snapping can engage.
 */
#define SNAP_MIN_ACTIVITY 12

struct angle_input_data {
    int32_t pending_x;
    int32_t pending_y;

    struct angle_remainder remainder;

    int32_t vertical_history;
    int32_t horizontal_history;

    int32_t horizontal_fraction;
};

struct angle_input_config {
    const struct device *source;
    int32_t angle_deg;
};

static int32_t snap_abs(int32_t value)
{
    if (value >= 0) {
        return value;
    }

    if (value == INT32_MIN) {
        return INT32_MAX;
    }

    return -value;
}

/*
 * Apply vertical-only snapping.
 *
 * X is the future screen-vertical axis.
 * Y is the future screen-horizontal axis.
 *
 * Normal horizontal and diagonal
 * movements should remain unchanged.
 */
static void apply_vertical_snapping(
    struct angle_input_data *data,
    int32_t *x,
    int32_t *y)
{
    int32_t vertical = snap_abs(*x);
    int32_t horizontal = snap_abs(*y);

    /*
     * Update directional history.
     */
    data->vertical_history =
        (data->vertical_history *
         SNAP_HISTORY_WEIGHT) /
         SNAP_HISTORY_DIVISOR +
         vertical;

    data->horizontal_history =
        (data->horizontal_history *
         SNAP_HISTORY_WEIGHT) /
         SNAP_HISTORY_DIVISOR +
         horizontal;

    /*
     * Decide whether movement is
     * sufficiently vertical.
     */
    int64_t activity =
        (int64_t)data->vertical_history +
        data->horizontal_history;

    bool vertical_dominant =
        data->vertical_history > 0 &&
        activity >= SNAP_MIN_ACTIVITY &&
        (int64_t)data->horizontal_history *
            SNAP_DIRECTION_RATIO <=
        data->vertical_history;

    if (!vertical_dominant ||
        VERTICAL_SNAP_STRENGTH == 0) {

        /*
         * Exit snapping immediately.
         * Do not carry fractional
         * suppression into other motions.
         */
        data->horizontal_fraction = 0;
        return;
    }

    /*
     * Preserve 4/10 of horizontal motion
     * at snapping strength 6/10.
     *
     * Fractional output is accumulated
     * to prevent integer truncation
     * from producing excessive dead zones.
     */
    int32_t numerator =
        *y * (10 - VERTICAL_SNAP_STRENGTH) +
        data->horizontal_fraction;

    *y = numerator / 10;

    data->horizontal_fraction =
        numerator - (*y * 10);
}

/*
 * Receive PMW3610 movement.
 */
static void angle_input_callback(
    struct input_event *event,
    void *user_data)
{
    const struct device *virtual_dev = user_data;

    if (virtual_dev == NULL || event == NULL) {
        return;
    }

    struct angle_input_data *data =
        virtual_dev->data;

    if (event->type != INPUT_EV_REL) {
        return;
    }

    switch (event->code) {
    case INPUT_REL_X:
        data->pending_x += event->value;
        break;

    case INPUT_REL_Y:
        data->pending_y += event->value;
        break;

    default:
        return;
    }

    if (!event->sync) {
        return;
    }

    int32_t corrected_x = 0;
    int32_t corrected_y = 0;

    /*
     * Step 1: Existing clockwise
     * 10-degree correction.
     */
    angle_rotate_10(
        data->pending_x,
        data->pending_y,
        &corrected_x,
        &corrected_y,
        &data->remainder
    );

    data->pending_x = 0;
    data->pending_y = 0;

    /*
     * Step 2: New vertical-only snapping.
     */
    apply_vertical_snapping(
        data,
        &corrected_x,
        &corrected_y
    );

    /*
     * Step 3: Forward to virtual device.
     *
     * Original ZMK listener still handles
     * XY transformation, temporary layer,
     * and Response Curve processing.
     */
    int ret_x = input_report_rel(
        virtual_dev,
        INPUT_REL_X,
        corrected_x,
        false,
        K_NO_WAIT
    );

    int ret_y = input_report_rel(
        virtual_dev,
        INPUT_REL_Y,
        corrected_y,
        true,
        K_NO_WAIT
    );

    if (ret_x < 0 || ret_y < 0) {
        LOG_WRN(
            "Motion forwarding failed: X=%d Y=%d",
            ret_x,
            ret_y
        );
    }
}

/*
 * Initialize virtual input device.
 */
static int angle_input_init(
    const struct device *dev)
{
    const struct angle_input_config *config =
        dev->config;

    if (!device_is_ready(config->source)) {
        LOG_ERR("PMW3610 source not ready");
        return -ENODEV;
    }

    LOG_INF(
        "Trackball ready: angle=%d, vertical snap=%d/10",
        config->angle_deg,
        VERTICAL_SNAP_STRENGTH
    );

    return 0;
}

/*
 * Create virtual input devices.
 */
#define ANGLE_INPUT_DEFINE(n)                             \
                                                         \
    BUILD_ASSERT(                                         \
        DT_INST_PROP(n, angle_deg) == 10,                 \
        "Only 10-degree correction supported"            \
    );                                                   \
                                                         \
    static struct angle_input_data                        \
        angle_input_data_##n;                            \
                                                         \
    static const struct angle_input_config                \
        angle_input_config_##n = {                       \
            .source = DEVICE_DT_GET(                      \
                DT_INST_PHANDLE(n, source)                \
            ),                                           \
            .angle_deg = DT_INST_PROP(n, angle_deg),      \
        };                                               \
                                                         \
    DEVICE_DT_INST_DEFINE(                                \
        n,                                               \
        angle_input_init,                                \
        NULL,                                            \
        &angle_input_data_##n,                           \
        &angle_input_config_##n,                         \
        POST_KERNEL,                                     \
        ANGLE_INPUT_INIT_PRIORITY,                       \
        NULL                                             \
    );                                                   \
                                                         \
    INPUT_CALLBACK_DEFINE_NAMED(                          \
        DEVICE_DT_GET(                                   \
            DT_INST_PHANDLE(n, source)                    \
        ),                                               \
        angle_input_callback,                            \
        (void *)DEVICE_DT_INST_GET(n),                   \
        angle_input_callback_##n                         \
    );

DT_INST_FOREACH_STATUS_OKAY(ANGLE_INPUT_DEFINE)
