
/*
 * ZMK Trackball Angle Input Device
 *
 * PMW3610 -> XY Rotation -> Virtual Input Device
 *
 * Clockwise 10-degree compensation.
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

/*
 * PMW3610 initializes at POST_KERNEL priority 33
 * in the current build.
 *
 * Initialize the virtual device afterward.
 */
#define ANGLE_INPUT_INIT_PRIORITY 50

/*
 * Runtime data for the virtual input device.
 */
struct angle_input_data {
    int32_t pending_x;
    int32_t pending_y;

    struct angle_remainder remainder;
};

/*
 * Configuration for the virtual device.
 */
struct angle_input_config {
    const struct device *source;
    int32_t angle_deg;
};

/*
 * Receive movement events from PMW3610.
 *
 * The physical sensor reports relative X and Y
 * movement as separate events.
 *
 * Wait for the synchronized event before applying
 * the two-dimensional rotation.
 */
static void angle_input_callback(
    struct input_event *event,
    void *user_data)
{
    const struct device *virtual_dev = user_data;

    if (virtual_dev == NULL || event == NULL) {
        return;
    }

    struct angle_input_data *data = virtual_dev->data;

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

    /*
     * Wait until the complete input report
     * has been received.
     */
    if (!event->sync) {
        return;
    }

    int32_t corrected_x = 0;
    int32_t corrected_y = 0;

    /*
     * Rotate the complete XY vector clockwise
     * by 10 degrees.
     *
     * angle_math.h preserves fractional
     * remainders between reports.
     */
    angle_rotate_10(
        data->pending_x,
        data->pending_y,
        &corrected_x,
        &corrected_y,
        &data->remainder
    );

    /*
     * Clear accumulated raw movement.
     */
    data->pending_x = 0;
    data->pending_y = 0;

    /*
     * Forward corrected coordinates through
     * the virtual input device.
     *
     * X: sync = false
     * Y: sync = true
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
            "Failed to forward corrected XY movement: X=%d Y=%d",
            ret_x,
            ret_y
        );
    }
}

/*
 * Initialize the virtual input device.
 *
 * The physical PMW3610 must already
 * be initialized and ready.
 */
static int angle_input_init(
    const struct device *dev)
{
    const struct angle_input_config *config = dev->config;

    if (!device_is_ready(config->source)) {
        LOG_ERR("PMW3610 source sensor not ready");
        return -ENODEV;
    }

    LOG_INF(
        "Virtual trackball initialized, correction=%d degrees",
        config->angle_deg
    );

    return 0;
}

/*
 * Create one virtual device for each enabled
 * zmk,angle-input Devicetree instance.
 *
 * Current implementation supports
 * clockwise 10-degree correction only.
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
