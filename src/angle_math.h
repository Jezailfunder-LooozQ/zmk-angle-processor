
/*
 * Trackball 2D angle correction math
 * Fixed-point Q14 implementation
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ZMK_ANGLE_MATH_H
#define ZMK_ANGLE_MATH_H

#include <stdint.h>

#define ANGLE_Q14_SCALE 16384

/* cos(10 degrees) * 16384 */
#define ANGLE_COS_10 16135

/* sin(10 degrees) * 16384 */
#define ANGLE_SIN_10 2845

struct angle_remainder {
    int32_t x;
    int32_t y;
};

/*
 * Rotate a screen-coordinate movement vector
 * clockwise by 10 degrees.
 *
 * x, y: input movement
 * out_x, out_y: corrected movement
 * remainder: retains fractional movement
 */
static inline void angle_rotate_10(
    int32_t x,
    int32_t y,
    int32_t *out_x,
    int32_t *out_y,
    struct angle_remainder *remainder)
{
    int64_t rotated_x =
        (int64_t)x * ANGLE_COS_10 -
        (int64_t)y * ANGLE_SIN_10 +
        remainder->x;

    int64_t rotated_y =
        (int64_t)x * ANGLE_SIN_10 +
        (int64_t)y * ANGLE_COS_10 +
        remainder->y;

    *out_x = (int32_t)(
        rotated_x / ANGLE_Q14_SCALE
    );

    *out_y = (int32_t)(
        rotated_y / ANGLE_Q14_SCALE
    );

    remainder->x = (int32_t)(
        rotated_x -
        (int64_t)(*out_x) * ANGLE_Q14_SCALE
    );

    remainder->y = (int32_t)(
        rotated_y -
        (int64_t)(*out_y) * ANGLE_Q14_SCALE
    );
}

#endif
