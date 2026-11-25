#ifndef COLLISION_H
#define COLLISION_H

#include <stdint.h>
#include "imu.h"

/*
 * Initial RoboGuard collision threshold.
 * 400 = 4.00 g.
 *
 * This is a first experimentally-derived threshold.
 * It will be calibrated again on the assembled robot.
 */
#define COLLISION_THRESHOLD_CENTI_G 400U

uint8_t collision_detect(
    const imu_sample_t *sample
);

uint32_t collision_magnitude_centi_g(
    const imu_sample_t *sample
);

#endif
