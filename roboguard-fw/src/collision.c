#include "collision.h"

#define ACCEL_LSB_PER_G 2048U


static uint32_t accel_squared(
    const imu_sample_t *sample
)
{
    int64_t ax = sample->ax;
    int64_t ay = sample->ay;
    int64_t az = sample->az;

    uint64_t sum =
        (uint64_t)(ax * ax) +
        (uint64_t)(ay * ay) +
        (uint64_t)(az * az);

    return (uint32_t)sum;
}


/*
 * Integer square root.
 *
 * Avoids floating point on MIK32.
 */
static uint32_t isqrt_u32(
    uint32_t value
)
{
    uint32_t result = 0;
    uint32_t bit = 1UL << 30;

    while (bit > value)
    {
        bit >>= 2;
    }

    while (bit != 0)
    {
        if (value >= result + bit)
        {
            value -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }

        bit >>= 2;
    }

    return result;
}


uint8_t collision_detect(
    const imu_sample_t *sample
)
{
    /*
     * 4.00 g * 2048 LSB/g = 8192 raw units.
     */
    uint32_t threshold_raw =
        (COLLISION_THRESHOLD_CENTI_G *
         ACCEL_LSB_PER_G) / 100U;

    uint32_t threshold_squared =
        threshold_raw * threshold_raw;

    return (
        accel_squared(sample) >=
        threshold_squared
    );
}


uint32_t collision_magnitude_centi_g(
    const imu_sample_t *sample
)
{
    uint32_t raw_magnitude =
        isqrt_u32(
            accel_squared(sample)
        );

    return (
        raw_magnitude * 100U
    ) / ACCEL_LSB_PER_G;
}
