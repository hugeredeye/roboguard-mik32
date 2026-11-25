#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>

typedef struct
{
    uint32_t seq;

    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

} telemetry_t;

#endif
