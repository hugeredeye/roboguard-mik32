#ifndef FAST_RING_H
#define FAST_RING_H

#include <stdint.h>

#include "telemetry.h"


#define FAST_RING_SIZE 64U


typedef struct
{
    telemetry_t data[FAST_RING_SIZE];

    uint16_t head;
    uint16_t count;

} fast_ring_t;


void fast_ring_init(
    fast_ring_t *ring
);


void fast_ring_push(
    fast_ring_t *ring,
    const telemetry_t *sample
);


const telemetry_t *fast_ring_get(
    const fast_ring_t *ring,
    uint16_t chronological_index
);


uint16_t fast_ring_count(
    const fast_ring_t *ring
);


#endif
