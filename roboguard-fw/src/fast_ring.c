#include "fast_ring.h"


void fast_ring_init(
    fast_ring_t *ring
)
{
    if (ring == 0)
    {
        return;
    }

    ring->head = 0;
    ring->count = 0;
}


void fast_ring_push(
    fast_ring_t *ring,
    const telemetry_t *sample
)
{
    if (
        ring == 0 ||
        sample == 0
    )
    {
        return;
    }


    ring->data[ring->head] =
        *sample;


    ring->head++;

    if (
        ring->head >=
        FAST_RING_SIZE
    )
    {
        ring->head = 0;
    }


    if (
        ring->count <
        FAST_RING_SIZE
    )
    {
        ring->count++;
    }
}


const telemetry_t *fast_ring_get(
    const fast_ring_t *ring,
    uint16_t chronological_index
)
{
    if (
        ring == 0 ||
        chronological_index >=
        ring->count
    )
    {
        return 0;
    }


    uint16_t oldest =
        (uint16_t)(
            ring->head +
            FAST_RING_SIZE -
            ring->count
        );


    if (
        oldest >=
        FAST_RING_SIZE
    )
    {
        oldest -=
            FAST_RING_SIZE;
    }


    uint16_t index =
        (uint16_t)(
            oldest +
            chronological_index
        );


    if (
        index >=
        FAST_RING_SIZE
    )
    {
        index -=
            FAST_RING_SIZE;
    }


    return &ring->data[index];
}


uint16_t fast_ring_count(
    const fast_ring_t *ring
)
{
    if (ring == 0)
    {
        return 0;
    }

    return ring->count;
}
