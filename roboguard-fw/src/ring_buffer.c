#include "ring_buffer.h"

void ring_buffer_init(
    ring_buffer_t *rb
)
{
    rb->head = 0;
    rb->count = 0;
}

void ring_buffer_push(
    ring_buffer_t *rb,
    const telemetry_t *sample
)
{
    rb->data[rb->head] = *sample;

    rb->head++;

    if (rb->head >= RING_BUFFER_SIZE)
    {
        rb->head = 0;
    }

    if (rb->count < RING_BUFFER_SIZE)
    {
        rb->count++;
    }
}

uint16_t ring_buffer_count(
    const ring_buffer_t *rb
)
{
    return rb->count;
}

const telemetry_t *ring_buffer_latest(
    const ring_buffer_t *rb
)
{
    if (rb->count == 0)
    {
        return 0;
    }

    uint16_t index;

    if (rb->head == 0)
    {
        index = RING_BUFFER_SIZE - 1;
    }
    else
    {
        index = rb->head - 1;
    }

    return &rb->data[index];
}

const telemetry_t *ring_buffer_get(
    const ring_buffer_t *rb,
    uint16_t chronological_index
)
{
    if (
        chronological_index >= rb->count
    )
    {
        return 0;
    }

    uint16_t oldest =
        (uint16_t)(
            rb->head +
            RING_BUFFER_SIZE -
            rb->count
        );

    if (oldest >= RING_BUFFER_SIZE)
    {
        oldest -= RING_BUFFER_SIZE;
    }

    uint16_t index =
        (uint16_t)(
            oldest +
            chronological_index
        );

    if (index >= RING_BUFFER_SIZE)
    {
        index -= RING_BUFFER_SIZE;
    }

    return &rb->data[index];
}
