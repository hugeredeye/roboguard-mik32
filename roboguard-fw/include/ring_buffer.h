#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include "telemetry.h"

/*
 * 400 records * 16 bytes = 6400 bytes.
 * At ~100 Hz this is about 4 seconds of history.
 */
#define RING_BUFFER_SIZE 400

typedef struct
{
    telemetry_t data[RING_BUFFER_SIZE];

    uint16_t head;
    uint16_t count;

} ring_buffer_t;

void ring_buffer_init(
    ring_buffer_t *rb
);

void ring_buffer_push(
    ring_buffer_t *rb,
    const telemetry_t *sample
);

uint16_t ring_buffer_count(
    const ring_buffer_t *rb
);

const telemetry_t *ring_buffer_latest(
    const ring_buffer_t *rb
);

/*
 * Returns records in chronological order:
 * index 0 = oldest,
 * index count-1 = newest.
 */
const telemetry_t *ring_buffer_get(
    const ring_buffer_t *rb,
    uint16_t chronological_index
);

#endif
