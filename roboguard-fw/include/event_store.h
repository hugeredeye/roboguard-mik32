#ifndef EVENT_STORE_H
#define EVENT_STORE_H

#include <stdint.h>

#include "imu.h"
#include "ring_buffer.h"
#include "fast_ring.h"


#define EVENT_STORE_OFFSET          0x00780000UL
#define EVENT_STORE_XIP_BASE        0x80780000UL

#define EVENT_STORE_SLOT_SIZE       0x00002000UL
#define EVENT_STORE_SLOT_COUNT      64U

#define EVENT_STORE_PAYLOAD_OFFSET  0x00000100UL
#define EVENT_STORE_COMMIT_OFFSET   0x00001FF0UL


#define EVENT_MAGIC                 0x31564752UL
#define EVENT_COMMIT_MAGIC          0x54494D43UL

#define EVENT_FORMAT_VERSION        1U

#define EVENT_TYPE_ACCEL_TRIGGER    1UL
#define EVENT_TYPE_MANUAL_CAPTURE   2UL
#define EVENT_TYPE_HEARTBEAT_LOST   3UL
#define EVENT_TYPE_STALL_CANDIDATE  4UL

#define EVENT_FLAG_ACCEL_SAT        (1UL << 0)
#define EVENT_FLAG_GYRO_SAT         (1UL << 1)


typedef struct
{
    uint32_t magic;

    uint16_t version;
    uint16_t header_size;

    uint32_t event_sequence;
    uint32_t event_type;
    uint32_t flags;

    uint32_t trigger_fast_seq;
    uint32_t trigger_telemetry_seq;

    uint32_t record_count;
    uint32_t post_count;

    uint32_t first_seq;
    uint32_t last_seq;

    int16_t trigger_ax;
    int16_t trigger_ay;
    int16_t trigger_az;

    int16_t trigger_gx;
    int16_t trigger_gy;
    int16_t trigger_gz;

    uint16_t trigger_magnitude_centi_g;
    uint16_t fast_count;

    uint32_t payload_bytes;
    uint32_t payload_crc32;

    uint32_t header_crc32;

} event_header_t;


typedef struct
{
    uint32_t magic;

    uint32_t event_sequence;
    uint32_t header_crc32;
    uint32_t payload_crc32;

} event_commit_t;


typedef struct
{
    event_header_t header;

    uint32_t slot_offset;

    uint16_t slot_index;
    uint16_t oldest_index;
    uint16_t fast_oldest_index;

} event_pending_t;


typedef struct
{
    uint8_t found;

    uint16_t slot_index;

    event_header_t header;

} event_summary_t;


int event_store_find_latest(
    event_summary_t *summary
);


int event_store_prepare(
    event_pending_t *pending,
    const ring_buffer_t *history,
    const fast_ring_t *fast_history,
    uint32_t event_type,

    uint32_t trigger_fast_seq,
    uint32_t trigger_telemetry_seq,

    const imu_sample_t *trigger_sample,

    uint32_t trigger_magnitude_centi_g,

    uint32_t post_count
);

#endif
