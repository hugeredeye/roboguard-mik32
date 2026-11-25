#include "event_store.h"

#include <stddef.h>
#include <stdint.h>

#include "crc32.h"


_Static_assert(
    sizeof(telemetry_t) == 16,
    "telemetry_t must remain 16 bytes"
);

_Static_assert(
    sizeof(event_header_t) == 72,
    "event_header_t layout changed"
);

_Static_assert(
    sizeof(event_commit_t) == 16,
    "event_commit_t layout changed"
);


static uint32_t event_header_crc(
    const event_header_t *header
)
{
    return crc32_compute(
        header,
        (uint32_t)
        offsetof(
            event_header_t,
            header_crc32
        )
    );
}


static uint32_t event_payload_crc_flash(
    uint16_t slot,
    uint32_t payload_bytes
)
{
    const uint8_t *payload =
        (const uint8_t *)(
            EVENT_STORE_XIP_BASE +
            ((uint32_t)slot *
             EVENT_STORE_SLOT_SIZE) +
            EVENT_STORE_PAYLOAD_OFFSET
        );

    return crc32_compute(
        payload,
        payload_bytes
    );
}


static uint32_t event_payload_crc_rings(
    const ring_buffer_t *history,
    const fast_ring_t *fast_history
)
{
    uint32_t crc =
        crc32_begin();

    for (
        uint16_t i = 0;
        i < history->count;
        i++
    )
    {
        const telemetry_t *sample =
            ring_buffer_get(
                history,
                i
            );

        crc =
            crc32_update(
                crc,
                sample,
                sizeof(telemetry_t)
            );
    }

    for (
        uint16_t i = 0;
        i < fast_history->count;
        i++
    )
    {
        const telemetry_t *sample =
            fast_ring_get(
                fast_history,
                i
            );

        crc =
            crc32_update(
                crc,
                sample,
                sizeof(telemetry_t)
            );
    }

    return crc32_end(crc);
}


static uint32_t telemetry_flags(
    const telemetry_t *sample
)
{
    uint32_t flags = 0;

    if (
        sample->ax == INT16_MIN ||
        sample->ax == INT16_MAX ||
        sample->ay == INT16_MIN ||
        sample->ay == INT16_MAX ||
        sample->az == INT16_MIN ||
        sample->az == INT16_MAX
    )
    {
        flags |=
            EVENT_FLAG_ACCEL_SAT;
    }

    if (
        sample->gx == INT16_MIN ||
        sample->gx == INT16_MAX ||
        sample->gy == INT16_MIN ||
        sample->gy == INT16_MAX ||
        sample->gz == INT16_MIN ||
        sample->gz == INT16_MAX
    )
    {
        flags |=
            EVENT_FLAG_GYRO_SAT;
    }

    return flags;
}


static uint32_t trigger_flags(
    const imu_sample_t *sample
)
{
    telemetry_t temporary;

    temporary.seq = 0;

    temporary.ax = sample->ax;
    temporary.ay = sample->ay;
    temporary.az = sample->az;

    temporary.gx = sample->gx;
    temporary.gy = sample->gy;
    temporary.gz = sample->gz;

    return telemetry_flags(
        &temporary
    );
}


int event_store_find_latest(
    event_summary_t *summary
)
{
    summary->found = 0;
    summary->slot_index = 0;

    uint32_t best_sequence = 0;


    for (
        uint16_t slot = 0;
        slot < EVENT_STORE_SLOT_COUNT;
        slot++
    )
    {
        uint32_t slot_address =
            EVENT_STORE_XIP_BASE +
            ((uint32_t)slot *
             EVENT_STORE_SLOT_SIZE);


        const event_header_t *header =
            (const event_header_t *)
            slot_address;


        const event_commit_t *commit =
            (const event_commit_t *)(
                slot_address +
                EVENT_STORE_COMMIT_OFFSET
            );


        if (
            header->magic !=
            EVENT_MAGIC
        )
        {
            continue;
        }


        if (
            header->version !=
            EVENT_FORMAT_VERSION
        )
        {
            continue;
        }


        if (
            header->header_size !=
            sizeof(event_header_t)
        )
        {
            continue;
        }


        if (
            header->record_count >
            RING_BUFFER_SIZE
        )
        {
            continue;
        }


        if (
            header->fast_count >
            FAST_RING_SIZE
        )
        {
            continue;
        }


        uint32_t expected_payload_bytes =
            (
                header->record_count +
                header->fast_count
            ) *
            sizeof(telemetry_t);


        if (
            header->payload_bytes !=
            expected_payload_bytes
        )
        {
            continue;
        }


        if (
            header->payload_bytes >
            (
                EVENT_STORE_COMMIT_OFFSET -
                EVENT_STORE_PAYLOAD_OFFSET
            )
        )
        {
            continue;
        }


        if (
            event_header_crc(header) !=
            header->header_crc32
        )
        {
            continue;
        }


        if (
            commit->magic !=
            EVENT_COMMIT_MAGIC
        )
        {
            continue;
        }


        if (
            commit->event_sequence !=
            header->event_sequence ||
            commit->header_crc32 !=
            header->header_crc32 ||
            commit->payload_crc32 !=
            header->payload_crc32
        )
        {
            continue;
        }


        if (
            event_payload_crc_flash(
                slot,
                header->payload_bytes
            ) !=
            header->payload_crc32
        )
        {
            continue;
        }


        if (
            summary->found == 0 ||
            header->event_sequence >
            best_sequence
        )
        {
            summary->found = 1;

            summary->slot_index =
                slot;

            summary->header =
                *header;

            best_sequence =
                header->event_sequence;
        }
    }


    return summary->found
        ? 0
        : -1;
}


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
)
{
    if (
        pending == 0 ||
        history == 0 ||
        fast_history == 0 ||
        trigger_sample == 0 ||
        history->count == 0 ||
        fast_history->count == 0
    )
    {
        return -1;
    }


    event_summary_t latest;

    int latest_found =
        event_store_find_latest(
            &latest
        ) == 0;


    if (latest_found)
    {
        pending->slot_index =
            (uint16_t)(
                latest.slot_index + 1U
            );

        if (
            pending->slot_index >=
            EVENT_STORE_SLOT_COUNT
        )
        {
            pending->slot_index = 0;
        }

        pending->header.event_sequence =
            latest.header.event_sequence +
            1U;
    }
    else
    {
        pending->slot_index = 0;

        pending->header.event_sequence =
            1U;
    }


    pending->slot_offset =
        EVENT_STORE_OFFSET +
        ((uint32_t)pending->slot_index *
         EVENT_STORE_SLOT_SIZE);


    pending->oldest_index =
        (uint16_t)(
            history->head +
            RING_BUFFER_SIZE -
            history->count
        );

    if (
        pending->oldest_index >=
        RING_BUFFER_SIZE
    )
    {
        pending->oldest_index -=
            RING_BUFFER_SIZE;
    }


    pending->fast_oldest_index =
        (uint16_t)(
            fast_history->head +
            FAST_RING_SIZE -
            fast_history->count
        );

    if (
        pending->fast_oldest_index >=
        FAST_RING_SIZE
    )
    {
        pending->fast_oldest_index -=
            FAST_RING_SIZE;
    }


    const telemetry_t *first =
        ring_buffer_get(
            history,
            0
        );

    const telemetry_t *last =
        ring_buffer_latest(
            history
        );


    uint32_t flags =
        trigger_flags(
            trigger_sample
        );


    for (
        uint16_t i = 0;
        i < history->count;
        i++
    )
    {
        const telemetry_t *sample =
            ring_buffer_get(
                history,
                i
            );

        flags |=
            telemetry_flags(
                sample
            );
    }


    for (
        uint16_t i = 0;
        i < fast_history->count;
        i++
    )
    {
        const telemetry_t *sample =
            fast_ring_get(
                fast_history,
                i
            );

        flags |=
            telemetry_flags(
                sample
            );
    }


    event_header_t *header =
        &pending->header;


    header->magic =
        EVENT_MAGIC;

    header->version =
        EVENT_FORMAT_VERSION;

    header->header_size =
        sizeof(event_header_t);

    header->event_type =
        event_type;

    header->flags =
        flags;

    header->trigger_fast_seq =
        trigger_fast_seq;

    header->trigger_telemetry_seq =
        trigger_telemetry_seq;

    header->record_count =
        history->count;

    header->post_count =
        post_count;

    header->first_seq =
        first->seq;

    header->last_seq =
        last->seq;


    header->trigger_ax =
        trigger_sample->ax;

    header->trigger_ay =
        trigger_sample->ay;

    header->trigger_az =
        trigger_sample->az;

    header->trigger_gx =
        trigger_sample->gx;

    header->trigger_gy =
        trigger_sample->gy;

    header->trigger_gz =
        trigger_sample->gz;


    header->trigger_magnitude_centi_g =
        (uint16_t)
        trigger_magnitude_centi_g;

    header->fast_count =
        fast_history->count;


    header->payload_bytes =
        (
            history->count +
            fast_history->count
        ) *
        sizeof(telemetry_t);


    if (
        header->payload_bytes >
        (
            EVENT_STORE_COMMIT_OFFSET -
            EVENT_STORE_PAYLOAD_OFFSET
        )
    )
    {
        return -2;
    }


    header->payload_crc32 =
        event_payload_crc_rings(
            history,
            fast_history
        );

    header->header_crc32 = 0;

    header->header_crc32 =
        event_header_crc(
            header
        );


    return 0;
}
