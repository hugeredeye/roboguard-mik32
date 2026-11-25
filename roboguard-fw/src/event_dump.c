#include "event_dump.h"

#include <stdint.h>

#include "telemetry.h"
#include "xprintf.h"


void event_dump_csv(
    const event_summary_t *summary
)
{
    if (
        summary == 0 ||
        summary->found == 0
    )
    {
        return;
    }


    const event_header_t *header =
        &summary->header;


    uint32_t payload_address =
        EVENT_STORE_XIP_BASE +
        ((uint32_t)summary->slot_index *
         EVENT_STORE_SLOT_SIZE) +
        EVENT_STORE_PAYLOAD_OFFSET;


    const telemetry_t *records =
        (const telemetry_t *)
        payload_address;


    xprintf("\n");
    xprintf("=== EVENT FAST TRIGGER ===\n");

    xprintf(
        "event_seq=%u\n",
        header->event_sequence
    );

    xprintf(
        "fast_seq=%u\n",
        header->trigger_fast_seq
    );

    xprintf(
        "telemetry_seq_hint=%u\n",
        header->trigger_telemetry_seq
    );

    xprintf(
        "acceleration=%u.%02u g\n",
        header->trigger_magnitude_centi_g / 100U,
        header->trigger_magnitude_centi_g % 100U
    );

    xprintf(
        "raw_accel=%d,%d,%d\n",
        header->trigger_ax,
        header->trigger_ay,
        header->trigger_az
    );

    xprintf(
        "raw_gyro=%d,%d,%d\n",
        header->trigger_gx,
        header->trigger_gy,
        header->trigger_gz
    );

    xprintf("=== EVENT FAST TRIGGER END ===\n");


    xprintf("\n");
    xprintf("=== EVENT CSV BEGIN ===\n");

    xprintf(
        "event_seq=%u,slot=%u,trigger_seq=%u,records=%u\n",
        header->event_sequence,
        summary->slot_index,
        header->trigger_telemetry_seq,
        header->record_count
    );

    xprintf(
        "seq,ax,ay,az,gx,gy,gz,trigger_hint\n"
    );


    for (
        uint32_t i = 0;
        i < header->record_count;
        i++
    )
    {
        const telemetry_t *sample =
            &records[i];


        uint32_t trigger_hint =
            (
                sample->seq ==
                header->trigger_telemetry_seq
            )
            ? 1U
            : 0U;


        xprintf(
            "%u,%d,%d,%d,%d,%d,%d,%u\n",

            sample->seq,

            sample->ax,
            sample->ay,
            sample->az,

            sample->gx,
            sample->gy,
            sample->gz,

            trigger_hint
        );
    }


    xprintf("=== EVENT CSV END ===\n");
    xprintf("\n");

    if (header->fast_count == 0U)
    {
        return;
    }

    /* The Flash payload stores the long history first, then the fast window. */
    const telemetry_t *fast_records = records + header->record_count;
    xprintf("=== FAST FLASH CSV BEGIN ===\n");
    xprintf(
        "event_seq=%u,slot=%u,trigger_fast_seq=%u,records=%u,event_type=%u\n",
        header->event_sequence,
        summary->slot_index,
        header->trigger_fast_seq,
        header->fast_count,
        header->event_type
    );
    xprintf("fast_seq,ax,ay,az,gx,gy,gz,trigger\n");

    for (uint16_t i = 0U; i < header->fast_count; i++)
    {
        const telemetry_t *sample = &fast_records[i];
        xprintf(
            "%u,%d,%d,%d,%d,%d,%d,%u\n",
            sample->seq,
            sample->ax,
            sample->ay,
            sample->az,
            sample->gx,
            sample->gy,
            sample->gz,
            (sample->seq == header->trigger_fast_seq) ? 1U : 0U
        );
    }

    xprintf("=== FAST FLASH CSV END ===\n\n");
}
