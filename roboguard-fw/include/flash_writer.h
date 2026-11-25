#ifndef FLASH_WRITER_H
#define FLASH_WRITER_H

#include "event_store.h"
#include "ring_buffer.h"
#include "fast_ring.h"

void flash_writer_save_event_and_wait_reset(
    const event_pending_t *pending,
    const ring_buffer_t *history,
    const fast_ring_t *fast_history
)
__attribute__((noreturn));

#endif
