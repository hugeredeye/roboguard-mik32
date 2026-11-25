#ifndef CRC32_H
#define CRC32_H

#include <stdint.h>

uint32_t crc32_begin(void);

uint32_t crc32_update(
    uint32_t crc,
    const void *data,
    uint32_t length
);

uint32_t crc32_end(
    uint32_t crc
);

uint32_t crc32_compute(
    const void *data,
    uint32_t length
);

#endif
