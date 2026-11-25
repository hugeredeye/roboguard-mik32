#include "crc32.h"

uint32_t crc32_begin(void)
{
    return 0xFFFFFFFFUL;
}

uint32_t crc32_update(
    uint32_t crc,
    const void *data,
    uint32_t length
)
{
    const uint8_t *bytes =
        (const uint8_t *)data;

    for (
        uint32_t i = 0;
        i < length;
        i++
    )
    {
        crc ^= bytes[i];

        for (
            uint8_t bit = 0;
            bit < 8;
            bit++
        )
        {
            uint32_t mask =
                0U - (crc & 1U);

            crc =
                (crc >> 1) ^
                (0xEDB88320UL & mask);
        }
    }

    return crc;
}

uint32_t crc32_end(
    uint32_t crc
)
{
    return crc ^ 0xFFFFFFFFUL;
}

uint32_t crc32_compute(
    const void *data,
    uint32_t length
)
{
    return crc32_end(
        crc32_update(
            crc32_begin(),
            data,
            length
        )
    );
}
