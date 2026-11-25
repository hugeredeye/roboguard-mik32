#include "flash_writer.h"

#include <stdint.h>

#include "mik32_hal_spifi.h"
#include "mik32_hal_usart.h"


#define RAMFUNC \
    __attribute__((section(".ram_text"), noinline, used))

#define RG_TIMEOUT           1000000UL
#define RG_BUSY_ATTEMPTS     800000UL
#define RG_UART_TIMEOUT      1000000UL


static uint8_t g_page_buffer[256]
    __attribute__((aligned(4)));

static uint8_t g_verify_buffer[256]
    __attribute__((aligned(4)));


static RAMFUNC void rg_uart_char(char c)
{
    uint32_t timeout =
        RG_UART_TIMEOUT;

    while (
        ((UART_0->FLAGS &
          UART_FLAGS_TXE_M) == 0U) &&
        timeout-- != 0U
    )
    {
    }

    UART_0->TXDATA = c;

    timeout =
        RG_UART_TIMEOUT;

    while (
        ((UART_0->FLAGS &
          UART_FLAGS_TC_M) == 0U) &&
        timeout-- != 0U
    )
    {
    }
}


static RAMFUNC void rg_uart_newline(void)
{
    rg_uart_char('\r');
    rg_uart_char('\n');
}


static RAMFUNC void rg_disable_interrupts(void)
{
    uint32_t mask = 0x8U;

    __asm__ volatile(
        "csrc mstatus, %0"
        :
        : "r"(mask)
        : "memory"
    );
}


static RAMFUNC void rg_delay(void)
{
    volatile uint32_t i;

    for (
        i = 0;
        i < 150000U;
        i++
    )
    {
        __asm__ volatile("nop");
    }
}


static RAMFUNC int rg_wait_reset_clear(void)
{
    uint32_t timeout =
        RG_TIMEOUT;

    while (timeout-- != 0U)
    {
        if (
            (SPIFI_CONFIG->STAT &
             SPIFI_CONFIG_STAT_RESET_M) == 0U
        )
        {
            return 0;
        }
    }

    return -1;
}


static RAMFUNC int rg_wait_command(void)
{
    uint32_t timeout =
        RG_TIMEOUT;

    while (timeout-- != 0U)
    {
        if (
            (SPIFI_CONFIG->STAT &
             SPIFI_CONFIG_STAT_INTRQ_M) != 0U
        )
        {
            return 0;
        }
    }

    return -1;
}


static RAMFUNC int rg_send_opcode(
    uint8_t opcode,
    uint32_t fieldform
)
{
    uint32_t command =
        SPIFI_DIRECTION_INPUT |

        SPIFI_CONFIG_CMD_INTLEN(0) |

        SPIFI_CONFIG_CMD_FIELDFORM(
            fieldform
        ) |

        SPIFI_CONFIG_CMD_FRAMEFORM(
            SPIFI_FRAMEFORM_OPCODE
        ) |

        SPIFI_CONFIG_CMD_OPCODE(
            opcode
        );


    SPIFI_CONFIG->STAT |=
        SPIFI_CONFIG_STAT_INTRQ_M;

    SPIFI_CONFIG->ADDR = 0;
    SPIFI_CONFIG->IDATA = 0;

    SPIFI_CONFIG->CMD =
        command |
        SPIFI_CONFIG_CMD_DATALEN(0);


    return rg_wait_command();
}


static RAMFUNC int rg_read_opcode(
    uint8_t opcode,
    uint8_t *data,
    uint16_t length
)
{
    uint16_t i;


    uint32_t command =
        SPIFI_DIRECTION_INPUT |

        SPIFI_CONFIG_CMD_INTLEN(0) |

        SPIFI_CONFIG_CMD_FIELDFORM(
            SPIFI_FIELDFORM_ALL_SERIAL
        ) |

        SPIFI_CONFIG_CMD_FRAMEFORM(
            SPIFI_FRAMEFORM_OPCODE
        ) |

        SPIFI_CONFIG_CMD_OPCODE(
            opcode
        );


    SPIFI_CONFIG->STAT |=
        SPIFI_CONFIG_STAT_INTRQ_M;

    SPIFI_CONFIG->ADDR = 0;
    SPIFI_CONFIG->IDATA = 0;

    SPIFI_CONFIG->CMD =
        command |
        SPIFI_CONFIG_CMD_DATALEN(
            length
        );


    for (
        i = 0;
        i < length;
        i++
    )
    {
        data[i] =
            (uint8_t)
            SPIFI_CONFIG->DATA8;
    }


    return rg_wait_command();
}


static RAMFUNC int rg_flash_reset_to_spi(void)
{
    if (
        rg_send_opcode(
            0x66,
            SPIFI_FIELDFORM_ALL_PARALLEL
        ) != 0
    )
    {
        return -1;
    }

    if (
        rg_send_opcode(
            0x99,
            SPIFI_FIELDFORM_ALL_PARALLEL
        ) != 0
    )
    {
        return -2;
    }

    rg_delay();


    if (
        rg_send_opcode(
            0x66,
            SPIFI_FIELDFORM_ALL_SERIAL
        ) != 0
    )
    {
        return -3;
    }

    if (
        rg_send_opcode(
            0x99,
            SPIFI_FIELDFORM_ALL_SERIAL
        ) != 0
    )
    {
        return -4;
    }

    rg_delay();

    return 0;
}


static RAMFUNC int rg_read_status(
    uint8_t *status
)
{
    return rg_read_opcode(
        0x05,
        status,
        1
    );
}


static RAMFUNC int rg_write_enable(void)
{
    uint8_t status = 0;


    if (
        rg_send_opcode(
            0x06,
            SPIFI_FIELDFORM_ALL_SERIAL
        ) != 0
    )
    {
        return -1;
    }


    if (
        rg_read_status(
            &status
        ) != 0
    )
    {
        return -2;
    }


    if ((status & 0x02U) == 0U)
    {
        return -3;
    }


    return 0;
}


static RAMFUNC int rg_wait_not_busy(void)
{
    uint32_t attempts =
        RG_BUSY_ATTEMPTS;


    while (attempts-- != 0U)
    {
        uint8_t status = 0xFF;


        if (
            rg_read_status(
                &status
            ) != 0
        )
        {
            return -1;
        }


        if ((status & 0x01U) == 0U)
        {
            return 0;
        }
    }


    return -2;
}


static RAMFUNC int rg_sector_erase_4k(
    uint32_t address
)
{
    uint32_t command;


    if (rg_write_enable() != 0)
    {
        return -1;
    }


    command =
        SPIFI_DIRECTION_INPUT |

        SPIFI_CONFIG_CMD_INTLEN(0) |

        SPIFI_CONFIG_CMD_FIELDFORM(
            SPIFI_FIELDFORM_ALL_SERIAL
        ) |

        SPIFI_CONFIG_CMD_FRAMEFORM(
            SPIFI_FRAMEFORM_OPCODE_3ADDR
        ) |

        SPIFI_CONFIG_CMD_OPCODE(0x20);


    SPIFI_CONFIG->STAT |=
        SPIFI_CONFIG_STAT_INTRQ_M;

    SPIFI_CONFIG->ADDR =
        address;

    SPIFI_CONFIG->IDATA = 0;

    SPIFI_CONFIG->CMD =
        command |
        SPIFI_CONFIG_CMD_DATALEN(0);


    if (rg_wait_command() != 0)
    {
        return -2;
    }


    if (rg_wait_not_busy() != 0)
    {
        return -3;
    }


    return 0;
}


static RAMFUNC int rg_page_program(
    uint32_t address,
    const uint8_t *data,
    uint16_t length
)
{
    uint32_t command;


    if (
        length == 0U ||
        length > 256U
    )
    {
        return -10;
    }


    if (rg_write_enable() != 0)
    {
        return -1;
    }


    command =
        SPIFI_DIRECTION_OUTPUT |

        SPIFI_CONFIG_CMD_INTLEN(0) |

        SPIFI_CONFIG_CMD_FIELDFORM(
            SPIFI_FIELDFORM_ALL_SERIAL
        ) |

        SPIFI_CONFIG_CMD_FRAMEFORM(
            SPIFI_FRAMEFORM_OPCODE_3ADDR
        ) |

        SPIFI_CONFIG_CMD_OPCODE(0x02);


    SPIFI_CONFIG->STAT |=
        SPIFI_CONFIG_STAT_INTRQ_M;

    SPIFI_CONFIG->ADDR =
        address;

    SPIFI_CONFIG->IDATA = 0;

    SPIFI_CONFIG->CMD =
        command |
        SPIFI_CONFIG_CMD_DATALEN(
            length
        );


    for (
        uint16_t i = 0;
        i < length;
        i++
    )
    {
        SPIFI_CONFIG->DATA8 =
            data[i];
    }


    if (rg_wait_command() != 0)
    {
        return -2;
    }


    if (rg_wait_not_busy() != 0)
    {
        return -3;
    }


    return 0;
}


static RAMFUNC int rg_read_data(
    uint32_t address,
    uint8_t *data,
    uint16_t length
)
{
    uint32_t command;


    command =
        SPIFI_DIRECTION_INPUT |

        SPIFI_CONFIG_CMD_INTLEN(0) |

        SPIFI_CONFIG_CMD_FIELDFORM(
            SPIFI_FIELDFORM_ALL_SERIAL
        ) |

        SPIFI_CONFIG_CMD_FRAMEFORM(
            SPIFI_FRAMEFORM_OPCODE_3ADDR
        ) |

        SPIFI_CONFIG_CMD_OPCODE(0x03);


    SPIFI_CONFIG->STAT |=
        SPIFI_CONFIG_STAT_INTRQ_M;

    SPIFI_CONFIG->ADDR =
        address;

    SPIFI_CONFIG->IDATA = 0;

    SPIFI_CONFIG->CMD =
        command |
        SPIFI_CONFIG_CMD_DATALEN(
            length
        );


    for (
        uint16_t i = 0;
        i < length;
        i++
    )
    {
        data[i] =
            (uint8_t)
            SPIFI_CONFIG->DATA8;
    }


    return rg_wait_command();
}


static RAMFUNC int rg_compare(
    const uint8_t *a,
    const uint8_t *b,
    uint16_t length
)
{
    for (
        uint16_t i = 0;
        i < length;
        i++
    )
    {
        if (a[i] != b[i])
        {
            return -1;
        }
    }

    return 0;
}


static RAMFUNC void rg_fail(
    char stage
)
{
    rg_uart_char('!');
    rg_uart_char(stage);
    rg_uart_newline();

    while (1)
    {
        __asm__ volatile("nop");
    }
}


RAMFUNC
__attribute__((noreturn))
void flash_writer_save_event_and_wait_reset(
    const event_pending_t *pending,
    const ring_buffer_t *history,
    const fast_ring_t *fast_history
)
{
    uint8_t jedec[3] =
        {0xFF, 0xFF, 0xFF};


    rg_disable_interrupts();

    rg_uart_char('0');


    SPIFI_CONFIG->STAT =
        SPIFI_CONFIG_STAT_RESET_M;


    if (rg_wait_reset_clear() != 0)
    {
        rg_fail('X');
    }


    SPIFI_CONFIG->ADDR = 0;
    SPIFI_CONFIG->IDATA = 0;
    SPIFI_CONFIG->CLIMIT = 0;


    rg_uart_char('R');


    if (
        rg_flash_reset_to_spi() != 0
    )
    {
        rg_fail('Q');
    }


    if (
        rg_read_opcode(
            0x9F,
            jedec,
            3
        ) != 0
    )
    {
        rg_fail('J');
    }


    if (
        jedec[0] != 0xEF ||
        jedec[1] != 0x40 ||
        jedec[2] != 0x17
    )
    {
        rg_fail('J');
    }


    rg_uart_char('J');


    if (
        rg_sector_erase_4k(
            pending->slot_offset
        ) != 0
    )
    {
        rg_fail('E');
    }


    if (
        rg_sector_erase_4k(
            pending->slot_offset +
            0x1000UL
        ) != 0
    )
    {
        rg_fail('E');
    }


    rg_uart_char('E');


    /*
     * Header first.
     * Commit marker is intentionally NOT written yet.
     */
    const uint8_t *header_bytes =
        (const uint8_t *)
        &pending->header;


    if (
        rg_page_program(
            pending->slot_offset,
            header_bytes,
            sizeof(event_header_t)
        ) != 0
    )
    {
        rg_fail('H');
    }


    if (
        rg_read_data(
            pending->slot_offset,
            g_verify_buffer,
            sizeof(event_header_t)
        ) != 0
    )
    {
        rg_fail('H');
    }


    if (
        rg_compare(
            header_bytes,
            g_verify_buffer,
            sizeof(event_header_t)
        ) != 0
    )
    {
        rg_fail('H');
    }


    rg_uart_char('H');


    /*
     * Telemetry payload.
     *
     * 16 records * 16 bytes = one 256-byte page.
     */
    uint16_t record_position = 0;

    uint32_t flash_address =
        pending->slot_offset +
        EVENT_STORE_PAYLOAD_OFFSET;


    while (
        record_position <
        pending->header.record_count
    )
    {
        uint16_t records_this_page =
            (uint16_t)(
                pending->header.record_count -
                record_position
            );


        if (records_this_page > 16U)
        {
            records_this_page = 16U;
        }


        uint16_t bytes_this_page =
            (uint16_t)(
                records_this_page *
                sizeof(telemetry_t)
            );


        for (
            uint16_t r = 0;
            r < records_this_page;
            r++
        )
        {
            uint16_t index =
                (uint16_t)(
                    pending->oldest_index +
                    record_position +
                    r
                );


            if (index >= RING_BUFFER_SIZE)
            {
                index -= RING_BUFFER_SIZE;
            }


            const uint8_t *source =
                (const uint8_t *)
                &history->data[index];


            for (
                uint16_t b = 0;
                b < sizeof(telemetry_t);
                b++
            )
            {
                g_page_buffer[
                    r *
                    sizeof(telemetry_t) +
                    b
                ] = source[b];
            }
        }


        if (
            rg_page_program(
                flash_address,
                g_page_buffer,
                bytes_this_page
            ) != 0
        )
        {
            rg_fail('P');
        }


        if (
            rg_read_data(
                flash_address,
                g_verify_buffer,
                bytes_this_page
            ) != 0
        )
        {
            rg_fail('P');
        }


        if (
            rg_compare(
                g_page_buffer,
                g_verify_buffer,
                bytes_this_page
            ) != 0
        )
        {
            rg_fail('P');
        }


        flash_address +=
            bytes_this_page;

        record_position +=
            records_this_page;
    }


    /*
     * High-rate collision window follows the slow payload.
     *
     * Access the fast ring directly here. This function runs from RAM
     * after XIP is disabled, so it must not call normal Flash-resident
     * helpers such as fast_ring_get().
     */
    record_position = 0;


    while (
        record_position <
        pending->header.fast_count
    )
    {
        uint16_t records_this_page =
            (uint16_t)(
                pending->header.fast_count -
                record_position
            );


        /*
         * The slow payload may end in the middle of a 256-byte
         * Flash page if the collision happens before the slow ring
         * has filled. Never let Page Program cross a page boundary.
         */
        uint16_t page_bytes_remaining =
            (uint16_t)(
                256U -
                (flash_address & 0xFFU)
            );

        uint16_t page_records_remaining =
            (uint16_t)(
                page_bytes_remaining /
                sizeof(telemetry_t)
            );


        if (
            records_this_page >
            page_records_remaining
        )
        {
            records_this_page =
                page_records_remaining;
        }


        if (records_this_page > 16U)
        {
            records_this_page = 16U;
        }


        uint16_t bytes_this_page =
            (uint16_t)(
                records_this_page *
                sizeof(telemetry_t)
            );


        for (
            uint16_t r = 0;
            r < records_this_page;
            r++
        )
        {
            uint16_t index =
                (uint16_t)(
                    pending->fast_oldest_index +
                    record_position +
                    r
                );


            if (index >= FAST_RING_SIZE)
            {
                index -= FAST_RING_SIZE;
            }


            const uint8_t *source =
                (const uint8_t *)
                &fast_history->data[index];


            for (
                uint16_t b = 0;
                b < sizeof(telemetry_t);
                b++
            )
            {
                g_page_buffer[
                    r *
                    sizeof(telemetry_t) +
                    b
                ] = source[b];
            }
        }


        if (
            rg_page_program(
                flash_address,
                g_page_buffer,
                bytes_this_page
            ) != 0
        )
        {
            rg_fail('P');
        }


        if (
            rg_read_data(
                flash_address,
                g_verify_buffer,
                bytes_this_page
            ) != 0
        )
        {
            rg_fail('P');
        }


        if (
            rg_compare(
                g_page_buffer,
                g_verify_buffer,
                bytes_this_page
            ) != 0
        )
        {
            rg_fail('P');
        }


        flash_address +=
            bytes_this_page;

        record_position +=
            records_this_page;
    }


    if (
        flash_address !=
        (
            pending->slot_offset +
            EVENT_STORE_PAYLOAD_OFFSET +
            pending->header.payload_bytes
        )
    )
    {
        rg_fail('L');
    }


    rg_uart_char('P');


    /*
     * Commit written LAST.
     *
     * A power loss before this point leaves the slot invalid.
     */
    event_commit_t commit;

    commit.magic =
        EVENT_COMMIT_MAGIC;

    commit.event_sequence =
        pending->header.event_sequence;

    commit.header_crc32 =
        pending->header.header_crc32;

    commit.payload_crc32 =
        pending->header.payload_crc32;


    const uint8_t *commit_bytes =
        (const uint8_t *)
        &commit;


    uint32_t commit_address =
        pending->slot_offset +
        EVENT_STORE_COMMIT_OFFSET;


    if (
        rg_page_program(
            commit_address,
            commit_bytes,
            sizeof(event_commit_t)
        ) != 0
    )
    {
        rg_fail('C');
    }


    if (
        rg_read_data(
            commit_address,
            g_verify_buffer,
            sizeof(event_commit_t)
        ) != 0
    )
    {
        rg_fail('C');
    }


    if (
        rg_compare(
            commit_bytes,
            g_verify_buffer,
            sizeof(event_commit_t)
        ) != 0
    )
    {
        rg_fail('C');
    }


    rg_uart_char('C');
    rg_uart_newline();


    /*
     * Never return to XIP.
     * WDT reboots the MIK32.
     */
    while (1)
    {
        __asm__ volatile("nop");
    }
}
