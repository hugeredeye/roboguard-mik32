#ifndef FLASH_PERSIST_TEST_H
#define FLASH_PERSIST_TEST_H

#include <stdint.h>

#define FLASH_TEST_OFFSET       0x007FF000UL
#define FLASH_TEST_XIP_ADDRESS  0x807FF000UL

void flash_persist_test_prepare(void);

int flash_persist_test_present(void);

void flash_persist_test_write_and_wait_reset(void);

#endif
