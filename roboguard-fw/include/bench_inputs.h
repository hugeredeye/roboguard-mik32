#ifndef BENCH_INPUTS_H
#define BENCH_INPUTS_H

#include <stdint.h>
#include "mik32_hal_i2c.h"

/*
 * Optional build flags for the wired bench:
 * RG_LEFT_PORT and RG_HEARTBEAT_PORT are GPIO_0, GPIO_1 or GPIO_2.
 * RG_LEFT_PIN and RG_HEARTBEAT_PIN are GPIO_PIN_n masks.
 * RG_INA219_ADDRESS is the module's 7-bit I2C address.
 * RG_SHUNT_MILLIOHMS is the measured or marked shunt resistance.
 * RG_STALL_MIN_MA enables the stall candidate when the left input and
 * INA219 current are both configured. Set from measured motor behavior.
 */
void bench_inputs_init(void);
uint32_t bench_inputs_poll(I2C_HandleTypeDef *i2c, uint32_t now_us, uint32_t right_pulses);
void bench_inputs_report_encoder_changes(void);
void bench_inputs_report(void);
#ifdef RG_INA219_ADDRESS
HAL_StatusTypeDef bench_inputs_read_bus_mv(I2C_HandleTypeDef *i2c, uint32_t *value_mv);
#endif

#endif
