#ifndef ROBOGUARD_MOTOR_BENCH_H
#define ROBOGUARD_MOTOR_BENCH_H

#include "mik32_hal_i2c.h"
#include "mik32_hal_usart.h"

_Noreturn void motor_bench_run(USART_HandleTypeDef *uart, I2C_HandleTypeDef *i2c);

#endif
