#ifdef RG_MOTOR_BENCH

#include "motor_bench.h"
#include "bench_inputs.h"
#include "mik32_hal_gpio.h"
#include "mik32_hal_timer32.h"
#include "mik32_hal_wdt.h"
#include "xprintf.h"

// ELBEAR ACE-UNO header pins, all on PORT1.
#define MOTOR_AIN1 GPIO_PIN_8  // D7
#define MOTOR_AIN2 GPIO_PIN_9  // D8
#define MOTOR_BIN1 GPIO_PIN_1  // D11
#define MOTOR_BIN2 GPIO_PIN_0  // D12
#define MOTOR_STBY GPIO_PIN_2  // D13
#define MOTOR_INPUTS (MOTOR_AIN1 | MOTOR_AIN2 | MOTOR_BIN1 | MOTOR_BIN2 | MOTOR_STBY)

#define RIGHT_ENCODER GPIO_PIN_8  // D4, PORT0
#define LEFT_ENCODER GPIO_PIN_1   // D5, PORT0

#define MOTOR_PULSE_US 300000U
#define SECOND_MOTOR_DELAY_US 180000U
#define PWM_PERIOD_US 1000U
#define SINGLE_ON_US 500U
#define PAIRED_ON_US 850U
#define SECOND_MOTOR_KICK_US 100000U
#define DRIVER_WAKE_US 1000U
#define VOLTAGE_SAMPLE_DELAY_US 100000U

typedef struct {
    HAL_PinsTypeDef first_motor;
    HAL_PinsTypeDef second_motor;
} motor_drive_t;

static void report_voltage(const char *stage, HAL_StatusTypeDef status, uint32_t voltage_mv)
{
    if (status == HAL_OK)
    {
        xprintf("INA219 %s=%u mV\n", stage, voltage_mv);
    }
    else
    {
        xprintf("INA219 %s unavailable (%d)\n", stage, status);
    }
}

static void run_pulse(USART_HandleTypeDef *uart, I2C_HandleTypeDef *i2c, motor_drive_t drive)
{
    uint32_t right_pulses = 0;
    uint32_t left_pulses = 0;
    GPIO_PinState right = HAL_GPIO_ReadPin(GPIO_0, RIGHT_ENCODER);
    GPIO_PinState left = HAL_GPIO_ReadPin(GPIO_0, LEFT_ENCODER);
    HAL_PinsTypeDef active_pins = 0U;
    HAL_StatusTypeDef voltage_status = HAL_ERROR;
    uint32_t sampled_mv = 0U;
    uint8_t voltage_sampled = 0U;

    HAL_GPIO_WritePin(GPIO_1, MOTOR_STBY, GPIO_PIN_HIGH);
    HAL_Time_TIM32_DelayUs(DRIVER_WAKE_US);
    const uint32_t lead_us = drive.second_motor != 0U ? SECOND_MOTOR_DELAY_US : 0U;
    const uint32_t on_us = lead_us != 0U ? PAIRED_ON_US : SINGLE_ON_US;
    uint32_t start_us = HAL_Time_TIM32_Micros();
    uint32_t elapsed_us;
    while ((elapsed_us = HAL_Time_TIM32_Micros() - start_us) < lead_us + MOTOR_PULSE_US)
    {
        if (HAL_USART_RXNE_ReadFlag(uart) && HAL_USART_ReadByte(uart) == '0')
        {
            break;
        }

        if (!voltage_sampled && elapsed_us >= lead_us + VOLTAGE_SAMPLE_DELAY_US)
        {
            // Drop the active inputs before the blocking I2C read of the last ADC sample.
            HAL_GPIO_WritePin(GPIO_1, active_pins, GPIO_PIN_LOW);
            active_pins = 0U;
            voltage_status = bench_inputs_read_bus_mv(i2c, &sampled_mv);
            voltage_sampled = 1U;
            continue;
        }

        uint32_t phase = elapsed_us % PWM_PERIOD_US;
        HAL_PinsTypeDef next_pins = phase < on_us || elapsed_us < lead_us ? drive.first_motor : 0U;
        if (drive.second_motor != 0U && elapsed_us >= lead_us)
        {
            const uint32_t second_phase = (phase + PWM_PERIOD_US / 2U) % PWM_PERIOD_US;
            if (elapsed_us - lead_us < SECOND_MOTOR_KICK_US || second_phase < on_us)
            {
                next_pins |= drive.second_motor;
            }
        }
        if (next_pins != active_pins)
        {
            HAL_GPIO_WritePin(GPIO_1, active_pins & ~next_pins, GPIO_PIN_LOW);
            HAL_GPIO_WritePin(GPIO_1, next_pins & ~active_pins, GPIO_PIN_HIGH);
            active_pins = next_pins;
        }

        GPIO_PinState next_right = HAL_GPIO_ReadPin(GPIO_0, RIGHT_ENCODER);
        GPIO_PinState next_left = HAL_GPIO_ReadPin(GPIO_0, LEFT_ENCODER);
        right_pulses += right == GPIO_PIN_LOW && next_right == GPIO_PIN_HIGH;
        left_pulses += left == GPIO_PIN_LOW && next_left == GPIO_PIN_HIGH;
        right = next_right;
        left = next_left;
    }

    HAL_GPIO_WritePin(GPIO_1, MOTOR_INPUTS, GPIO_PIN_LOW);
    xprintf("Motor stopped: right pulses=%u left pulses=%u\n", right_pulses, left_pulses);
    if (voltage_sampled)
    {
        report_voltage("pulse sample", voltage_status, sampled_mv);
    }
}

_Noreturn void motor_bench_run(USART_HandleTypeDef *uart, I2C_HandleTypeDef *i2c)
{
    __HAL_PCC_GPIO_0_CLK_ENABLE();
    __HAL_PCC_GPIO_1_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIO_1, MOTOR_INPUTS, GPIO_PIN_LOW);
    HAL_GPIO_PinConfig(GPIO_1, MOTOR_INPUTS, HAL_GPIO_MODE_GPIO_OUTPUT,
                      HAL_GPIO_PULL_DOWN, HAL_GPIO_DS_2MA);
    HAL_GPIO_PinConfig(GPIO_0, RIGHT_ENCODER | LEFT_ENCODER, HAL_GPIO_MODE_GPIO_INPUT,
                      HAL_GPIO_PULL_UP, HAL_GPIO_DS_2MA);
    HAL_Time_TIM32_Init(TIMER32_0);

    WDT_HandleTypeDef watchdog = {
        .Instance = WDT,
        .Init = { .Clock = HAL_WDT_OSC32K, .ReloadMs = 750U },
    };
    if (HAL_WDT_Init(&watchdog, WDT_TIMEOUT_DEFAULT) != HAL_OK ||
        HAL_WDT_Start(&watchdog, WDT_TIMEOUT_DEFAULT) != HAL_OK)
    {
        xprintf("Motor test disabled: watchdog initialization failed\n");
        for (;;) { }
    }

    const motor_drive_t drives[] = {
        { MOTOR_AIN1, 0U },
        { MOTOR_AIN2, 0U },
        { MOTOR_BIN1, 0U },
        { MOTOR_BIN2, 0U },
        // Start the right motor first, with both wheels driving toward the caster.
        { MOTOR_AIN2, MOTOR_BIN1 },
    };
    xprintf("Motor test ready: 1/2 right, 3/4 left, 5 right-first boost, v voltage, 0 stop\n");
    for (;;)
    {
        // Allow the watchdog to expire if a pulse ever fails to return.
        if (HAL_WDT_Refresh(&watchdog, WDT_TIMEOUT_DEFAULT) != HAL_OK)
        {
            continue;
        }
        if (HAL_USART_RXNE_ReadFlag(uart))
        {
            char command = HAL_USART_ReadByte(uart);
            if (command >= '1' && command <= '5')
            {
                run_pulse(uart, i2c, drives[command - '1']);
            }
            else if (command == 'v')
            {
                uint32_t voltage_mv = 0U;
                HAL_StatusTypeDef status = bench_inputs_read_bus_mv(i2c, &voltage_mv);
                report_voltage("idle", status, voltage_mv);
            }
        }
        HAL_Time_TIM32_DelayMs(5);
    }
}

#endif
