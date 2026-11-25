#include "bench_inputs.h"

#include "event_store.h"
#include "mik32_hal_gpio.h"
#include "xprintf.h"

#if defined(RG_LEFT_PORT) != defined(RG_LEFT_PIN)
#error Configure both RG_LEFT_PORT and RG_LEFT_PIN
#endif
#if defined(RG_HEARTBEAT_PORT) != defined(RG_HEARTBEAT_PIN)
#error Configure both RG_HEARTBEAT_PORT and RG_HEARTBEAT_PIN
#endif
#if defined(RG_SHUNT_MILLIOHMS) && RG_SHUNT_MILLIOHMS <= 0
#error RG_SHUNT_MILLIOHMS must be positive
#endif
#if defined(RG_STALL_MIN_MA) && (!defined(RG_LEFT_PIN) || !defined(RG_INA219_ADDRESS) || !defined(RG_SHUNT_MILLIOHMS))
#error Stall detection needs both encoders and a calibrated INA219 shunt
#endif
#if defined(RG_STALL_MIN_MA) && RG_STALL_MIN_MA <= 0
#error RG_STALL_MIN_MA must be positive
#endif

#ifdef RG_LEFT_PIN
static uint8_t left_level;
static uint32_t left_pulses;
#endif

#ifdef RG_HEARTBEAT_PIN
#define HEARTBEAT_TIMEOUT_US 600000U
static uint8_t heartbeat_level;
static uint8_t heartbeat_edges;
static uint32_t last_heartbeat_us;
#endif

#ifdef RG_INA219_ADDRESS
#define INA219_SHUNT_REGISTER 0x01U
#define INA219_BUS_REGISTER 0x02U
#define INA219_TIMEOUT 4000U
#define INA219_REGISTER_PERIOD_US 20000U
static uint32_t last_ina219_us;
static uint8_t ina219_register = INA219_SHUNT_REGISTER;
static uint8_t shunt_valid;
static uint8_t bus_valid;
static int32_t shunt_uv;
static uint32_t bus_mv;

static HAL_StatusTypeDef ina219_read(I2C_HandleTypeDef *i2c, uint8_t reg, uint16_t *value)
{
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
        i2c, RG_INA219_ADDRESS, &reg, 1U, INA219_TIMEOUT
    );
    if (status != HAL_OK)
    {
        return status;
    }
    uint8_t data[2];
    status = HAL_I2C_Master_Receive(i2c, RG_INA219_ADDRESS, data, 2U, INA219_TIMEOUT);
    if (status == HAL_OK)
    {
        *value = ((uint16_t)data[0] << 8) | data[1];
    }
    return status;
}

static uint32_t ina219_bus_mv(uint16_t raw)
{
    return (uint32_t)(raw >> 3) * 4U;
}

HAL_StatusTypeDef bench_inputs_read_bus_mv(I2C_HandleTypeDef *i2c, uint32_t *value_mv)
{
    uint16_t raw;
    HAL_StatusTypeDef status = ina219_read(i2c, INA219_BUS_REGISTER, &raw);
    if (status == HAL_OK)
    {
        *value_mv = ina219_bus_mv(raw);
    }
    else
    {
        HAL_I2C_Reset(i2c);
    }
    return status;
}
#endif

#ifdef RG_STALL_MIN_MA
#define STALL_WINDOW_US 400000U
static uint32_t stall_start_us;
static uint32_t last_right_pulses;
static uint8_t stall_tracking;
static uint8_t stall_reported;
#endif

void bench_inputs_init(void)
{
#if defined(RG_LEFT_PIN) || defined(RG_HEARTBEAT_PIN)
    __HAL_PCC_GPIO_0_CLK_ENABLE();
    __HAL_PCC_GPIO_1_CLK_ENABLE();
    __HAL_PCC_GPIO_2_CLK_ENABLE();
#endif
#ifdef RG_LEFT_PIN
    HAL_GPIO_PinConfig(RG_LEFT_PORT, RG_LEFT_PIN, HAL_GPIO_MODE_GPIO_INPUT,
                      HAL_GPIO_PULL_UP, HAL_GPIO_DS_2MA);
    left_level = HAL_GPIO_ReadPin(RG_LEFT_PORT, RG_LEFT_PIN) == GPIO_PIN_HIGH;
    xprintf("FC03 LEFT ready: level=%u\n", left_level);
#endif
#ifdef RG_HEARTBEAT_PIN
    HAL_GPIO_PinConfig(RG_HEARTBEAT_PORT, RG_HEARTBEAT_PIN, HAL_GPIO_MODE_GPIO_INPUT,
                      HAL_GPIO_PULL_DOWN, HAL_GPIO_DS_2MA);
    heartbeat_level = HAL_GPIO_ReadPin(RG_HEARTBEAT_PORT, RG_HEARTBEAT_PIN) == GPIO_PIN_HIGH;
    heartbeat_edges = 0U;
#endif
#ifdef RG_INA219_ADDRESS
    shunt_valid = 0U;
    bus_valid = 0U;
#endif
#ifdef RG_STALL_MIN_MA
    stall_tracking = 0U;
    stall_reported = 0U;
#endif
}

uint32_t bench_inputs_poll(I2C_HandleTypeDef *i2c, uint32_t now_us, uint32_t right_pulses)
{
    (void)i2c;
    (void)now_us;
    (void)right_pulses;
    uint32_t event_type = 0U;

#ifdef RG_LEFT_PIN
    uint8_t left_now = HAL_GPIO_ReadPin(RG_LEFT_PORT, RG_LEFT_PIN) == GPIO_PIN_HIGH;
    uint8_t left_edge = left_now && !left_level;
    left_pulses += left_edge;
    left_level = left_now;
#endif

#ifdef RG_HEARTBEAT_PIN
    uint8_t heartbeat_now = HAL_GPIO_ReadPin(RG_HEARTBEAT_PORT, RG_HEARTBEAT_PIN) == GPIO_PIN_HIGH;
    if (heartbeat_now && !heartbeat_level)
    {
        heartbeat_edges = now_us - last_heartbeat_us < HEARTBEAT_TIMEOUT_US && heartbeat_edges
            ? 2U : 1U;
        last_heartbeat_us = now_us;
    }
    heartbeat_level = heartbeat_now;
    if (heartbeat_edges == 2U && now_us - last_heartbeat_us >= HEARTBEAT_TIMEOUT_US)
    {
        event_type = EVENT_TYPE_HEARTBEAT_LOST;
        heartbeat_edges = 0U;
    }
#endif

#ifdef RG_INA219_ADDRESS
    if (now_us - last_ina219_us >= INA219_REGISTER_PERIOD_US)
    {
        last_ina219_us = now_us;
        uint16_t raw = 0U;
        uint8_t valid = ina219_read(i2c, ina219_register, &raw) == HAL_OK;
        if (!valid)
        {
            HAL_I2C_Reset(i2c);
            shunt_valid = 0U;
            bus_valid = 0U;
        }
        if (ina219_register == INA219_SHUNT_REGISTER)
        {
            shunt_valid = valid;
            if (valid)
            {
                shunt_uv = (int32_t)(int16_t)raw * 10;
            }
            ina219_register = INA219_BUS_REGISTER;
        }
        else
        {
            bus_valid = valid;
            if (valid)
            {
                bus_mv = ina219_bus_mv(raw);
            }
            ina219_register = INA219_SHUNT_REGISTER;
        }
    }
#endif

#ifdef RG_STALL_MIN_MA
    uint8_t wheel_moved = left_edge || right_pulses != last_right_pulses;
    last_right_pulses = right_pulses;
    int32_t current_ma = shunt_uv / RG_SHUNT_MILLIOHMS;
    if (!shunt_valid || current_ma < RG_STALL_MIN_MA || wheel_moved)
    {
        stall_tracking = 0U;
        stall_reported = 0U;
    }
    else if (!stall_tracking)
    {
        stall_tracking = 1U;
        stall_start_us = now_us;
    }
    else if (!stall_reported && event_type == 0U && now_us - stall_start_us >= STALL_WINDOW_US)
    {
        stall_reported = 1U;
        event_type = EVENT_TYPE_STALL_CANDIDATE;
    }
#endif
    return event_type;
}

void bench_inputs_report_encoder_changes(void)
{
#ifdef RG_LEFT_PIN
    static uint32_t reported_left_pulses;
    if (left_pulses != reported_left_pulses)
    {
        xprintf("FC03 LEFT pulses=%u level=%u\n", left_pulses, left_level);
        reported_left_pulses = left_pulses;
    }
#endif
}

void bench_inputs_report(void)
{
#ifdef RG_LEFT_PIN
    xprintf("FC03 LEFT pulses=%u level=%u\n", left_pulses, left_level);
#endif
#ifdef RG_HEARTBEAT_PIN
    xprintf("Heartbeat active=%u\n", heartbeat_edges == 2U);
#endif
#ifdef RG_INA219_ADDRESS
    if (bus_valid)
    {
        xprintf("INA219 bus=%u mV\n", bus_mv);
    }
    if (shunt_valid)
    {
        xprintf("INA219 shunt=%d uV\n", shunt_uv);
#ifdef RG_SHUNT_MILLIOHMS
        xprintf("INA219 current=%d mA\n", shunt_uv / RG_SHUNT_MILLIOHMS);
#endif
    }
    if (!shunt_valid || !bus_valid)
    {
        xprintf("INA219 incomplete reading (shunt=%u bus=%u)\n", shunt_valid, bus_valid);
    }
#endif
}
