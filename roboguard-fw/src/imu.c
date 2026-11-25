#include "imu.h"
#include "xprintf.h"
#include "mik32_hal_gpio.h"
#include "mik32_hal_pcc.h"

#define IMU_ADDR               0x68

#define REG_SMPLRT_DIV         0x19
#define REG_CONFIG             0x1A
#define REG_GYRO_CONFIG        0x1B
#define REG_ACCEL_CONFIG       0x1C
#define REG_ACCEL_CONFIG2      0x1D
#define REG_ACCEL_XOUT_H       0x3B
#define REG_PWR_MGMT_1         0x6B
#define REG_WHO_AM_I           0x75

#define ACCEL_CONFIG_16G       0x18
#define GYRO_CONFIG_2000       0x18

#define SAMPLE_DIV             1
#define DLPF_CONFIG            0x01


#define I2C1_SDA_PIN GPIO_PIN_12
#define I2C1_SCL_PIN GPIO_PIN_13


static void i2c_line_release(HAL_PinsTypeDef pin)
{
    HAL_GPIO_PinConfig(
        GPIO_1,
        pin,
        HAL_GPIO_MODE_GPIO_INPUT,
        HAL_GPIO_PULL_UP,
        HAL_GPIO_DS_2MA
    );
}


static void i2c_line_low(HAL_PinsTypeDef pin)
{
    /*
     * Never drive an I2C line high.
     * Pull low as output; release high as input + pull-up.
     */
    HAL_GPIO_WritePin(
        GPIO_1,
        pin,
        GPIO_PIN_LOW
    );

    HAL_GPIO_PinConfig(
        GPIO_1,
        pin,
        HAL_GPIO_MODE_GPIO_OUTPUT,
        HAL_GPIO_PULL_NONE,
        HAL_GPIO_DS_2MA
    );
}


static HAL_StatusTypeDef i2c_bus_recover(
    I2C_HandleTypeDef *i2c
)
{
    uint8_t clocks = 0;

    HAL_I2C_Disable(i2c);
    i2c->Instance->CR2 = 0;

    __HAL_PCC_GPIO_1_CLK_ENABLE();

    /*
     * Release both lines first.
     */
    i2c_line_release(I2C1_SDA_PIN);
    i2c_line_release(I2C1_SCL_PIN);

    HAL_DelayMs(1);

    xprintf(
        "BUS RECOVERY: before SDA=%d SCL=%d\n",
        HAL_GPIO_ReadPin(GPIO_1, I2C1_SDA_PIN),
        HAL_GPIO_ReadPin(GPIO_1, I2C1_SCL_PIN)
    );

    /*
     * If a slave is holding SDA low, provide up to 9 clock pulses.
     */
    while (
        HAL_GPIO_ReadPin(GPIO_1, I2C1_SDA_PIN) == GPIO_PIN_LOW &&
        clocks < 9
    )
    {
        i2c_line_low(I2C1_SCL_PIN);
        HAL_DelayMs(1);

        i2c_line_release(I2C1_SCL_PIN);
        HAL_DelayMs(1);

        clocks++;
    }

    /*
     * Generate a STOP:
     * SDA low -> SCL released high -> SDA released high.
     */
    i2c_line_low(I2C1_SDA_PIN);
    HAL_DelayMs(1);

    i2c_line_release(I2C1_SCL_PIN);
    HAL_DelayMs(1);

    i2c_line_release(I2C1_SDA_PIN);
    HAL_DelayMs(1);

    xprintf(
        "BUS RECOVERY: clocks=%u after SDA=%d SCL=%d\n",
        clocks,
        HAL_GPIO_ReadPin(GPIO_1, I2C1_SDA_PIN),
        HAL_GPIO_ReadPin(GPIO_1, I2C1_SCL_PIN)
    );

    /*
     * Restore normal I2C pin mux/timing.
     */
    if (HAL_I2C_Init(i2c) != HAL_OK)
    {
        xprintf("BUS RECOVERY: HAL_I2C_Init failed\n");
        return HAL_ERROR;
    }

    return HAL_OK;
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(
        ((uint16_t)p[0] << 8) |
        (uint16_t)p[1]
    );
}


static HAL_StatusTypeDef write_reg(
    imu_t *imu,
    uint8_t reg,
    uint8_t value
)
{
    uint8_t tx[2] = {reg, value};

    return HAL_I2C_Master_Transmit(
        imu->i2c,
        imu->address,
        tx,
        2,
        I2C_TIMEOUT_DEFAULT
    );
}


static HAL_StatusTypeDef read_bytes(
    imu_t *imu,
    uint8_t reg,
    uint8_t *data,
    uint8_t len
)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(
        imu->i2c,
        imu->address,
        &reg,
        1,
        I2C_TIMEOUT_DEFAULT
    );

    if (status != HAL_OK)
    {
        xprintf(
            "I2C TX FAIL: reg=0x%02X status=%d err=%d ISR=%08X CR1=%08X CR2=%08X\n",
            reg,
            status,
            imu->i2c->ErrorCode,
            imu->i2c->Instance->ISR,
            imu->i2c->Instance->CR1,
            imu->i2c->Instance->CR2
        );

        return status;
    }

    status = HAL_I2C_Master_Receive(
        imu->i2c,
        imu->address,
        data,
        len,
        I2C_TIMEOUT_DEFAULT
    );

    if (status != HAL_OK)
    {
        xprintf(
            "I2C RX FAIL: reg=0x%02X status=%d err=%d ISR=%08X CR1=%08X CR2=%08X\n",
            reg,
            status,
            imu->i2c->ErrorCode,
            imu->i2c->Instance->ISR,
            imu->i2c->Instance->CR1,
            imu->i2c->Instance->CR2
        );
    }

    return status;
}


HAL_StatusTypeDef imu_init(
    imu_t *imu,
    I2C_HandleTypeDef *i2c
)
{
    HAL_StatusTypeDef status;

    imu->i2c = i2c;
    imu->address = IMU_ADDR;
    imu->who_am_i = 0;

    /*
     * Give the sensor/bus a little time before first transaction.
     */
    HAL_DelayMs(200);

    xprintf("IMU init: reading WHO_AM_I...\n");

    /*
     * A few retries make startup more robust and also tell us
     * whether this is only a transient first transaction failure.
     */
    for (uint8_t attempt = 1; attempt <= 5; attempt++)
    {
        status = read_bytes(
            imu,
            REG_WHO_AM_I,
            &imu->who_am_i,
            1
        );

        if (status == HAL_OK)
        {
            break;
        }

        xprintf(
            "IMU init: WHO_AM_I attempt %u failed, status=%d, i2c_error=%d\n",
            attempt,
            status,
            imu->i2c->ErrorCode
        );
        xprintf("IMU init: attempting bus recovery\n");

        if (
            i2c_bus_recover(
                imu->i2c
            ) != HAL_OK
        )
        {
            xprintf(
                "IMU init: bus recovery failed\n"
            );
        }

        HAL_DelayMs(100);
    }

    if (status != HAL_OK)
    {
        xprintf("IMU init FAILED at WHO_AM_I transaction\n");
        return status;
    }

    xprintf(
        "IMU init: WHO_AM_I = 0x%02X\n",
        imu->who_am_i
    );

    if (
        imu->who_am_i != 0x68 &&
        imu->who_am_i != 0x70
    )
    {
        xprintf("IMU init FAILED: unsupported WHO_AM_I\n");
        return HAL_ERROR;
    }


    xprintf("IMU init: PWR_MGMT_1...\n");

    status = write_reg(
        imu,
        REG_PWR_MGMT_1,
        0x01
    );

    if (status != HAL_OK)
    {
        xprintf(
            "IMU init FAILED at PWR_MGMT_1, status=%d, i2c_error=%d\n",
            status,
            imu->i2c->ErrorCode
        );
        return status;
    }

    HAL_DelayMs(100);


    xprintf("IMU init: CONFIG...\n");

    status = write_reg(
        imu,
        REG_CONFIG,
        DLPF_CONFIG
    );

    if (status != HAL_OK)
    {
        xprintf(
            "IMU init FAILED at CONFIG, status=%d, i2c_error=%d\n",
            status,
            imu->i2c->ErrorCode
        );
        return status;
    }


    xprintf("IMU init: SMPLRT_DIV...\n");

    status = write_reg(
        imu,
        REG_SMPLRT_DIV,
        SAMPLE_DIV
    );

    if (status != HAL_OK)
    {
        xprintf(
            "IMU init FAILED at SMPLRT_DIV, status=%d, i2c_error=%d\n",
            status,
            imu->i2c->ErrorCode
        );
        return status;
    }


    xprintf("IMU init: GYRO_CONFIG...\n");

    status = write_reg(
        imu,
        REG_GYRO_CONFIG,
        GYRO_CONFIG_2000
    );

    if (status != HAL_OK)
    {
        xprintf(
            "IMU init FAILED at GYRO_CONFIG, status=%d, i2c_error=%d\n",
            status,
            imu->i2c->ErrorCode
        );
        return status;
    }


    xprintf("IMU init: ACCEL_CONFIG...\n");

    status = write_reg(
        imu,
        REG_ACCEL_CONFIG,
        ACCEL_CONFIG_16G
    );

    if (status != HAL_OK)
    {
        xprintf(
            "IMU init FAILED at ACCEL_CONFIG, status=%d, i2c_error=%d\n",
            status,
            imu->i2c->ErrorCode
        );
        return status;
    }


    if (imu->who_am_i == 0x70)
    {
        xprintf("IMU init: ACCEL_CONFIG2...\n");

        status = write_reg(
            imu,
            REG_ACCEL_CONFIG2,
            0x01
        );

        if (status != HAL_OK)
        {
            xprintf(
                "IMU init FAILED at ACCEL_CONFIG2, status=%d, i2c_error=%d\n",
                status,
                imu->i2c->ErrorCode
            );
            return status;
        }
    }


    xprintf("IMU init: OK\n");

    return HAL_OK;
}


HAL_StatusTypeDef imu_read(
    imu_t *imu,
    imu_sample_t *sample
)
{
    uint8_t raw[14];

    HAL_StatusTypeDef status;

    status = read_bytes(
        imu,
        REG_ACCEL_XOUT_H,
        raw,
        sizeof(raw)
    );

    if (status != HAL_OK)
    {
        return status;
    }

    sample->ax = be16(&raw[0]);
    sample->ay = be16(&raw[2]);
    sample->az = be16(&raw[4]);

    sample->gx = be16(&raw[8]);
    sample->gy = be16(&raw[10]);
    sample->gz = be16(&raw[12]);

    return HAL_OK;
}
