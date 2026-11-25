#include <stdint.h>

#include "mik32_hal_i2c.h"
#include "mik32_hal_usart.h"
#include "mik32_hal_wdt.h"
#include "mik32_hal_timer32.h"
#include "mik32_hal_gpio.h"

#include "xprintf.h"

#include "imu.h"
#include "telemetry.h"
#include "ring_buffer.h"
#include "fast_ring.h"
#include "collision.h"

#include "event_store.h"
#include "flash_writer.h"
#include "event_dump.h"
#include "bench_inputs.h"
#ifdef RG_MOTOR_BENCH
#include "motor_bench.h"
#endif


#define BAUDRATE 115200UL

#define TELEMETRY_DECIMATION 5
#define FAST_SAMPLE_PERIOD_US 2000U

#define POST_EVENT_SAMPLES 100

#define FAST_POST_EVENT_SAMPLES 31U
#define IMU_READ_RETRIES 3U


typedef enum
{
    RECORDER_ARMED = 0,
    RECORDER_POST_EVENT,
    RECORDER_FROZEN

} recorder_state_t;


I2C_HandleTypeDef hi2c;
USART_HandleTypeDef husart;
WDT_HandleTypeDef hwdt;

imu_t imu;
ring_buffer_t history;
fast_ring_t fast_history;


static void SystemClock_Config(void);
static void I2C_Init(void);
static void IMU_WaitUntilReady(void);
static void USART_Init(void);
static void WDT_Init(void);


int main(void)
{
    SystemClock_Config();
    USART_Init();
    I2C_Init();
#ifdef RG_MOTOR_BENCH
    motor_bench_run(&husart, &hi2c);
#endif
    WDT_Init();


    xprintf("\n");
    xprintf(
        "=== ROBOGUARD EVENT RECORDER ===\n"
    );


    event_summary_t saved_event;

    if (
        event_store_find_latest(
            &saved_event
        ) == 0
    )
    {
        xprintf(
            "FLASH EVENT FOUND\n"
        );

        xprintf(
            "Event seq=%u slot=%u type=%u\n",
            saved_event.header.event_sequence,
            saved_event.slot_index,
            saved_event.header.event_type
        );

        xprintf(
            "Records=%u post=%u\n",
            saved_event.header.record_count,
            saved_event.header.post_count
        );

        xprintf(
            "Telemetry seq: first=%u trigger~=%u last=%u\n",
            saved_event.header.first_seq,
            saved_event.header.trigger_telemetry_seq,
            saved_event.header.last_seq
        );

        xprintf(
            "Trigger acceleration=%u.%02u g\n",
            saved_event.header.trigger_magnitude_centi_g / 100U,
            saved_event.header.trigger_magnitude_centi_g % 100U
        );

        xprintf(
            "Flags: accel_sat=%u gyro_sat=%u\n",
            (
                saved_event.header.flags &
                EVENT_FLAG_ACCEL_SAT
            ) ? 1U : 0U,
            (
                saved_event.header.flags &
                EVENT_FLAG_GYRO_SAT
            ) ? 1U : 0U
        );

        xprintf(
            "CRC OK: header=%08X payload=%08X\n",
            saved_event.header.header_crc32,
            saved_event.header.payload_crc32
        );

        event_dump_csv(&saved_event);
    }
    else
    {
        xprintf(
            "No committed event in Flash.\n"
        );
    }


    IMU_WaitUntilReady();


    ring_buffer_init(
        &history
    );

    fast_ring_init(
        &fast_history
    );

    xprintf(
        "Fast ring: %u full-rate records\n",
        FAST_RING_SIZE
    );


    xprintf(
        "Ring buffer: %u telemetry records\n",
        RING_BUFFER_SIZE
    );

    xprintf(
        "Collision threshold: %u.%02u g\n",
        COLLISION_THRESHOLD_CENTI_G / 100U,
        COLLISION_THRESHOLD_CENTI_G % 100U
    );

    xprintf(
        "Post-event capture: %u telemetry records\n",
        POST_EVENT_SAMPLES
    );
    xprintf("UART: send c to capture a labeled test, s for sensor status\n");


    /*
     * FC-03 RIGHT bring-up.
     * ELBEAR D4 = PORT 0.8.
     */
    __HAL_PCC_GPIO_0_CLK_ENABLE();

    HAL_GPIO_PinConfig(
        GPIO_0,
                GPIO_PIN_8,
        HAL_GPIO_MODE_GPIO_INPUT,
        HAL_GPIO_PULL_UP,
        HAL_GPIO_DS_2MA
    );

    uint8_t fc03_right_level =
        (
            HAL_GPIO_ReadPin(
                GPIO_0,
                GPIO_PIN_8
            ) == GPIO_PIN_HIGH
        ) ? 1U : 0U;

    uint32_t fc03_right_pulses = 0U;
    uint32_t fc03_right_reported_pulses = 0U;
    uint16_t fc03_right_report_divider = 0U;

    xprintf(
        "FC03 RIGHT ready: D4 / PORT0.8, level=%u\n",
        fc03_right_level
    );
    bench_inputs_init();

    uint32_t telemetry_sequence = 0;
    uint32_t fast_sequence = 0;

    uint32_t event_fast_sequence = 0;
    uint32_t event_telemetry_sequence = 0;

    imu_sample_t event_trigger_sample = {0};

    uint32_t event_trigger_magnitude = 0;
    uint32_t event_type = EVENT_TYPE_ACCEL_TRIGGER;


    uint8_t decimation_counter = 0;
    uint16_t report_counter = 0;

    uint16_t post_event_counter = 0;

    uint8_t fast_post_remaining = 0;
    uint8_t fast_capture_frozen = 0;


    recorder_state_t recorder_state =
        RECORDER_ARMED;


    /*
     * TIMER32_0 is used as a free-running microsecond timebase.
     * Sampling is scheduled against absolute deadlines so loop
     * execution time does not accumulate as sampling-period drift.
     */
    HAL_Time_TIM32_Init(TIMER32_0);

    uint32_t next_sample_us =
        HAL_Time_TIM32_Micros() +
        FAST_SAMPLE_PERIOD_US;

    uint32_t sample_deadline_misses = 0;
    uint32_t imu_read_failures = 0U;
    uint8_t consecutive_imu_errors = 0U;


    /*
     * Diagnostic acceleration-step measurement.
     *
     * delta-|a| is measured in centi-g between adjacent
     * successful full-rate IMU samples.
     *
     * Keep 32 pre-trigger deltas + trigger delta so the
     * diagnostic window matches the collision fast window.
     */
    uint32_t previous_magnitude_centi_g = 0;
    uint8_t previous_magnitude_valid = 0U;

    uint32_t delta_history[33] = {0};
    uint8_t delta_history_head = 0U;
    uint8_t delta_history_count = 0U;

    uint32_t event_trigger_delta_centi_g = 0;
    uint32_t event_peak_delta_centi_g = 0;


    while (1)
    {
        uint32_t now_us;

        /*
         * Wait for the next absolute 2 ms sampling deadline.
         * Signed subtraction keeps the comparison safe across
         * the 32-bit timer wraparound.
         */
        do
        {
            now_us = HAL_Time_TIM32_Micros();
        }
        while (
            (int32_t)(now_us - next_sample_us) < 0
        );


        /*
         * If processing took longer than one sampling period,
         * do not "catch up" with back-to-back IMU reads.
         * Skip missed deadlines and return to the time grid.
         */
        uint32_t lateness_us =
            now_us - next_sample_us;

        if (
            lateness_us >=
            FAST_SAMPLE_PERIOD_US
        )
        {
            uint32_t missed =
                lateness_us /
                FAST_SAMPLE_PERIOD_US;

            sample_deadline_misses +=
                missed;

            next_sample_us +=
                (missed + 1U) *
                FAST_SAMPLE_PERIOD_US;
        }
        else
        {
            next_sample_us +=
                FAST_SAMPLE_PERIOD_US;
        }


        imu_sample_t imu_sample;


        if (
            imu_read(
                &imu,
                &imu_sample
            ) != HAL_OK
        )
        {
            xprintf(
                "ERROR: IMU read failed code=%d\n",
                hi2c.ErrorCode
            );

            HAL_I2C_Reset(
                &hi2c
            );

            /*
             * Do not calculate a delta across a missing sample.
             */
            previous_magnitude_valid = 0U;
            imu_read_failures++;

            if (++consecutive_imu_errors >= IMU_READ_RETRIES)
            {
                xprintf("IMU recovery: capture history reset\n");
                HAL_I2C_Disable(&hi2c);
                I2C_Init();
                IMU_WaitUntilReady();
                bench_inputs_init();

                ring_buffer_init(&history);
                fast_ring_init(&fast_history);
                recorder_state = RECORDER_ARMED;
                fast_capture_frozen = 0U;
                decimation_counter = 0U;
                delta_history_head = 0U;
                delta_history_count = 0U;
                consecutive_imu_errors = 0U;
                next_sample_us = HAL_Time_TIM32_Micros() + FAST_SAMPLE_PERIOD_US;
            }

            continue;
        }
        consecutive_imu_errors = 0U;
        /*
         * FC-03 RIGHT pulse counter.
         *
         * Observed with this sensor:
         *   clear slot   -> 0
         *   blocked slot -> 1
         *
         * Count only rising edges: 0 -> 1.
         */
        uint8_t fc03_right_now =
            (
                HAL_GPIO_ReadPin(
                    GPIO_0,
                    GPIO_PIN_8
                ) == GPIO_PIN_HIGH
            ) ? 1U : 0U;

        if (fc03_right_now != fc03_right_level)
        {
            if (
                (fc03_right_level == 0U) &&
                (fc03_right_now == 1U)
            )
            {
                fc03_right_pulses++;
            }

            fc03_right_level =
                fc03_right_now;
        }

        /*
         * Temporary report about once per second.
         * Print only when pulse count changed.
         */
        fc03_right_report_divider++;

        if (fc03_right_report_divider >= 500U)
        {
            fc03_right_report_divider = 0U;

            if (
                fc03_right_pulses !=
                fc03_right_reported_pulses
            )
            {
                xprintf(
                    "FC03 RIGHT pulses=%u level=%u\n",
                    fc03_right_pulses,
                    fc03_right_level
                );

                fc03_right_reported_pulses =
                    fc03_right_pulses;
            }
            bench_inputs_report_encoder_changes();
        }

        /*
         * Diagnostic only: measure the change in acceleration
         * magnitude between adjacent 500 Hz acquisition samples.
         *
         * This does NOT participate in collision classification yet.
         */
        uint32_t current_magnitude_centi_g =
            collision_magnitude_centi_g(
                &imu_sample
            );

        uint32_t current_delta_centi_g = 0U;

        if (previous_magnitude_valid != 0U)
        {
            if (
                current_magnitude_centi_g >=
                previous_magnitude_centi_g
            )
            {
                current_delta_centi_g =
                    current_magnitude_centi_g -
                    previous_magnitude_centi_g;
            }
            else
            {
                current_delta_centi_g =
                    previous_magnitude_centi_g -
                    current_magnitude_centi_g;
            }
        }

        previous_magnitude_centi_g =
            current_magnitude_centi_g;

        previous_magnitude_valid = 1U;


        delta_history[delta_history_head] =
            current_delta_centi_g;

        delta_history_head++;

        if (delta_history_head >= 33U)
        {
            delta_history_head = 0U;
        }

        if (delta_history_count < 33U)
        {
            delta_history_count++;
        }


        /*
         * Once an event has triggered, extend the peak measurement
         * through the remaining full-rate fast-capture samples only.
         */
        if (
            recorder_state == RECORDER_POST_EVENT &&
            fast_capture_frozen == 0U &&
            current_delta_centi_g >
                event_peak_delta_centi_g
        )
        {
            event_peak_delta_centi_g =
                current_delta_centi_g;
        }


        fast_sequence++;


        if (
            fast_capture_frozen == 0U
        )
        {
            telemetry_t fast_telemetry;

            fast_telemetry.seq =
                fast_sequence;

            fast_telemetry.ax =
                imu_sample.ax;

            fast_telemetry.ay =
                imu_sample.ay;

            fast_telemetry.az =
                imu_sample.az;

            fast_telemetry.gx =
                imu_sample.gx;

            fast_telemetry.gy =
                imu_sample.gy;

            fast_telemetry.gz =
                imu_sample.gz;


            fast_ring_push(
                &fast_history,
                &fast_telemetry
            );
        }


        uint8_t manual_capture = 0U;
        if (HAL_USART_RXNE_ReadFlag(&husart))
        {
            char command = HAL_USART_ReadByte(&husart);
            manual_capture = command == 'c';
            if (command == 's' && recorder_state == RECORDER_ARMED)
            {
                xprintf("=== SENSOR STATUS ===\n");
                xprintf("Right encoder pulses = %u\n", fc03_right_pulses);
                bench_inputs_report();
                xprintf("=== SENSOR STATUS END ===\n");
            }
        }

        uint32_t bench_event = bench_inputs_poll(&hi2c, now_us, fc03_right_pulses);
        if (recorder_state == RECORDER_ARMED &&
            (manual_capture || bench_event != 0U || collision_detect(&imu_sample)))
        {
            event_type = bench_event != 0U ? bench_event : EVENT_TYPE_ACCEL_TRIGGER;
            if (manual_capture)
            {
                event_type = EVENT_TYPE_MANUAL_CAPTURE;
            }
            event_trigger_magnitude =
                current_magnitude_centi_g;


            event_fast_sequence =
                fast_sequence;

            event_telemetry_sequence =
                telemetry_sequence;

            event_trigger_sample =
                imu_sample;


            event_trigger_delta_centi_g =
                current_delta_centi_g;

            /*
             * Find the largest delta-|a| in the 32 samples before
             * the trigger plus the trigger sample itself.
             */
            event_peak_delta_centi_g = 0U;

            for (
                uint8_t i = 0U;
                i < delta_history_count;
                i++
            )
            {
                if (
                    delta_history[i] >
                    event_peak_delta_centi_g
                )
                {
                    event_peak_delta_centi_g =
                        delta_history[i];
                }
            }


            recorder_state =
                RECORDER_POST_EVENT;

            post_event_counter = 0;

            fast_post_remaining =
                FAST_POST_EVENT_SAMPLES;

            fast_capture_frozen = 0U;


        }


        if (
            recorder_state ==
            RECORDER_POST_EVENT &&

            fast_capture_frozen == 0U &&

            fast_sequence >
            event_fast_sequence
        )
        {
            if (
                fast_post_remaining > 0U
            )
            {
                fast_post_remaining--;
            }


            if (
                fast_post_remaining == 0U
            )
            {
                fast_capture_frozen = 1U;

            }
        }


        decimation_counter++;


        if (
            decimation_counter >=
            TELEMETRY_DECIMATION
        )
        {
            decimation_counter = 0;


            if (
                recorder_state !=
                RECORDER_FROZEN
            )
            {
                telemetry_t telemetry;

                telemetry.seq =
                    telemetry_sequence++;

                telemetry.ax =
                    imu_sample.ax;

                telemetry.ay =
                    imu_sample.ay;

                telemetry.az =
                    imu_sample.az;

                telemetry.gx =
                    imu_sample.gx;

                telemetry.gy =
                    imu_sample.gy;

                telemetry.gz =
                    imu_sample.gz;


                ring_buffer_push(
                    &history,
                    &telemetry
                );


                if (
                    recorder_state ==
                    RECORDER_POST_EVENT
                )
                {
                    post_event_counter++;


                    if (
                        post_event_counter >=
                        POST_EVENT_SAMPLES
                    )
                    {
                        recorder_state =
                            RECORDER_FROZEN;


                        const telemetry_t *latest =
                            ring_buffer_latest(
                                &history
                            );


                        xprintf("\n");
                        xprintf(
                            "=== EVENT CAPTURE COMPLETE ===\n"
                        );

                        xprintf(
                            "Buffer frozen: %u/%u\n",
                            ring_buffer_count(
                                &history
                            ),
                            RING_BUFFER_SIZE
                        );

                        xprintf(
                            "Trigger telemetry seq ~= %u\n",
                            event_telemetry_sequence
                        );

                        if (latest != 0)
                        {
                            xprintf(
                                "Last telemetry seq = %u\n",
                                latest->seq
                            );
                        }

                        xprintf(
                            "Post-event records = %u\n",
                            post_event_counter
                        );

                        xprintf(
                            "Sampling deadline misses = %u\n",
                            sample_deadline_misses
                        );
                        xprintf("IMU read failures = %u\n", imu_read_failures);
                        xprintf("Bench inputs at end of capture:\n");
                        xprintf("Right encoder pulses = %u\n", fc03_right_pulses);
                        bench_inputs_report();

                        xprintf(
                            "Trigger delta-|a| = %u.%02u g\n",
                            event_trigger_delta_centi_g / 100U,
                            event_trigger_delta_centi_g % 100U
                        );

                        xprintf(
                            "Peak delta-|a| = %u.%02u g\n",
                            event_peak_delta_centi_g / 100U,
                            event_peak_delta_centi_g % 100U
                        );


                        xprintf(
                            "\n=== FAST RAM CSV BEGIN ===\n"
                        );

                        xprintf(
                            "fast_seq,ax,ay,az,gx,gy,gz,trigger\n"
                        );


                        for (
                            uint16_t i = 0;
                            i < fast_ring_count(
                                &fast_history
                            );
                            i++
                        )
                        {
                            const telemetry_t *fast_sample =
                                fast_ring_get(
                                    &fast_history,
                                    i
                                );

                            if (fast_sample != 0)
                            {
                                xprintf(
                                    "%u,%d,%d,%d,%d,%d,%d,%u\n",

                                    fast_sample->seq,

                                    fast_sample->ax,
                                    fast_sample->ay,
                                    fast_sample->az,

                                    fast_sample->gx,
                                    fast_sample->gy,
                                    fast_sample->gz,

                                    (
                                        fast_sample->seq ==
                                        event_fast_sequence
                                    )
                                        ? 1U
                                        : 0U
                                );
                            }
                        }


                        xprintf(
                            "=== FAST RAM CSV END ===\n\n"
                        );


                        event_pending_t pending_event;


                        if (
                            event_store_prepare(
                                &pending_event,
                                &history,
                                &fast_history,
                                event_type,

                                event_fast_sequence,
                                event_telemetry_sequence,

                                &event_trigger_sample,

                                event_trigger_magnitude,

                                post_event_counter
                            ) != 0
                        )
                        {
                            xprintf(
                                "ERROR: event_store_prepare failed\n"
                            );

                            while (1)
                            {
                                HAL_DelayMs(1000);
                            }
                        }


                        xprintf(
                            "Persisting event seq=%u slot=%u\n",
                            pending_event.header.event_sequence,
                            pending_event.slot_index
                        );

                        xprintf(
                            "Records=%u first=%u last=%u\n",
                            pending_event.header.record_count,
                            pending_event.header.first_seq,
                            pending_event.header.last_seq
                        );

                        xprintf(
                            "Payload CRC=%08X\n",
                            pending_event.header.payload_crc32
                        );

                        xprintf(
                            "RAM writer stages: 0 R J E H P C\n"
                        );


                        if (
                            HAL_WDT_Start(
                                &hwdt,
                                WDT_TIMEOUT_DEFAULT
                            ) != HAL_OK
                        )
                        {
                            xprintf(
                                "ERROR: cannot start WDT\n"
                            );

                            while (1)
                            {
                                HAL_DelayMs(1000);
                            }
                        }


                        flash_writer_save_event_and_wait_reset(
                            &pending_event,
                            &history,
                            &fast_history
                        );
                    }
                }


                report_counter++;


            }
        }


    }
}


static void SystemClock_Config(void)
{
    PCC_InitTypeDef clock = {0};

    clock.OscillatorEnable =
        PCC_OSCILLATORTYPE_ALL;

    clock.FreqMon.OscillatorSystem =
        PCC_OSCILLATORTYPE_OSC32M;

    clock.FreqMon.ForceOscSys =
        PCC_FORCE_OSC_SYS_UNFIXED;

    clock.FreqMon.Force32KClk =
        PCC_FREQ_MONITOR_SOURCE_OSC32K;

    clock.AHBDivider = 0;
    clock.APBMDivider = 0;
    clock.APBPDivider = 0;

    clock.HSI32MCalibrationValue = 128;
    clock.LSI32KCalibrationValue = 8;

    clock.RTCClockSelection =
        PCC_RTC_CLOCK_SOURCE_AUTO;

    clock.RTCClockCPUSelection =
        PCC_CPU_RTC_CLOCK_SOURCE_OSC32K;

    HAL_PCC_Config(
        &clock
    );
}


static void I2C_Init(void)
{
    hi2c.Instance = I2C_1;

    hi2c.Init.Mode =
        HAL_I2C_MODE_MASTER;

    hi2c.Init.DigitalFilter =
        I2C_DIGITALFILTER_OFF;

    hi2c.Init.AnalogFilter =
        I2C_ANALOGFILTER_DISABLE;

    hi2c.Init.AutoEnd =
        I2C_AUTOEND_ENABLE;

    hi2c.Clock.PRESC = 1;
    hi2c.Clock.SCLDEL = 15;
    hi2c.Clock.SDADEL = 15;
    hi2c.Clock.SCLH = 75;
    hi2c.Clock.SCLL = 75;

    if (
        HAL_I2C_Init(
            &hi2c
        ) != HAL_OK
    )
    {
        xprintf(
            "ERROR: I2C init failed\n"
        );
    }
}


static void IMU_WaitUntilReady(void)
{
    while (imu_init(&imu, &hi2c) != HAL_OK)
    {
        xprintf("WARN: IMU initialization failed; retrying in 500 ms\n");
        HAL_I2C_Disable(&hi2c);
        hi2c.Instance->CR2 = 0U;
        HAL_DelayMs(5);
        if (HAL_I2C_Init(&hi2c) != HAL_OK)
        {
            xprintf("WARN: I2C re-init failed\n");
        }
        HAL_DelayMs(500);
    }
    xprintf("IMU ready. WHO_AM_I = 0x%02X\n", imu.who_am_i);
}


static void USART_Init(void)
{
    husart.Instance = UART_0;

    husart.transmitting = Enable;
    husart.receiving = Enable;

    husart.frame = Frame_8bit;

    husart.parity_bit = Disable;
    husart.parity_bit_inversion = Disable;

    husart.bit_direction = LSB_First;

    husart.data_inversion = Disable;
    husart.tx_inversion = Disable;
    husart.rx_inversion = Disable;

    husart.swap = Disable;
    husart.lbm = Disable;

    husart.stop_bit = StopBit_1;
    husart.mode = Asynchronous_Mode;

    husart.xck_mode = XCK_Mode3;
    husart.last_byte_clock = Disable;

    husart.overwrite = Disable;
    husart.rts_mode = AlwaysEnable_mode;

    husart.dma_tx_request = Disable;
    husart.dma_rx_request = Disable;

    husart.channel_mode = Duplex_Mode;
    husart.tx_break_mode = Disable;

    husart.Interrupt.ctsie = Disable;
    husart.Interrupt.eie = Disable;
    husart.Interrupt.idleie = Disable;
    husart.Interrupt.lbdie = Disable;
    husart.Interrupt.peie = Disable;
    husart.Interrupt.rxneie = Disable;
    husart.Interrupt.tcie = Disable;
    husart.Interrupt.txeie = Disable;

    husart.Modem.rts = Disable;
    husart.Modem.cts = Disable;
    husart.Modem.dtr = Disable;
    husart.Modem.dcd = Disable;
    husart.Modem.dsr = Disable;
    husart.Modem.ri = Disable;
    husart.Modem.ddis = Disable;

    husart.baudrate =
        BAUDRATE;

    HAL_USART_Init(
        &husart
    );
}


static void WDT_Init(void)
{
    hwdt.Instance = WDT;

    hwdt.Init.Clock =
        HAL_WDT_OSC32K;

    /*
     * Two sector erases + 25 payload pages
     * + read-back verification.
     */
    hwdt.Init.ReloadMs =
        20000;

    if (
        HAL_WDT_Init(
            &hwdt,
            WDT_TIMEOUT_DEFAULT
        ) != HAL_OK
    )
    {
        xprintf(
            "ERROR: WDT init failed\n"
        );
    }
}

