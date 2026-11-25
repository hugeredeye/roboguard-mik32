# RoboGuard

The MIK32 recorder saves IMU history and a 64-sample window around a
trigger. On restart it verifies the saved record and prints both windows
on UART at 115200 baud. The current trigger is acceleration of at least
4 g. Send the single character `c` on UART for a manual capture, including
trials below 4 g. Wait four seconds after boot for the long history to fill.

Run `pio run` to build. Check the Windows COM port before uploading.
The existing uploader command in `platformio.ini` targets COM3.

From the parent directory, convert a Device Monitor log to CSV, PNG
and a JSON summary:

```powershell
python main.py path/to/trial.log --label impact_01 --out path/to/report
```

Plotting needs matplotlib. Use `--no-plots` for CSV and JSON only.
The time axis uses the configured sample periods. Check the UART
diagnostics for missed deadlines and failed reads when comparing timings.

The right FC-03 input uses D4 / PORT0.8 and the left uses D5 / PORT0.1.
Their pulse counters are reported separately. To enable other inputs, set
the build flags described in `include/bench_inputs.h` to match the wired
GPIOs, INA219 address and shunt resistance. Verify the GPIO mapping
against the [board documentation](https://docs.elron.tech/arduino/elbear-ace-uno/).
Send `s` over UART to read sensor status while the recorder is armed.
The heartbeat input expects
separate pulses from ESP32 with rising edges spaced less than 600 ms.
After two such edges, losing that signal for 600 ms triggers a record.
The stall candidate requires a calibrated forward-current threshold
and both wheels without pulses for 400 ms. The INA219 conversion follows
the [TI register description](https://www.ti.com/lit/ds/symlink/ina219.pdf).
The record stores the trigger reason. UART also shows bench readings
at the end of the capture.
