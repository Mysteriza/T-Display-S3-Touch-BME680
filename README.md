# T-Display-S3 Touch BME680 Monitor

Offline environmental monitor firmware for LilyGO T-Display-S3 + BME680/BME688 with Bosch BSEC2.

This README explains how the system works and the meaning of every number on the screen based on the current code implementation, so users do not need to read the source code.

## Design Prototype

![UI Design Prototype](UI%20Design%20Prototype.png)

## Flow & Architecture

The system runs in two main FreeRTOS loops on separate cores:

- **Sensor loop**: Runs the BSEC2 scheduler continuously, reads BME68x data, and builds data snapshots.
- **UI loop**: Reads the latest snapshot data and renders it to the display.

Important principles:

- The sensor is processed in real-time by BSEC (`run()` is called frequently), not every 30 seconds.
- Sensor data publishing remains every `kSensorRefreshMs` (default 30 seconds), while UI labels can refresh each second to keep status text responsive.
- If data is temporarily invalid, the UI retains the last valid snapshot (anti-flicker), and will display NO DATA if it truly becomes stale.

## Project Structure Overview

```text
include/
  config.h          # Tuning parameters (pins, timings, colors)
  power_mgmt.h      # Screen timeout, wake logic
  sensor_manager.h  # BSEC2 pipeline, sensor reading
  ui_controller.h   # LVGL UI and rendering
src/
  main.cpp          # App entry point
  power_mgmt.cpp    # Power management logic
  sensor_manager.cpp# Sensor processing
  ui_controller.cpp # UI lifecycle
```

## Features

- **Real-time Environment Data**: Temperature, Humidity, Air Pressure, and calculated Altitude.
- **AQI Monitoring**: Real-time Gas Resistance and Indoor Air Quality (IAQ) from Bosch BSEC2.
- **Adaptive IAQ Statusing (Page 02)**: Hybrid IAQ banding that keeps BSEC output as baseline and applies bounded gas-history correction with confidence + fallback safeguards.
- **System Telemetry**: UI task load estimate, Uptime counter, and Battery Percentage.
- **Power Optimization**: Background sensor processing with reduced screen redraws.
- **Serial Diagnostics**: Built-in CLI for status checks and manual calibration.
- **Automatic Recovery**: Detects and clears stuck IAQ states automatically.
- **Extended Boot Self-Check**: Verifies display, touch, sensor init, data pipeline validity, and IAQ pipeline readiness before entering normal runtime.
- **Advanced Battery SOC Estimator (No Extra Hardware)**: Non-linear Li-ion OCV curve, load-compensated voltage recovery, dual-path filtering, and bounded-rate SOC fusion to reduce jumpy and misleading percentage output.

## Runtime Defaults

- Display timeout: **15 seconds** of inactivity.
- Sensor publish interval to UI: **30 seconds** (`kSensorRefreshMs`).
- UI value refresh cadence: **1 second** (`kUiValuesRefreshMs`).
- Uptime label refresh: **1 second**.
- Boot data/IAQ verification window: **up to 15 seconds**.

Temperature compensation policy:

- Current default BSEC temperature offset is **0.0 C** (`kBsecTemperatureOffsetC`) because the sensor placement is thermally separated from the main board.
- For a future compact enclosure where the sensor is close to board heat, start with **0.5 C** (see `kBsecTemperatureOffsetCompactC`) and validate against a reference thermometer before increasing.

## Battery Percentage Model

Battery percentage is estimated in software from the board's existing battery ADC path (no added gauge IC).

Estimator pipeline:

- ADC sampling with calibration and multi-sample averaging.
- Dual voltage filtering:
  - Fast filter for responsive voltage tracking.
  - Slow filter for rest-state baseline detection.
- Dynamic OCV compensation:
  - Compensates load sag based on display power state and UI load activity.
  - Applies extra recovery in near-rest conditions.
- Non-linear Li-ion OCV-to-SOC mapping (piecewise interpolation, not linear 3.2V-4.2V mapping).
- Fusion + slew-rate limiting:
  - Smooths SOC to avoid false jumps.
  - Limits physically unrealistic rise/drop speed.

Important note:

- This is a high-quality voltage-model SOC estimator, not true coulomb-counting fuel gauging.
- Runtime prediction in minutes still depends heavily on battery age, internal resistance, temperature, and load profile.

## Hardware & Wiring

- LilyGO T-Display-S3 (touch)
- BME680 or BME688 module (via I2C)
- USB-C cable

| Sensor   | T-Display-S3             |
| -------- | ------------------------ |
| VCC      | 3V3                      |
| GND      | GND                      |
| SDA      | GPIO18                   |
| SCL      | GPIO17                   |
| CS/CSB   | 3V3                      |
| SDO/ADDR | GND (0x76) or 3V3 (0x77) |

## Installation

You need PlatformIO to build and flash this project.

1. Clone this repository.
2. Open the project folder in PlatformIO.
3. Build the project firmware:
   ```bash
   platformio run -e lilygo-t-display-s3
   ```
4. Upload to the board:
   ```bash
   platformio run -e lilygo-t-display-s3 -t upload
   ```
   _(If you need a specific port, use: `platformio run -e lilygo-t-display-s3 -t upload --upload-port COM5`)_

## Install From GitHub Release (Single BIN, Recommended)

To avoid confusion, each firmware release now provides only one flash-ready binary file:

- `firmware-merged.bin`

You do not need `bootloader.bin`, `partitions.bin`, or app-only binaries.

### Windows (Step-by-step)

1. Open the repository **Releases** page.
2. Download `firmware-merged.bin` from the latest release.
3. Install Python 3 (if not installed):

- https://www.python.org/downloads/

4. Open **PowerShell**.
5. Install esptool:

```powershell
py -m pip install --upgrade pip
py -m pip install esptool
```

6. Connect ESP32-S3 board with USB.
7. Find your COM port in Device Manager (example: `COM5`).
8. Move to the folder that contains `firmware-merged.bin`:

```powershell
cd C:\path\to\download
```

9. Flash the board (offset must be `0x0`):

```powershell
py -m esptool --chip esp32s3 --port COM5 --baud 460800 write_flash 0x0 firmware-merged.bin
```

10. Wait until flashing is successful and the board reboots.

### Linux / macOS (Step-by-step)

1. Download `firmware-merged.bin` from the latest release.
2. Install esptool:

```bash
python3 -m pip install --upgrade pip
python3 -m pip install esptool
```

3. Connect ESP32-S3 board via USB.
4. Find serial port:

- Linux: typically `/dev/ttyUSB0` or `/dev/ttyACM0`
- macOS: typically `/dev/cu.usbserial-*`

5. Flash (offset `0x0`):

```bash
python3 -m esptool --chip esp32s3 --port /dev/ttyUSB0 --baud 460800 write_flash 0x0 firmware-merged.bin
```

### If Flashing Fails

- Hold the **BOOT** button while plugging USB, then retry the flash command.
- Close any Serial Monitor app that may be locking the COM/serial port.
- Lower baud rate to `115200` in the same command if USB cable quality is poor.
- Use a data-capable USB cable (not charge-only).

## Usage

Once flashed, the monitor will immediately boot up and begin calibrating the sensor. The touch screen can be used to navigate between logical pages:

- **Swipe Left/Right** on the touch screen to move between Environment, AQI, and System Telemetry pages.
- The UI will automatically hide after inactivity to preserve battery but will continue sampling in the background. Tap the screen or press the wake button to turn on the screen.
- AQI page intentionally focuses on Gas, IAQ value, Status, and Accuracy only (no duplicate Temp/Humidity rows).

### Serial interface (CLI)

Use the built-in CLI to check real-time statuses and perform manual calibrations:

Open the serial monitor:

```bash
platformio device monitor -b 115200
```

Available commands:

- `help`: Command list
- `status`: Show sensor status + latest data
- `get slp`: Get the current sea-level pressure setting
- `set slp <hPa>`: Manually set the sea-level pressure
- `set alt <meter>`: Calibrate sea-level pressure from reference altitude
- `reset slp`: Reset sea-level pressure to default
- `sensor reinit`: Force BSEC/sensor pipeline re-initialization
- `i2c scan`: Scan the I2C bus for devices
- `debug detail on`: Turn on periodic verbose debugging
- `debug detail off`: Turn off verbose debugging
- `iaq model status`: Show IAQ adaptive model diagnostics (confidence, state, reference gas, delta)
- `iaq model digest`: Show concise IAQ adaptive runtime digest (state, confidence, readiness, history, effective IAQ)
- `iaq model reset`: Reset IAQ adaptive model and clear local learning history

Adaptive IAQ production safeguards:

- Adaptive correction only becomes active when run-in and stabilization are ready and confidence history is sufficient.
- Anti-regression guard continuously compares adaptive output against a gas-derived proxy target.
- Automatic rollback resets adaptive correction to baseline if adaptive path degrades repeatedly.
- Periodic digest output is rate-limited (hourly in detailed debug mode) to avoid serial spam and extra power draw.
- During early warmup (run-in/stabilization not ready, low accuracy), Page 02 shows `Warming up...` and `Calibrating` instead of a misleading fixed-good status.

Boot production checklist:

- Firmware prints a boot-time readiness checklist to serial for quick deployment validation.
- Checklist covers LCD, touch, sensor init, fresh data availability, IAQ core validity, and IAQ model sanity.
- Final verdict is reported as `READY` or `DEGRADED`.

Notes:

- Serial commands are executed when you press Enter (no idle auto-submit).

## Troubleshooting

- **Sensor not detected**: Check the SDA/SCL wiring and the 0x76/0x77 address configuration.
- **Accuracy is stuck at "Very Low"**: Leave it running continuously; monitor the `status` via Serial.
- **Screen numbers flicker/disappear**: The firmware pauses data publishing when I2C is unstable to prevent flickering. Provide better power or check connections.
- **Boot self-check shows temporary data/IAQ fail**: Keep the device powered for initial warm-up; the firmware now waits up to 15 seconds for fresh pipeline data during boot validation.
