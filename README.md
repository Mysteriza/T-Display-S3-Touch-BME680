# T-Display-S3 Touch BME680 Monitor

Offline environmental monitor firmware for LilyGO T-Display-S3 + BME680/BME688 with Bosch BSEC2.

This README explains how the system works and the meaning of every number on the screen based on the current code implementation, so users do not need to read the source code.

## Flow & Architecture

The system runs in two main FreeRTOS loops on separate cores:

- **Sensor loop**: Runs the BSEC2 scheduler continuously, reads BME68x data, and builds data snapshots.
- **UI loop**: Reads the latest snapshot data and renders it to the display.

Important principles:

- The sensor is processed in real-time by BSEC (`run()` is called frequently), not every 30 seconds.
- Data published to the UI is synchronized every `kSensorRefreshMs` (default 30 seconds) to reduce redraws and save battery.
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
- **System Telemetry**: CPU Load estimate (%), Uptime counter, and Battery Percentage.
- **Power Optimization**: Background sensor processing with reduced screen redraws.
- **Serial Diagnostics**: Built-in CLI for status checks and manual calibration.
- **Automatic Recovery**: Detects and clears stuck IAQ states automatically.
- **Extended Boot Self-Check**: Verifies display, touch, sensor init, data pipeline validity, and IAQ pipeline readiness before entering normal runtime.

## Runtime Defaults

- Display timeout: **15 seconds** of inactivity.
- Sensor publish interval to UI: **30 seconds** (`kSensorRefreshMs`).
- Uptime label refresh: **1 second**.

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

## Troubleshooting

- **Sensor not detected**: Check the SDA/SCL wiring and the 0x76/0x77 address configuration.
- **Accuracy is stuck at "Very Low"**: Leave it running continuously; monitor the `status` via Serial.
- **Screen numbers flicker/disappear**: The firmware pauses data publishing when I2C is unstable to prevent flickering. Provide better power or check connections.
