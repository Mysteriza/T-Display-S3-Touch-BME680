# T-Display-S3 Touch BME680 Monitor

Offline environmental monitor firmware for LilyGO T-Display-S3 with BME680/BME688 and Bosch BSEC2.

## Design Prototype

![UI Design Prototype](UI%20Design%20Prototype.png)

## Features

- Fully offline operation (no cloud, no Wi-Fi required).
- Live measurements: temperature, humidity, pressure, altitude, gas resistance, IAQ.
- IAQ calibration visibility: IAQ level, IAQ accuracy, run-in status, stabilization status.
- Boot self-check for display, touch, and sensor.
- LVGL UI with swipe pages.
- Smart backlight timeout + wake button.
- Automatic sensor recovery if data becomes stale.
- BSEC state save/restore so calibration can continue across reboots.
- Serial command interface for calibration and diagnostics.

## What This Firmware Does

The firmware runs two core loops:

- Sensor loop: reads BME68x through BSEC2 scheduler, updates IAQ pipeline, and publishes fresh snapshots.
- UI loop: renders values on screen and handles touch/wake behavior.

The architecture is split into dedicated modules:

- `include/config.h`: all tunable constants (`cfg::pins`, `cfg::timing`, `cfg::sensor`, `cfg::color`, etc.).
- `include/sensor_manager.h` + `src/sensor_manager.cpp`: BSEC2 pipeline, NVS persistence, health checks, and serial command backend.
- `include/ui_controller.h` + `src/ui_controller.cpp`: LVGL display/touch controller and all page rendering logic.
- `include/power_mgmt.h` + `src/power_mgmt.cpp`: display timeout, wake ISR/button handling, and CPU frequency profile switching.

IAQ stability logic includes:

- Scheduler-aware BSEC run handling (no false reinit from normal no-output cycles).
- Fresh-sample publish path so IAQ updates appear as soon as BSEC emits data.
- Adaptive stale timeout and automatic reinitialization when sensor data is truly stale.
- Accuracy-aware reporting so users can see calibration progress from 0 to 3.

## Hardware

- LilyGO T-Display-S3 (touch version)
- BME680 or BME688 module (I2C)
- USB-C cable
- Jumper wires

## Wiring

| Sensor   | T-Display-S3             |
| -------- | ------------------------ |
| VCC      | 3V3                      |
| GND      | GND                      |
| SDA      | GPIO18                   |
| SCL      | GPIO17                   |
| CS/CSB   | 3V3                      |
| SDO/ADDR | GND (0x76) or 3V3 (0x77) |

## Installation

### Option A - Build and Upload from Source (PlatformIO)

1. Install Visual Studio Code.
2. Install PlatformIO extension in VS Code.
3. Open this project folder.
4. Connect board with USB.
5. Build:

```bash
platformio run -e lilygo-t-display-s3
```

6. Upload:

```bash
platformio run -e lilygo-t-display-s3 -t upload
```

7. If needed, specify port:

```bash
platformio run -e lilygo-t-display-s3 -t upload --upload-port COM5
```

### Option B - Install from Prebuilt .bin (GitHub Actions Artifact)

This repository includes CI workflow:

- Workflow file: `.github/workflows/build-firmware.yml`
- Trigger: pull request, non-main push, or manual run
- Artifact name: `t-display-s3-firmware`
- Artifact contents:
  - `firmware-merged.bin` (recommended for end users, flash at 0x0)
  - `firmware-app.bin` (app only, flash at 0x10000)
  - `bootloader.bin` (0x0)
  - `partitions.bin` (0x8000)

#### Beginner Flashing Steps (Windows)

1. Open GitHub repository page.
2. Open Actions tab.
3. Open the latest successful "Build Firmware" run.
4. Download artifact `t-display-s3-firmware`.
5. Extract ZIP to a folder, for example `C:\firmware`.
6. Install Python 3 from https://www.python.org/downloads/.
7. Open PowerShell.
8. Install esptool:

```powershell
py -m pip install --upgrade pip
py -m pip install esptool
```

9. Find board COM port from Device Manager (for example COM5).
10. In PowerShell, go to extracted folder:

```powershell
cd C:\firmware
```

11. Flash merged binary:

```powershell
py -m esptool --chip esp32s3 --port COM5 --baud 460800 write_flash 0x0 firmware-merged.bin
```

12. Wait until process shows success and board reboots.
13. Open serial monitor at 115200 to verify startup.

If flashing fails:

- Hold BOOT button while connecting USB, then run flash command again.
- Verify COM port is not used by another serial monitor.

### Option C - Install from GitHub Release (Recommended for End Users)

For end users, this is the easiest path because firmware is already built and tagged:

1. Open repository page on GitHub.
2. Open **Releases**.
3. Download latest `firmware-merged.bin` from latest version tag.
4. Flash at offset `0x0`.

Example command:

```powershell
py -m esptool --chip esp32s3 --port COM5 --baud 460800 write_flash 0x0 firmware-merged.bin
```

## Usage

- Power on board.
- Wait for self-check to complete.
- Swipe left/right to change pages.
- Device keeps collecting data even when display backlight is off.
- Press wake button (GPIO14 path) to turn display back on.

## Project Structure

```text
include/
  config.h
  power_mgmt.h
  sensor_manager.h
  ui_controller.h
src/
  main.cpp
  power_mgmt.cpp
  sensor_manager.cpp
  ui_controller.cpp
```

### IAQ Accuracy Meaning

- 0: Very Low (initial warm-up)
- 1: Low
- 2: Medium
- 3: High (calibrated)

For best IAQ results, keep device powered continuously during first calibration period.

## Serial Commands

Open monitor:

```bash
platformio device monitor -b 115200
```

| Command            | Description                                                    |
| ------------------ | -------------------------------------------------------------- |
| `help`             | Show command list                                              |
| `status`           | Show sensor, IAQ, accuracy, calibration, and connection status |
| `get slp`          | Show current sea-level pressure setting                        |
| `set slp <hPa>`    | Set sea-level pressure manually                                |
| `set alt <meter>`  | Set known altitude to auto-adjust sea-level pressure           |
| `reset slp`        | Reset sea-level pressure to default                            |
| `sensor reinit`    | Force sensor/BSEC reinitialization                             |
| `i2c scan`         | Scan I2C bus and show detected devices                         |
| `debug detail on`  | Enable verbose periodic debug output                           |
| `debug detail off` | Disable verbose periodic debug output                          |

## Quick Troubleshooting

- Sensor not detected:
  - Check SDA/SCL wiring (GPIO18/GPIO17).
  - Confirm address 0x76 or 0x77 via `i2c scan`.
- IAQ accuracy stays low:
  - Keep device running longer (calibration requires time).
  - Check `status` for `runin` and `stab` progression.
- IAQ appears flat:
  - Verify environment is actually changing (airflow, VOC source, humidity/temperature changes).
  - Run `sensor reinit` once, then monitor `status` for several minutes.
  - Firmware now auto-recovers once if IAQ remains near 50 with accuracy 0 for extended time.
- Altitude incorrect:
  - Use `set slp <hPa>` or `set alt <meter>`.

UI note:

- IAQ and Gas are shown as integer values for cleaner dashboard readability.

## Change Log

- Replaced legacy `sensors.*` with class-based `sensor_manager.*`.
- Replaced legacy `ui_pages.*` with class-based `ui_controller.*`.
- Extracted serial command parsing into dedicated `SerialCLI` class.
- Refactored power handling to singleton `PowerManager` with ISR-safe wake flow.
- Centralized magic numbers into namespaced `cfg::*` constants in `config.h`.
- Kept offline-only behavior (no Wi-Fi, Bluetooth, or cloud dependency).
- Fixed pressure/altitude scaling path so pressure stays in hPa and altitude calculation is consistent.
- Reduced noisy runtime I2C link probing and added NVS state-key guard to avoid recurring serial error spam.
- Improved IAQ gauge status color mapping to follow IAQ value bands and tightened calibrating-state conditions.
- Normalized AQI colors to semantic defaults: green for good/normal, yellow for warning/caution, red for bad/error conditions.
- Simplified Accuracy display text to label-only format (Very Low/Low/Medium/High) without numeric suffix.
