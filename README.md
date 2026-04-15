# T-Display-S3 Touch BME680 Monitor

Offline environmental monitor firmware for LilyGO T-Display-S3 with BME680/BME688 and Bosch BSEC2.

This document uses English only and documents direct GPIO I2C wiring as the primary user wiring method.

## Design Prototype

![UI Design](UI%20Design%20Prototype.png)

## Features

- Fully offline operation (no Wi-Fi required).
- Boot self-check for LCD, touch controller, and sensor.
- LVGL UI (320x170 landscape) with left/right swipe navigation.
- Live values for temperature, humidity, pressure, derived altitude, gas resistance, IAQ, and IAQ accuracy.
- Smart backlight timeout (default 15 seconds), wake by GPIO14 button when display is off.
- BSEC state persistence to NVS every 24 hours.
- Runtime calibration commands (`set slp`, `set alt`) with immediate altitude update.
- Quiet serial monitor by default (detailed debug is opt-in).
- Stabilized battery percentage display using ADC averaging + filtering + hysteresis.
- Dynamic CPU frequency scaling: high frequency while display is on, reduced frequency while display is off.
- Touch controller enters sleep when display is off and wakes when GPIO14 wake button turns display on.
- Lower UI task activity while display is off to reduce power without stopping sensor updates.
- Header connectivity indicator:
  - Env_monitor text is cyan when sensor is healthy and data is fresh.
  - Env_monitor text turns red when sensor is disconnected/not healthy or no fresh update arrives within the refresh interval.

## Required Hardware

- LilyGO T-Display-S3 (touch variant).
- BME680/BME688 module (I2C).
- Jumper wires for direct GPIO I2C wiring.
- USB-C cable.
- PC with Visual Studio Code and PlatformIO extension.

## Sensor Wiring (Primary: GPIO I2C)

Use direct GPIO wiring between the board and the sensor.

### 1. Connect Sensor to GPIO I2C

| BME680/BME688 Pin | T-Display-S3 Pin         |
| ----------------- | ------------------------ |
| VCC               | 3V3                      |
| GND               | GND                      |
| SDA               | GPIO18                   |
| SCL               | GPIO17                   |
| CS/CSB            | 3V3                      |
| SDO/ADDR          | GND (0x76) or 3V3 (0x77) |

### 2. Bus Preference in Firmware

For current hardware setup, the primary and recommended wiring is:

- Main bus: SDA GPIO18, SCL GPIO17.

Firmware probes MAIN bus first (GPIO18/GPIO17), then falls back to ALT bus for compatibility.

### 3. Sensor I2C Address Notes

Valid BME68x addresses:

- 0x76
- 0x77

Firmware auto-detects both addresses at initialization.

For modules exposing CS/SDO jumpers:

- CS/CSB must be HIGH (3V3) for I2C mode.
- SDO usually selects address (GND -> 0x76, 3V3 -> 0x77).

### 4. GPIO Wiring Verification Steps

After firmware upload:

1. Open serial monitor at 115200.
2. Run `status`.
3. Confirm sensor status is `OK` and bus/address are reported.
4. Run `i2c scan` and confirm 0x76/0x77 is visible on main bus.

If `status` is not valid immediately after boot, wait until the first BSEC sample is produced.

## Build and Upload

Run commands from project root.

### Build

```bash
platformio run -e lilygo-t-display-s3
```

### Upload

```bash
platformio run -e lilygo-t-display-s3 -t upload
```

### Upload to specific port

```bash
platformio run -e lilygo-t-display-s3 -t upload --upload-port COMx
```

## Serial Monitor and Commands

### Open serial monitor

```bash
platformio device monitor -b 115200
```

### Logging modes

- Default: quiet (no periodic detailed debug spam).
- Enable detailed debug: `debug detail on`
- Disable detailed debug: `debug detail off`

### Command list

- `help`
- `status`
- `get slp`
- `set slp <hPa>`
- `set alt <meter>`
- `reset slp`
- `sensor reinit`
- `i2c scan`
- `debug detail on`
- `debug detail off`

Command behavior notes:

- Commands run on Enter.
- If a terminal does not send CR/LF, commands auto-run after a short idle timeout.
- `set slp` and `set alt` immediately refresh displayed altitude from the latest pressure snapshot.

## Runtime Behavior

- Boot self-check displays LCD/touch/sensor status as OK/FAIL.
- UI starts on page 1 and supports three swipe pages.
- Backlight timeout default: 15000 ms.
- When display is ON: touch works for swipe navigation and keeps activity alive.
- When display is OFF: touch input is ignored and touch controller is put into sleep mode.
- GPIO14 wake button is the only wake source for the display timeout path.
- When GPIO14 wakes the display, the touch controller is reactivated.
- Sensor task runs separately (FreeRTOS) and retries initialization on failure.
- If sensor data becomes stale for too long, firmware schedules automatic sensor re-initialization.
- UI loop is throttled while display is off, while sensor sampling continues in the background.

## Main Configuration

Common settings in include/config.h:

- `DISPLAY_TIMEOUT` (default 15000 ms)
- `SENSOR_REFRESH` (default 15000 ms)
- `CPU_FREQ_ACTIVE_MHZ` (default 240)
- `CPU_FREQ_SLEEP_MHZ` (default 80)
- `PIN_I2C_SDA` (default 18)
- `PIN_I2C_SCL` (default 17)

Sea-level pressure calibration is stored in NVS namespace `sensorcfg` with key `slp_hpa`.

## Project Structure

- include/config.h: pins, timing, color constants.
- include/sensors.h + src/sensors.cpp: BSEC2 pipeline, sensor snapshot, serial commands, persistent state/config.
- include/ui_pages.h + src/ui_pages.cpp: LVGL display/touch initialization and page rendering.
- include/power_mgmt.h + src/power_mgmt.cpp: backlight timeout and wake logic.
- src/main.cpp: boot flow, self-check, task startup.
- platformio.ini: board/environment/dependency configuration.

## Sensor Troubleshooting

### Sensor not detected

- Confirm SDA/SCL are connected to GPIO18/GPIO17.
- Confirm the module is BME680/BME688 (CHIP_ID must be 0x61).
- Run `i2c scan` and verify 0x76 or 0x77 appears.
- Run `sensor reinit` to force re-detection without reboot.

### Altitude value looks wrong

- Run `set slp <hPa>` using local sea-level pressure, or
- Run `set alt <meter>` using known local elevation.
- Re-check with `status`.

### Serial monitor is too noisy

- Use `debug detail off`.

## Dependencies

Managed in platformio.ini:

- lvgl/lvgl@8.3.11
- bodmer/TFT_eSPI@2.5.43
- Bosch BSEC2 Library
- Bosch BME68x Library
- lewisxhe/SensorLib
