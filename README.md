# T-Display-S3 Touch BME680 Monitor

Offline environmental monitor firmware for LilyGO T-Display-S3.

The project displays live environmental data on a 320x170 landscape UI built with LVGL, uses BME680 + Bosch BSEC2 for IAQ processing, supports swipe navigation via capacitive touch, and includes smart power behavior (screen timeout + wake input).

## Main Functions

- Runs fully offline (no Wi-Fi required).
- Shows a boot self-check screen for LCD, touch controller, and sensor.
- Reads and processes:
  - Temperature
  - Humidity
  - Pressure
  - Altitude (derived)
  - Gas resistance
  - IAQ + IAQ accuracy (from BSEC2)
- Provides 3 touch-navigable pages with left/right swipe.
- Turns display backlight off after inactivity and wakes on interaction.
- Saves BSEC state to NVS periodically (every 24 hours) for better long-term IAQ behavior.

## Hardware and Tools Required

- LilyGO T-Display-S3 board
- BME680 module (I2C)
- USB-C cable
- PC with:
  - Visual Studio Code
  - PlatformIO IDE extension

Optional:

- External wake button connected to GPIO14 (if not using onboard button mapping)

## Firmware Stack

- MCU: ESP32-S3 (Arduino framework)
- Display: ST7789, 8-bit parallel, landscape right
- UI: LVGL 8.3
- Sensor processing: Bosch BSEC2 + BME68x library
- Touch: SensorLib TouchDrvCSTXXX (auto-detect touch address)
- Build system: PlatformIO

## Wiring

### 1) Onboard Peripherals (already wired on T-Display-S3)

These are handled by firmware and board hardware:

- LCD backlight: GPIO38
- LCD power: GPIO15
- Touch reset: GPIO21
- Touch interrupt: GPIO16
- Wake button input: GPIO14

### 2) External BME680 Wiring (I2C)

Connect BME680 to T-Display-S3 as follows:

| BME680 Pin | T-Display-S3 Pin |
| ---------- | ---------------- |
| VCC        | 3V3              |
| GND        | GND              |
| SDA        | GPIO18           |
| SCL        | GPIO17           |

Notes:

- Expected I2C address is 0x76.
- If your module is 0x77, update BME680_I2C_ADDR in include/config.h.

## Project Structure

- include/config.h: Pin map, timing values, color constants
- include/sensors.h + src/sensors.cpp: BSEC2 pipeline, sensor snapshot, NVS state handling
- include/ui_pages.h + src/ui_pages.cpp: LVGL display/touch init and page rendering
- include/power_mgmt.h + src/power_mgmt.cpp: Backlight timeout and wake logic
- src/main.cpp: Boot sequence and FreeRTOS task startup
- platformio.ini: Board config, build flags, dependencies

## Build

From project root:

```bash
platformio run -e lilygo-t-display-s3
```

If PlatformIO is not in PATH on Windows:

```bash
C:\Users\<your-user>\.platformio\penv\Scripts\platformio.exe run -e lilygo-t-display-s3
```

## Upload

```bash
platformio run -e lilygo-t-display-s3 -t upload
```

To use a specific serial port:

```bash
platformio run -e lilygo-t-display-s3 -t upload --upload-port COMx
```

## Serial Monitor

```bash
platformio device monitor -b 115200
```

## Runtime Behavior

- Boot performs hardware checks and shows OK or FAIL states.
- UI starts on Page 1.
- Swipe left or right to change page.
- Display timeout default is 10 seconds of inactivity.
- Any touch activity or wake button event resets the timeout.

## Configuration

Edit include/config.h for common adjustments:

- DISPLAY_TIMEOUT (default 10000 ms)
- SENSOR_REFRESH (default 30000 ms)
- I2C pins and addresses
- UI color constants

## Troubleshooting

- Touch not responding:
  - Confirm board is T-Display-S3 touch variant.
  - Firmware probes touch at 0x15 and 0x1A automatically.
- Sensor FAIL on boot:
  - Recheck SDA/SCL wiring and power.
  - Verify BME680 I2C address (0x76 or 0x77).
- Build command not found:
  - Use full PlatformIO executable path from your user profile.

## Dependencies

Managed in platformio.ini:

- lvgl/lvgl@8.3.11
- bodmer/TFT_eSPI@2.5.43
- Bosch BSEC2 Library
- Bosch BME68x Library
- lewisxhe/SensorLib

## Change Log (Current)

- Added modular firmware architecture for display, touch, sensor, and power management.
- Added robust touch initialization with address probing and coordinate mapping.
- Added swipe-only page navigation and smart display timeout.
- Added BSEC2 state persistence to NVS.
- Added complete PlatformIO dependency and build configuration.
- Added battery percentage indicator at top-center on all pages using built-in board ADC battery sensing.
- Updated Page 03 system info cadence: CPU Load and Storage now refresh every 5 seconds for lower power impact.
- Updated Page 03 storage text to numeric free/total MB format without float printf.
- Updated gas resistance unit fallback to ASCII `kOhm` for font compatibility on-board.
