# T-Display-S3 Touch BME680 Monitor

Offline environmental monitor firmware for LilyGO T-Display-S3.

The project displays live environmental data on a 320x170 landscape UI built with LVGL, uses BME680 + Bosch BSEC2 for IAQ processing, supports swipe navigation via capacitive touch, and includes smart power behavior (screen timeout + wake input).

## Design Prototype
![UI Design](UI%20Design%20Prototype.png)

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
| CS / CSB   | 3V3              |
| SDO / ADDR | GND (0x76)       |

Notes:

- This project uses I2C mode, so CS/CSB must be tied HIGH (3V3).
- SDO/ADDR selects I2C address:
  - SDO to GND -> 0x76 (default in this project)
  - SDO to 3V3 -> 0x77
- If your module is 0x77, update BME680_I2C_ADDR in include/config.h.
- If your sensor board has a QWIIC/STEMMA-QT connector, it is still the same I2C bus (SDA/SCL). You may use either QWIIC cable or direct GPIO wiring.

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

Firmware now prints I2C/BME680 diagnostics every 5 seconds. Look for these lines:

- `[I2C] main(...)` full scan on primary bus (GPIO18/GPIO17)
- `[I2C] alt(...)` full scan on fallback bus (GPIO44/GPIO43)
- `[I2C] ...` quick probe of touch and BME680 addresses on both buses
- `[BME680] ...` sensor detection status and latest data snapshot

Firmware will auto-probe BME680 on both buses and pick the first valid one:

- MAIN bus: GPIO18 (SDA), GPIO17 (SCL)
- ALT bus: GPIO44 (SDA), GPIO43 (SCL)

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
  - For I2C mode modules with CS/SDO pins: CS must be tied to 3V3.
  - SDO to GND = 0x76, SDO to 3V3 = 0x77.
  - Verify BME680 I2C address (0x76 or 0x77).
  - Open Serial Monitor and read `[I2C]` / `[BME680]` debug output every 5 seconds.
  - If BME appears only on `alt(...)` scan, move wiring to GPIO44/43 or update your cable breakout accordingly.
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
- Clarified BME680 I2C wiring for modules with CS/SDO pins (CS to 3V3, SDO for 0x76/0x77 address select).
- Added periodic serial diagnostics (every 5 seconds) for I2C probe and BME680 detection/data status.
- Added dual-bus BME680 auto-probe (main 18/17 and fallback 44/43) with detailed serial monitor output.
