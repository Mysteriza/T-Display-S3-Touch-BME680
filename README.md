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
| SDA        | GPIO43           |
| SCL        | GPIO44           |
| CS / CSB   | 3V3              |
| SDO / ADDR | GND (0x76)       |

Notes:

- This project uses I2C mode, so CS/CSB must be tied HIGH (3V3).
- SDO/ADDR selects I2C address:
  - SDO to GND -> 0x76 (default in this project)
  - SDO to 3V3 -> 0x77
- Firmware auto-detects BME680 at 0x76 or 0x77. You can keep default config for most setups.
- If your sensor board has a QWIIC/STEMMA-QT connector, it is still the same I2C bus (SDA/SCL). You may use either QWIIC cable or direct GPIO wiring.
- Firmware will try sensor bus GPIO43/GPIO44 first, then fallback to GPIO18/GPIO17.

### 3) Alternative: BME680 via QWIIC Port

If both boards provide QWIIC/STEMMA-QT sockets, use a 4-pin JST-SH cable directly:

- T-Display-S3 QWIIC <-> BME680 QWIIC/STEMMA-QT
- No separate SDA/SCL jumper wires are needed.

QWIIC signal map (standard 4-wire I2C):

| QWIIC Signal | Function  |
| ------------ | --------- |
| GND          | Ground    |
| 3V3          | Power     |
| SDA          | I2C Data  |
| SCL          | I2C Clock |

Important notes for QWIIC usage:

- QWIIC is only a physical connector; protocol remains standard I2C.
- Firmware still uses the same auto-probe logic: GPIO43/44 preferred, GPIO18/17 fallback.
- For modules exposing CS/SDO pins or jumpers:
  - CS/CSB must be HIGH (3V3) for I2C mode.
  - SDO selects address: 0x76 (GND) or 0x77 (3V3). Firmware auto-detects both.

Quick verification after switching to QWIIC:

1. Upload firmware as usual.
2. Open serial monitor at 115200.
3. Confirm BME680 appears in `[I2C] alt(...)` or `[I2C] main(...)` scan at 0x76/0x77.
4. Confirm boot self-check reports sensor `[OK]`.

Validated QWIIC wiring that works in this project:

- VCC -> 3V3
- GND -> GND
- SCL -> QWIIC SCL / GPIO44
- SDA -> QWIIC SDA / GPIO43
- CS/CSB -> 3V3 (force I2C mode)

Observed runtime result with this wiring:

- Sensor detected on ALT bus (GPIO43/GPIO44)
- Address detected at 0x77
- CHIP_ID detected as 0x61 (valid BME68x)

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

- `[I2C] main(...)` full scan on shared bus (GPIO18/GPIO17, touch bus)
- `[I2C] alt(...)` full scan on preferred sensor bus (GPIO43/GPIO44)
- `[I2C] ...` quick probe of touch and BME680 addresses on both buses
- `[BME680] ...` sensor detection status and latest data snapshot

Firmware will auto-probe BME680 on both buses and pick the first valid one:

- Preferred bus: GPIO43 (SDA), GPIO44 (SCL)
- Fallback bus: GPIO18 (SDA), GPIO17 (SCL)

## Runtime Behavior

- Boot performs hardware checks and shows OK or FAIL states.
- UI starts on Page 1.
- Swipe left or right to change page.
- Display timeout default is 15 seconds of inactivity.
- Any touch activity or wake button event resets the timeout.
- If BME680 init fails during early boot timing, firmware retries sensor init automatically in background.
- Firmware now publishes the first sensor snapshot as soon as the first valid BSEC sample is available (no 30-second initial wait).

## Configuration

Edit include/config.h for common adjustments:

- DISPLAY_TIMEOUT (default 15000 ms)
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
  - Firmware auto-detects both 0x76 and 0x77; ensure SDO strap is stable and not floating.
  - BME680/BME688 must report CHIP_ID `0x61`. If serial says device exists at 0x76/0x77 but CHIP_ID is not `0x61`, the connected module is likely not a BME68x sensor.
  - If serial shows `bsec_status=14` during init, firmware now auto-falls back across BSEC sample-rate modes (LP -> ULP -> CONT) to find a compatible subscription.
  - Open Serial Monitor and read `[I2C]` / `[BME680]` debug output every 5 seconds.
  - If BME appears only on `alt(...)` scan, keep wiring on GPIO43/44.
  - If BME appears only on `main(...)` scan, move wiring to GPIO18/17.
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
- Updated Page 03 system info cadence: CPU Load refreshes every 2 seconds, while Storage/Battery refresh every 5 seconds.
- Updated Page 03 storage text to numeric free/total MB format without float printf.
- Updated gas resistance unit fallback to ASCII `kOhm` for font compatibility on-board.
- Clarified BME680 I2C wiring for modules with CS/SDO pins (CS to 3V3, SDO for 0x76/0x77 address select).
- Added periodic serial diagnostics (every 5 seconds) for I2C probe and BME680 detection/data status.
- Added dual-bus BME680 auto-probe with preferred sensor bus 43/44 and fallback 18/17, plus detailed serial monitor output.
- Improved top header symmetry by aligning `Env_monitor` vertically with battery indicator and PAGE labels on all pages.
- Changed default display timeout from 10 seconds to 15 seconds.
- Lowered and re-centered IAQ gauge placement on Page 02 to keep safe spacing from the battery indicator.
- Removed duplicate `Uptime` subtitle on Page 03 and kept uptime context in the top-right page header only.
- Simplified battery label text to icon + percentage only (including USB-powered mode), kept top-center alignment.
- Fine-tuned `Env_monitor` vertical offset by 1px upward for tighter alignment with battery and PAGE header text.
- Added explicit QWIIC wiring alternative section (direct cable method, address notes, and verification steps).
- Improved BME680 robustness with multi-address auto-detect (0x76/0x77) and automatic init retry when early boot probe fails.
- Added BME68x CHIP_ID (0x61) validation in diagnostics to detect wrong devices responding at 0x76/0x77.
- Added automatic BSEC subscription fallback across sample-rate modes (LP/ULP/CONT) to handle samplerate mismatch warnings (`bsec_status=14`).
- Fixed pressure scaling path (removed double conversion), which restores realistic pressure and derived altitude values.
- Improved first-sample UX by publishing sensor data immediately after first valid BSEC output instead of waiting for periodic interval.
- Updated Page 01 numeric card typography (smaller value font) to avoid overlap with card titles.
- Updated Temp/Humidity/Pressure formatting to 2 decimal places.
- Added IAQ status/risk mapping with dynamic labels and IAQ color state on Page 02.
