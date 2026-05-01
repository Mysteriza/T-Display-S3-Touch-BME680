# T-Display-S3 Touch BME680 Monitor

Environmental monitor firmware for LilyGO T-Display-S3 Touch + BME680/BME688 with Bosch BSEC2.

## Features

- **Page 01 (ENV)**: Temperature, Humidity, Pressure, Altitude
- **Page 02 (GAS)**: Gas Resistance, Gas Status, Gas Trend
- **Page 03 (SYSTEM)**: Uptime, CPU Load, Free Memory, Battery Voltage
- **Tap Navigation**: Tap anywhere to change page
- **Smart Timeout**: Screen turns off after 15s of inactivity
- **Boot Diagnostics**: LCD, Touch, Sensor checks with progress bar

## Hardware

- LilyGO T-Display-S3 (Touch)
- BME680 or BME688 (I2C)

## Wiring

| Sensor   | T-Display-S3 |
| -------- | ------------- |
| VCC      | 3V3           |
| GND      | GND           |
| SDA      | GPIO18        |
| SCL      | GPIO17        |
| SDO/ADDR | GND (0x76)    |

## Installation

```bash
# Build
platformio run

# Upload
platformio run -t upload
```

## Specifications

- Display timeout: 15 seconds
- Sensor refresh: 30 seconds
- UI refresh: 30 seconds

## Version History

- v0.3.0 - Added battery card, improved boot UI, fixed gas value display
- v0.2.0 - Initial release with tap navigation
- v0.1.0 - First implementation