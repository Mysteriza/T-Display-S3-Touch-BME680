# T-Display-S3 Touch BME680 Monitor

Firmware monitor lingkungan offline untuk LilyGO T-Display-S3 + BME680/BME688 dengan Bosch BSEC2.

README ini menjelaskan cara kerja sistem dan arti setiap angka di layar berdasarkan implementasi kode saat ini, supaya pengguna tidak perlu membaca source code.

## Design Prototype

![UI Design Prototype](UI%20Design%20Prototype.png)

## Ringkasan Cara Kerja

Sistem berjalan dalam dua loop utama (FreeRTOS, beda core):

- Loop sensor: menjalankan scheduler BSEC2 terus-menerus, membaca data BME68x, lalu membangun snapshot data.
- Loop UI: membaca snapshot terakhir dan merender ke layar.

Prinsip penting:

- Sensor diproses real-time oleh BSEC (`run()` dipanggil sering), bukan tiap 30 detik.
- Data yang dipublikasikan ke UI disinkronkan setiap `kSensorRefreshMs` (default 30 detik) untuk mengurangi redraw dan hemat baterai.
- Jika data invalid sementara, UI menahan snapshot valid terakhir dulu (anti flicker), lalu menampilkan `No Data` bila memang stale.

## Struktur Modul

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

- `config.h`: semua parameter yang bisa dituning (pin, timing, warna, batas validasi).
- `sensor_manager.*`: pipeline BSEC2, health sensor, simpan/restore state kalibrasi, command serial.
- `ui_controller.*`: semua tampilan LVGL dan mapping angka ke label/status/warna.
- `power_mgmt.*`: timeout layar, wake button, dan profile frekuensi CPU.

## Data Di Setiap Page

### Page 1 (Environment)

- `Temperature (C)`
  - Sumber: `BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE`.
  - Ini suhu yang sudah dikompensasi panas sensor, lebih representatif daripada suhu mentah.

- `Humidity (%)`
  - Sumber: `BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY`.
  - Juga sudah dikompensasi terhadap efek panas internal.

- `Pressure (hPa)`
  - Sumber: `BSEC_OUTPUT_RAW_PRESSURE` (wrapper BSEC2 Arduino sudah dalam hPa di proyek ini).

- `Altitude (m)`
  - Bukan nilai langsung dari sensor.
  - Dihitung dari pressure + sea-level pressure (SLP/QNH) dengan rumus barometrik:
  - $alt = 44330 \times (1 - (P / SLP)^{0.1903})$

### Page 2 (AQI)

- `Gas resistance (kOhm)`
  - Sumber: `BSEC_OUTPUT_RAW_GAS` (Ohm), lalu dibagi 1000 menjadi kOhm.

- `IAQ`
  - Sumber utama: `BSEC_OUTPUT_IAQ`.
  - Fallback: jika IAQ utama invalid, pakai `BSEC_OUTPUT_STATIC_IAQ`.
  - Nilai ditampilkan sebagai integer untuk keterbacaan.

- `Status`
  - Dibentuk dari nilai IAQ:
  - `0..50 Excellent`, `51..100 Good`, `101..150 Moderate`, `151..200 Poor`, `201..300 Unhealthy`, `>300 Hazardous`.
  - Warna mengikuti severity status (hijau -> kuning -> merah).

- `Accuracy`
  - Sumber: `out.accuracy` dari output IAQ BSEC (0..3).
  - Label: `Very Low`, `Low`, `Medium`, `High`.
  - Warna:
  - `Very Low/Low = merah`, `Medium = kuning`, `High = hijau`.

- `Temp` dan `Humidity` mini di page AQI
  - Nilainya sama dengan data environment yang sudah dikompensasi BSEC.

### Page 3 (System)

- `Uptime`
  - Dihitung dari `esp_timer_get_time()` lalu diformat `HH:MM:SS`.

- `CPU Freq (MHz)`
  - Bukan “estimasi load”.
  - Nilai real dari `getCpuFrequencyMhz()`.
  - Akan berubah sesuai mode power manager (aktif/sleep).

- `Storage (MB)`
  - `Total`: `ESP.getFlashChipSize()`.
  - `Free`: `ESP.getFreeSketchSpace()`.
  - Ini metrik ruang firmware flash, bukan sisa RAM heap.

- `Battery (%)`
  - Dari ADC pin battery, dikalibrasi (`esp_adc_cal_raw_to_voltage`), dikali 2 karena divider.
  - Dihaluskan, lalu dipetakan ke 0..100% dengan batas `kMinMv..kMaxMv`.

## Data Real (Bukan Dummy)

Yang diukur langsung dari sensor/BSEC:

- Temperature, humidity, pressure, gas resistance, IAQ, static IAQ, run-in status, stabilization status, IAQ accuracy.

Yang dihitung dari data nyata:

- Altitude (dari pressure + SLP), status IAQ (dari nilai IAQ), label accuracy (dari level accuracy), battery %, uptime, storage string.

Yang bukan sensor lingkungan:

- CPU frequency (telemetri sistem), storage (telemetri flash).

Sistem tidak menggunakan angka random/fake untuk nilai sensor.

## Kenapa IAQ Bisa Stuck di 50 dan Cara Recovery

Secara teori BSEC, pada fase awal kalibrasi IAQ bisa lama di sekitar 50 dan accuracy rendah.

Proteksi di firmware:

- Deteksi pola stuck IAQ sekitar 50 + accuracy rendah dalam durasi panjang (`kIaqStuckTimeoutMs`).
- Jika terpenuhi, firmware melakukan recovery satu kali:
  - hapus state blob BSEC,
  - reinit pipeline,
  - lanjut kalibrasi dari kondisi baru.

State kalibrasi normal tetap disimpan periodik dan saat accuracy naik, agar progress tidak hilang setiap reboot.

## Sinkronisasi Refresh dan Hemat Baterai

- Publish snapshot sensor ke UI: tiap `kSensorRefreshMs` (default 30 detik).
- Refresh nilai UI: interval yang sama (`kUiValuesRefreshMs = kSensorRefreshMs`).

Dampak:

- Redraw lebih sedikit.
- Konsumsi daya layar/CPU lebih rendah.
- Tampilan lebih stabil (tidak berkedip update terus).

## Hardware

- LilyGO T-Display-S3 (touch)
- BME680/BME688 via I2C
- USB-C cable

## Wiring

| Sensor   | T-Display-S3               |
| -------- | -------------------------- |
| VCC      | 3V3                        |
| GND      | GND                        |
| SDA      | GPIO18                     |
| SCL      | GPIO17                     |
| CS/CSB   | 3V3                        |
| SDO/ADDR | GND (0x76) atau 3V3 (0x77) |

## Build dan Upload

Build:

```bash
platformio run -e lilygo-t-display-s3
```

Upload:

```bash
platformio run -e lilygo-t-display-s3 -t upload
```

Jika perlu port spesifik:

```bash
platformio run -e lilygo-t-display-s3 -t upload --upload-port COM5
```

## Serial Commands

Buka monitor:

```bash
platformio device monitor -b 115200
```

| Command            | Fungsi                                               |
| ------------------ | ---------------------------------------------------- |
| `help`             | Daftar command                                       |
| `status`           | Status sensor + data terbaru                         |
| `get slp`          | Lihat sea-level pressure saat ini                    |
| `set slp <hPa>`    | Set sea-level pressure manual                        |
| `set alt <meter>`  | Kalibrasi sea-level pressure dari altitude referensi |
| `reset slp`        | Reset sea-level pressure ke default                  |
| `sensor reinit`    | Paksa reinit sensor/BSEC                             |
| `i2c scan`         | Scan device di bus I2C                               |
| `debug detail on`  | Aktifkan debug verbose periodik                      |
| `debug detail off` | Matikan debug verbose                                |

## Troubleshooting Cepat

- Sensor tidak terdeteksi:
  - Cek wiring SDA/SCL dan alamat 0x76/0x77.

- Accuracy lama di `Very Low`:
  - Jalankan perangkat kontinu lebih lama.
  - Pantau `status` (run-in dan stabilization).

- IAQ terlihat tidak banyak berubah:
  - Uji dengan perubahan lingkungan nyata (ventilasi, sumber VOC aman, perubahan kelembapan).
  - Cek `status` berkala untuk melihat snapshot baru dan progression accuracy.

- Angka berkedip/hilang:
  - Firmware menahan snapshot valid saat gangguan singkat.
  - Jika data benar-benar stale, UI akan tampil `No Data`.
