# 🛠️ GLOBAL AI AGENT RULES: EMBEDDED ARCHITECT MODE (ALWAYS ON)

## 1. CODE QUALITY, STYLE & BEST PRACTICES

- Produce clean, modular, well-structured, maintainable, professional, readable, and tested code. If you can make it optimal, think again about making it even more optimal than you think.
- Inline comments must be minimized; used only for critical documentation. Comments should be short, concise, and simple, explaining complex logic.
- Avoid duplicated/redundant logic; refactor for simplification. Avoid race conditions, dead code, and anti-patterns.
- Follow official C++ style guides and industry-standard best practices. Avoid over-engineering.
- Output must be deterministic. Generated code must be executable and free from basic runtime errors.

## 2. PROJECT SPECIFICATION: T-DISPLAY S3 MONITOR

- **MCU:** ESP32-S3 (Lilygo T-Display S3).
- **Display:** 170x320 8-bit Parallel (ST7789), GPIO 38 Backlight. Orientation: Landscape Right.
- **Sensor:** BME680 (I2C) with Bosch BSEC 2.x (IAQ calculation). NO WiFi.
- **Input:** Touch CST816 & Physical Wake Button (GPIO 14).
- **Navigation:** Only Swipe Left/Right (LV_EVENT_GESTURE). No other buttons on data pages.

## 3. ARCHITECTURE & MODULARITY

- Strictly split code into:
  - `config.h`: Adjustable constants (DISPLAY_TIMEOUT 10000ms, REFRESH_INTERVAL 30000ms, Pins).
  - `sensor_manager.cpp/h`: BSEC2 logic, NVS state persistence (save state every 24h).
  - `ui_controller.cpp/h`: LVGL screens (3 Pages) and layout.
  - `power_mgmt.cpp/h`: Smart timeout logic and GPIO 14 interrupt.
- **Concurrency:** Use FreeRTOS. Core 0: Sensors/BSEC. Core 1: UI/LVGL.

## 4. FUNCTIONAL LOGIC & UI UX

- **Boot Sequence:** Perform hardware check (LCD, Touch, BME680). Display results: [OK] in Green, [FAIL] in Red.
- **Smart Timeout:** - Screen turns OFF after 10s (default) of inactivity.
  - Reset timer on: Touch event OR GPIO 14 press. Both MUST wake the screen.
- **Uptime Logic:** Implement a robust formatter to display uptime as `HH:MM:SS` from `esp_timer_get_time()`.
- **UI Design:** Replicate the provided screenshot exactly.
  - Theme: Pure Black (#000000).
  - Accents: Cyan (#00FFFF) for values, Neon Green (#39FF14) for status.
  - AVOID AI-generic colors (purple/violet). Prioritize clarity and professionalism.

## 5. REPOSITORY & FILE HYGIENE

- Keep project structure minimal and logical.
- Every change to code or dependencies MUST be reflected in `README.md`.
- `README.md` must include: Installation steps, usage, feature list, and change log.
- Do not commit updates without permission.

## 6. AGENT BEHAVIOR

- Respond directly to the core request; no introductions or closings.
- Be concise and action-oriented. Disable "nice mode".
- Be honest and critical; challenge incorrect or weak requirements based on technical correctness.
- If requirements are ambiguous, ask one concise clarification question.
- Always check the "Problems" tab (@current_problems) and fix issues immediately.
- If you encounter an error, provide a clear, actionable solution. Do not just state the error.
- Do not generate code that is not directly related to the specified requirements.
- NEVER PUT changelog in README.md file! Make it clean and concise but instructive and informative. And ALWAYS use English in README.md, no other languages.
