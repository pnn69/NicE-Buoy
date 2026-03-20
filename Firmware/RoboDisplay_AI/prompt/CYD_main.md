# Prompt: CYD Hello World with ESP-IDF
## Objective
Create a "Hello World" display for the Cheap Yellow Display (CYD) / ESP32-2432S028 using the Espressif IoT Development Framework (ESP-IDF) and LVGL.

## Target Hardware
- **Board:** ESP32-2432S028 (CYD)
- **LCD Driver:** ILI9341
- **LCD Pins (SPI2):**
  - MOSI: 13
  - SCK: 14
  - MISO: 12
  - CS: 15
  - DC: 2
  - Backlight: 21 (Active High)
- **RGB LED Pins:**
  - Red: 4
  - Green: 16
  - Blue: 17
  - (Note: LEDs are typically active-low on this board)
- **Touch Driver:** XPT2046
- **Touch Pins (SPI3):**
  - MOSI: 32
  - MISO: 39
  - SCK: 25
  - CS: 33
- **Framework:** ESP-IDF v5.x
- **UI Library:** LVGL v9.x
- **Mirroring:** Horizontal mirroring enabled (`mirror_x = true`) to fix inverted text.
- **Touch Alignment:** XY swapping enabled (`swap_xy = true`) for correct coordinate mapping.

## Features
- "CYD Click Counter" title label.
- Click counter display ("Clicks: X").
- "Push me" button in the center.
- Button interaction: 
    - Increments click counter.
    - Updates "Clicks: X" label on display.
    - Prints "I'm pushed! Total clicks: X" to the serial port.
- **RGB LED Control:**
    - Three buttons at the bottom: "Red", "Green", "Blue".
    - Toggling a button switches the corresponding on-board RGB LED component.
- Periodic Serial Output: Prints "Hello World" every second at 115200 baud.

## Automation Workflow
When this prompt is executed, the following steps must be performed automatically:
1.  **Dependencies:** Ensure `main/idf_component.yml` includes:
    - `espressif/esp_lcd_ili9341`
    - `lvgl/lvgl`
    - `espressif/esp_lvgl_port`
    - `espressif/esp_lcd_touch`
    - `atanisoft/esp_lcd_touch_xpt2046`
2.  **Code Implementation:** Update `main/main.c` with the click counter logic and dual SPI bus configuration (SPI2 for LCD, SPI3 for Touch) as defined in the Target Hardware section.
3.  **Build & Flash:** 
    - Compile the project using `idf.py build`.
    - Identify the correct serial port (defaulting to COM4 if available).
    - Flash the firmware and start the serial monitor at 115200 baud.

## How to Build
1.  Ensure you have **ESP-IDF v5.x** installed and configured in your shell.
2.  Navigate to the project root directory.
3.  Run `idf.py build` to compile the project. The ESP-IDF Component Manager will automatically download LVGL and the ILI9341 driver.
4.  Flash the board using `idf.py -p <PORT> flash monitor`.

## Note on Hardware Revisions
If your CYD uses an ST7789 driver instead of ILI9341, you will need to:
1. Update `main/idf_component.yml` to use `espressif/esp_lcd_panel_st7789`.
2. Update `main/main.c` to include `esp_lcd_panel_st7789.h` and use `esp_lcd_new_panel_st7789()`.









# Prompt 2

You are a senior embedded software engineer. Generate a **complete, working ESP32 project** for the Cheap Yellow Display (CYD ESP32-2432S028) using **ESP-IDF v5.x** and **LVGL v9.x**. The project must implement a "Hello World" display, a click counter button, RGB LED controls, and periodic serial output. All code must be fully functional and ready to build/flash.

---

# 1️⃣ PROJECT OVERVIEW
- Name: CYD Hello World
- Goal: Display "CYD Click Counter", implement click counter, control RGB LEDs, and print periodic serial messages.
- Languages: C
- Platform: ESP32-2432S028
- Framework: ESP-IDF v5.x
- UI Library: LVGL v9.x

---

# 2️⃣ HARDWARE CONFIGURATION

## LCD (SPI2)
- Driver: ILI9341
- Pins: MOSI:13, SCK:14, MISO:12, CS:15, DC:2, Backlight:21 (Active High)
- Horizontal mirroring: mirror_x = true

## Touch (SPI3)
- Driver: XPT2046
- Pins: MOSI:32, MISO:39, SCK:25, CS:33
- XY swapping: swap_xy = true

## RGB LEDs
- Red: 4, Green: 16, Blue: 17 (active-low)

---

# 3️⃣ FEATURES
1. Title label: "CYD Click Counter"
2. Click counter label: "Clicks: X"
3. Center button "Push me":
   - Increment click counter
   - Update label
   - Print "I'm pushed! Total clicks: X" to serial
4. Bottom buttons "Red", "Green", "Blue" to toggle LEDs
5. Periodic serial output: "Hello World" every second at 115200 baud

---

# 4️⃣ AUTOMATION WORKFLOW

### Dependencies (`main/idf_component.yml`)
- espressif/esp_lcd_ili9341
- lvgl/lvgl
- espressif/esp_lvgl_port
- espressif/esp_lcd_touch
- atanisoft/esp_lcd_touch_xpt2046

### Code Implementation (`main/main.c`)
- Initialize SPI2 (LCD) and SPI3 (Touch)
- Configure LVGL display and input drivers
- Implement click counter and RGB LED button logic
- Periodic serial output every second
- Include proper horizontal mirroring and XY swap

### Build & Flash
- Compile: `idf.py build`
- Flash: `idf.py -p <PORT> flash monitor` (115200 baud)
- Ensure ESP-IDF v5.x environment is sourced

---

# 5️⃣ OUTPUT FORMAT
1. Show **project tree** first
2. Then **each file separately** with headers, e.g.:

--- main.c ---
<code>

3. Include `idf_component.yml` with all dependencies
4. Include small usage/test snippets if applicable
5. Do **not** skip code or leave placeholders

---

# 6️⃣ REGENERATION RULE
- Must be fully reproducible
- Keep pin assignments, LVGL config, SPI2/3 setup, button and LED logic consistent
- Include optional instructions for ST7789 driver if needed

---

# 7️⃣ IMPORTANT INSTRUCTIONS
- Comment non-obvious logic
- Keep modular, readable C code
- Include SPI2/3 initialization, LVGL setup, GPIO control, and periodic serial output
- Do not summarize or omit any part
- Ensure project can build and flash without manual edits

---

# 8️⃣ GENERATE FILES
- `main/main.c` with full implementation
- `main/idf_component.yml` with dependencies
- Full project tree (folders/files)