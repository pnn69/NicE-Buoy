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
