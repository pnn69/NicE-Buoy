# AI Generator Prompt: Resistive Cheap Yellow Display (CYD) ESP32-2432S028R Firmware

Copy and paste this prompt into any AI coding assistant to fully generate or rebuild this modular, over-the-air (OTA) enabled resistive Cheap Yellow Display (CYD) project with perfect hardware settings, correct libraries, and flicker-free visual update feedback.

---

### System Role & Objective
You are an expert embedded software engineer specializing in ESP32 development, PlatformIO, and the Arduino framework. Your task is to generate a modular, highly organized, and robust C++ project for the **resistive Cheap Yellow Display (CYD)**, also known as the **ESP32-2432S028R**.

### Key Hardware Requirements
1. **Target Board:** ESP32 Dev Module (WROOM-32).
2. **Display Controller:** ILI9341 (configured as `ILI9341_2_DRIVER` under TFT_eSPI) connected via HSPI.
3. **Display Backlight Control:** GPIO 21 (Active `HIGH`).
4. **Touchscreen Controller:** Resistive SPI-based XPT2046 connected via VSPI.
5. **Touch Controller Pins:** `CLK: 25`, `MISO: 39`, `MOSI: 32`, `CS: 33`, `IRQ: 36`.
6. **Onboard RGB LED Pins:** Red: GPIO 4, Green: GPIO 16, Blue: GPIO 17 (Active `LOW`).

### Architecture & Design Rules
- **Modular Design:** Do not write a single monolithic file. Separate responsibilities into dedicated modules: display control, touch sensing, networking, LED drivers, and the main game.
- **Dynamic Orientation-Independent Drawing:** Never hardcode display sizes (e.g., 320 or 240) in your graphics, coordinates, or bounds checking. Always query `tft.width()` and `tft.height()` dynamically so that switching orientation (e.g., Rotation 0, 1, 2, or 3) automatically adjusts layout and bouncing physics.
- **Flicker-Free OTA Updates:** Screen clearing during updates causes high-frequency flickering. Cache the progress percentage as an integer, only redraw screen graphics when the percentage changes, and only clear the text bounding box region rather than the whole screen.
- **Clockwise Border Progress Loader:** On OTA start, draw a crisp white border rectangle around the screen outer edge. As progress climbs, fill the border clockwise with a red line (0-25% top edge, 25-50% right edge, 50-75% bottom edge, 75-100% left edge) for a highly polished, visual loading effect.

---

## File-by-File Implementation Details

### 1. Project Configuration: `platformio.ini`
Configure PlatformIO with Espressif32, standard partitions, Deep library dependency finder, and TFT_eSPI user setup flags explicitly defined in the build flags:

```ini
[platformio]
src_dir = .
default_envs = cyd

[env]
platform = espressif32@6.5.0
board = esp32dev
framework = arduino
lib_ldf_mode = deep
lib_deps =
        bodmer/TFT_eSPI@^2.5.42
        https://github.com/PaulStoffregen/XPT2046_Touchscreen.git
        lvgl/lvgl@^8.3.6

monitor_speed = 115200
monitor_port = COM9
monitor_filters = esp32_exception_decoder
upload_speed = 921600
upload_port = COM15
board_build.partitions = min_spiffs.csv
build_flags =
        -I./src/
        -I./src/ui/
        -DUSER_SETUP_LOADED
        -DILI9341_2_DRIVER
        -DUSE_HSPI_PORT
        -DTFT_WIDTH=240
        -DTFT_HEIGHT=320
        -DTFT_MISO=12
        -DTFT_MOSI=13
        -DTFT_SCLK=14
        -DTFT_CS=15
        -DTFT_DC=2
        -DTFT_RST=-1
        -DTFT_BL=21
        -DTFT_BACKLIGHT_ON=HIGH
        -DTFT_BACKLIGHT_OFF=LOW
        -DLOAD_GLCD
        -DSPI_FREQUENCY=55000000
        -DSPI_READ_FREQUENCY=20000000
        -DSPI_TOUCH_FREQUENCY=2500000
        -DLOAD_FONT2
        -DLOAD_FONT4
        -DLOAD_FONT6
        -DLOAD_FONT7
        -DLOAD_FONT8
        -DLOAD_GFXFF

[env:cyd]
build_flags =
        ${env.build_flags}
        -DTFT_INVERSION_OFF
```

*Note: Since PlatformIO attempts to compile the listed `lvgl` dependency, copy or create a dummy `lv_conf.h` template in `.pio/libdeps/cyd/lv_conf.h` to avoid compilation errors.*

### 2. Onboard LED Driver: `src/RGBledDriver.h`
```cpp
#ifndef _RGB_LED_DRIVER_H_
#define _RGB_LED_DRIVER_H_

void ChangeRGBColor(uint32_t color);            // uses 32-bit color code (e.g. 0xFFd251)
void setColor(uint8_t R, uint8_t G, uint8_t B); // uses individual 8-bit values for R, G, and B
void initRGBled();

#define RGB_COLOR_1 0xFF0000
#define RGB_COLOR_2 0x00FF00
#define RGB_COLOR_3 0x0000FF
#define RGB_COLOR_4 0xFF00ff
#endif // _RGB_LED_DRIVER_H_
```

### 3. Onboard LED Driver: `src/RGBledDriver.cpp`
```cpp
#include <Arduino.h>
#include "RGBledDriver.h"

#define RED_LED_PIN 4
#define GREEN_LED_PIN 16
#define BLUE_LED_PIN 17

uint8_t getRedValueFromColor(uint32_t c) { return (c >> 16); }
uint8_t getGreenValueFromColor(uint32_t c) { return (c >> 8); }
uint8_t getBlueValueFromColor(uint32_t c) { return (c); }

void setColor(uint8_t R, uint8_t G, uint8_t B) {
    // Note: CYD RGB LEDs are Active-LOW (HIGH is OFF, LOW is ON)
    analogWrite(RED_LED_PIN, 255 - R);
    analogWrite(GREEN_LED_PIN, 255 - G);
    analogWrite(BLUE_LED_PIN, 255 - B);
}

void ChangeRGBColor(uint32_t color) {
    setColor(getRedValueFromColor(color), getGreenValueFromColor(color), getBlueValueFromColor(color));
}

void initRGBled() {
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(GREEN_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);
}
```

### 4. Display Module: `src/cyd_display.h`
```cpp
#ifndef CYD_DISPLAY_H
#define CYD_DISPLAY_H

#include <TFT_eSPI.h>

extern TFT_eSPI tft;

void init_display();

#endif // CYD_DISPLAY_H
```

### 5. Display Module: `src/cyd_display.cpp`
```cpp
#include <Arduino.h>
#include "cyd_display.h"

TFT_eSPI tft = TFT_eSPI();

void init_display() {
    // Backlight on GPIO 21 must be explicitly driven HIGH
    #ifdef TFT_BL
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);
    #else
    pinMode(21, OUTPUT);
    digitalWrite(21, HIGH);
    #endif

    tft.begin();
    tft.setRotation(0); // Choose 0 for portrait, 1 for landscape, 2 for portrait inverted, 3 for landscape inverted
}
```

### 6. Touchscreen Module: `src/cyd_touch.h`
```cpp
#ifndef CYD_TOUCH_H
#define CYD_TOUCH_H

void init_touch();
bool get_touch_point(int &x, int &y);

#endif // CYD_TOUCH_H
```

### 7. Touchscreen Module: `src/cyd_touch.cpp`
```cpp
#include <Arduino.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include "cyd_touch.h"

#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

SPIClass touchSpi(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);

void init_touch() {
    touchSpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
    ts.begin(touchSpi);
    ts.setRotation(0); // Set to same rotation as display
}

bool get_touch_point(int &x, int &y) {
    if (ts.tirqTouched() && ts.touched()) {
        TS_Point p = ts.getPoint();

        // Calibration bounds for resistive touchscreen on CYD
        const uint16_t ts_minx = 300;
        const uint16_t ts_maxx = 3800;
        const uint16_t ts_miny = 260;
        const uint16_t ts_maxy = 3800;

        // Map and scale touch ranges relative to the current rotation.
        // For Rotation 0: (Portrait USB Top)
        int touchX = map(p.x, ts_minx, ts_maxx, 1, 240);
        int touchY = map(p.y, ts_miny, ts_maxy, 1, 320);
        
        // Note: For Landscape Rotation 1: map(p.x, ts_minx, ts_maxx, 1, 320) & map(p.y, ts_miny, ts_maxy, 1, 240)
        // Note: For Portrait Rotation 2: map(p.x, ts_maxx, ts_minx, 1, 240) & map(p.y, ts_maxy, ts_miny, 1, 320)
        // Note: For Landscape Rotation 3: map(p.x, ts_maxx, ts_minx, 1, 320) & map(p.y, ts_maxy, ts_miny, 1, 240)

        x = constrain(touchX, 1, 240); // Keep within tft.width() limit
        y = constrain(touchY, 1, 320); // Keep within tft.height() limit

        return true;
    }
    return false;
}
```

### 8. WiFi & OTA Module: `src/cyd_wifi.h`
```cpp
#ifndef CYD_WIFI_H
#define CYD_WIFI_H

void init_wifi_and_ota();
void handle_ota();

#endif // CYD_WIFI_H
```

### 9. WiFi & OTA Module: `src/cyd_wifi.cpp`
```cpp
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "cyd_wifi.h"
#include "cyd_display.h"

bool scan_and_connect_wifi() {
    Serial.println("Starting WiFi scan...");
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.drawString("WiFi Scan...", 20, 20);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    int n = WiFi.scanNetworks();
    Serial.printf("Scan done: %d networks found\n", n);

    bool robobuoy_found = false;
    bool nice_wifi_found = false;

    for (int i = 0; i < n; ++i) {
        String ssid = WiFi.SSID(i);
        if (ssid == "ROBOBUOY") {
            robobuoy_found = true;
        } else if (ssid == "NicE_WiFi") {
            nice_wifi_found = true;
        }
    }

    if (robobuoy_found) {
        Serial.println("Found 'ROBOBUOY'. Connecting...");
        tft.drawString("Connecting to ROBOBUOY...", 20, 50);
        WiFi.begin("ROBOBUOY", "");
    } else if (nice_wifi_found) {
        Serial.println("Found 'NicE_WiFi'. Connecting...");
        tft.drawString("Connecting to NicE_WiFi...", 20, 50);
        WiFi.begin("NicE_WiFi", "!Ni1001100110");
    } else {
        Serial.println("No configured WiFi found. AP mode fallback...");
        tft.drawString("WiFi not found!", 20, 50);
        tft.drawString("Starting AP: RoboCYD", 20, 80);
        WiFi.softAP("RoboCYD", "");
        return false;
    }

    unsigned long conn_timeout = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - conn_timeout > 15000) {
            Serial.println("\nConnection timed out. Starting AP RoboCYD...");
            tft.drawString("WiFi Timeout!", 20, 80);
            tft.drawString("Starting AP: RoboCYD", 20, 110);
            WiFi.softAP("RoboCYD", "");
            return false;
        }
    }

    Serial.println("\nWiFi CONNECTED");
    WiFi.setSleep(WIFI_PS_NONE); // Disable sleep for high throughput and stable OTA
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    char buf[64];
    sprintf(buf, "IP: %s", ip.toString().c_str());
    tft.drawString("Connected!", 20, 80);
    tft.drawString(buf, 20, 110);
    delay(2000);
    return true;
}

void setup_Arduino_OTA() {
    byte mac[6];
    WiFi.macAddress(mac);
    char hostname[32];
    sprintf(hostname, "CYD_%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    ArduinoOTA.setHostname(hostname);
    
    ArduinoOTA.onStart([]() {
        Serial.println("OTA Start");
        tft.fillScreen(TFT_BLACK);
        
        int w = tft.width();
        int h = tft.height();
        
        // Draw initial white boundary border dynamically
        tft.drawRect(0, 0, w, h, TFT_WHITE);
        
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextSize(3);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("OTA Update...", w / 2, h / 2 - 30);
    });
    
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA End");
        tft.fillScreen(TFT_BLACK);
        tft.drawString("Restarting...", tft.width() / 2, tft.height() / 2);
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static int lastPercent = -1;
        int percent = progress / (total / 100);
        
        if (percent != lastPercent) {
            lastPercent = percent;
            
            Serial.printf("Progress: %u%%\r", percent);
            
            int w = tft.width();
            int h = tft.height();
            
            // Draw progress text (only clearing text area dynamically to avoid flicker)
            tft.fillRect(20, h / 2 + 10, w - 40, 40, TFT_BLACK);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setTextSize(3);
            tft.setTextDatum(MC_DATUM);
            
            char buf[32];
            sprintf(buf, "Progress: %d%%", percent);
            tft.drawString(buf, w / 2, h / 2 + 30);

            // Dynamic progress bar wrapping clockwise around screen border (filling red on white)
            int w_idx = w - 1;
            int h_idx = h - 1;
            
            if (percent <= 25) {
                int x_end = map(percent, 0, 25, 0, w_idx);
                tft.drawLine(0, 0, x_end, 0, TFT_RED);
            } else {
                tft.drawLine(0, 0, w_idx, 0, TFT_RED); // Fully draw top line
                if (percent <= 50) {
                    int y_end = map(percent, 25, 50, 0, h_idx);
                    tft.drawLine(w_idx, 0, w_idx, y_end, TFT_RED);
                } else {
                    tft.drawLine(w_idx, 0, w_idx, h_idx, TFT_RED); // Fully draw right line
                    if (percent <= 75) {
                        int x_end = map(percent, 50, 75, w_idx, 0);
                        tft.drawLine(w_idx, h_idx, x_end, h_idx, TFT_RED);
                    } else {
                        tft.drawLine(w_idx, h_idx, 0, h_idx, TFT_RED); // Fully draw bottom line
                        int y_end = map(percent, 75, 100, h_idx, 0);
                        tft.drawLine(0, h_idx, 0, y_end, TFT_RED);
                    }
                }
            }
        }
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]\n", error);
        ESP.restart();
    });
    
    ArduinoOTA.begin();
    MDNS.begin(hostname);
    Serial.printf("OTA hostname set to: %s\n", hostname);
}

void init_wifi_and_ota() {
    scan_and_connect_wifi();
    setup_Arduino_OTA();
}

void handle_ota() {
    ArduinoOTA.handle();
}
```

### 10. Main Orchestrator: `src/main.cpp`
This file connects to all modules, sets up peripheral configurations, and drives the interactive bouncing red dot game loop using dynamic boundaries:

```cpp
#include <Arduino.h>
#include "cyd_display.h"
#include "cyd_touch.h"
#include "cyd_wifi.h"
#include "RGBledDriver.h"

// Dot variables
float dotX = 120;
float dotY = 160;
float dotDX = 0;
float dotDY = 0;
int dotSize = 10; // Red target dot radius

// State variables
bool showYes = false;
unsigned long yesStartTime = 0;

void resetDot() {
    dotX = tft.width() / 2;
    dotY = tft.height() / 2;

    // Pick a random direction angle
    float angle = random(0, 360) * PI / 180.0;
    float speed = 3.5; // Dot motion speed

    dotDX = cos(angle) * speed;
    dotDY = sin(angle) * speed;

    if (abs(dotDX) < 0.5) dotDX = (dotDX < 0) ? -0.5 : 0.5;
    if (abs(dotDY) < 0.5) dotDY = (dotDY < 0) ? -0.5 : 0.5;

    tft.invertDisplay(false);
    tft.fillScreen(TFT_BLACK);
}

void setup() {
    Serial.begin(115200);

    // Initialize Onboard LED
    initRGBled();
    ChangeRGBColor(RGB_COLOR_2); // Green status LED

    // Initialize Display & Touch Panels
    init_display();
    init_touch();

    // Connect to Network & Start OTA Listener
    init_wifi_and_ota();

    randomSeed(analogRead(0));
    resetDot();
}

void loop() {
    handle_ota(); // Continuously maintain wireless updates

    if (showYes) {
        if (millis() - yesStartTime > 5000) {
            showYes = false;
            resetDot(); // Resume game
        }
        return;
    }

    // 1. Clear previous dot position
    tft.fillCircle((int)dotX, (int)dotY, dotSize, TFT_BLACK);

    // 2. Advance ball position
    dotX += dotDX;
    dotY += dotDY;

    // 3. Dynamic Bounce off Screen Edges
    int screenWidth = tft.width();
    int screenHeight = tft.height();

    if (dotX - dotSize <= 0) {
        dotDX = abs(dotDX);
        dotX = dotSize;
    } else if (dotX + dotSize >= screenWidth) {
        dotDX = -abs(dotDX);
        dotX = screenWidth - dotSize;
    }

    if (dotY - dotSize <= 0) {
        dotDY = abs(dotDY);
        dotY = dotSize;
    } else if (dotY + dotSize >= screenHeight) {
        dotDY = -abs(dotDY);
        dotY = screenHeight - dotSize;
    }

    // 4. Draw updated red ball
    tft.fillCircle((int)dotX, (int)dotY, dotSize, TFT_RED);

    // 5. Detect and Process Touches
    int touchX, touchY;
    if (get_touch_point(touchX, touchY)) {
        float dist = sqrt(pow(touchX - dotX, 2) + pow(touchY - dotY, 2));

        if (dist < 40) { // Catch box size
            showYes = true;
            yesStartTime = millis();

            tft.invertDisplay(true); // Invert display colors
            tft.fillScreen(TFT_BLACK); // Appears white under inversion

            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setTextSize(4);
            tft.setTextDatum(MC_DATUM);
            tft.drawString("YESSSSS", screenWidth / 2, screenHeight / 2);
        }
    }

    delay(15); // Frame rate lock (approx 60 FPS)
}
```
