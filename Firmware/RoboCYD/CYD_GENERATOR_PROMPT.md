# AI Generator Prompt: Resistive Cheap Yellow Display (CYD) ESP32-2432S028R Firmware

Copy and paste this prompt into any AI coding assistant to fully generate or rebuild this modular, over-the-air (OTA) enabled resistive Cheap Yellow Display (CYD) project with perfect hardware settings, software bit-bang touchscreen libraries, robust electromagnetic noise filtering, and a dual-state 3-buoy telemetry controller dashboard.

---

### System Role & Objective
You are an expert embedded software engineer specializing in ESP32 development, PlatformIO, and the Arduino framework. Your task is to generate a modular, highly organized, and robust C++ project for the **resistive Cheap Yellow Display (CYD)**, also known as the **ESP32-2432S028R**.

### Key Hardware Requirements & Wiring
1. **Target Board:** ESP32 Dev Module (WROOM-32).
2. **Display Controller:** ILI9341 (configured as `ILI9341_2_DRIVER` under TFT_eSPI) connected via HSPI.
3. **Display Backlight Control:** GPIO 21 (Active `HIGH`).
4. **Touchscreen Controller:** Resistive SPI-based XPT2046 connected via a software-emulated (bit-banged) SPI bus on pins:
   - `CLK: 25`, `MISO: 39`, `MOSI: 32`, `CS: 33`, `IRQ: 36`.
5. **SX1278 Ra-02 433MHz LoRa Module:** Wired using the microSD card's hardware `VSPI` bus pins:
   - `SCK: 18`, `MISO: 19`, `MOSI: 23`, `CS: 5` (NSS), `RST: 27`, `DIO0: 35`.
6. **Onboard RGB LED Pins:** Red: GPIO 4, Green: GPIO 16, Blue: GPIO 17 (Active `LOW`).

### Architecture & Design Rules
- **Modular Design:** Do not write a single monolithic file. Separate responsibilities into dedicated modules: display control, touch sensing, networking, LED drivers, and the main controller.
- **Dedicated SPI Bus Separation:** To avoid SPI hardware conflicts, the LoRa module runs on the hardware `VSPI` bus, while the slow touchscreen runs on a dedicated Software SPI (bit-bang) bus using the `XPT2046_Bitbang_Slim` library.
- **90-Degree Hardware Axis Calibration (USB Top):** When the screen is set to Portrait orientation (USB port at top, Rotation 0), the physical touchscreen's coordinate axes are swapped 90 degrees relative to the display grid. Map `touchX` dynamically from raw **`p2.yRaw`** (scaled inversely: `ts_maxy` down to `ts_miny`) and `touchY` dynamically from raw **`p2.xRaw`** (scaled standardly: `ts_minx` to `ts_maxx`) to align touch coordinates with LCD buttons perfectly.
- **Electromagnetic Noise Filtering (Anti-Ghost Touch):** High-power LoRa (433MHz) and WiFi (2.4GHz) radio waves can couple into the resistive screen traces and trigger false ghost touches. To prevent this:
  - Raise the pressure threshold to **`800`** (finger presses are typically `1500` to `4000` points).
  - Implement a **double-read software debounce**: pause for `5ms` after the first trigger and confirm the pressure is still above `800` before registering a valid touch.
- **Dynamic Orientation-Independent Drawing:** Never hardcode display sizes (e.g., 320 or 240) in your graphics or coordinates. Always query `tft.width()` and `tft.height()` dynamically so that switching orientation (e.g., Rotation 0, 1, 2, or 3) automatically adjusts layout.
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
        nitek/XPT2046_Bitbang_Slim@^2.0.0
        lvgl/lvgl@^8.3.6
        sandeepmistry/LoRa@^0.8.0

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
    tft.setRotation(0); // Choose 0 for portrait (USB top), 1 for landscape, 2 for portrait inverted (USB bottom), 3 for landscape inverted
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
#include <XPT2046_Bitbang.h>
#include "cyd_touch.h"

#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

// Instantiate the bitbang touch object on the dedicated software pins (240x320 Portrait)
XPT2046_Bitbang ts(XPT2046_MOSI, XPT2046_MISO, XPT2046_CLK, XPT2046_CS, 240, 320);

void init_touch() {
    ts.begin();
    // Set baseline calibration bounds (raw X, raw Y)
    ts.setCalibration(300, 3800, 260, 3800);
}

bool get_touch_point(int &x, int &y) {
    TouchPoint p = ts.getTouch();
    
    // Raise the pressure threshold to 800 (finger touches are typically 1500 to 4000)
    // to place the trigger point far above any transient electromagnetic noise.
    if (p.zRaw > 800) {
        // Debounce: Wait 5ms and take a second reading to ensure it's a real finger press
        // and not a transient analog noise spike from high-power LoRa/WiFi transmissions.
        delay(5);
        TouchPoint p2 = ts.getTouch();
        
        if (p2.zRaw > 800) {
            // Calibration ranges established earlier
            const uint16_t ts_minx = 300;
            const uint16_t ts_maxx = 3800;
            const uint16_t ts_miny = 260;
            const uint16_t ts_maxy = 3800;

            // In standard Portrait mode (USB top / rotation 0):
            // Swap xRaw and yRaw, and invert the X axis (yRaw scaled max-to-min) to align touch axes perfectly!
            int touchX = map(p2.yRaw, ts_maxy, ts_miny, 1, 240);
            int touchY = map(p2.xRaw, ts_minx, ts_maxx, 1, 320);

            x = constrain(touchX, 1, 240); // Keep within tft.width() limit
            y = constrain(touchY, 1, 320); // Keep within tft.height() limit

            return true;
        }
    }
    return false;
}
```

### 8. WiFi & OTA Module: `src/cyd_wifi.h`
```cpp
#ifndef CYD_WIFI_H
#define CYD_WIFI_H

#include <Arduino.h>

void init_wifi_and_ota();
void handle_ota();
void udp_broadcast(const String &message);

#endif // CYD_WIFI_H
```

### 9. WiFi & OTA Module: `src/cyd_wifi.cpp`
```cpp
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <AsyncUDP.h>
#include "cyd_wifi.h"
#include "cyd_display.h"
#include "buoy_data.h"

AsyncUDP udp;

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

    // PRIORITIZE "NicE_WiFi" to join same subnet as RoboLora module (192.168.1.x)
    if (nice_wifi_found) {
        Serial.println("Found 'NicE_WiFi'. Connecting...");
        tft.drawString("Connecting to NicE_WiFi...", 20, 50);
        WiFi.begin("NicE_WiFi", "!Ni1001100110");
    } else if (robobuoy_found) {
        Serial.println("Found 'ROBOBUOY'. Connecting...");
        tft.drawString("Connecting to ROBOBUOY...", 20, 50);
        WiFi.begin("ROBOBUOY", "");
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
    
    // Set up AsyncUDP listener on port 1001 for buoy broadcasts
    if (udp.listen(1001)) {
        Serial.println("Listening on UDP port 1001 for Buoy telemetry...");
        udp.onPacket([](AsyncUDPPacket packet) {
            String stringUdpIn = String((const char *)packet.data(), packet.length());
            parse_buoy_packet(stringUdpIn, "UDP");
        });
    } else {
        Serial.println("Failed to bind UDP port 1001!");
    }
}

void handle_ota() {
    ArduinoOTA.handle();
}

void udp_broadcast(const String &message) {
    // Broadcast packets on UDP port 1001
    udp.broadcast(message.c_str());
}
```

### 10. LoRa Subsystem Module: `src/cyd_lora.h`
```cpp
#ifndef CYD_LORA_H
#define CYD_LORA_H

#include <Arduino.h>

void init_lora();
void check_lora_packets();
void send_lora_packet(const String &message);

#endif // CYD_LORA_H
```

### 11. LoRa Subsystem Module: `src/cyd_lora.cpp`
```cpp
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "cyd_lora.h"
#include "cyd_display.h"
#include "buoy_data.h"

#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_CS 5
#define LORA_RST 27
#define LORA_DIO0 35

#define LORA_BAND 433E6

SPIClass loraSpi(VSPI);

void init_lora() {
    Serial.println("Initializing LoRa on SD-Card SPI Pins...");
    
    loraSpi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
    LoRa.setSPI(loraSpi);
    LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(LORA_BAND)) {
        Serial.println("Starting LoRa failed!");
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.setTextSize(1);
        tft.setTextDatum(BC_DATUM);
        tft.drawString("LoRa Init Failed!", tft.width() / 2, tft.height() - 10);
    } else {
        Serial.println("LoRa Init Success! Enabling Hardware CRC...");
        LoRa.enableCrc(); // Enable receiver hardware CRC checking to match RoboLora
        
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextSize(1);
        tft.setTextDatum(BC_DATUM);
        tft.drawString("LoRa Active (433MHz)", tft.width() / 2, tft.height() - 10);
    }
}

void check_lora_packets() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        String message = "";
        while (LoRa.available()) {
            message += (char)LoRa.read();
        }
        Serial.print("Received LoRa packet: ");
        Serial.println(message);
        
        // Parse incoming packet and update our buoy data structure
        parse_buoy_packet(message, "LoRa");
    }
}

void send_lora_packet(const String &message) {
    Serial.print("Sending LoRa packet: ");
    Serial.println(message);
    
    LoRa.beginPacket();
    LoRa.print(message);
    LoRa.endPacket();
}
```

### 12. Buoy Data Manager: `src/buoy_data.h`
```cpp
#ifndef BUOY_DATA_H
#define BUOY_DATA_H

#include <Arduino.h>

struct BuoyData {
    String id = "";
    bool present = false;
    unsigned long last_seen_ms = 0;
    
    // Telemetry fields
    String status = "UNKNOWN";
    float mag_dir = 0;
    float gps_dir = 0;
    float tg_dir = 0;
    float tg_dist = 0;
    float wind_dir = 0;
    float wind_std = 0;
    float bb_power = 0;
    float sb_power = 0;
    float pid_i = 0;
    float pid_r = 0;
    float battery_v = 0;
    float battery_pct = 0;
    String lat = "N/A";
    String lon = "N/A";
    String gps_fix = "N/A";
    int gps_sat = 0;
    float current = 0;
};

extern BuoyData buoys[3];
extern int selected_buoy_idx;

void parse_buoy_packet(const String &packetStr, const String &source);
void send_buoy_command(const String &buoy_id, int cmd_code);

#endif // BUOY_DATA_H
```

### 13. Buoy Data Manager: `src/buoy_data.cpp`
```cpp
#include <Arduino.h>
#include <vector>
#include "buoy_data.h"
#include "cyd_wifi.h"
#include "cyd_lora.h"

BuoyData buoys[3];
int selected_buoy_idx = -1; // -1 for main menu, 0-2 for details

uint8_t calculate_crc(const String &content) {
    uint8_t crc = 0;
    for (int i = 0; i < content.length(); i++) {
        crc ^= content[i];
    }
    return crc;
}

void parse_buoy_packet(const String &packetStr, const String &source) {
    // Robust parsing: Find the dollar sign wherever it starts (handles "D$", "UDP$", etc. dynamically!)
    int dollarIdx = packetStr.indexOf("$");
    int starIdx = packetStr.indexOf("*");
    if (dollarIdx == -1 || starIdx == -1 || starIdx <= dollarIdx) return;
    
    String cleanPacket = packetStr.substring(dollarIdx);
    starIdx = cleanPacket.indexOf("*"); // Recalculate relative to the clean start
    
    String content = cleanPacket.substring(1, starIdx);
    String crcHex = cleanPacket.substring(starIdx + 1);
    crcHex.trim();
    
    uint8_t calculated_crc = calculate_crc(content);
    uint8_t received_crc = strtol(crcHex.c_str(), NULL, 16);
    if (calculated_crc != received_crc) {
        Serial.printf("CRC error: calculated %02X, received %02X\n", calculated_crc, received_crc);
        return;
    }
    
    // Split content by comma
    std::vector<String> fields;
    int prev_idx = 0;
    int next_idx = content.indexOf(",");
    while (next_idx != -1) {
        fields.push_back(content.substring(prev_idx, next_idx));
        prev_idx = next_idx + 1;
        next_idx = content.indexOf(",", prev_idx);
    }
    fields.push_back(content.substring(prev_idx));
    
    if (fields.size() < 5) return;
    
    String sender_id = fields[1];
    sender_id.trim();
    
    // Ignore commands sent from ourselves (Remote ID 99)
    if (sender_id == "99") return;
    
    // Search for an existing registered buoy
    int buoy_idx = -1;
    for (int i = 0; i < 3; i++) {
        if (buoys[i].id == sender_id) {
            buoy_idx = i;
            break;
        }
    }
    
    // If not found, assign to the first empty slot
    if (buoy_idx == -1) {
        for (int i = 0; i < 3; i++) {
            if (buoys[i].id == "") {
                buoys[i].id = sender_id;
                buoy_idx = i;
                break;
            }
        }
    }
    
    if (buoy_idx == -1) return; // No slot available
    
    // Update timestamps and online presence
    buoys[buoy_idx].present = true;
    buoys[buoy_idx].last_seen_ms = millis();
    
    int cmd = atoi(fields[3].c_str());
    int status_code = atoi(fields[4].c_str());
    
    // Status text mapping
    if (status_code == 7) buoys[buoy_idx].status = "IDLE";
    else if (status_code == 12) buoys[buoy_idx].status = "LOCKING";
    else if (status_code == 13) buoys[buoy_idx].status = "LOCKED";
    else if (status_code == 15) buoys[buoy_idx].status = "DOCKING";
    else if (status_code == 16) buoys[buoy_idx].status = "DOCKED";
    else if (status_code == 25) buoys[buoy_idx].status = "REMOTE";
    else buoys[buoy_idx].status = "MODE " + String(status_code);
    
    // Parse TOPDATA (CMD = 51)
    if (cmd == 51 && fields.size() >= 21) {
        buoys[buoy_idx].mag_dir = atof(fields[5].c_str());
        buoys[buoy_idx].gps_dir = atof(fields[6].c_str());
        buoys[buoy_idx].tg_dir = atof(fields[7].c_str());
        buoys[buoy_idx].tg_dist = atof(fields[8].c_str());
        buoys[buoy_idx].wind_dir = atof(fields[9].c_str());
        buoys[buoy_idx].wind_std = atof(fields[10].c_str());
        buoys[buoy_idx].bb_power = atof(fields[11].c_str());
        buoys[buoy_idx].sb_power = atof(fields[12].c_str());
        buoys[buoy_idx].pid_i = atof(fields[13].c_str());
        buoys[buoy_idx].pid_r = atof(fields[14].c_str());
        buoys[buoy_idx].battery_v = atof(fields[15].c_str());
        buoys[buoy_idx].battery_pct = atof(fields[16].c_str());
        buoys[buoy_idx].lat = fields[17];
        buoys[buoy_idx].lon = fields[18];
        buoys[buoy_idx].gps_fix = fields[19] == "1" ? "3D" : fields[19] == "2" ? "2D" : "NoFix";
        buoys[buoy_idx].gps_sat = atoi(fields[20].c_str());
        if (fields.size() > 21) {
            buoys[buoy_idx].current = atof(fields[21].c_str());
        }
    }
    // Parse BUOYPOS (CMD = 19)
    else if (cmd == 19 && fields.size() >= 14) {
        buoys[buoy_idx].lat = fields[5];
        buoys[buoy_idx].lon = fields[6];
        buoys[buoy_idx].mag_dir = atof(fields[7].c_str());
        buoys[buoy_idx].wind_dir = atof(fields[8].c_str());
        buoys[buoy_idx].wind_std = atof(fields[9].c_str());
        
        // fields[10] is topAccuP
        buoys[buoy_idx].battery_pct = atof(fields[11].c_str()); // subAccuP
        
        // Estimate voltage if battery_v is currently zero
        if (buoys[buoy_idx].battery_v == 0) {
            buoys[buoy_idx].battery_v = 17.0 + (8.2 * buoys[buoy_idx].battery_pct / 100.0);
        }
        
        buoys[buoy_idx].gps_fix = fields[12] == "1" ? "3D" : fields[12] == "2" ? "2D" : "NoFix";
        buoys[buoy_idx].gps_sat = atoi(fields[13].c_str());
    }
}

void send_buoy_command(const String &buoy_id, int cmd_code) {
    // Standard command formatting: $Target,Sender,ACK,CMD,Status,Data1,Data2...*CRC
    // To change mode: $buoy_id,99,3,cmd_code,cmd_code,,,,,,*
    String cmdStr = buoy_id + ",99,3," + String(cmd_code) + "," + String(cmd_code) + ",,,,,,";
    
    uint8_t crc = calculate_crc(cmdStr);
    char crc_buf[8];
    sprintf(crc_buf, "*%02X", crc);
    
    String finalPacket = "$" + cmdStr + String(crc_buf);
    
    Serial.printf("Broadcasting Command: %s\n", finalPacket.c_str());
    
    // Send over both LoRa and UDP!
    send_lora_packet(finalPacket);
    udp_broadcast(finalPacket);
}
```

### 14. Main Orchestrator: `src/main.cpp`
This file connects to all modules, sets up peripheral configurations, and drives the dual-state (3-buoy list and detailed telemetry sub-page) Portrait dashboard layout with dynamic redraw optimization and full quadrant button hitboxes:

```cpp
#include <Arduino.h>
#include <WiFi.h>
#include "cyd_display.h"
#include "cyd_touch.h"
#include "cyd_wifi.h"
#include "cyd_lora.h"
#include "buoy_data.h"
#include "RGBledDriver.h"

unsigned long lastUIUpdate = 0;
int lastKnownState = -2; // Used to trigger a static redraw on state change

// State caches to avoid redrawing menu buttons unnecessarily (Portrait size)
String last_drawn_ids[3] = {"", "", ""};
int last_drawn_present[3] = {-2, -2, -2}; // -2 uninitialized, -1 empty, 0 offline, 1 online

void reset_button_draw_cache() {
    for (int i = 0; i < 3; i++) {
        last_drawn_ids[i] = "RESET"; // Force mismatch
        last_drawn_present[i] = -2;   // Force mismatch
    }
}

void draw_resting_ui() {
    int w = tft.width();
    int h = tft.height();
    
    tft.fillScreen(TFT_BLACK);
    
    if (selected_buoy_idx == -1) {
        // --- Static Menu Screen (240x320 Portrait) ---
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setTextSize(2);
        tft.setTextDatum(TC_DATUM);
        tft.drawString("ROBOBUOY", w / 2, 15);
        tft.drawString("CONTROLLER", w / 2, 40);
        
        tft.drawFastHLine(15, 70, w - 30, TFT_WHITE);
        
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.setTextSize(1);
        tft.setTextDatum(BC_DATUM);
        IPAddress ip = WiFi.localIP();
        char ip_buf[48];
        sprintf(ip_buf, "IP: %s | LoRa: 433M", ip.toString().c_str());
        tft.drawString(ip_buf, w / 2, h - 10);
    } else {
        // --- Static Details Screen (240x320 Portrait) ---
        int idx = selected_buoy_idx;
        tft.setTextColor(TFT_CYAN, TFT_BLACK);
        tft.setTextSize(2);
        tft.setTextDatum(TC_DATUM);
        tft.drawString("BUOY: " + buoys[idx].id, w / 2, 10);
        
        tft.drawFastHLine(15, 35, w - 30, TFT_WHITE);
        
        // Draw static text headers (left column)
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);
        tft.drawString("Status:", 15, 45);
        tft.drawString("Bat:", 15, 65);
        tft.drawString("Cur:", 15, 85);
        tft.drawString("GPS:", 15, 105);
        tft.drawString("Lat:", 15, 125);
        tft.drawString("Lon:", 15, 145);
        
        // Draw static text headers (right column)
        tft.drawString("Mag:", 130, 45);
        tft.drawString("TgD:", 130, 65);
        tft.drawString("TgS:", 130, 85);
        tft.drawString("Wnd:", 130, 105);
        tft.drawString("Std:", 130, 125);
        
        tft.drawFastHLine(15, 170, w - 30, TFT_WHITE);
        
        tft.drawString("BB (Bow):", 15, 180);
        tft.drawString("SB (Stern):", 15, 210);
        
        tft.drawFastHLine(15, 245, w - 30, TFT_WHITE);
        
        // Back Button
        tft.fillRoundRect(10, 260, 105, 40, 5, TFT_BLUE);
        tft.setTextColor(TFT_WHITE, TFT_BLUE);
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("BACK", 62, 280);
        
        // Lock Button (color/text determined dynamically during loop)
    }
}

void update_dynamic_ui() {
    int w = tft.width();
    int h = tft.height();
    unsigned long now = millis();
    
    // Check if buoys went offline (> 10 seconds since last transmission)
    for (int i = 0; i < 3; i++) {
        if (buoys[i].id != "") {
            if (now - buoys[i].last_seen_ms > 10000) { // 10 seconds timeout
                buoys[i].present = false;
            }
        }
    }
    
    if (selected_buoy_idx == -1) {
        // --- Menu Screen (Optimized with State Caching to prevent redraw flicker) ---
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        
        for (int i = 0; i < 3; i++) {
            int y = 80 + i * 55;
            int current_present = (buoys[i].id == "") ? -1 : (buoys[i].present ? 1 : 0);
            
            // Only draw/redraw if the ID or the online presence changed!
            if (buoys[i].id != last_drawn_ids[i] || current_present != last_drawn_present[i]) {
                last_drawn_ids[i] = buoys[i].id;
                last_drawn_present[i] = current_present;
                
                // Clear only this button's area first
                tft.fillRect(10, y, w - 20, 45, TFT_BLACK);
                
                if (current_present == -1) {
                    // Empty slot
                    tft.drawRoundRect(10, y, w - 20, 45, 5, TFT_DARKGREY);
                    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
                    tft.drawString("Buoy " + String(i+1) + ": [Waiting]", w / 2, y + 22);
                } else if (current_present == 1) {
                    // Present: Green button
                    tft.fillRoundRect(10, y, w - 20, 45, 5, TFT_GREEN);
                    tft.setTextColor(TFT_BLACK, TFT_GREEN);
                    tft.drawString("Buoy " + String(i+1) + ": " + buoys[i].id, w / 2, y + 22);
                } else {
                    // Offline: Red button
                    tft.fillRoundRect(10, y, w - 20, 45, 5, TFT_RED);
                    tft.setTextColor(TFT_WHITE, TFT_RED);
                    tft.drawString("Buoy " + String(i+1) + ": [Offline]", w / 2, y + 22);
                }
            }
        }
    } else {
        // --- Details Screen (240x320 Portrait) ---
        int idx = selected_buoy_idx;
        BuoyData &b = buoys[idx];
        
        tft.setTextSize(1);
        tft.setTextDatum(TL_DATUM);
        
        // Status color code
        uint16_t statusColor = TFT_WHITE;
        if (b.status == "LOCKED") statusColor = TFT_GREEN;
        else if (b.status == "LOCKING" || b.status == "DOCKING") statusColor = TFT_YELLOW;
        else if (b.status == "DOCKED") statusColor = TFT_CYAN;
        else if (b.status == "IDLE") statusColor = TFT_LIGHTGREY;
        
        // Print values with standard background-clearing text drawing (no flicker!)
        tft.setTextColor(statusColor, TFT_BLACK);
        tft.drawString(b.status + "      ", 65, 45);
        
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        char buf[64];
        sprintf(buf, "%0.1fV (%0.0f%%)   ", b.battery_v, b.battery_pct);
        tft.drawString(buf, 65, 65);
        
        sprintf(buf, "%0.1fA    ", b.current);
        tft.drawString(buf, 65, 85);
        
        sprintf(buf, "%s (%d sat)   ", b.gps_fix.c_str(), b.gps_sat);
        tft.drawString(buf, 65, 105);
        
        tft.drawString(b.lat + "          ", 65, 125);
        tft.drawString(b.lon + "          ", 65, 145);
        
        // Right column values
        sprintf(buf, "%0.0f deg    ", b.mag_dir);
        tft.drawString(buf, 165, 45);
        
        sprintf(buf, "%0.0f deg    ", b.tg_dir);
        tft.drawString(buf, 165, 65);
        
        sprintf(buf, "%0.1fm      ", b.tg_dist);
        tft.drawString(buf, 165, 85);
        
        sprintf(buf, "%0.0f deg    ", b.wind_dir);
        tft.drawString(buf, 165, 105);
        
        sprintf(buf, "%0.0f deg    ", b.wind_std);
        tft.drawString(buf, 165, 125);
        
        // Thrusters power values
        tft.setTextSize(2);
        
        // BB Bow
        if (b.bb_power == 0) tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        else tft.setTextColor(b.bb_power > 0 ? TFT_GREEN : TFT_RED, TFT_BLACK);
        sprintf(buf, "%0.0f%%       ", b.bb_power);
        tft.drawString(buf, 130, 180);
        
        // SB Stern
        if (b.sb_power == 0) tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        else tft.setTextColor(b.sb_power > 0 ? TFT_GREEN : TFT_RED, TFT_BLACK);
        sprintf(buf, "%0.0f%%       ", b.sb_power);
        tft.drawString(buf, 130, 210);
        
        // Action Button: LOCK if idle, IDLE if locking/locked/docking/docked
        uint16_t btnColor = TFT_DARKGREEN;
        String btnText = "LOCK";
        if (b.status == "LOCKED" || b.status == "LOCKING" || b.status == "DOCKED" || b.status == "DOCKING") {
            btnColor = TFT_MAROON;
            btnText = "IDLE";
        }
        
        tft.fillRoundRect(125, 260, 105, 40, 5, btnColor);
        tft.setTextColor(TFT_WHITE, btnColor);
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        tft.drawString(btnText, 177, 280);
    }
}

void setup() {
    Serial.begin(115200);

    // Setup RGB LED
    initRGBled();
    ChangeRGBColor(RGB_COLOR_2); // Start with a Green LED

    // Initialize display and touch controllers
    init_display();
    init_touch();

    // Setup WiFi, OTA & UDP listener
    init_wifi_and_ota();

    // Initialize LoRa radio using MicroSD card SPI pins
    init_lora();

    randomSeed(analogRead(0));

    // Initially draw the resting menu dashboard
    draw_resting_ui();
}

void loop() {
    handle_ota();

    // Process incoming LoRa telemetry packets
    check_lora_packets();

    // Trigger dynamic interface redraws on screen state transitions
    if (selected_buoy_idx != lastKnownState) {
        lastKnownState = selected_buoy_idx;
        reset_button_draw_cache(); // Clear button draw cache to force complete redraw
        draw_resting_ui();
    }

    // Process and sense touchscreen button touches (MAPPED for 240x320 Portrait!)
    int touchX, touchY;
    if (get_touch_point(touchX, touchY)) {
        Serial.printf("Touch at: X=%d, Y=%d\n", touchX, touchY);
        
        if (selected_buoy_idx == -1) {
            // --- MAIN MENU INTERACTION (240x320 Portrait) ---
            // Button 1: Y 80 to 125, X 10 to 230
            if (touchY >= 80 && touchY <= 125 && touchX >= 10 && touchX <= 230) {
                if (buoys[0].id != "" && buoys[0].present) {
                    selected_buoy_idx = 0;
                    ChangeRGBColor(RGB_COLOR_3); // Shift LED to blue indicating view
                }
            }
            // Button 2: Y 135 to 180, X 10 to 230
            else if (touchY >= 135 && touchY <= 180 && touchX >= 10 && touchX <= 230) {
                if (buoys[1].id != "" && buoys[1].present) {
                    selected_buoy_idx = 1;
                    ChangeRGBColor(RGB_COLOR_3);
                }
            }
            // Button 3: Y 190 to 235, X 10 to 230
            else if (touchY >= 190 && touchY <= 235 && touchX >= 10 && touchX <= 230) {
                if (buoys[2].id != "" && buoys[2].present) {
                    selected_buoy_idx = 2;
                    ChangeRGBColor(RGB_COLOR_3);
                }
            }
        } else {
            // --- DETAILS PAGE INTERACTION (Optimized with massive hitboxes for edge taps) ---
            // Back Button: Entire bottom-left quadrant (X: 0 to 120, Y >= 240)
            if (touchY >= 240 && touchX >= 0 && touchX <= 120) {
                selected_buoy_idx = -1;
                ChangeRGBColor(RGB_COLOR_2); // Back to green status LED
            }
            // Lock/Action Button: Entire bottom-right quadrant (X: 121 to 240, Y >= 240)
            else if (touchY >= 240 && touchX >= 121 && touchX <= 240) {
                String currentStatus = buoys[selected_buoy_idx].status;
                
                tft.fillRect(20, 100, 200, 100, TFT_BLACK);
                tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                tft.setTextSize(2);
                tft.setTextDatum(MC_DATUM);
                
                if (currentStatus == "LOCKED" || currentStatus == "LOCKING" || currentStatus == "DOCKED" || currentStatus == "DOCKING") {
                    tft.drawString("SENDING IDLE...", tft.width() / 2, tft.height() / 2);
                    send_buoy_command(buoys[selected_buoy_idx].id, 8); // Send IDLING (8)
                } else {
                    tft.drawString("SENDING LOCK...", tft.width() / 2, tft.height() / 2);
                    send_buoy_command(buoys[selected_buoy_idx].id, 12); // Send LOCKING (12)
                }
                
                delay(600); // Hold message briefly for feedback
                reset_button_draw_cache(); // Force complete refresh
                draw_resting_ui(); // Fully redraw screen state
            }
        }
    }

    // Refresh dynamic screen details every 250 milliseconds
    if (millis() - lastUIUpdate > 250) {
        lastUIUpdate = millis();
        update_dynamic_ui();
    }

    delay(20);
}
```
