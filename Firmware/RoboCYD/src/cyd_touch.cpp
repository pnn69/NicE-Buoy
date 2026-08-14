#include <Arduino.h>
#include <XPT2046_Bitbang.h>
#include <Preferences.h>
#include "cyd_touch.h"

// Touch Screen pins for CYD
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

// Instantiate the bitbang touch object on the dedicated pins (240x320 Portrait)
XPT2046_Bitbang ts(XPT2046_MOSI, XPT2046_MISO, XPT2046_CLK, XPT2046_CS, 240, 320);

// Global calibration bounds
uint16_t ts_minx = 300;
uint16_t ts_maxx = 3800;
uint16_t ts_miny = 260;
uint16_t ts_maxy = 3800;

void init_touch() {
    ts.begin();
    
    // One-time self-clearing reset of broken calibration bounds
    Preferences prefs;
    prefs.begin("touch-cal", false);
    if (!prefs.isKey("cal_reset_v3")) {
        prefs.clear();
        prefs.putBool("cal_reset_v3", true);
        Serial.println("One-time touch calibration reset triggered!");
    }
    prefs.end();
    
    // Load calibration bounds from non-volatile Preferences
    prefs.begin("touch-cal", true);
    ts_minx = prefs.getUShort("minx", 300);
    ts_maxx = prefs.getUShort("maxx", 3800);
    ts_miny = prefs.getUShort("miny", 260);
    ts_maxy = prefs.getUShort("maxy", 3800);
    prefs.end();
    
    ts.setCalibration(ts_minx, ts_maxx, ts_miny, ts_maxy);
}

bool get_touch_point(int &x, int &y) {
    static unsigned long last_touch_ms = 0;
    
    // Block touch screen for 200 milliseconds after any valid touch to prevent double-clicks!
    if (millis() - last_touch_ms < 200) {
        return false;
    }
    
    TouchPoint p = ts.getTouch();
    
    // Raise the pressure threshold to 800 (finger touches are typically 1500 to 4000)
    // This places the trigger point far above any transient analog noise floor.
    if (p.zRaw > 800) {
        // Debounce: Wait 5ms and take a second reading to ensure it's a real finger press
        // and not a single transient electromagnetic spike from the LoRa or WiFi radios.
        delay(5);
        TouchPoint p2 = ts.getTouch();
        
        if (p2.zRaw > 800) {
            // In Inverted Portrait mode (USB bottom / rotation 2), the physical touch sensor's hardware axes
            // are swapped 90 degrees relative to the display grid, and both axes are inverted relative to rotation 0.
            int touchX = map(p2.yRaw, ts_miny, ts_maxy, 1, 240);
            int touchY = map(p2.xRaw, ts_maxx, ts_minx, 1, 320);

            x = constrain(touchX, 1, 240);
            y = constrain(touchY, 1, 320);

            // Record this valid touch timestamp
            last_touch_ms = millis();
            return true;
        }
    }
    return false;
}

bool get_raw_touch_point(int &rawX, int &rawY) {
    static unsigned long last_raw_touch_ms = 0;
    if (millis() - last_raw_touch_ms < 300) {
        return false;
    }
    TouchPoint p = ts.getTouch();
    if (p.zRaw > 800) {
        delay(5);
        TouchPoint p2 = ts.getTouch();
        if (p2.zRaw > 800) {
            rawX = p2.xRaw;
            rawY = p2.yRaw;
            last_raw_touch_ms = millis();
            return true;
        }
    }
    return false;
}

void apply_calibration(uint16_t minx, uint16_t maxx, uint16_t miny, uint16_t maxy) {
    ts_minx = minx;
    ts_maxx = maxx;
    ts_miny = miny;
    ts_maxy = maxy;
    ts.setCalibration(ts_minx, ts_maxx, ts_miny, ts_maxy);
}
