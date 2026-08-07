#include <Arduino.h>
#include <XPT2046_Bitbang.h>
#include "cyd_touch.h"

// Touch Screen pins for CYD
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

// Instantiate the bitbang touch object on the dedicated pins (240x320 Portrait)
XPT2046_Bitbang ts(XPT2046_MOSI, XPT2046_MISO, XPT2046_CLK, XPT2046_CS, 240, 320);

void init_touch() {
    ts.begin();
    // Set baseline calibration bounds (raw X, raw Y)
    ts.setCalibration(300, 3800, 260, 3800);
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
            // Calibration ranges established earlier
            const uint16_t ts_minx = 300;
            const uint16_t ts_maxx = 3800;
            const uint16_t ts_miny = 260;
            const uint16_t ts_maxy = 3800;

            // In Portrait mode (USB top / rotation 0), the physical touch sensor's hardware axes 
            // are swapped 90 degrees relative to the display grid. 
            // Swapping xRaw/yRaw and inverting the X axis (yRaw mapped max-to-min) aligns touch perfectly!
            int touchX = map(p2.yRaw, ts_maxy, ts_miny, 1, 240);
            int touchY = map(p2.xRaw, ts_minx, ts_maxx, 1, 320);

            x = constrain(touchX, 1, 240);
            y = constrain(touchY, 1, 320);

            // Record this valid touch timestamp
            last_touch_ms = millis();
            return true;
        }
    }
    return false;
}
