#include <Arduino.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include "cyd_touch.h"

// Touch Screen pins for CYD
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
    ts.setRotation(0); // Align with display portrait mode (USB on top)
}

bool get_touch_point(int &x, int &y) {
    if (ts.tirqTouched() && ts.touched()) {
        TS_Point p = ts.getPoint();

        // Calibration ranges established earlier
        const uint16_t ts_minx = 300;
        const uint16_t ts_maxx = 3800;
        const uint16_t ts_miny = 260;
        const uint16_t ts_maxy = 3800;

        // In rotation 0 (portrait, USB at top), raw coordinate axes are mapped normally
        int touchX = map(p.x, ts_minx, ts_maxx, 1, 240);
        int touchY = map(p.y, ts_miny, ts_maxy, 1, 320);

        x = constrain(touchX, 1, 240);
        y = constrain(touchY, 1, 320);

        return true;
    }
    return false;
}
