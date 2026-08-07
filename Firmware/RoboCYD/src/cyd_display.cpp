#include <Arduino.h>
#include "cyd_display.h"

TFT_eSPI tft = TFT_eSPI();

void init_display() {
    // Note: Backlight control for resistive board is on GPIO 21.
    // TFT_eSPI uses TFT_BL build flag from platformio.ini but we ensure it is on.
    #ifdef TFT_BL
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);
    #else
    pinMode(21, OUTPUT);
    digitalWrite(21, HIGH);
    #endif

    tft.begin();
    tft.setRotation(2); // Portrait orientation inverted (USB on bottom)
}
