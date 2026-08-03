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
int dotSize = 10; // A 10px radius dot is easier to catch than a tiny one

// State variables
bool showYes = false;
unsigned long yesStartTime = 0;

void resetDot() {
    dotX = tft.width() / 2;
    dotY = tft.height() / 2;

    // Pick a random direction angle
    float angle = random(0, 360) * PI / 180.0;
    float speed = 3.5; // Adjust speed of the dot here

    dotDX = cos(angle) * speed;
    dotDY = sin(angle) * speed;

    // Ensure we don't get a perfectly horizontal or vertical bounce
    // to keep the game interesting
    if (abs(dotDX) < 0.5) dotDX = (dotDX < 0) ? -0.5 : 0.5;
    if (abs(dotDY) < 0.5) dotDY = (dotDY < 0) ? -0.5 : 0.5;

    // Reset display colors
    tft.invertDisplay(false);
    tft.fillScreen(TFT_BLACK);
}

void setup() {
    Serial.begin(115200);

    // Setup RGB LED
    initRGBled();
    ChangeRGBColor(RGB_COLOR_2); // Start with a Green LED

    // Initialize display and touch controllers
    init_display();
    init_touch();

    // Setup WiFi & OTA with on-screen logging
    init_wifi_and_ota();

    // Seed random generator using unconnected analog pin noise
    randomSeed(analogRead(0));

    resetDot();
}

void loop() {
    handle_ota();

    // If we caught the dot, show YESSSSS and wait 5 seconds
    if (showYes) {
        if (millis() - yesStartTime > 5000) {
            showYes = false;
            resetDot(); // Start over!
        }
        return;
    }

    // 1. Erase old dot
    tft.fillCircle((int)dotX, (int)dotY, dotSize, TFT_BLACK);

    // 2. Update position
    dotX += dotDX;
    dotY += dotDY;

    // 3. Bounce off edges (completely dynamic based on current screen dimensions)
    int screenWidth = tft.width();
    int screenHeight = tft.height();

    if (dotX - dotSize <= 0) {
        dotDX = abs(dotDX); // Force move right
        dotX = dotSize;
    } else if (dotX + dotSize >= screenWidth) {
        dotDX = -abs(dotDX); // Force move left
        dotX = screenWidth - dotSize;
    }

    if (dotY - dotSize <= 0) {
        dotDY = abs(dotDY); // Force move down
        dotY = dotSize;
    } else if (dotY + dotSize >= screenHeight) {
        dotDY = -abs(dotDY); // Force move up
        dotY = screenHeight - dotSize;
    }

    // 4. Draw new dot
    tft.fillCircle((int)dotX, (int)dotY, dotSize, TFT_RED);

    // 5. Check if the screen is touched
    int touchX, touchY;
    if (get_touch_point(touchX, touchY)) {
        // Distance formula to check if the touch is near the dot's center
        float dist = sqrt(pow(touchX - dotX, 2) + pow(touchY - dotY, 2));

        // If touched within 40 pixels of the dot center (generous hit box)
        if (dist < 40) {
            showYes = true;
            yesStartTime = millis();

            // Invert screen colors
            tft.invertDisplay(true);
            tft.fillScreen(TFT_BLACK); // Will actually appear White because of hardware inversion

            // Draw text dynamically centered
            tft.setTextColor(TFT_WHITE, TFT_BLACK); // Will appear Black on White
            tft.setTextSize(4);
            tft.setTextDatum(MC_DATUM); // Middle Center alignment
            tft.drawString("YESSSSS", screenWidth / 2, screenHeight / 2);
        }
    }

    // Small delay to control frame rate (approx 60 FPS)
    delay(15);
}
