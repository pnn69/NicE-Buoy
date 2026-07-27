/*
    Leds on board updated
    Put new value in que and the leds will be updated.
*/
#include <Arduino.h>
#include <FastLED.h>
#include <WiFi.h>
#include "main.h"
#include "leds.h"
#include "io_sub.h"

#define NUM_LEDS 3
#define LEDBB 2
#define LEDSTATUS 1
#define LEDSB 0

CRGB ledData;
CRGB leds[NUM_LEDS];

QueueHandle_t ledPwr;    // speed bb,sb
QueueHandle_t ledStatus; // Accu status
static uint8_t blinkcnt = 0;
static unsigned long blinktimer = millis();
static LedData ledStatusData;
static PwrData ledPwrData;
static bool blink, blinkFast;

/**
 * @brief Initializes the FreeRTOS queues for LED state commands.
 * Creates separate queues for general status LEDs and ESC power/speed indicator LEDs.
 * 
 * @return true if queues are successfully created.
 */
bool initledqueue(void)
{
    ledStatus = xQueueCreate(1, sizeof(LedData));
    ledPwr = xQueueCreate(1, sizeof(PwrData));
    return true;
}

/**
 * @brief Configures the FastLED library and hardware pin for WS2812B LEDs.
 * Sets the initial colors to RGB (Red, Green, Blue) on startup.
 */
void initLed(void)
{
    FastLED.addLeds<WS2812B, LEDS_PIN, GRB>(leds, NUM_LEDS);
    FastLED.clear();
    leds[0] = CRGB::Red;
    leds[1] = CRGB::Green;
    leds[2] = CRGB::Blue;
    FastLED.setBrightness(LED_BRIGHTNESS);
    FastLED.show();
}

/**
 * @brief FreeRTOS task handling LED animations and state updates.
 * 1. Initializes the FastLED driver.
 * 2. Monitors `ledStatus` for system state colors (e.g., GPS lock, connection status).
 * 3. Monitors `ledPwr` for motor speed data to drive the LED thruster indicators.
 * 4. Manages global blink animations (Fast and Slow) for any LED that requires it.
 * 5. Calls FastLED.show() only when the state or animation step changes.
 * 
 * @param arg Unused FreeRTOS task argument.
 */
void LedTask(void *arg)
{
    initLed();
    Serial.println("Led task running!");
    unsigned long lastUpdate = millis();
    
    while (1)
    {
        bool changed = false;

        if (xQueueReceive(ledStatus, (void *)&ledStatusData, 0) == pdTRUE)
        {
            leds[LEDSTATUS] = ledStatusData.color;
            changed = true;
        }
        
        if (xQueueReceive(ledPwr, (void *)&ledPwrData, 0) == pdTRUE)
        {
            int bbVal = ledPwrData.ledBb;
            int sbVal = ledPwrData.ledSb;

            // BB Motor Indicator (LED 2)
            if (bbVal > 5) { ledPwrData.bb = CRGB::Green; }
            else if (bbVal < -5) { ledPwrData.bb = CRGB::Red; }
            else { ledPwrData.bb = CRGB::Black; }

            // SB Motor Indicator (LED 0)
            if (sbVal > 5) { ledPwrData.sb = CRGB::Green; }
            else if (sbVal < -5) { ledPwrData.sb = CRGB::Red; }
            else { ledPwrData.sb = CRGB::Black; }

            leds[LEDBB] = ledPwrData.bb;
            leds[LEDSB] = ledPwrData.sb;
            
            changed = true;
        }

        if (millis() - lastUpdate >= 50) // Update animations at 20Hz
        {
            lastUpdate = millis();
            blinkcnt++;
            blinkFast = !blinkFast;
            if (blinkcnt >= 10) {
                blinkcnt = 0;
                blink = !blink;
            }

            bool anyBlink = false;

            // --- Status LED ---
            if (ledStatusData.blink == BLINK_FAST) { leds[LEDSTATUS] = blinkFast ? ledStatusData.color : CRGB::Black; anyBlink = true; }
            else if (ledStatusData.blink == BLINK_SLOW) { leds[LEDSTATUS] = blink ? ledStatusData.color : CRGB::Black; anyBlink = true; }
            
            // --- BB Motor LED ---
            if (ledPwrData.blinkBb == BLINK_FAST) { leds[LEDBB] = blinkFast ? ledPwrData.bb : CRGB::Black; anyBlink = true; }
            else if (ledPwrData.blinkBb == BLINK_SLOW) { leds[LEDBB] = blink ? ledPwrData.bb : CRGB::Black; anyBlink = true; }

            // --- SB Motor LED ---
            if (ledPwrData.blinkSb == BLINK_FAST) { leds[LEDSB] = blinkFast ? ledPwrData.sb : CRGB::Black; anyBlink = true; }
            else if (ledPwrData.blinkSb == BLINK_SLOW) { leds[LEDSB] = blink ? ledPwrData.sb : CRGB::Black; anyBlink = true; }

            if (anyBlink) {
                changed = true;
            }
        }

        // If the wifi connection is active, override the 3rd LED (LEDBB) to Magenta
        bool wifiActive = (WiFi.status() == WL_CONNECTED || WiFi.softAPgetStationNum() > 0);
        static bool lastWifiActive = false;
        if (wifiActive != lastWifiActive) {
            lastWifiActive = wifiActive;
            changed = true;
        }

        if (wifiActive) {
            leds[LEDBB] = CRGB::Magenta;
        }

        if (changed)
        {
            FastLED.show();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}