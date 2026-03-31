/*
    Leds on board updated
    Put new value in que and the leds will be updated.
*/
#include <Arduino.h>
#include <FastLED.h>
#include "main.h"
#include "leds.h"
#include "io_top.h"

#define NUM_LEDS 3 + 20
#define LEDSTATUS 0
#define LEDUTIL 1
#define LEDGPS 2

CRGB ledData;
CRGB leds[NUM_LEDS];

QueueHandle_t ledStatus; // speed status bb,sb
QueueHandle_t ledUtil;   // Accu status
QueueHandle_t ledGps;    // Accu status
QueueHandle_t ledPwr;    // power bar

static uint8_t blinkcnt = 0;
static unsigned long blinktimer = millis();
static LedData ledDataStatus;
static LedData ledDataUtil;
static LedData ledDataGps;
static PwrData ledPwrData;
static bool blink, blinkFast;

/**
 * @brief Initializes the FreeRTOS queues for LED commands.
 * 
 * @return true if queues were created successfully.
 */
bool initledqueue(void)
{
    // ledStatus = xQueueCreate(10, sizeof(CRGB));
    ledStatus = xQueueCreate(10, sizeof(LedData));
    ledUtil = xQueueCreate(10, sizeof(LedData));
    ledGps = xQueueCreate(10, sizeof(LedData));
    ledPwr = xQueueCreate(10, sizeof(PwrData));
    Serial.println("Led queue created!");
    return true;
}

/**
 * @brief Initializes the FastLED library and clears all LEDs.
 */
void initLed(void)
{
    FastLED.addLeds<WS2812B, LEDS_PIN, GRB>(leds, NUM_LEDS);
    FastLED.clear();
    leds[LEDSTATUS] = CRGB::Black;
    leds[LEDUTIL] = CRGB::Black;
    leds[LEDGPS] = CRGB::Black;
    FastLED.setBrightness(100);
    FastLED.show();
}

/**
 * @brief FreeRTOS task responsible for handling LED animations and updates.
 * 
 * Continuously polls the LED queues (status, utility, GPS, power) and 
 * drives the FastLED array with colors and blink/fade animations at 20Hz.
 * 
 * @param arg Task arguments (unused).
 */
void LedTask(void *arg)
{
    initLed();
    Serial.println("Led task running!");
    unsigned long lastUpdate = millis();
    
    while (1)
    {
        bool changed = false;

        if (xQueueReceive(ledStatus, (void *)&ledDataStatus, 0) == pdTRUE)
        {
            leds[LEDSTATUS] = ledDataStatus.color;
            changed = true;
        }
        if (xQueueReceive(ledUtil, (void *)&ledDataUtil, 0) == pdTRUE)
        {
            leds[LEDUTIL] = ledDataUtil.color;
            changed = true;
        }
        if (xQueueReceive(ledGps, (void *)&ledDataGps, 0) == pdTRUE)
        {
            leds[LEDGPS] = ledDataGps.color;
            changed = true;
        }
        if (xQueueReceive(ledPwr, (void *)&ledPwrData, 0) == pdTRUE)
        {
            int bbVal = ledPwrData.ledBb;
            int sbVal = ledPwrData.ledSb;

            for (int i = 1; i < 11; i++)
            {
                // BB Motor Bar (LEDs 3 to 12)
                if (bbVal >= 10) { leds[2 + i] = CRGB::Red; bbVal -= 10; }
                else if (bbVal <= -10) { leds[2 + i] = CRGB::Green; bbVal += 10; }
                else { leds[2 + i] = CRGB::Black; }

                // SB Motor Bar (LEDs 13 to 22)
                if (sbVal >= 10) { leds[2 + 10 + i] = CRGB::Red; sbVal -= 10; }
                else if (sbVal <= -10) { leds[2 + 10 + i] = CRGB::Green; sbVal += 10; }
                else { leds[2 + 10 + i] = CRGB::Black; }
            }
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

            // --- Status LED ---
            if (ledDataStatus.blink == BLINK_FAST) leds[LEDSTATUS] = blinkFast ? ledDataStatus.color : CRGB::Black;
            else if (ledDataStatus.blink == BLINK_SLOW) leds[LEDSTATUS] = blink ? ledDataStatus.color : CRGB::Black;
            else if (ledDataStatus.blink == FADE_ON) {
                leds[LEDSTATUS] = ledDataStatus.color;
                leds[LEDSTATUS].nscale8(ledDataStatus.brightness);
                ledDataStatus.brightness += ledDataStatus.fadeAmount;
                if (ledDataStatus.brightness <= 10 || ledDataStatus.brightness >= 245) ledDataStatus.fadeAmount = -ledDataStatus.fadeAmount;
            }
            
            // --- Util LED ---
            if (ledDataUtil.blink == BLINK_FAST) leds[LEDUTIL] = blinkFast ? ledDataUtil.color : CRGB::Black;
            else if (ledDataUtil.blink == BLINK_SLOW) leds[LEDUTIL] = blink ? ledDataUtil.color : CRGB::Black;
            else if (ledDataUtil.blink == FADE_ON) {
                leds[LEDUTIL] = ledDataUtil.color;
                leds[LEDUTIL].nscale8(ledDataUtil.brightness);
                ledDataUtil.brightness += ledDataUtil.fadeAmount;
                if (ledDataUtil.brightness <= 10 || ledDataUtil.brightness >= 245) ledDataUtil.fadeAmount = -ledDataUtil.fadeAmount;
            }

            // --- GPS LED ---
            if (ledDataGps.blink == BLINK_FAST) leds[LEDGPS] = blinkFast ? ledDataGps.color : CRGB::Black;
            else if (ledDataGps.blink == BLINK_SLOW) leds[LEDGPS] = blink ? ledDataGps.color : CRGB::Black;
            else if (ledDataGps.blink == FADE_ON) {
                leds[LEDGPS] = ledDataGps.color;
                leds[LEDGPS].nscale8(ledDataGps.brightness);
                ledDataGps.brightness += ledDataGps.fadeAmount;
                if (ledDataGps.brightness <= 10 || ledDataGps.brightness >= 245) ledDataGps.fadeAmount = -ledDataGps.fadeAmount;
            }
            changed = true;
        }

        if (changed)
        {
            FastLED.show();
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}