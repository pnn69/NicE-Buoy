/*
    Leds on board updated
    Put new value in que and the leds will be updated.
*/
#include <Arduino.h>
#include <FastLED.h>
#include "main.h"
#include "leds.h"
#include "io_top.h"

#define NUM_LEDS 3
#define LEDSTATUS 0
#define LEDUTIL 1
#define LEDGPS 2

CRGB ledData;
CRGB leds[NUM_LEDS];

QueueHandle_t ledStatus; // speed status bb,sb
QueueHandle_t ledUtil;   // Accu status
QueueHandle_t ledGps;    // Accu status
static uint8_t blinkcnt = 0;
static unsigned long blinktimer = millis();
static LedData ledDataStatus;
static LedData ledDataUtil;
static LedData ledDataGps;
static bool blink, blinkFast;

bool initledqueue(void)
{
    // ledStatus = xQueueCreate(10, sizeof(CRGB));
    ledStatus = xQueueCreate(10, sizeof(LedData));
    ledUtil = xQueueCreate(10, sizeof(LedData));
    ledGps = xQueueCreate(10, sizeof(LedData));
    Serial.println("Led queue created!");
    return true;
}

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