/*
    Leds on board updated
    Put new value in que and the leds will be updated.
*/
#include <Arduino.h>
#include <FastLED.h>
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
static bool blink, blinkfast;
static uint8_t blinkcnt = 0;
static unsigned long blinktimer = millis();
static LedData ledDataStatus;
static LedData ledDataUtil;
static LedData ledDataGps;

void initLedTask(void)
{
    ledStatus = xQueueCreate(10, sizeof(CRGB));
    ledUtil = xQueueCreate(10, sizeof(CRGB));
    ledGps = xQueueCreate(10, sizeof(CRGB));
    FastLED.addLeds<WS2812B, LEDS_PIN, GRB>(leds, NUM_LEDS);
    FastLED.clear();
    leds[LEDSTATUS] = CRGB::Red;
    leds[LEDUTIL] = CRGB::White;
    leds[LEDGPS] = CRGB::Blue;
    FastLED.setBrightness(100);
    FastLED.show();
}

void LedTask(void *arg)
{
    Serial.println("Led task running!");
    while (1)
    {

        if (xQueueReceive(ledStatus, (void *)&ledDataStatus, 0) == pdTRUE)
        {
            leds[LEDSTATUS] = ledDataStatus.color;
            FastLED.show();
        }
        if (xQueueReceive(ledUtil, (void *)&ledDataUtil, 0) == pdTRUE)
        {
            leds[LEDUTIL] = ledDataUtil.color;
            FastLED.show();
        }
        if (xQueueReceive(ledGps, (void *)&ledDataGps, 0) == pdTRUE)
        {
            leds[LEDGPS] = ledDataGps.color;
            FastLED.show();
        }

        if (blinktimer + 100 < millis())
        {
            blinktimer = millis();
            blinkcnt++;
            blinkfast = !blinkfast;
            if (blinkcnt >= 10)
            {
                blinkcnt = 0;
                blink = !blink;
            }
        }
        vTaskDelay(100);
    }
}