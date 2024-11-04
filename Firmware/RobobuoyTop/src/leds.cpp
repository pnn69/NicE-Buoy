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
    while (1)
    {

        if (xQueueReceive(ledStatus, (void *)&ledDataStatus, 0) == pdTRUE)
        {
            leds[LEDSTATUS] = ledDataStatus.color;
            if (ledDataStatus.blink == 0 || ledDataStatus.blink == BLINK_OFF)
            {
                FastLED.show();
            }
        }
        if (xQueueReceive(ledUtil, (void *)&ledDataUtil, 0) == pdTRUE)
        {
            leds[LEDUTIL] = ledDataUtil.color;
            if (ledDataUtil.blink == 0 || ledDataUtil.blink == BLINK_OFF)
            {
                FastLED.show();
            }
        }
        if (xQueueReceive(ledGps, (void *)&ledDataGps, 0) == pdTRUE)
        {
            leds[LEDGPS] = ledDataGps.color;
            if (ledDataGps.blink == 0 || ledDataGps.blink == BLINK_OFF)
            {
                FastLED.show();
            }
        }
        /*
            Blink stuff
        */
        if (blinktimer + 100 < millis())
        {
            blinktimer = millis();
            blinkcnt++;
            blinkFast = !blinkFast;
            if (ledDataGps.blink == BLINK_FAST)
            {
                if (blinkFast == true)
                {
                    leds[LEDGPS] = ledDataGps.color;
                }
                else
                {
                    leds[LEDGPS] = CRGB ::Black;
                }
                FastLED.show();
            }
            if (ledDataStatus.blink == BLINK_FAST)
            {
                if (blinkFast == true)
                {
                    leds[LEDSTATUS] = ledDataStatus.color;
                }
                else
                {
                    leds[LEDSTATUS] = CRGB ::Black;
                }
                FastLED.show();
            }
            if (ledDataUtil.blink == BLINK_FAST)
            {
                if (blinkFast == true)
                {
                    leds[LEDUTIL] = ledDataUtil.color;
                }
                else
                {
                    leds[LEDUTIL] = CRGB ::Black;
                }
                FastLED.show();
            }
            if (blinkcnt >= 10)
            {
                blinkcnt = 0;
                blink = !blink;
                if (ledDataGps.blink == BLINK_SLOW)
                {
                    if (blink == true)
                    {
                        leds[LEDGPS] = ledDataGps.color;
                    }
                    else
                    {
                        leds[LEDGPS] = CRGB ::Black;
                    }
                    FastLED.show();
                }
                if (ledDataStatus.blink == BLINK_SLOW)
                {
                    if (blink == true)
                    {
                        leds[LEDSTATUS] = ledDataStatus.color;
                    }
                    else
                    {
                        leds[LEDSTATUS] = CRGB ::Black;
                    }
                    FastLED.show();
                }
                if (ledDataUtil.blink == BLINK_SLOW)
                {
                    if (blink == true)
                    {
                        leds[LEDUTIL] = ledDataUtil.color;
                    }
                    else
                    {
                        leds[LEDUTIL] = CRGB ::Black;
                    }
                    FastLED.show();
                }
            }
        }
        vTaskDelay(1);
    }
}