/*
    Leds on board updated
    Put new value in que and the leds will be updated.
*/
#include <Arduino.h>
#include <FastLED.h>
#include "main.h"
#include "leds.h"
#include "io_sub.h"

#define NUM_LEDS 3
#define LEDBB 0
#define LEDSTATUS 1
#define LEDSB 2

CRGB ledData;
CRGB leds[NUM_LEDS];

QueueHandle_t ledPwr;    // speed bb,sb
QueueHandle_t ledStatus; // Accu status
static uint8_t blinkcnt = 0;
static unsigned long blinktimer = millis();
static LedData ledStatusData;
static PwrData ledPwrData;
static bool blink, blinkFast;

bool initledqueue(void)
{
    ledStatus = xQueueCreate(10, sizeof(LedData));
    ledPwr = xQueueCreate(10, sizeof(PwrData));
    return true;
}

void initLed(void)
{
    FastLED.addLeds<WS2812B, LEDS_PIN, GRB>(leds, NUM_LEDS);
    FastLED.clear();
    leds[LEDSTATUS] = CRGB::Red;
    leds[LEDBB] = CRGB::White;
    leds[LEDSB] = CRGB::Blue;
    FastLED.setBrightness(100);
    FastLED.show();
}

void LedTask(void *arg)
{
    initLed();
    Serial.println("Led task running!");
    while (1)
    {

        if (xQueueReceive(ledStatus, (void *)&ledStatusData, 0) == pdTRUE)
        {
            leds[LEDSTATUS] = ledStatusData.color;
            if (ledStatusData.blink == 0 || ledStatusData.blink == BLINK_OFF)
            {
                FastLED.show();
            }
        }
        if (xQueueReceive(ledPwr, (void *)&ledPwrData, 0) == pdTRUE)
        {
            leds[LEDBB] = ledPwrData.sb;
            leds[LEDSB] = ledPwrData.bb;
            FastLED.show();
        }
        /*
            Blink stuff
        */
        if (blinktimer + 100 < millis())
        {
            blinktimer = millis();
            blinkcnt++;
            blinkFast = !blinkFast;
            if (ledStatusData.blink == BLINK_FAST)
            {
                if (blinkFast == true)
                {
                    leds[LEDSTATUS] = ledStatusData.color;
                }
                else
                {
                    leds[LEDSTATUS] = CRGB ::Black;
                }
                FastLED.show();
            }
            if (ledPwrData.blinkBb == BLINK_FAST)
            {
                if (blinkFast == true)
                {
                    leds[LEDBB] = ledPwrData.bb;
                }
                else
                {
                    leds[LEDBB] = CRGB ::Black;
                }
                FastLED.show();
            }
            if (ledPwrData.blinkSb == BLINK_FAST)
            {
                if (blinkFast == true)
                {
                    leds[LEDSB] = ledPwrData.sb;
                }
                else
                {
                    leds[LEDSB] = CRGB ::Black;
                }
                FastLED.show();
            }
            if (blinkcnt >= 10)
            {
                blinkcnt = 0;
                blink = !blink;
                if (ledStatusData.blink == BLINK_SLOW)
                {
                    if (blink == true)
                    {
                        // printf("led on\r\n");
                        leds[LEDSTATUS] = ledStatusData.color;
                    }
                    else
                    {
                        // printf("led off\r\n");
                        leds[LEDSTATUS] = CRGB ::Black;
                    }
                    FastLED.show();
                }
                if (ledPwrData.blinkBb == BLINK_SLOW)
                {
                    if (blink == true)
                    {
                        leds[LEDBB] = ledPwrData.bb;
                    }
                    else
                    {
                        leds[LEDBB] = CRGB ::Black;
                    }
                    FastLED.show();
                }
                if (ledPwrData.blinkSb == BLINK_SLOW)
                {
                    if (blink == true)
                    {
                        leds[LEDSB] = ledPwrData.sb;
                    }
                    else
                    {
                        leds[LEDSB] = CRGB ::Black;
                    }
                    FastLED.show();
                }
            }
        }
        vTaskDelay(1);
    }
}