/*
Indicator
*/
#include <Arduino.h>
#include "indicator.h"
#include <FastLED.h>
#include "io.h"
#include "general.h"
#define NUM_STRIPS 1
#define NUM_LEDS 3

QueueHandle_t indicatorque;
QueueHandle_t statusque;

CRGB leds[NUM_LEDS];

void InitFastled(void)
{
    FastLED.addLeds<NEOPIXEL, LEDSTRIPPIN>(leds, NUM_LEDS);
}

void IndicatorTask(void *arg)
{
    LMessage msg;
    int bb = 0, sb = 0;
    InitFastled();
    indicatorque = xQueueCreate(10, sizeof(LMessage));
    statusque = xQueueCreate(10, sizeof(SMessage));
    while (1)
    {
        if (xQueueReceive(indicatorque, (void *)&msg, 0) == pdTRUE)
        {
            if (msg.speedbb < 0)
            {
                bb = map(msg.speedbb, 0, 100, 0, 255);
                leds[0] = CRGB(bb, 0, 0);
            }
            else
            {
                bb = map(msg.speedbb, 0, -100, 0, 255);
                leds[0] = CRGB(0, bb, 0);
            }
            if (msg.speedsb < 0)
            {
                sb = map(msg.speedsb, 0, 100, 0, 255);
                leds[1] = CRGB(sb, 0, 0);
            }
            else
            {
                sb = map(msg.speedsb, 0, -100, 0, 255);
                leds[1] = CRGB(0, sb, 0);
            }
            FastLED.show();
        }
        if (xQueueReceive(statusque, (void *)&msg, 0) == pdTRUE)
        {
            leds[2] = msg.ledstatus;
            FastLED.show();
        }
        delay(1);
    }
}
