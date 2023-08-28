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

CRGB leds[NUM_LEDS];

void InitFastled(void)
{
    if (LEDSTRIP == true)
    {
        FastLED.addLeds<NEOPIXEL, LEDSTRIPPIN>(leds, NUM_LEDS);
    }
}

void IndicatorTask(void *arg)
{
    LMessage msg;
    int bb = 0, sb = 0;
    InitFastled();
    indicatorque = xQueueCreate(10, sizeof(LMessage));
    while (1)
    {
        if (xQueueReceive(indicatorque, (void *)&msg, 0) == pdTRUE)
        {
            bb = map(msg.speedbb, 0, 100, 0, 255);
            sb = map(msg.speedsb, 0, 100, 0, 255);
            if (bb < 0)
            {
                leds[0] = CHSV(bb, 0, 0);
            }
            else
            {
                leds[0] = CHSV(0, bb, 0);
            }
            if (sb < 0)
            {
                leds[1] = CHSV(sb, 0, 0);
            }
            else
            {
                leds[1] = CHSV(0, sb, 0);
            }
            leds[2] = msg.ledstatus;
            if (LEDSTRIP == true)
            {
                FastLED.show();
            }
        }
        delay(1);
    }
}
