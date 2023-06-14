/*
Indicator
*/
#include <Arduino.h>
#include "indicator.h"
#include <FastLED.h>
#include "io.h"
#define NUM_STRIPS 2
#define NUM_LEDS_PER_STRIP 11

QueueHandle_t indicatorque;

CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];

void InitFastled(void)
{
    FastLED.addLeds<NEOPIXEL, LEDSTRIP1>(leds[0], NUM_LEDS_PER_STRIP);
    FastLED.addLeds<NEOPIXEL, LEDSTRIP2>(leds[1], NUM_LEDS_PER_STRIP);
}

void IndicatorTask(void *arg)
{
    LMessage msg;
    CRGB bbcolor;
    CRGB sbcolor;
    int bb = 0, sb = 0;
    InitFastled();
    indicatorque = xQueueCreate(10, sizeof(LMessage));
    while (1)
    {
        if (xQueueReceive(indicatorque, (void *)&msg, 0) == pdTRUE)
        {
            bb = msg.speedbb;
            sb = msg.speedsb;
            bbcolor = CRGB::Green;
            if (bb < 0)
            {
                bbcolor = CRGB::Red;
                bb = bb * -1;
            }

            sbcolor = CRGB::Green;
            if (sb < 0)
            {
                sbcolor = CRGB::Red;
                sb = sb * -1;
            }

            for (int i = 0; i < NUM_LEDS_PER_STRIP - 1; i++)
            {
                if (bb > 0)
                {
                    leds[0][i] = bbcolor;
                    bb = bb - 100 / NUM_LEDS_PER_STRIP - 1;
                }
                else
                {
                    leds[0][i] = CRGB::Black;
                }
                if (sb > 0)
                {
                    leds[1][i] = sbcolor;
                    sb = sb - 100 / NUM_LEDS_PER_STRIP - 1;
                }
                else
                {
                    leds[1][i] = CRGB::Black;
                }
            }
            leds[0][10] = msg.ledstatus1;
            leds[1][10] = msg.ledstatus2;
            FastLED.show();
        }
        delay(1);
    }
}
