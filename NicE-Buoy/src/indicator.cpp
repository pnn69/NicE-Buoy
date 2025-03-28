/*
Indicator
*/
#include <Arduino.h>
#include "indicator.h"
#include <FastLED.h>
#include "io.h"
#include "general.h"
#define NUM_STRIPS 1
#define NUM_BARR 20
#define BARR_OFFSET 3

QueueHandle_t indicatorqueSp; // speed que
QueueHandle_t indicatorqueSt; // status que
QueueHandle_t indicatorqueBb; // status que
QueueHandle_t indicatorqueSb; // status que

CRGB leds[NUM_BARR * 2 + BARR_OFFSET];

void InitFastled(void)
{
    FastLED.addLeds<NEOPIXEL, LEDSTRIPPIN>(leds, NUM_BARR * 2 + BARR_OFFSET);
}

void IndicatorTask(void *arg)
{
    MessageSP msgSP;
    MessageSq msgSt;
    MessageSq msgBb;
    MessageSq msgSb;

    CRGB bbcolor;
    CRGB sbcolor;
    int bb = 0, sb = 0;
    InitFastled();
    indicatorqueSp = xQueueCreate(10, sizeof(MessageSP));
    indicatorqueSt = xQueueCreate(10, sizeof(CRGB));
    indicatorqueBb = xQueueCreate(10, sizeof(CRGB));
    indicatorqueSb = xQueueCreate(10, sizeof(CRGB));

    while (1)
    {
        if (xQueueReceive(indicatorqueSp, (void *)&msgSP, 0) == pdTRUE)
        {
            bb = msgSP.speedbb;
            sb = msgSP.speedsb;
            //determ collor
            if (bb < 0)
            {
                bbcolor = CRGB::Red;
                bb = bb * -1;
            }
            else
            {
                bbcolor = CRGB::Green;
            }
            if (sb < 0)
            {
                sbcolor = CRGB::Red;
                sb = sb * -1;
            }
            else
            {
                sbcolor = CRGB::Green;
            }
            //fill buffer
            for (int i = 0; i < (NUM_BARR); i++)
            {
                if (bb > 0)
                {
                    leds[BARR_OFFSET + i] = bbcolor;
                    bb = bb - 100 / NUM_BARR;
                }
                else
                {
                    leds[BARR_OFFSET + i] = CRGB::Black;
                }
                if (sb > 0)
                {
                    leds[BARR_OFFSET - 1 + NUM_BARR * 2 - i] = sbcolor;
                    sb = sb - 100 / NUM_BARR;
                }
                else
                {
                    leds[BARR_OFFSET - 1 + NUM_BARR * 2 - i] = CRGB::Black;
                }
            }
            FastLED.show();
        }

        if (xQueueReceive(indicatorqueBb, (void *)&msgBb, 0) == pdTRUE)
        {
            leds[0] = msgBb.ledstatus;
            FastLED.show();
        }
        if (xQueueReceive(indicatorqueSb, (void *)&msgSb, 0) == pdTRUE)
        {
            leds[1] = msgSb.ledstatus;
            FastLED.show();
        }
        if (xQueueReceive(indicatorqueSt, (void *)&msgSt, 0) == pdTRUE)
        {
            leds[2] = msgSt.ledstatus;
            FastLED.show();
        }
        delay(1);
    }
}
