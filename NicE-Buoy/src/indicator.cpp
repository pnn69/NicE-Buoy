/*
Indicator
*/
#include <FastLED.h>
#define NUM_STRIPS 2
#define NUM_LEDS_PER_STRIP 5
CRGB leds[NUM_STRIPS][NUM_LEDS_PER_STRIP];

void InitFastled(void)
{
    FastLED.addLeds<NEOPIXEL, 2>(leds[0], NUM_LEDS_PER_STRIP);
    FastLED.addLeds<NEOPIXEL, 3>(leds[1], NUM_LEDS_PER_STRIP);
}

void IndicatorTask(void *arg)
{
    while (1)
    {
        for (int x = 0; x < NUM_STRIPS; x++)
        {
            // This inner loop will go over each led in the current strip, one at a time
            for (int i = 0; i < NUM_LEDS_PER_STRIP; i++)
            {
                leds[x][i] = CRGB::Red;
                FastLED.show();
                leds[x][i] = CRGB::Black;
            }
        }
        delay(100);
    }
}