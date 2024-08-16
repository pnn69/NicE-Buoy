/*
    Leds on board updated
    Put new value in que and the leds will be updated.
*/
#include <Arduino.h>
#include <FastLED.h>
#include "leds.h"
#include "io_sub.h"

#define NUM_LEDS 3
#define ACCU 0
#define BB 1
#define SB 2

CRGB leds[NUM_LEDS];

QueueHandle_t ledsSpeed;   // speed status bb,sb
QueueHandle_t ledAccuVoltage; // Accu status

void initLedTask(void){
    ledsSpeed = xQueueCreate(10, sizeof(SpeedMessage));
    ledAccuVoltage = xQueueCreate(10, sizeof(AccuMessage));
    FastLED.addLeds<WS2812B, LEDS_PIN, GRB>(leds, NUM_LEDS);
    FastLED.clear();
    leds[ACCU] = CRGB::Green;
    FastLED.show();
}

void LedTask(void *arg)
{
    SpeedMessage speed;
    AccuMessage accu;
    Serial.println("Led task running!");
    while (1)
    {
        if (xQueueReceive(ledsSpeed, (void *)&speed, 0) == pdTRUE)
        {
            if (speed.speedbb == 0)
            {
                leds[BB] = CRGB::Black;
            }
            else if (speed.speedbb < 0)
            {
                leds[BB] = CRGB(abs(speed.speedbb),0,0);
            }
            else
            {
                leds[BB] = CRGB(0,speed.speedbb,0);
            }

            if (speed.speedsb == 0)
            {
                leds[BB] = CRGB::Black;
            }
            else if (speed.speedsb < 0)
            {
                leds[SB] = CRGB(abs(speed.speedsb),0,0);
            }
            else
            {
                leds[SB] = CRGB(0,speed.speedsb,0);
            }


            FastLED.show();
        }
        if (xQueueReceive(ledAccuVoltage, (void *)&accu, 0) == pdTRUE)
        {
            float in = accu.accuvoltage;
            if (in < 20)
            {
                leds[ACCU] = CRGB::Red;
            }
            else if (in < 22)
            {
                leds[ACCU] = CRGB::OrangeRed;
            }
            else if (in < 24)
            {
                leds[ACCU] = CRGB::Orange;
            }
            else
            {
                leds[ACCU] = CRGB::Green;
            }

            FastLED.show();
        }
        vTaskDelay(100);

    }
}