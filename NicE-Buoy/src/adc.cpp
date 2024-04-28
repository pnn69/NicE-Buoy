#include <Arduino.h>
#include "io.h"
#include <Arduino.h>
#include "general.h"
#include "buzzer.h"

#define DEBOUNCE 50

MessageBuzz snd_buzz;

/*
  This switch has 3 positions top middle bottom
  adc reads more than 2000 mV in top positon
  adc reads less than 1000 mV in bottom positon
  Midddle postition between 1000mV and 2000mV
*/
static int skip = 0;
void adc_switch(void)
{
    if (skip == 0)
    {
        snd_buzz.repeat = 0;
        snd_buzz.pauze = 0;
        int adc_result = analogReadMilliVolts(SWITCH1);
        if (adc_result > 2000)
        {
            if (frontsw.switch1upact == 0)
            {
                frontsw.switch1upcnt = 1;
                frontsw.switch1upact = 1;
            }
            else
            {
                frontsw.switch1upcnt++;
            }
            snd_buzz.time = BUZZTIME;
            xQueueSend(Buzzerque, (void *)&snd_buzz, 10);
            skip = DEBOUNCE;
        }
        else if (frontsw.switch1upact == 1)
        {
            frontsw.switch1upact = 0;
        }

        if (adc_result < 1000)
        {
            if (frontsw.switch1dwnact == 0)
            {
                frontsw.switch1dwncnt = 1;
            }
            else
            {
                frontsw.switch1dwncnt++;
            }
            frontsw.switch1upact = 1;
            snd_buzz.time = BUZZTIME;
            xQueueSend(Buzzerque, (void *)&snd_buzz, 10);
            skip = DEBOUNCE;
        }
        else if (frontsw.switch1dwnact == 1)
        {
            frontsw.switch1dwnact = 0;
        }

        adc_result = analogReadMilliVolts(SWITCH2);
        if (adc_result > 2000)
        {
            if (frontsw.switch2upact == 0)
            {
                frontsw.switch2upcnt = 1;
            }
            else
            {
                frontsw.switch2upcnt++;
            }
            frontsw.switch2upact = 1;
            snd_buzz.time = BUZZTIME;
            xQueueSend(Buzzerque, (void *)&snd_buzz, 10);
            skip = DEBOUNCE;
        }
        else if (frontsw.switch2upact == 1)
        {
            frontsw.switch2upact = 0;
        }

        if (adc_result < 1000)
        {
            if (frontsw.switch2dwnact == 0)
            {
                frontsw.switch2dwncnt = 1;
            }
            else
            {
                frontsw.switch2dwncnt++;
            }
            frontsw.switch2upact = 1;
            snd_buzz.time = BUZZTIME;
            xQueueSend(Buzzerque, (void *)&snd_buzz, 10);
            skip = DEBOUNCE;
        }
        else if (frontsw.switch2dwnact == 1)
        {
            frontsw.switch2dwnact = 0;
        }
        adc_result = analogReadMilliVolts(VBATT);
        buoy.vbatt = adc_result * 0.013339;
        float perc = map(buoy.vbatt * 100.0, 3.6 * 600.0, 4.2 * 600.0, 0, 10000);
        buoy.vperc = (constrain(perc, 0, 10000)) / 100.0;
        //Serial.printf("Vbat=%2.2lf",buoy.vbatt);
    }
    else
    {
        skip--;
    }
}