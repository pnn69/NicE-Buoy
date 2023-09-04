#include <Arduino.h>
#include "io.h"
#include "general.h"

/*
  This switch has 3 positions top middle bottom
  adc reads more than 2000 mV in top positon
  adc reads less than 1000 mV in bottom positon
  Midddle postition between 1000mV and 2000mV
*/
void adc_switch(void)
{
    int adc_result = analogReadMilliVolts(SWITCH1);
    if (adc_result > 2000)
    {
        if (frontsw.switch1upact == 0)
        {
            frontsw.switch1upcnt = 1;
        }
        else
        {
            frontsw.switch1upcnt++;
        }
        frontsw.switch1upact = 1;
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
    }
    else if (frontsw.switch2dwnact == 1)
    {
        frontsw.switch2dwnact = 0;
    }
}