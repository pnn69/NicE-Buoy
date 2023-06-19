#include "Arduino.h"
#include "adc.h"
#include "io.h"
#include "general.h"

struct adcDataType adc;

/*
    Potentiometer connected to vcc (3.3V)
    range of 4095 bits
    for speed
    map 47 to -100
    map 4047 to 100
    so 2047 = off
    for rudder
    map 47 to -197
    map 4047 to 197
    so 2047 = straight
*/
void readAdc(void)
{
    int tmp;
    tmp = analogRead(POT_RUDDER);
    tmp = constrain(tmp,47,4047);
    if (adc.rawr - 10 > tmp || adc.rawr + 10 < tmp)
    {
        adc.rudder = map(tmp, 47, 4047, -135, 135); // 4095;
        adc.rawr = tmp;
        adc.newdata = true;
    }
    tmp = analogRead(POT_SPEED);
    tmp = constrain(tmp,47,4047);
    if (adc.raws - 10 > tmp || adc.raws + 10 < tmp)
    {
        adc.raws = tmp;
        tmp = map(tmp, 47, 4047, -100, 100); // 4095;
        if( -5 < tmp && tmp < 5){
            tmp = 0;
        }
        adc.speed = tmp;
        adc.newdata = true;
    }
}