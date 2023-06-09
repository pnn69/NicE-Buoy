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
void readAdc(void){
    int tmp;
    tmp = analogRead(POT_RUDDER);
    if(adc.rawr - 10 > tmp || adc.rawr +10 < tmp){
        adc.rudder = map(tmp,47,4047,-179,179);// 4095;
        adc.newdata = true;
    }
    tmp = analogRead(POT_SPEED);
    if(adc.raws - 10 > adc.raws || adc.raws + 10 <tmp){
        adc.rudder = map(tmp,47,4047,-100,100);// 4095;
        adc.newdata = true;
    }
}