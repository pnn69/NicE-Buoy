#include <Arduino.h>
#include "io_top.h"

void battVoltage(float &vbatt, int &vperc)
{
    int adc_result = analogReadMilliVolts(VBATT);
    vbatt = adc_result / 500.0;
    vperc = (int)map(vbatt * 100, 3.0 * 100, 4.2 * 100, 0, 100); // nominal 3.0-4.2V  breakdown 2.3-4.7V
    vperc = constrain(vperc, 0, 100);
    //Serial.printf("Vraw=%d Vbat=%2.2lf Perc:%d\r\n", adc_result, vbatt, vperc);
}