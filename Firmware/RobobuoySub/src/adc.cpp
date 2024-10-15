#include <Arduino.h>
#include "io_sub.h"
#include <Arduino.h>

void battVoltage(float *vbatt, int *vperc)
{
    int adc_result = analogReadMilliVolts(VBATT);
    *vbatt = adc_result * 0.013339;
    float perc = map(*vbatt * 100.0, 3.6 * 600.0, 4.2 * 600.0, 0, 10000);
    *vperc = (int)((constrain(perc, 0, 10000)) / 100.0);
    Serial.printf("Vbat=%2.2lf", *vbatt);
}