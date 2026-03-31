#include <Arduino.h>
#include "io_top.h"

static float smoothed_vbatt = 0;
static bool first_read = true;

/**
 * @brief Reads the battery voltage and calculates the percentage.
 * 
 * Uses an IIR filter to smooth the analog reading and map the voltage 
 * to a 0-100 percentage based on the typical Li-ion 6S pack voltage range.
 * 
 * @param vbatt Output parameter to store the smoothed voltage.
 * @param vperc Output parameter to store the calculated battery percentage.
 */
void battVoltage(float &vbatt, int &vperc)
{
    int adc_result = analogReadMilliVolts(VBATT);
    float current_v = adc_result * 0.021f;
    
    if (first_read) {
        smoothed_vbatt = current_v;
        first_read = false;
    } else {
        // Simple IIR filter: 90% old, 10% new
        smoothed_vbatt = (smoothed_vbatt * 0.9f) + (current_v * 0.1f);
    }

    vbatt = smoothed_vbatt;
    vperc = (int)map(vbatt * 100, (3.0f * 6) * 100, (4.2f * 6) * 100, 0, 100); 
    vperc = constrain(vperc, 0, 100);
}