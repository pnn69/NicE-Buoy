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
    
    // Resistor divider: R1 = 6200 Ohm, R2 = 500 Ohm (Theoretical ratio: 13.4)
    // Empirical calibration: Vin = 21.97V when Vpin = 1.630V (Ratio: 13.4785)
    float v_batt_now = (adc_result / 1000.0f) * (21.97f / 1.630f);
    
    if (first_read) {
        smoothed_vbatt = v_batt_now;
        first_read = false;
    } else {
        // Simple IIR filter: 90% old, 10% new
        smoothed_vbatt = (smoothed_vbatt * 0.9f) + (v_batt_now * 0.1f);
    }

    vbatt = smoothed_vbatt;
    vperc = (int)map(vbatt * 100, (3.0f * 6) * 100, (4.2f * 6) * 100, 0, 100); 
    vperc = constrain(vperc, 0, 100);
}