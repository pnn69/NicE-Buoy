#include <Arduino.h>
#include "io_sub.h"

static float smoothed_vbatt = 0;
static float smoothed_current = 0;
static bool first_read_v = true;
static bool first_read_i = true;

/**
 * @brief Reads and filters the battery voltage.
 * 1. Reads the battery voltage from the ADC in millivolts.
 * 2. Applies a simple IIR filter to smooth out noise (90% old, 10% new).
 * 3. Maps the smoothed voltage to a percentage (assuming 6-cell Li-ion, 18V-25.2V).
 * 
 * @param vbatt Output parameter for the smoothed battery voltage.
 * @param vperc Output parameter for the battery percentage (0-100%).
 */
void battVoltage(float &vbatt, int &vperc)
{
    int adc_result = analogReadMilliVolts(VBATT);
    
    // Resistor divider: R1 = 6200 Ohm, R2 = 500 Ohm (Theoretical ratio: 13.4)
    // Empirical calibration: Vin = 21.97V when Vpin = 1.630V (Ratio: 13.4785)
    float v_batt_now = (adc_result / 1000.0f) * (21.97f / 1.630f);

    if (first_read_v) {
        smoothed_vbatt = v_batt_now;
        first_read_v = false;
    } else {
        // Simple IIR filter: 90% old, 10% new
        smoothed_vbatt = (smoothed_vbatt * 0.9f) + (v_batt_now * 0.1f);
    }

    vbatt = smoothed_vbatt;
    vperc = (int)map(vbatt * 100, (3.0f * 6) * 100, (4.2f * 6) * 100, 0, 100);
    vperc = constrain(vperc, 0, 100);
}

/**
 * @brief Reads and filters the battery current (V3 only).
 * Formula: 0.1V = -20A, 1.65V = 0A, 3.20V = 20A
 * 
 * @param current_a Output parameter for the smoothed current in Amperes.
 */
void battCurrent(float &current_a, float &imon_v)
{
#ifdef IMON_PIN
    int adc_mv = analogReadMilliVolts(IMON_PIN);
    imon_v = adc_mv / 1000.0f;

    // Formula: I = (V - 1.65V) * (20A / 1.55V)
    // I = (adc_mv - 1650) * (20 / 1550)
    // Adjusted by / 100.0f as reported by user
    float instant_current = ((float)(adc_mv - 1650) * (20.0f / 1550.0f)) / 100.0f;

    if (first_read_i) {
        smoothed_current = instant_current;
        first_read_i = false;
    } else {
        // Simple IIR filter: 90% old, 10% new
        smoothed_current = (smoothed_current * 0.9f) + (instant_current * 0.1f);
    }
    current_a = smoothed_current;
#else
    imon_v = 0.0f;
    current_a = 0.0f;
#endif
}