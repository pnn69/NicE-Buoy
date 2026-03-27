#include "Arduino.h"
#include "adc.h"
#include "io.h"
#include "esp_adc_cal.h"

#define MUTE 200
#define MUTE_RUDDER 200
#define JITTER 20
adcDataType adc;

char sw_pos = SW_MID;

uint32_t readADC_Cal(int ADC_Raw)
{
    esp_adc_cal_characteristics_t adc_chars;

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

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
void readAdc(adcDataType *adc)
{
    static float smoothedRudder = 2048;
    static float smoothedSpeed = 2048;
    const float alpha = 0.2; // Smoothing factor (0.0 to 1.0)

    // EMA Filtering for Rudder/Heading
    smoothedRudder = (alpha * analogRead(POT_RUDDER)) + ((1.0 - alpha) * smoothedRudder);
    adc->heading = map((int)smoothedRudder, 0, 4095, 360, 0);
  
    // EMA Filtering for Speed
    int rawSpeed = analogRead(POT_SPEED);
    smoothedSpeed = (alpha * rawSpeed) + ((1.0 - alpha) * smoothedSpeed);
    
    int val = (int)smoothedSpeed;
    int newValue = 0;
    int center = 2048;
    int deadzone = 700;

    if (val < center - deadzone) {
        newValue = map(val, 0, center - deadzone, -100, 0);
    } else if (val > center + deadzone) {
        newValue = map(val, center + deadzone, 4095, 0, 100);
    }

    if (abs((int)adc->raws - val) > JITTER)
    {
        adc->raws = val;
        adc->speed = newValue;
        adc->newdata = true;
    }

    // Mode Switch
    int swRaw = analogRead(SWITCH_PIN_REMOTE_IDLE_LOCK);
    int newSwPos = adc->swPos;

    if (swRaw < 1300) newSwPos = SW_LEFT;
    else if (swRaw > 1700 && swRaw < 2800) newSwPos = SW_MID;
    else if (swRaw > 3200) newSwPos = SW_RIGHT;
    
    // Simple software debounce
    static int debouncedSwPos = SW_MID;
    static int swPosStableCount = 0;
    
    if (newSwPos != debouncedSwPos) {
        swPosStableCount++;
        if (swPosStableCount > 3) { // Require 3 stable readings (approx 150ms)
            debouncedSwPos = newSwPos;
            adc->swPos = debouncedSwPos;
            swPosStableCount = 0;
        }
    } else {
        swPosStableCount = 0;
    }
}