#include "Arduino.h"
#include "adc.h"
#include "io.h"
#include "general.h"
#include "esp_adc_cal.h"

#define MUTE 400
#define MUTE_RUDDER 100
#define JITTER 20
struct adcDataType adc;

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
void readAdc(void)
{
    int tmp, newValue;
    tmp = 0;
    for (int i = 0; i < 20; i++)
    {
        tmp += analogRead(POT_RUDDER) / 20;
    }
    // tmp = analogRead(POT_RUDDER);
    tmp = constrain(tmp, 47, 4047);
    if (tmp < 2000 - MUTE_RUDDER)
    {
        newValue = map(tmp, 47, 2000 - MUTE_RUDDER, -135, 0); // 4095;
    }
    else if (tmp > 2000 + MUTE)
    {
        newValue = map(tmp, 2000 + MUTE_RUDDER, 4047, 0, 135); // 4095;
    }
    else
    {
        newValue = 0;
    }
    if (adc.rawr - JITTER > tmp || adc.rawr + JITTER < tmp)
    {
        adc.rawr = tmp;
        adc.rudder = newValue;
        adc.newdata = true;
    }

    // tmp = analogRead(POT_SPEED);
    tmp = 0;
    for (int i = 0; i < 20; i++)
    {
        tmp += analogRead(POT_SPEED) / 20;
    }

    tmp = constrain(tmp, 47, 4047);
    if (tmp < 2000 - MUTE)
    {
        newValue = map(tmp, 47, 2000 - MUTE, -100, 0); // 4095;
    }
    else if (tmp > 2000 + MUTE)
    {
        newValue = map(tmp, 2000 + MUTE, 4047, 0, 100); // 4095;
    }
    else
    {
        newValue = 0;
    }

    if (adc.raws - JITTER > tmp || adc.raws + JITTER < tmp)
    {
        adc.raws = tmp;
        adc.speed = newValue;
        adc.newdata = true;
    }
    tmp = analogRead(SWITCH_PIN_REMOTE_IDLE_LOCK);
    if (tmp < 1500)
    {
        sw_pos = SW_LEFT;
    }
    else if (tmp > 3000)
    {
        sw_pos = SW_RIGHT;
    }
    else
    {
        sw_pos = SW_MID;
    }

    // Serial.printf("Dir raw:%04d converted:%04d,Speed Raw:%04d converted:%04d\r\n", adc.rawr, adc.rudder, adc.raws, adc.speed);
}