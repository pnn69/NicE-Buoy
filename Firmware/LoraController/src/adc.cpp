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
    //*************************************************************************************
    // Course
    //************************************************************************************* 
    int tmp, newValue;
    tmp = 0;
    for (int i = 0; i < 10; i++)
    {
        tmp += analogRead(POT_RUDDER);
    }
    tmp = tmp / 10; // average over 20 samples
    adc->heading = constrain(map(tmp, 0, 4095, 0, 360), 360, 0); // 4095;
  
    //*************************************************************************************
    // Speed  
    //************************************************************************************* 
    tmp = 0;
    for (int i = 0; i < 20; i++)
    {
        tmp += analogRead(POT_SPEED);
    }
    tmp = tmp / 20; // average over 20 samples
    if (tmp < 4095/2 -700)
    {
        newValue = map(tmp, 0,  4095/2 -700, 0, -100); // 4095;
    }
    else if (tmp >  4095/2 -200 + 700)
    {
        newValue = map(tmp, 4095/2 +700,  4094, 100, 0); // 4095;
    }
    else
    {
        newValue = 0;
    }

    if (adc->raws - JITTER > tmp || adc->raws + JITTER < tmp)
    {
        adc->raws = tmp;
        adc->speed = newValue;
        adc->newdata = true;
    }
    tmp = analogRead(SWITCH_PIN_REMOTE_IDLE_LOCK);
    if (tmp < 1500)
    {
        adc->swPos = SW_LEFT;
    }
    else if (tmp > 3000)
    {
        adc->swPos = SW_RIGHT;
    }
    else
    {
        adc->swPos = SW_MID;
    }
    //Serial.printf("Dir raw:%04d converted:%04d,Speed Raw:%04d converted:%04d\r\n", adc->rawr, adc->rudder, adc->raws, adc->speed);
}