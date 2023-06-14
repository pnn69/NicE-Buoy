#ifndef ADC_H_
#define ADC_H_

struct adcDataType
{
    int rudder;
    unsigned int rawr;
    int speed;
    unsigned int raws;
    bool newdata;
};

extern adcDataType adc;

void readAdc(void);

#endif /* ADC_H_ */