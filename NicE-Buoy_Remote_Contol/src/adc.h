#ifndef ADC_H_
#define ADC_H_
 
 struct adcDataType
{
    unsigned int rudder;
    unsigned int speed;
    bool newdata;
};

extern adcDataType adc;

void readAdc(void); 


#endif /* ADC_H_ */