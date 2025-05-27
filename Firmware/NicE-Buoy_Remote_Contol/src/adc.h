#ifndef ADC_H_
#define ADC_H_

struct adcDataType
{
    int rudder;
    unsigned int rawr;
    int speed;
    unsigned int raws;
    bool newdata;
    float vbat;
};

typedef enum
{
    SW_LEFT = 1,
    SW_MID,
    SW_RIGHT
} Swtch_t;

extern adcDataType adc;
extern char sw_pos;

void readAdc(void);

#endif /* ADC_H_ */