#ifndef INDICATOR_H_
#define INDICATOR_H_
#include <FastLED.h>

extern QueueHandle_t indicatorque;
extern QueueHandle_t indicatorqueSt;

typedef struct LMessage
{
    int speedbb;
    int speedsb;
    CRGB ledstatus1;
    CRGB ledstatus2;
} LMessage;

void IndicatorTask(void *arg);

#endif /* INDICATOR_H_ */
