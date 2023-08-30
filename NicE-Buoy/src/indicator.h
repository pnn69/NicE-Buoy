#ifndef INDICATOR_H_
#define INDICATOR_H_
#include <FastLED.h>

extern QueueHandle_t indicatorque;
extern QueueHandle_t indicatorqueSt;
extern QueueHandle_t statusque;

typedef struct LMessage
{
    int speedbb;
    int speedsb;
    CRGB ledstatus;
} LMessage;

typedef struct SMessage
{
    CRGB ledstatus;
} SMessage;


void IndicatorTask(void *arg);

#endif /* INDICATOR_H_ */
