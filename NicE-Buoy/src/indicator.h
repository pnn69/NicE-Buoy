#ifndef INDICATOR_H_
#define INDICATOR_H_
#include <FastLED.h>

extern QueueHandle_t indicatorqueSp;
extern QueueHandle_t indicatorqueSt;
extern QueueHandle_t indicatorqueBb;
extern QueueHandle_t indicatorqueSb;

typedef struct LMessage
{
    int speedbb;
    int speedsb;
} MessageSP;

typedef struct QStatus
{
    CRGB ledstatus;
} MessageSq;


void IndicatorTask(void *arg);

#endif /* INDICATOR_H_ */
