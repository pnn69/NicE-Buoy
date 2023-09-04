#ifndef INDICATOR_H_
#define INDICATOR_H_
#include <FastLED.h>

extern QueueHandle_t indicatorqueSp;
extern QueueHandle_t indicatorqueSt;

typedef struct LMessage
{
    int speedbb;
    int speedsb;
} MessageSP;

typedef struct SMessage
{
    CRGB ledstatus;
    CRGB ledstatusbb;
    CRGB ledstatussb;
} MessageST;

void IndicatorTask(void *arg);

#endif /* INDICATOR_H_ */
