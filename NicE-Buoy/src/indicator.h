#ifndef INDICATOR_H_
#define INDICATOR_H_

extern QueueHandle_t indicatorque;
typedef struct LMessage
{
    int speedbb;
    int speedsb;
} LMessage;

void IndicatorTask(void* arg);

#endif /* INDICATOR_H_ */
