#ifndef ESC_H_
#define ESC_H_
#include "fastled.h"

extern QueueHandle_t escspeed;
typedef struct Message
{
    int speedbb;
    int speedsb;
    CRGB ledstatus[2];
} Message;

void EscTask(void *arg);

#endif /* ESC_H_ */
