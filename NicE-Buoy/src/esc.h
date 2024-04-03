#ifndef ESC_H_
#define ESC_H_
#include "fastled.h"

extern QueueHandle_t escspeed;
typedef struct Message
{
    int speedbb;
    int speedsb;
} Message;

void InitEsc(void);
void beepESC(void);
void triggerESC(void);
void EscTask(void *arg);

#endif /* ESC_H_ */
