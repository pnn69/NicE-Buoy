#ifndef ESC_H_
#define ESC_H_

extern QueueHandle_t escspeed;
typedef struct Message
{
    int speedbb;
    int speedsb;
} Message;

void EscTask(void* arg);

#endif /* ESC_H_ */