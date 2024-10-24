#ifndef LORATOP_H_
#define LORATOP_H_
#include "main.h"
#define LoRa_frequency 433E6

struct lorabuf
{
    int ack = 0;
    int msg = 0;
    char data[MAXSTRINGLENG] = "";
};

extern QueueHandle_t loraOut;
extern QueueHandle_t loraIn;

void initloraqueue(void);
void LoraTask(void *arg);

#endif /* LORATOP_H_ */
