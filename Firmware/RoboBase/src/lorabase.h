#ifndef LORABASE_H_
#define LORABASE_H_
#include "main.h"
#define LoRa_frequency 433E6

struct lorabuf
{
    unsigned long mac = -1;   // Fixed-size array for the MAC address (12 characters + null terminator)
    unsigned long macIn = -1; // Fixed-size array for the MAC address (12 characters + null terminator)
    int msg = 0;
    int ack = 0;
    int retry = 0;
    char data[MAXSTRINGLENG] = "";
};

extern QueueHandle_t loraOut;
extern QueueHandle_t loraIn;

void initloraqueue(void);
void LoraTask(void *arg);

#endif /* LORBASE_H_ */
