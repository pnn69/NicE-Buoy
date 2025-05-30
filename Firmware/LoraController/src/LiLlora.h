#ifndef LORA_H_
#define LORA_H_
#include "main.h"
#define LoRa_frequency 433E6

extern QueueHandle_t loraOut;
extern QueueHandle_t loraIn;
extern QueueHandle_t loraToMain;
extern QueueHandle_t loraOutSerial; // Queue for serial output

void initloraqueue(void);
void LoraTask(void *arg);

#endif /* LORA_H_ */
