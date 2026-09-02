#ifndef LORATOP_H_
#define LORATOP_H_
#include "main.h"
#define LoRa_frequency 433E6

extern QueueHandle_t loraOut;
extern QueueHandle_t loraIn;

void initloraqueue(void);
void LoraTask(void *arg);

// LoRa link quality - what this node hears and how well, see loratop.cpp.
void linkReportService(void);
String linkReportJson(void);

#endif /* LORATOP_H_ */
