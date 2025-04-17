#ifndef SERCOM_H_
#define SERCOM_H_
#include "main.h"

extern QueueHandle_t serOut;
extern QueueHandle_t serIn;

void initserqueue(void);
void SercomTask(void *arg);

#endif /* SERCOM_H_ */
