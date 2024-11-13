#ifndef BASEWIFI_H_
#define BASEWIFI_H_
#include "main.h"

extern QueueHandle_t udpOut;
extern QueueHandle_t udpIn;


unsigned long initwifiqueue(void);
void WiFiTask(void *arg);

#endif /* BASEWIFI_H_ */
