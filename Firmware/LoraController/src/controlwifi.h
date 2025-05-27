#ifndef CONTROLWIFI_H_
#define CONTROLWIFI_H_
#include "main.h"
extern QueueHandle_t udpOut;
extern QueueHandle_t udpIn;

unsigned long espMac(void);
unsigned long initwifiqueue(void);
void udpSend(String data);
void WiFiTask(void *arg);

#endif /* CONTROLWIFI_H_ */
