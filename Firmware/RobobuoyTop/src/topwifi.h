#ifndef TOPWIFI_H_
#define TOPWIFI_H_
#include "main.h"
extern QueueHandle_t udpOut;
extern QueueHandle_t udpIn;
struct UdpData
{
    unsigned char adress[4];
    unsigned int port;
    int msgId;
    char msg[MAXSTRINGLENG];
};

unsigned long initwifiqueue(void);
void udpSend(String data);
void WiFiTask(void *arg);

#endif /* TOPWIFI_H_ */
