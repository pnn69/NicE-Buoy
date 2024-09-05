#ifndef SUBWIFI_H_
#define SUBWIFI_H_

extern QueueHandle_t udpOut;
typedef struct UdpMsg
{
    unsigned char adress[4];
    unsigned int port;
    char msg[100];
} UdpData;

bool initwifiqueue(void);
void WiFiTask(void *arg);

#endif /* SUBWIFI_H_ */
