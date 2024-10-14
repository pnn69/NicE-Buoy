#ifndef SUBWIFI_H_
#define SUBWIFI_H_

extern QueueHandle_t udpOut;
typedef struct UdpMsg
{
    unsigned char adress[4];
    unsigned int port;
    int msgId;
    char msg;
} UdpData;

bool initwifiqueue(void);
void WiFiTask(void *arg);

#endif /* SUBWIFI_H_ */
