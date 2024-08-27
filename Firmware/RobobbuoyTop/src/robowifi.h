#ifndef ROBOWIFI_H_
#define ROBOWIFI_H_

extern QueueHandle_t udpOut;
typedef struct UdpMsg
{
    unsigned char adress[4];
    unsigned int port;
    char msg[100];

} UdpData;
bool wifi_ap(char set);
bool initwifiqueue(void);
void WiFiTask(void *arg);

#endif /* ROBOWIFI_H_ */
