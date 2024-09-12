#ifndef TOPWIFI_H_
#define TOPWIFI_H_

extern QueueHandle_t udpOut;
extern QueueHandle_t udpIn;
struct UdpData
{
    unsigned char adress[4];
    unsigned int port;
    char msg[100];
};

bool initwifiqueue(void);
void WiFiTask(void *arg);

#endif /* TOPWIFI_H_ */
