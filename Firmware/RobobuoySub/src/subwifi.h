#ifndef SUBWIFI_H_
#define SUBWIFI_H_

extern QueueHandle_t udpOut;
extern QueueHandle_t udpIn;

#define MAXSTRINGLENG 150

unsigned long initwifiqueue(void);
void WiFiTask(void *arg);

#endif /* SUBWIFI_H_ */
