#ifndef TOPWIFI_H_
#define TOPWIFI_H_
#include "main.h"
#include <WebServer.h>

extern QueueHandle_t udpOut;
extern QueueHandle_t udpIn;
extern WebServer server;
extern RoboStruct mainData;
extern RoboStruct buoyPara[3];

// Plain-words reason for the last restart, also reported in /data as "ResetReason".
const char *resetReasonText();

// One line of network vitals for the serial log: WiFi task liveness, association state, RSSI,
// disconnect count and reason, requests served, and heap. Printed by the main loop.
String netHealthLine();

unsigned long espMac(void);
unsigned long initwifiqueue(void);
void udpSend(String data);
void WiFiTask(void *arg);

#endif /* TOPWIFI_H_ */
