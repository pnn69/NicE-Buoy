#ifndef LORATOP_H_
#define LORATOP_H_
#include "main.h"
#define LoRa_frequency 433E6

// Two transmit classes, strictly ordered. Do not queue onto these directly - loraSend() decides
// which one a frame belongs in, and that decision is the whole point. See loratop.cpp.
extern QueueHandle_t loraOutHi;
extern QueueHandle_t loraOutLo;
extern QueueHandle_t loraIn;

void initloraqueue(void);
void LoraTask(void *arg);

// The one door onto the radio. Commands - IDLE/LOCK/DOCK, a waypoint, an ACK - jump ahead of
// telemetry and are never dropped to make room for it. Telemetry is overwritten rather than
// queued, and yields the channel whenever a command wants it.
bool loraSend(const RoboStruct *msg);

// What the airtime governor is currently doing, as JSON, for /data and the debug log.
String loraAirJson(void);

// LoRa link quality - what this node hears and how well, see loratop.cpp.
void linkReportService(void);
String linkReportJson(void);

#endif /* LORATOP_H_ */
