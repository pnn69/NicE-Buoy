#ifndef LORATOP_H_
#define LORATOP_H_
#include "main.h"
#define LoRa_frequency 433E6

// Two transmit classes, strictly ordered. Do not queue onto these directly - loraSend() decides
// which one a frame belongs in, and that decision is the whole point. See loratop.cpp.
extern QueueHandle_t loraOutHi;
extern QueueHandle_t loraOutLo;
extern QueueHandle_t loraIn;

// ACK receipts, on their way from the radio task to the loop task.
//
// onReceive() consumes an ACK addressed to us and returns - it clears the LoRa retry table and
// the frame never reaches loraIn, which is right: an ACK is not a command and must not be
// dispatched as one. But the application above the radio has receipts of its own to retire (the
// track push in main.cpp waits on the ACK for its SETLOCKPOS), and it never got to see them.
//
// A queue rather than a direct call because onReceive() runs on LoraTask and the state being
// retired belongs to the loop task. Small on purpose: only the sender and the command are
// needed, so this is 16 bytes an entry rather than the ~500 a RoboStruct would cost.
struct LoraAckNote
{
    uint64_t from;  // who acknowledged
    int cmd;        // what they were acknowledging
};
extern QueueHandle_t loraAckIn;

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
