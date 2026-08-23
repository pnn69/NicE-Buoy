#ifndef UDPLOG_H_
#define UDPLOG_H_
#include <Arduino.h>

// Live debug channel. Every line is broadcast as plain text on UDP port 1002 - a different port
// from the 1001 protocol traffic on purpose, so debug output can never be mistaken for a frame.
void udpLogBegin(const char *tag);
void udpLog(const char *fmt, ...);

// Breadcrumb that survives a panic. RTC_NOINIT memory is not cleared by a reset, so the value
// left behind by a crash is still readable on the next boot - which is the one thing the UDP
// stream cannot give us, since the packet in flight when the chip dies is simply lost.
// One crumb per task. A single shared crumb was useless: whichever task ran last overwrote it,
// and the loop task - which is parked on a queue most of the time - always won. With a slot each,
// a panic tells us where EVERY task was, so the one that is somewhere unexpected is the suspect.
#define CRUMB_LOOP 0
#define CRUMB_LORA 1
#define CRUMB_WIFI 2
#define CRUMB_SER  3
#define CRUMB_SLOTS 4

void crumbAt(uint8_t slot, uint32_t v);

// Free stack words left for the task that owns this slot, and a one-line heap/stack summary.
// Every task registers itself the first time it drops a crumb, so this needs no plumbing through
// the task-creation code.
uint32_t crumbStackFree(uint8_t slot);
void udpLogHealth(void);
uint32_t crumbSlotAtLastReset(uint8_t slot);

static inline void crumb(uint32_t v) { crumbAt(CRUMB_LOOP, v); }
uint32_t crumbAtLastReset(void);

#endif /* UDPLOG_H_ */
