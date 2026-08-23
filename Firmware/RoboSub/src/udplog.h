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
void crumb(uint32_t v);
uint32_t crumbAtLastReset(void);

#endif /* UDPLOG_H_ */
