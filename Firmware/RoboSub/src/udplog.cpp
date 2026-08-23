#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_attr.h>
#include "udplog.h"

#define CRUMB_MAGIC 0xC0FFEE01u

// Not cleared on reset - see the note in udplog.h.
static RTC_NOINIT_ATTR uint32_t crumbMagic;
static RTC_NOINIT_ATTR uint32_t crumbLive;
static uint32_t crumbSurvived = 0;

// Synchronous WiFiUDP, not the AsyncUDP the protocol uses. A debug line has to be on the wire
// before the instruction that might crash us runs, not queued behind a callback.
static WiFiUDP logUdp;
static char logTag[20] = "?";
static bool ready = false;

void udpLogBegin(const char *tag)
{
    if (crumbMagic != CRUMB_MAGIC)
    {
        // First boot after a power cycle: RTC RAM holds garbage, so there is no crumb to report.
        crumbMagic = CRUMB_MAGIC;
        crumbLive = 0;
    }
    crumbSurvived = crumbLive;   // whatever we were doing when the last reset hit
    crumbLive = 0;
    snprintf(logTag, sizeof(logTag), "%s", tag);
    logUdp.begin(1002);
    ready = true;
    udpLog("boot - crumb at last reset was %lu", (unsigned long)crumbSurvived);
}

void crumb(uint32_t v) { crumbLive = v; }
uint32_t crumbAtLastReset(void) { return crumbSurvived; }

void udpLog(const char *fmt, ...)
{
    char body[200];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);

    char line[240];
    int n = snprintf(line, sizeof(line), "%s %lu %s\n", logTag, (unsigned long)millis(), body);
    Serial.print(line);          // still goes to the console for anyone with a cable

    if (!ready || n <= 0) return;
    // 255.255.255.255 rather than a subnet broadcast, so this works unchanged whether we are a
    // station on NicE_WiFi/Robo_WiFi or sitting on our own AP.
    if (WiFi.localIP() == IPAddress(0, 0, 0, 0) && WiFi.softAPIP() == IPAddress(0, 0, 0, 0)) return;
    if (logUdp.beginPacket(IPAddress(255, 255, 255, 255), 1002))
    {
        logUdp.write((const uint8_t *)line, (size_t)n);
        logUdp.endPacket();
    }
}
