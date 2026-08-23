#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_attr.h>
#include "udplog.h"

#define CRUMB_MAGIC 0xC0FFEE01u

// Not cleared on reset - see the note in udplog.h.
static RTC_NOINIT_ATTR uint32_t crumbMagic;
static RTC_NOINIT_ATTR uint32_t crumbLive[CRUMB_SLOTS];
static uint32_t crumbSurvived[CRUMB_SLOTS];

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
        for (uint8_t i = 0; i < CRUMB_SLOTS; i++) crumbLive[i] = 0;
    }
    for (uint8_t i = 0; i < CRUMB_SLOTS; i++)
    {
        crumbSurvived[i] = crumbLive[i];   // where each task was when the last reset hit
        crumbLive[i] = 0;
    }
    snprintf(logTag, sizeof(logTag), "%s", tag);
    logUdp.begin(1002);
    ready = true;
    udpLog("boot - crumbs at last reset: loop=%lu lora=%lu wifi=%lu ser=%lu",
           (unsigned long)crumbSurvived[CRUMB_LOOP], (unsigned long)crumbSurvived[CRUMB_LORA],
           (unsigned long)crumbSurvived[CRUMB_WIFI], (unsigned long)crumbSurvived[CRUMB_SER]);
}

static TaskHandle_t crumbTask[CRUMB_SLOTS] = {NULL};

void crumbAt(uint8_t slot, uint32_t v)
{
    if (slot >= CRUMB_SLOTS) return;
    crumbLive[slot] = v;
    if (crumbTask[slot] == NULL)
    {
        crumbTask[slot] = xTaskGetCurrentTaskHandle();
    }
}

uint32_t crumbStackFree(uint8_t slot)
{
    if (slot >= CRUMB_SLOTS || crumbTask[slot] == NULL) return 0;
    return (uint32_t)uxTaskGetStackHighWaterMark(crumbTask[slot]);
}

// Heap and the smallest stack margin each task has ever had. A panic with every task parked at an
// idle crumb points at memory rather than control flow, and these are the numbers that say which.
void udpLogHealth(void)
{
    udpLog("health heap=%lu min=%lu maxblk=%lu stack loop=%lu lora=%lu wifi=%lu ser=%lu",
           (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(),
           (unsigned long)ESP.getMaxAllocHeap(),
           (unsigned long)crumbStackFree(CRUMB_LOOP), (unsigned long)crumbStackFree(CRUMB_LORA),
           (unsigned long)crumbStackFree(CRUMB_WIFI), (unsigned long)crumbStackFree(CRUMB_SER));
}
uint32_t crumbSlotAtLastReset(uint8_t slot) { return slot < CRUMB_SLOTS ? crumbSurvived[slot] : 0; }
uint32_t crumbAtLastReset(void) { return crumbSurvived[CRUMB_LOOP]; }

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
