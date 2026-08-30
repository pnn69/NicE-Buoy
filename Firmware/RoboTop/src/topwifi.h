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

// ---------------------------------------------------------------------------------------------
// MAN CAL - manual Fourier compass calibration driven from this Top's web page.
//
// The Top owns no compass table of its own; the eight entries live in the Sub's NVS. This is the
// Top's working copy for the duration of a session: seeded from the Sub's answer to a
// STORE_INTERPOLATION_TABLE GET, edited by the page, and sent back as a SET when SAVE is pressed.
//
// mancalNoteTable() is called from handleSerialData() for every table frame the Sub sends up, so
// the copy tracks whatever the Sub really has rather than what we last asked for.
// ---------------------------------------------------------------------------------------------
void mancalNoteTable(const float *table, bool inEffect);

// Called from the main loop. A MAN CAL session driven from this Top's web page leaves the buoy in
// REMOTE with its harmonic correction switched off; if the browser goes away - crash, sleeping tab,
// WiFi drop - nothing else would ever put either back. This expires the session and does it.
void mancalSessionService();

unsigned long espMac(void);
unsigned long initwifiqueue(void);
void udpSend(String data);
void WiFiTask(void *arg);

#endif /* TOPWIFI_H_ */
