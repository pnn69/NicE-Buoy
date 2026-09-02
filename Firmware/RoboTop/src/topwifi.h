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
void mancalNoteTable(const float *table, bool usable);

// ---------------------------------------------------------------------------------------------
// Guided eight point calibration. The Top holds no state of its own here beyond a cache for its
// web page: the session lives on the Sub - see the block comment in RoboSub/src/compass.cpp - and
// the page presses its buttons over CAL8_SESSION frames. That is the whole point. The Top used to
// keep its own eight-entry working copy and its own idea of the arithmetic, which is one of the
// five copies that let a corrected heading end up stored as a raw one.
//
// Called from handleSerialData() for every CAL8_SESSION frame the Sub sends up.
// ---------------------------------------------------------------------------------------------
void cal8NoteState(bool active, int next, const float *captured, uint8_t mask, uint16_t seq);

// A calibration press, from this Top's page or from any client over LoRa or UDP, held until the Sub
// confirms it took. The serial link down to the Sub is a single half-duplex wire the Sub talks on
// continuously, and it drops most of what is sent: eight presses one second apart landed twice on
// the bench. Every interface would otherwise need its own retry loop, so the retry lives here,
// where the lossy hop is - and CAL8_SET names the leg it means, which is what makes repeating it
// safe. See CAL8_SESSION in RoboCompute.h.
// One press of the guided run, sent to the Sub and resent until the Sub shows it landed.
//
// seq is the press serial. Pass 0 for a press that originated HERE - this Top's own page - and one
// will be allocated. Pass the sender's own serial for a press that arrived over the air, so that
// the sender's retries and this Top's retries do not multiply: a remote press already carries a
// number, and renumbering each repeat would turn every one of them into a fresh capture.
void cal8NotePress(int action, int leg, uint16_t seq = 0);

// The direction the guided run is asking for next, as the SUB reports it. The page passes an
// explicit leg for a redo of an already captured direction, and this for a plain forward step.
int  cal8NextLeg(void);

// Called from the main loop. Resends the pending press until the Sub's reported state shows it
// landed, then stops. Gives up after CAL8_MAX_TRIES rather than hammering an unreachable buoy.
void cal8Service(void);

// Called from the main loop. A MAN CAL session driven from this Top's web page leaves the buoy in
// REMOTE with its harmonic correction switched off; if the browser goes away - crash, sleeping tab,
// WiFi drop - nothing else would ever put either back. This expires the session and does it.
void mancalSessionService();

unsigned long espMac(void);
unsigned long initwifiqueue(void);
void udpSend(String data);
void WiFiTask(void *arg);

#endif /* TOPWIFI_H_ */
