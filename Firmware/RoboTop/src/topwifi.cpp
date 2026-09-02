#include <WiFi.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <AsyncUDP.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include "main.h"
#include "topwifi.h"
#include "gpssim.h"
#include "leds.h"
#include "datastorage.h"
#include <esp_system.h>

// Why the Top last restarted, in plain words. Without this a buoy that "keeps crashing" is
// indistinguishable from one being power-cycled, browning out, or hitting a watchdog, and
// none of that is visible out on the water where there is no USB cable.
const char *resetReasonText()
{
    switch (esp_reset_reason())
    {
    case ESP_RST_POWERON:   return "power-on";      // cold start / power cycle
    case ESP_RST_EXT:       return "reset-pin";
    case ESP_RST_SW:        return "software";      // our own ESP.restart(): OTA or REBOOT
    case ESP_RST_PANIC:     return "PANIC";         // crash - exception or failed assert
    case ESP_RST_INT_WDT:   return "WATCHDOG-int";
    case ESP_RST_TASK_WDT:  return "WATCHDOG-task"; // a task stopped feeding the dog
    case ESP_RST_WDT:       return "WATCHDOG";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";      // supply dipped: power, not code
    case ESP_RST_DEEPSLEEP: return "deep-sleep";
    case ESP_RST_SDIO:      return "sdio";
    default:                return "unknown";
    }
}
#include "buzzer.h"
#include "loratop.h"
#include "udplog.h"
#include "sercom.h"

static int statik = IDLE;
static RoboStruct msgIdOut;
static RoboStruct topWifiIn;
static RoboStruct udpBuffer;
static RoboStruct udpBufferReceived;
static LedData wifiCollorUtil;
static bool ota = false;
static int8_t id = 0;
static char udpDataIn[MAXSTRINGLENG];
static IPAddress ipTop;
static unsigned long mac = 0;
AsyncUDP udp;
QueueHandle_t udpOut;
QueueHandle_t udpIn;
static unsigned long lastUpdMsg = 0;

//***************************************************************************************************
//      Network health instrumentation
//***************************************************************************************************
// A Top that "goes offline while the button still works" tells you almost nothing on its own: the
// button task lives on core 1 and WiFiTask on core 0, so a working button only proves the chip did
// not reboot. These counters, printed once every 10 s by the main loop, separate the three ways the
// link can die - the WiFi task stopped looping, the radio lost its association, or the heap ran out
// from under the web server - which is otherwise indistinguishable from the outside.
static volatile uint32_t netLoopTicks = 0;   // ++ every WiFiTask iteration; frozen means task wedged
static volatile uint32_t netHttpReqs = 0;    // pages and /data polls actually served
static volatile uint32_t netUdpTx = 0;       // frames broadcast
static volatile uint32_t netDisconnects = 0; // STA_DISCONNECTED events since boot
static volatile int netLastReason = 0;       // reason code of the most recent one
static volatile unsigned long netLastReqMs = 0;
static TaskHandle_t netWifiTask = NULL;

static const char *wifiStatusText(wl_status_t s)
{
    switch (s)
    {
    case WL_CONNECTED:       return "CONNECTED";
    case WL_NO_SSID_AVAIL:   return "NO-SSID";
    case WL_CONNECT_FAILED:  return "FAILED";
    case WL_CONNECTION_LOST: return "LOST";
    case WL_DISCONNECTED:    return "DISCONNECTED";
    case WL_IDLE_STATUS:     return "IDLE";
    case WL_SCAN_COMPLETED:  return "SCAN-DONE";
    default:                 return "?";
    }
}

// The reason codes are the 802.11 ones from esp_wifi_types.h. 200 BEACON_TIMEOUT and 201 NO_AP_FOUND
// point at range or a router that stopped answering, 15 4WAY_HANDSHAKE_TIMEOUT at the key exchange,
// 8 ASSOC_LEAVE at the AP deliberately kicking us off. Printing the number is the whole difference
// between "the WiFi broke" and knowing which end broke it.
static void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
    switch (event)
    {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        netDisconnects++;
        netLastReason = info.wifi_sta_disconnected.reason;
        printf("[NET] STA DISCONNECTED reason=%d (drop #%lu)\r\n", netLastReason, (unsigned long)netDisconnects);
        break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        printf("[NET] STA GOT IP %s rssi=%d\r\n", WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
        break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
        printf("[NET] STA LOST IP\r\n");
        break;
    default:
        break;
    }
}

// One line of network vitals. Called from the main loop rather than from the WiFi task on purpose:
// the main loop is on core 1, so the line still gets printed when the WiFi task is the part that
// died - and then reports loop=0, which says so outright.
String netHealthLine()
{
    static uint32_t prevTicks = 0, prevReqs = 0, prevTx = 0;
    uint32_t ticks = netLoopTicks, reqs = netHttpReqs, tx = netUdpTx;
    uint32_t dTicks = ticks - prevTicks, dReqs = reqs - prevReqs, dTx = tx - prevTx;
    prevTicks = ticks; prevReqs = reqs; prevTx = tx;

    unsigned long idle = (netLastReqMs == 0) ? 0 : (millis() - netLastReqMs) / 1000UL;
    char buf[320];
    snprintf(buf, sizeof(buf),
             "[NET] up=%lus loop=%lu wifi=%s ip=%s rssi=%d drops=%lu/last=%d http=%lu/idle=%lus udp=%lu "
             "heap=%lu min=%lu maxblk=%lu stack=%lu%s",
             (unsigned long)(millis() / 1000UL),
             (unsigned long)dTicks,
             wifiStatusText(WiFi.status()),
             WiFi.localIP().toString().c_str(),
             (int)WiFi.RSSI(),
             (unsigned long)netDisconnects, netLastReason,
             (unsigned long)dReqs, idle, (unsigned long)dTx,
             (unsigned long)ESP.getFreeHeap(),
             (unsigned long)ESP.getMinFreeHeap(),
             (unsigned long)ESP.getMaxAllocHeap(),
             (unsigned long)(netWifiTask ? uxTaskGetStackHighWaterMark(netWifiTask) : 0),
             dTicks == 0 ? "   <<< WiFi TASK STALLED" : "");
    return String(buf);
}

// Our identity on the network, filled in by build_identity() below. Declared up here because
// setup_OTA() needs the mDNS hostname and runs before any of the WiFi policy code.
static char ownApSsid[24] = "";  // TOP_b7a5099c
static char mdnsHost[24] = "";   // top-b7a5099c  ->  http://top-b7a5099c.local

WebServer server(80);

/**
 * @brief Sets up Over-The-Air (OTA) update functionality.
 */
bool setup_OTA()
{
    Serial.print("SETUP OTA...");
    // Same name mDNS advertises - see the note in start_mdns(). Also drops the underscore, which
    // is not legal in a hostname label.
    ArduinoOTA.setHostname(mdnsHost);
    ArduinoOTA.onStart([]() { Serial.println("OTA Start"); });
    ArduinoOTA.onEnd([]() { Serial.println("\nOTA End"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
    ArduinoOTA.onError([](ota_error_t error) { ESP.restart(); });
    ArduinoOTA.begin();
    Serial.println("READY");
    return true;
}

//***************************************************************************************************
//      Network policy: where we live, at home and in the field
//***************************************************************************************************
// One priority list, spelled the same way on the Top, the Sub and the CYD:
//
//      NicE_WiFi  ->  Robo_WiFi  ->  our own TOP_<id> AP
//
// Home first, deliberately. The CYD only raises Robo_WiFi when it cannot see NicE_WiFi, so at the
// edge of the garden the CYD can still be on home while the buoys are not. A buoy that preferred
// Robo_WiFi would then sit alone on its own AP with the CYD one network over. Home-first converges.
static const char *HOME_SSID   = "NicE_WiFi";
static const char *HOME_PASS   = "!Ni1001100110";
static const char *FIELD_SSID  = "Robo_WiFi";
static const char *FIELD_PASS  = "geenanker";
static const char *OWN_AP_PASS = "geenanker";

static DNSServer dnsServer;      // wildcard DNS, so joining our AP opens the dashboard by itself
static bool apActive = false;    // our own AP is currently up
// Short, readable identity. espMac() already folds the last four MAC bytes into the id the logs
// call b7a5099c, and a phone's WiFi list is far easier to search for TOP_b7a5099c than for the
// full twelve-hex MAC we used to advertise.
static void build_identity()
{
    snprintf(ownApSsid, sizeof(ownApSsid), "TOP_%08lx", (unsigned long)espMac());
    // mDNS labels may not contain '_', so the hostname is spelled with a hyphen instead.
    snprintf(mdnsHost, sizeof(mdnsHost), "top-%08lx", (unsigned long)espMac());
}

// The captive portal below only works while we are the access point - we will not hijack DNS on
// NicE_WiFi or on the CYD's network. mDNS covers the other half: http://top-b7a5099c.local
// resolves in both modes, so there is always a way in that is not a memorised IP address.
static void start_mdns()
{
    // Do NOT call MDNS.begin() or MDNS.end() here. ArduinoOTA.begin() already brings the mDNS
    // responder up under this same hostname, and standing a second one next to it - or tearing
    // it down while OTA still holds it - is exactly what the CYD file already warned about:
    // "dual-allocation heap responder collisions and panics". We only add our own HTTP record
    // on top of the responder OTA created, once.
    static bool advertised = false;
    if (advertised)
    {
        return;
    }
    if (MDNS.addService("http", "tcp", 80))
    {
        advertised = true;
        Serial.printf("[WiFi] reachable as http://%s.local\r\n", mdnsHost);
    }
}

// Try one network, without scanning for it first.
//
// WiFi.scanNetworks() is the blocking form: it walks all thirteen channels and parks this task for
// seconds at a time. Now that we always run a softAP alongside, that drags the single radio off
// the AP's channel for long enough that a connected phone gives up on us, and it stalls
// server.handleClient() and ArduinoOTA.handle() with it. begin() does its own short, targeted
// probe for the one SSID we care about and simply fails on timeout when it is not there - which
// is the only thing we ever wanted out of the scan.
static bool try_connect(const char *ssid, const char *pass, uint32_t timeoutMs)
{
    Serial.printf("[WiFi] trying '%s' ... ", ssid);
    WiFi.begin(ssid, pass);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - start > timeoutMs)
        {
            Serial.println("not there");
            WiFi.disconnect();
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Associated is not the same as usable. WL_CONNECTED fires on association, but bringing up
    // mDNS or OTA while the lease is still 0.0.0.0 takes the lwIP stack - and the chip - with it.
    while (WiFi.localIP() == IPAddress(0, 0, 0, 0))
    {
        if (millis() - start > timeoutMs + 6000)
        {
            Serial.println("no DHCP lease");
            WiFi.disconnect();
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    WiFi.setSleep(WIFI_PS_NONE);
    Serial.printf("joined, IP %s\r\n", WiFi.localIP().toString().c_str());
    return true;
}

/**
 * @brief Home first, then the CYD's field AP. No scan anywhere - trying them in order gives the
 *        same answer as scanning and then deciding, without taking the radio off channel.
 */
bool connect_known_wifi(IPAddress *tmp)
{
    if (!apActive)
    {
        WiFi.mode(WIFI_STA);
    }
    if (try_connect(HOME_SSID, HOME_PASS, 8000) || try_connect(FIELD_SSID, FIELD_PASS, 8000))
    {
        *tmp = WiFi.localIP();
        return true;
    }
    Serial.println("[WiFi] neither NicE_WiFi nor Robo_WiFi in reach");
    return false;
}

/**
 * @brief Brings up our own AP, plus the captive portal that makes it usable without an IP.
 */
void setup_wifi_ap(String ap, String ww, IPAddress *tmp)
{
    // AP_STA, not AP. WIFI_AP tears the station interface down, which is exactly why a buoy that
    // fell back to its own AP used to stay there forever: it had no radio left to look for
    // Robo_WiFi with, and only a power cycle brought it back. Keeping STA alive is what lets the
    // hunt carry on underneath the AP.
    WiFi.mode(WIFI_AP_STA);

    // The AP is its own gateway. We used to advertise 192.168.1.5, where nothing answers, and
    // Android reacts to a gateway that never replies by deciding the network is broken.
    IPAddress apIP(192, 168, 4, 1);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

    // max_connection defaults to 4 in the Arduino wrapper; the ESP32 silicon allows 15.
    if (WiFi.softAP(ap.c_str(), ww.length() ? ww.c_str() : NULL, 1, 0, 8))
    {
        WiFi.setSleep(WIFI_PS_NONE);
        *tmp = WiFi.softAPIP();
        apActive = true;

        // Wildcard DNS: every lookup resolves to us, so the phone's own connectivity probe lands
        // on our web server, gets the redirect below instead of the 204 it expected, and opens
        // the "sign in to network" sheet on the dashboard without anyone typing an address.
        dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
        dnsServer.start(53, "*", apIP);

        Serial.printf("[WiFi] AP '%s' up on %s\r\n", ap.c_str(), tmp->toString().c_str());
    }
    start_mdns();
}

/**
 * @brief Folds our own AP away once we are on a real network again.
 */
static void stop_own_ap()
{
    if (!apActive)
    {
        return;
    }
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    apActive = false;
    Serial.println("[WiFi] own AP folded away, we are on a real network now");
    start_mdns();
}

/**
 * @brief Retrieves the device's MAC address as an unsigned long.
 */
unsigned long espMac(void)
{
    byte macarr[6];
    if (mac == 0)
    {
        WiFi.macAddress(macarr);
        for (int i = 2; i < 6; i++)
        {
            mac = (mac << 8) | macarr[i];
        }
    }
    return mac;
}

/**
 * @brief Initializes the UDP input and output queues.
 */
unsigned long initwifiqueue(void)
{
    udpOut = xQueueCreate(10, sizeof(RoboStruct));
    udpIn = xQueueCreate(10, sizeof(RoboStruct));
    return espMac();
}

/**
 * @brief Initializes the Async UDP listener.
 */
bool udp_setup(int poort)
{
    if (udp.listen(poort))
    {
        udp.onPacket([](AsyncUDPPacket packet)
                     {
            String s((const char*)packet.data(), packet.length());
            if (s.startsWith("$")) {
                RoboStruct udpDataIn = {};
                rfDeCode(s, &udpDataIn);
                if (udpDataIn.IDs != -1 && udpDataIn.IDs != espMac()) {
                    xQueueSend(udpIn, (void *)&udpDataIn, 0);
                }
            } });
        return true;
    }
    return false;
}

/**
 * @brief Main Wi-Fi management task.
 */
//***************************************************************************************************
//  MAN CAL working copy - see topwifi.h
//***************************************************************************************************
static float mancalTable[8] = {0, 45, 90, 135, 180, 225, 270, 315};
static volatile bool mancalTableValid = false;

// ---- guided eight point calibration, mirrored from the Sub ----------------------------------
// Read-only as far as this Top is concerned. Every value here arrived in a CAL8_SESSION frame; the
// page renders it and nothing more, so it cannot get a step out of step with the hardware.
static volatile bool cal8Valid = false;   // the Sub has answered at least once
static volatile bool cal8Active = false;
static volatile int cal8Next = 0;
static volatile uint8_t cal8Mask = 0;     // bit i set once direction i has been captured
static volatile uint16_t cal8Seq = 0;     // serial of the last press the SUB applied
static float cal8Captured[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void cal8NoteState(bool active, int next, const float *captured, uint8_t mask, uint16_t seq)
{
    cal8Active = active;
    cal8Next = next;
    cal8Mask = mask;
    cal8Seq = seq;
    for (int i = 0; i < 8; i++) cal8Captured[i] = captured[i];
    cal8Valid = true;
    printf("CAL8: session %s, asking for %d, mask %02X, seq %u, captured:",
           active ? "running" : "idle", next, mask, seq);
    for (int i = 0; i < 8; i++) printf(" %.1f", cal8Captured[i]);
    printf("\r\n");
}

int cal8NextLeg(void) { return cal8Next; }

// One frame per press. ack SET carries an action; ack GET just asks. Either way the Sub answers
// with the state, which comes back through cal8NoteState().
//
// leg and seq are only meaningful for CAL8_SET. leg names the direction the press is for; seq
// numbers the press so the Sub can tell a resend from a new one. See cal8Service().
static void cal8Send(int ackKind, int action, int leg, uint16_t seq)
{
    RoboStruct msg = {};
    // Straight onto the serial link, NOT via udpIn. Every other command here goes through udpIn and
    // lets the RF handler relay it, but that handler now intercepts calibration SETs and routes them
    // back through cal8NotePress() so they get retried - so a SET posted to udpIn came straight back
    // to us, re-armed the pending press, and went round again without ever reaching the wire.
    msg.IDs = espMac();
    msg.IDr = BUOYIDALL;
    msg.cmd = CAL8_SESSION;
    msg.ack = ackKind;
    msg.cal8Action = action;
    msg.cal8Next = leg;
    msg.cal8Seq = seq;
    if (xQueueSend(serOut, (void *)&msg, pdMS_TO_TICKS(250)) != pdTRUE)
    {
        printf("ERROR: Failed to queue CAL8_SESSION to serOut!\r\n");
    }
}

// ---- press retry ----------------------------------------------------------------------------
// One press in flight at a time. There is never more than one: the operator is pressing buttons on
// a screen and the next press is only offered once this one has visibly landed.
#define CAL8_RETRY_MS   400
#define CAL8_MAX_TRIES  12
static volatile int cal8PendAction = -1;
static volatile int cal8PendLeg = 0;
static volatile uint16_t cal8PendSeq = 0;
static volatile int cal8PendTries = 0;
static volatile unsigned long cal8PendNextMs = 0;

void cal8NotePress(int action, int leg, uint16_t seq)
{
    if (action == CAL8_SET)
    {
        if (seq == 0)
        {
            // Ours to number: one past the last serial the Sub reported applying. Every resend
            // below carries this same number, so the Sub applies the first copy to arrive and
            // drops the rest - which is the whole point, now that a press can name a direction
            // that is already captured and so cannot be spotted as a duplicate by its leg alone.
            seq = (uint16_t)(cal8Seq + 1);
            if (seq == 0) seq = 1;   // 0 means "unnumbered", see cal8NextSeq() on the Sub
        }
        else
        {
            // Somebody else's press, already numbered. Two retry loops are now stacked - the
            // sender resending to us, us resending to the Sub - and the serial is what stops them
            // multiplying. Drop a repeat here rather than re-arm on it.
            if (seq == cal8Seq)
            {
                printf("CAL8: press seq %u already applied by the Sub, ignoring the repeat\r\n", seq);
                return;
            }
            if (cal8PendAction == CAL8_SET && seq == cal8PendSeq)
            {
                printf("CAL8: press seq %u is already in flight, ignoring the repeat\r\n", seq);
                return;
            }
        }
    }
    else
    {
        seq = 0;   // BEGIN, SAVE and CANCEL are idempotent and carry no serial
    }

    cal8PendAction = action;
    cal8PendLeg = leg;
    cal8PendSeq = seq;
    cal8PendTries = 1;
    cal8PendNextMs = millis() + CAL8_RETRY_MS;
    cal8Send(SET, action, leg, cal8PendSeq);
    printf("CAL8: press action %d leg %d seq %u sent, waiting for the Sub to confirm\r\n",
           action, leg, cal8PendSeq);
}

// Has the Sub's reported state caught up with what we asked for? This is the only stopping
// condition - a press is not "done" because it was transmitted, only because the buoy shows it.
static bool cal8PressLanded(void)
{
    if (!cal8Valid) return false;
    switch (cal8PendAction)
    {
    case CAL8_BEGIN:  return cal8Active && cal8Next == 0 && cal8Mask == 0;
    // The serial, not the cursor. A capture that redid an earlier direction leaves the cursor
    // exactly where it was, so "the cursor has moved past the leg we sent" would never come true
    // and the press would be resent until the retries ran out - twelve more captures of a hull
    // that is by then pointing somewhere else.
    case CAL8_SET:    return cal8Active && cal8Seq == cal8PendSeq;
    case CAL8_SAVE:   return !cal8Active;   // the Sub closes the session when it commits
    case CAL8_CANCEL: return !cal8Active;
    default:          return true;
    }
}

void cal8Service(void)
{
    if (cal8PendAction < 0) return;

    if (cal8PressLanded())
    {
        printf("CAL8: press action %d leg %d seq %u confirmed after %d attempt%s\r\n",
               cal8PendAction, cal8PendLeg, cal8PendSeq, cal8PendTries,
               cal8PendTries == 1 ? "" : "s");
        cal8PendAction = -1;
        return;
    }

    if ((long)(millis() - cal8PendNextMs) < 0) return;

    if (cal8PendTries >= CAL8_MAX_TRIES)
    {
        // Give up loudly and leave the session exactly as the Sub has it. Silently dropping the
        // press would leave the screen waiting on a step that is never coming.
        printf("CAL8: press action %d leg %d seq %u NEVER confirmed after %d attempts - giving up\r\n",
               cal8PendAction, cal8PendLeg, cal8PendSeq, cal8PendTries);
        cal8PendAction = -1;
        return;
    }

    cal8PendTries++;
    cal8PendNextMs = millis() + CAL8_RETRY_MS;
    cal8Send(SET, cal8PendAction, cal8PendLeg, cal8PendSeq);
}

void mancalNoteTable(const float *table, bool usable)
{
    // The Sub answers with the table that is IN EFFECT, not the one in NVS. When its table is not
    // usable - out of order, or measured under the old convention - what it reports is the identity
    // table, indistinguishable from a genuinely uncalibrated buoy unless it says so, which it does.
    // Latching it would report a calibrated buoy as having no corrections at all.
    if (!usable)
    {
        printf("MANCAL: ignoring table - the Sub reports it is not usable, so this is the "
               "identity table and not the stored one\r\n");
        return;
    }
    for (int i = 0; i < 8; i++) mancalTable[i] = table[i];
    mancalTableValid = true;
    printf("MANCAL: table from Sub:");
    for (int i = 0; i < 8; i++) printf(" %.1f", mancalTable[i]);
    printf("\r\n");
}

//***************************************************************************************************
//  MAN CAL session watchdog - see topwifi.h
//***************************************************************************************************
static volatile bool mancalActive = false;
static volatile unsigned long mancalLastPing = 0;
// Generous next to the page's own polling, so a slow link never drops a session out from under
// someone who is still standing there dialing it in.
#define MANCAL_TOP_TIMEOUT_MS (2 * 60 * 1000UL)

static void mancalSessionBegin()
{
    mancalActive = true;
    mancalLastPing = millis();
}

static void mancalSessionPing()
{
    if (mancalActive) mancalLastPing = millis();
}

static void mancalSessionEnd()
{
    mancalActive = false;
}

// Asks the Sub for the table it is running. Must be sent while the correction is still ON: the Sub
// answers with the table IN EFFECT, so with it off the reply is the identity table and every
// stored correction reads as zero.
void mancalSessionService()
{
    if (!mancalActive) return;
    if (millis() - mancalLastPing <= MANCAL_TOP_TIMEOUT_MS) return;

    mancalActive = false;
    printf("MANCAL: no contact from the page for %lu ms - stopping the hull\r\n",
           MANCAL_TOP_TIMEOUT_MS);
    // Stop turning, and nothing else. There is no correction to put back any more - the guided
    // flow never switches one off - and any captures the operator has made stay on the Sub, where
    // they cost nothing until SAVE. Walking away from a browser must not throw away work.
    mainData.tgSpeed = 0;
    mainData.tgDist = 0;
    mainData.status = IDLING;
}

static void mancalRequestTable()
{
    RoboStruct msg = {};
    msg.IDs = 0x99;
    msg.IDr = mainData.mac;
    msg.cmd = STORE_INTERPOLATION_TABLE;
    msg.ack = GET;
    xQueueSend(udpIn, (void *)&msg, 10);
}

void WiFiTask(void *arg)
{
    int wifiConfig = *(int *)arg;
    byte macarr[6];
    char macStr[20];
    WiFi.macAddress(macarr);
    sprintf(macStr, "%02x%02x%02x%02x%02x%02x", macarr[0], macarr[1], macarr[2], macarr[3], macarr[4], macarr[5]);
    
    netWifiTask = xTaskGetCurrentTaskHandle();
    WiFi.onEvent(onWiFiEvent);
    build_identity();

    IPAddress ip;
    String ap = "";
    String apww = "";
    static bool runBackgroundReconnection = false;

    if (wifiConfig == 1) {
        ap = "PAIR_ME_"; ap += macStr;
        setup_wifi_ap(ap, apww, &ip);
        runBackgroundReconnection = false;
    } else {
        wifiCollorUtil.color = CRGB::LightBlue;
        wifiCollorUtil.blink = FADE_ON;
        wifiCollorUtil.fadeAmount = 5;
        xQueueSend(ledStatus, (void *)&wifiCollorUtil, 10);
        
        if (!connect_known_wifi(&ip)) {
            // Nothing in reach - raise our own AP so there is still a way in out on the water.
            setup_wifi_ap(String(ownApSsid), String(OWN_AP_PASS), &ip);
        } else {
            start_mdns();
        }
        // Always on now, in both branches. A buoy sitting on its own AP has to keep hunting for
        // Robo_WiFi - that is the entire point of AP_STA - and a buoy that did get on the network
        // still has to cope with losing it later.
        runBackgroundReconnection = true;
    }
    
    wifiCollorUtil.color = CRGB::Black;
    wifiCollorUtil.blink = BLINK_OFF;
    xQueueSend(ledStatus, (void *)&wifiCollorUtil, 10);
    
    ota = setup_OTA();
    start_mdns();   // after OTA created the responder, not before
    udp_setup(1001);
    SPIFFS.begin(true);

    server.on("/", HTTP_GET, [](){
        netHttpReqs++;
        netLastReqMs = millis();
        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        server.sendHeader("Pragma", "no-cache");
        server.sendHeader("Expires", "-1");

        // Stream straight from SPIFFS rather than caching the ~46 KB page in a RAM String at boot.
        // File::readString() grows the String incrementally and, when a reallocation fails on a
        // fragmented heap, silently returns a SHORT string with no error: buoy 2 came up serving
        // 42351 of 46209 bytes, cut off mid-<script>, so none of the page JS ran. Streaming also
        // means a new SPIFFS upload takes effect without needing a reboot.
        File f = SPIFFS.open("/index.html", "r");
        if (!f) {
            server.send(404, "text/plain", "index.html not found on SPIFFS");
            return;
        }
        server.streamFile(f, "text/html");
        f.close();
    });

    // The MAN CAL page polls /data continuously, so it doubles as the session heartbeat:
    // while someone is watching the readout the session stays alive, and when the page goes
    // away it expires on its own instead of leaving the buoy in REMOTE.
    server.on("/data", HTTP_GET, []()
              {
        netHttpReqs++;
        netLastReqMs = millis();
        mancalSessionPing(); // heartbeat, see mancalSessionService()
        String json = "{\"buoys\":[";
        // Buoy 1
        json += "{";
        json += "\"ID\":\"" + String(mainData.mac, HEX) + "\",";
        json += "\"Status\":" + String(mainData.status) + ",";
        json += "\"Speed\":\"" + String(mainData.speedSet, 2) + "\",";
        json += "\"BB\":\"" + String(mainData.speedBb) + "\",";
        json += "\"SB\":\"" + String(mainData.speedSb) + "\",";
        json += "\"MagDir\":\"" + String(mainData.dirMag, 2) + "\",";
        // Imag - the heading before the eight point compass table and the trim. See
        // RoboStruct::imag; a calibration reads this rather than switching the correction off.
        json += "\"Imag\":\"" + String(mainData.imag, 2) + "\",";
        json += "\"TgDir\":\"" + String(mainData.tgDir, 2) + "\",";
        double local_tg_dist = mainData.tgDist;
        if (mainData.status == LOCKED || mainData.status == LOCKING || mainData.status == DOCKED || mainData.status == DOCKING) {
            local_tg_dist -= mainData.holdRad;
        }
        json += "\"TgDist\":\"" + String(local_tg_dist, 2) + "\",";
        json += "\"GpsDir\":\"" + String(mainData.gpsDir) + "\",";
        json += "\"WDir\":\"" + String(mainData.wDir, 2) + "\",";
        json += "\"WStd\":\"" + String(mainData.wStd, 2) + "\",";
        json += "\"SubVolt\":\"" + String(mainData.subAccuV, 2) + "\",";
        json += "\"SubCurr\":\"" + String(mainData.subAccuI, 2) + "\",";
        json += "\"SubPerc\":\"" + String(mainData.subAccuP) + "\",";
        // The Top runs its own battery and, unlike the Sub, does not disable the ESP32
        // brownout detector - so a dip here resets the Top while the Sub carries on.
        // Without these three that failure is invisible from the network.
        json += "\"TopVolt\":\"" + String(mainData.topAccuV, 2) + "\",";
        json += "\"TopCurr\":\"" + String(mainData.topAccuI, 2) + "\",";
        json += "\"TopPerc\":\"" + String(mainData.topAccuP) + "\",";
        json += "\"PIDI\":\"" + String(mainData.ip, 2) + "\",";
        json += "\"PIDR\":\"" + String(mainData.ir, 2) + "\",";
        json += "\"rev\":" + String(mainData.sub_status) + ",";
        json += "\"Kpr\":\"" + String(mainData.Kpr, 4) + "\",";
        json += "\"Kir\":\"" + String(mainData.Kir, 4) + "\",";
        json += "\"Kdr\":\"" + String(mainData.Kdr, 4) + "\",";
        json += "\"Kps\":\"" + String(mainData.Kps, 4) + "\",";
        json += "\"Kis\":\"" + String(mainData.Kis, 4) + "\",";
        json += "\"Kds\":\"" + String(mainData.Kds, 4) + "\",";
        json += "\"maxSpeed\":\"" + String(mainData.maxSpeed) + "\",";
        json += "\"minSpeed\":\"" + String(mainData.minSpeed) + "\",";
        json += "\"pivotSpeed\":\"" + String(mainData.pivotSpeed, 2) + "\",";
        json += "\"compassOffset\":\"" + String(mainData.compassOffset, 2) + "\",";
        json += "\"holdrad\":\"" + String(mainData.holdRad, 2) + "\",";
        json += "\"revBB\":\"" + String(mainData.revBB ? "true" : "false") + "\",";
        json += "\"revSB\":\"" + String(mainData.revSB ? "true" : "false") + "\",";
        json += "\"swap_BB_SB\":\"" + String(mainData.swap_BB_SB ? "true" : "false") + "\",";
        json += "\"compass_trim_enabled\":\"" + String(mainData.compass_trim_enabled ? "true" : "false") + "\",";
        json += "\"compass_trim\":\"" + String(mainData.compass_trim, 3) + "\",";
        // Always true: the compass table is applied unconditionally. Kept on the wire so an older
        // page still parses; the thing worth watching now is whether the table is USABLE.
        json += "\"dockAppDist\":\"" + String(mainData.dockApproachDist) + "\",";
        json += "\"dockAppDir\":\"" + String(mainData.dockApproachDir) + "\",";
        json += "\"dockToWP\":\"" + String(mainData.dockingToWaypoint ? "true" : "false") + "\",";
        json += "\"TgLat\":\"" + String(mainData.tgLat, 6) + "\",";
        json += "\"TgLng\":\"" + String(mainData.tgLng, 6) + "\",";
        json += "\"Lat\":\"" + String(mainData.lat, 6) + "\",";
        json += "\"Lng\":\"" + String(mainData.lng, 6) + "\",";
        json += "\"GpsFix\":\"" + String(mainData.gpsFix ? "true" : "false") + "\",";
        json += "\"trackPos\":" + String(mainData.trackPos) + ",";
        json += "\"SubOk\":\"" + String(subSerialAlive() ? "true" : "false") + "\",";
        // Was the GPS Fourier calibration's progress line. That run is gone; the field stays so
        // an older page still parses, and it is now always empty.
        json += "\"CalibMsg\":\"\",";
        json += "\"SimMsg\":\"" + String(gpsSimReport()) + "\",";
        // Diagnostics: seconds since boot, and what ended the previous run.
        json += "\"Uptime\":" + String(millis() / 1000) + ",";
        json += "\"Crumb\":" + String(crumbSlotAtLastReset(CRUMB_LOOP)) + ",";
        json += "\"CrumbLora\":" + String(crumbSlotAtLastReset(CRUMB_LORA)) + ",";
        json += "\"CrumbWifi\":" + String(crumbSlotAtLastReset(CRUMB_WIFI)) + ",";
        json += "\"CrumbSer\":" + String(crumbSlotAtLastReset(CRUMB_SER)) + ",";
        json += "\"StkLoop\":" + String(crumbStackFree(CRUMB_LOOP)) + ",";
        json += "\"StkLora\":" + String(crumbStackFree(CRUMB_LORA)) + ",";
        json += "\"StkWifi\":" + String(crumbStackFree(CRUMB_WIFI)) + ",";
        json += "\"StkSer\":" + String(crumbStackFree(CRUMB_SER)) + ",";
        json += "\"HeapFree\":" + String(ESP.getFreeHeap()) + ",";
        json += "\"HeapMin\":" + String(ESP.getMinFreeHeap()) + ",";
        json += "\"HeapMaxBlk\":" + String(ESP.getMaxAllocHeap()) + ",";
        json += "\"ResetReason\":\"" + String(resetReasonText()) + "\",";
        // MAN CAL working copy, so the page can show what the Sub is actually running rather than
        // a blank table. mancalValid stays false until the Sub has answered, which lets the page
        // tell "not reported yet" apart from "reported, and it is zero".
        json += "\"mancalValid\":" + String(mancalTableValid ? 1 : 0) + ",";
        json += "\"mancalTable\":[";
        for (int i = 0; i < 8; i++) json += String(mancalTable[i], 1) + (i < 7 ? "," : "");
        json += "],";
        // Guided calibration session as the SUB reports it. cal8Valid distinguishes "the Sub has
        // not answered yet" from "answered, and no session is running" - without it the page would
        // show a run that is under way as not started for as long as the first frame took.
        json += "\"cal8Valid\":" + String(cal8Valid ? 1 : 0) + ",";
        json += "\"cal8Active\":" + String(cal8Active ? 1 : 0) + ",";
        json += "\"cal8Next\":" + String(cal8Next) + ",";
        // Which directions are in. The cursor cannot say, because redoing one does not move it.
        json += "\"cal8Mask\":" + String((int)cal8Mask) + ",";
        json += "\"cal8\":[";
        for (int i = 0; i < 8; i++) json += String(cal8Captured[i], 1) + (i < 7 ? "," : "");
        json += "]";
        json += "},";

        for (int i = 1; i < 3; i++) {
            json += "{";
            json += "\"ID\":\"" + String(buoyPara[i].IDs, HEX) + "\",";
            json += "\"Status\":" + String(buoyPara[i].status) + ",";
            json += "\"Speed\":\"" + String(buoyPara[i].speedSet, 2) + "\",";
            json += "\"BB\":\"" + String(buoyPara[i].speedBb) + "\",";
            json += "\"SB\":\"" + String(buoyPara[i].speedSb) + "\",";
            json += "\"MagDir\":\"" + String(buoyPara[i].dirMag, 2) + "\",";
            json += "\"Imag\":\"" + String(buoyPara[i].imag, 2) + "\",";
            json += "\"TgDir\":\"" + String(buoyPara[i].tgDir, 2) + "\",";
            double buoy_tg_dist = buoyPara[i].tgDist;
            if (buoyPara[i].status == LOCKED || buoyPara[i].status == LOCKING || buoyPara[i].status == DOCKED || buoyPara[i].status == DOCKING) {
                buoy_tg_dist -= buoyPara[i].holdRad;
            }
            json += "\"TgDist\":\"" + String(buoy_tg_dist, 2) + "\",";
            json += "\"GpsDir\":\"" + String(buoyPara[i].gpsDir) + "\",";
            json += "\"WDir\":\"" + String(buoyPara[i].wDir, 2) + "\",";
            json += "\"WStd\":\"" + String(buoyPara[i].wStd, 2) + "\",";
            json += "\"SubVolt\":\"" + String(buoyPara[i].subAccuV, 2) + "\",";
            json += "\"SubCurr\":\"" + String(buoyPara[i].subAccuI, 2) + "\",";
            json += "\"SubPerc\":\"" + String(buoyPara[i].subAccuP) + "\",";
            json += "\"PIDI\":\"" + String(buoyPara[i].ip, 2) + "\",";
            json += "\"PIDR\":\"" + String(buoyPara[i].ir, 2) + "\",";
            json += "\"rev\":" + String(buoyPara[i].sub_status) + ",";
            json += "\"Kpr\":\"" + String(buoyPara[i].Kpr, 4) + "\",";
            json += "\"Kir\":\"" + String(buoyPara[i].Kir, 4) + "\",";
            json += "\"Kdr\":\"" + String(buoyPara[i].Kdr, 4) + "\",";
            json += "\"Kps\":\"" + String(buoyPara[i].Kps, 4) + "\",";
            json += "\"Kis\":\"" + String(buoyPara[i].Kis, 4) + "\",";
            json += "\"Kds\":\"" + String(buoyPara[i].Kds, 4) + "\",";
            json += "\"maxSpeed\":\"" + String(buoyPara[i].maxSpeed) + "\",";
            json += "\"minSpeed\":\"" + String(buoyPara[i].minSpeed) + "\",";
            json += "\"pivotSpeed\":\"" + String(buoyPara[i].pivotSpeed, 2) + "\",";
            json += "\"compassOffset\":\"" + String(buoyPara[i].compassOffset, 2) + "\",";
            json += "\"holdrad\":\"" + String(buoyPara[i].holdRad, 2) + "\",";
            json += "\"revBB\":\"" + String(buoyPara[i].revBB ? "true" : "false") + "\",";
            json += "\"revSB\":\"" + String(buoyPara[i].revSB ? "true" : "false") + "\",";
            json += "\"swap_BB_SB\":\"" + String(buoyPara[i].swap_BB_SB ? "true" : "false") + "\",";
            json += "\"compass_trim_enabled\":\"" + String(buoyPara[i].compass_trim_enabled ? "true" : "false") + "\",";
            json += "\"compass_trim\":\"" + String(buoyPara[i].compass_trim, 3) + "\",";
            json += "\"dockAppDist\":\"" + String(buoyPara[i].dockApproachDist) + "\",";
            json += "\"dockAppDir\":\"" + String(buoyPara[i].dockApproachDir) + "\",";
            json += "\"dockToWP\":\"" + String(buoyPara[i].dockingToWaypoint ? "true" : "false") + "\",";
            json += "\"TgLat\":\"" + String(buoyPara[i].tgLat, 6) + "\",";
            json += "\"TgLng\":\"" + String(buoyPara[i].tgLng, 6) + "\",";
            json += "\"Lat\":\"" + String(buoyPara[i].lat, 6) + "\",";
            json += "\"Lng\":\"" + String(buoyPara[i].lng, 6) + "\",";
            json += "\"GpsFix\":\"" + String(buoyPara[i].gpsFix ? "true" : "false") + "\",";
            json += "\"trackPos\":" + String(buoyPara[i].trackPos) + ",";
            // A remote buoy's Sub link is not observable from here - the telemetry we hold for it
            // comes from ITS Top over LoRa, and the protocol carries no sub-link flag. Report "true"
            // so SETUP stays available for remote buoys; if their Sub is mute the existing 5 s
            // "Buoy N did not send its setup" timeout in the dialog is what reports it.
            json += "\"SubOk\":\"true\"";
            json += "}";
            if (i < 2) json += ",";
        }
        json += "]}";
        server.send(200, "application/json", json); });

    server.on("/command", HTTP_GET, []()
              {
        int bid = server.arg("bid").toInt();
        String cmdStr = server.arg("cmd");
        int cmdEnum = -1;
        // Bench simulation of position and heading. Answered here and now: it changes nothing on
        // the buoy and sends nothing to the Sub, so it has no business going through the command
        // queues below. See gpssim.h.
        //   /command?cmd=GPSSIM&on=1&c0=0&a1=6&a2=4&mps=0.15
        //   /command?cmd=GPSSIM&on=0
        // ---- MAN CAL -------------------------------------------------------------------------
        // Answered here rather than falling through to the cmdEnum dispatch below: these drive the
        // Sub through several different frames each, and two of them have to be ORDERED.
        if (cmdStr.startsWith("MANCAL_"))
        {
            // Only this Top's own Sub. The frames below go straight down the serial link;
            // steering another buoy through here would need the whole exchange relayed over
            // LoRa, which is what the handheld is for.
            if (bid != 1)
            {
                server.send(400, "text/plain", "MAN CAL only drives this Top's own buoy");
                return;
            }
            if (cmdStr == "MANCAL_ENTER")
            {
                // Entry is only a heartbeat and a stop. It deliberately does NOT start a
                // calibration and does not touch the correction: a run already in progress on the
                // Sub is picked up exactly where it was left, even if it was started from the
                // buoy's own page or from the handheld.
                mancalSessionBegin();
                mancalTableValid = false;
                mancalRequestTable();
                // A plain GET, not through the retry: the page polls anyway, so a lost one costs
                // nothing, and it changes nothing on the buoy if it arrives twice.
                cal8Send(GET, CAL8_BEGIN, 0, 0);
                mainData.status = IDLING;
                server.send(200, "text/plain", "OK");
                return;
            }
            if (cmdStr == "MANCAL_BEGIN" || cmdStr == "MANCAL_SET" ||
                cmdStr == "MANCAL_SAVE" || cmdStr == "MANCAL_CANCEL")
            {
                // The four buttons of the guided run. Each is one frame; the Sub decides what it
                // means and answers with the resulting state. There is no arithmetic here on
                // purpose - see CAL8_SESSION in RoboCompute.h.
                mancalSessionPing();
                int action = cmdStr == "MANCAL_BEGIN" ? CAL8_BEGIN
                           : cmdStr == "MANCAL_SET"   ? CAL8_SET
                           : cmdStr == "MANCAL_SAVE"  ? CAL8_SAVE
                                                      : CAL8_CANCEL;
                // Through the retry, not straight down the wire. A press sent once mostly does not
                // arrive - see cal8Service().
                //
                // The leg defaults to the SUB's own cursor, so a page walking forwards need not
                // pass one; an explicit leg is how a direction that is already captured gets
                // redone. Either way the resend carries the same leg and the same serial as the
                // first attempt.
                int leg = server.hasArg("leg") ? server.arg("leg").toInt() : cal8Next;
                cal8NotePress(action, leg);
                if (action == CAL8_SAVE || action == CAL8_CANCEL)
                {
                    mainData.tgSpeed = 0;
                    mainData.tgDist = 0;
                    mainData.status = IDLING;
                }
                server.send(200, "text/plain", "OK");
                return;
            }
            // MANCAL_STEER used to live here: it pivoted the hull towards cal8Next * 45 as a
            // corrected heading so the buoy could aim itself at each direction. The directions are
            // stops on a mechanical fixture now, 45 degrees apart by construction and indexed round
            // from wherever it was aligned with Imag = 0, so a true heading of leg * 45 points
            // nowhere in particular. Turning the buoy is the fixture's job.
            if (cmdStr == "MANCAL_STOP")
            {
                mancalSessionPing();
                mainData.tgSpeed = 0;
                mainData.tgDist = 0;
                mainData.status = IDLING;
                server.send(200, "text/plain", "OK");
                return;
            }
            if (cmdStr == "MANCAL_EXIT")
            {
                // Stop turning and drop the heartbeat. A running calibration is left alone: it
                // lives on the Sub and has written nothing, so closing the tab loses no captures.
                mancalSessionEnd();
                mainData.tgSpeed = 0;
                mainData.tgDist = 0;
                mainData.status = IDLING;
                server.send(200, "text/plain", "OK");
                return;
            }
            server.send(400, "text/plain", "Unknown MANCAL command");
            return;
        }

        if (cmdStr == "GPSSIM")
        {
            bool on = (server.arg("on").toInt() != 0);
            double c0  = server.hasArg("c0")  ? server.arg("c0").toDouble()  : 0.0;
            double a1  = server.hasArg("a1")  ? server.arg("a1").toDouble()  : 6.0;
            double a2  = server.hasArg("a2")  ? server.arg("a2").toDouble()  : 4.0;
            double mps = server.hasArg("mps") ? server.arg("mps").toDouble() : 0.15;
            gpsSimSet(on, c0, a1, a2, mps);
            server.send(200, "text/plain", on ? "SIM ARMED" : "SIM DISARMED");
            return;
        }
        // printf("Web Command received: bid=%d, cmd=%s\r\n", bid, cmdStr.c_str());

        if (cmdStr == "LOCK") {
            if (bid == 1) {
                if (mainData.status == LOCKED || mainData.status == LOCKING) { mainData.status = IDLING; cmdEnum = IDLING; }
                else { mainData.status = LOCKING; cmdEnum = LOCKING; }
            } else {
                if (buoyPara[bid-1].status == LOCKED || buoyPara[bid-1].status == LOCKING) cmdEnum = IDLE;
                else cmdEnum = LOCKING;
            }
        }
        else if (cmdStr == "DOCK") {
            if (bid == 1) {
                if (mainData.status == DOCKING || mainData.status == DOCKED) { mainData.status = IDLING; cmdEnum = IDLING; }
                else { mainData.status = DOCKING; cmdEnum = DOCKING; }
            } else {
                if (buoyPara[bid-1].status == DOCKING || buoyPara[bid-1].status == DOCKED) cmdEnum = IDLE;
                else cmdEnum = DOCKING;
            }
        }
        else if (cmdStr == "SETUP" || cmdStr == "SUBSETUP" || cmdStr == "SETUPDATA") cmdEnum = SETUPDATA;
        else if (cmdStr == "IDLE") { if (bid == 1) mainData.status = IDLING; cmdEnum = IDLING; }
        else if (cmdStr == "DIRDIST") { 
            cmdEnum = DIRDIST; 
            if (bid == 1) {
                mainData.tgDir = server.arg("dir").toFloat();
                mainData.tgDist = server.arg("dist").toFloat();
                mainData.status = LOCKED;
            }
        }
        else if (cmdStr == "MAP") cmdEnum = NEWBUOYPOS;
        else if (cmdStr == "PIDRUDDER") cmdEnum = PIDRUDDERSET;
        else if (cmdStr == "PIDSPEED") cmdEnum = PIDSPEEDSET;
        else if (cmdStr == "LIMITS") cmdEnum = MAXMINPWRSET;
        else if (cmdStr == "COMPASSOFFSET") cmdEnum = STORE_COMPASS_OFFSET;
        else if (cmdStr == "CALIB_COMPASS") cmdEnum = INFIELD_CALIBRATE;
        else if (cmdStr == "MANUAL_CALIB") cmdEnum = CALIBRATE_MAGNETIC_COMPASS;
        else if (cmdStr == "CALIB_OFFSET") cmdEnum = INFIELD_OFFSET_CALIBRATE;
        else if (cmdStr == "SET_AS_NORTH") cmdEnum = SET_AS_NORTH;
        else if (cmdStr == "REBOOT") cmdEnum = REBOOT;
        else if (cmdStr == "ADAPTIVE_TRIM") cmdEnum = ADAPTIVE_TRIM;
        else if (cmdStr == "COMPUTESTART") cmdEnum = COMPUTESTART;
        else if (cmdStr == "COMPUTETRACK") cmdEnum = COMPUTETRACK;
        else if (cmdStr == "SETLOCKPOS") cmdEnum = SETLOCKPOS;

        // SETLOCKPOS carries a position, and a missing or malformed one parses to 0,0 - which is a
        // real coordinate off the coast of Africa that the buoy would dutifully set off towards.
        // Refuse it here rather than let it reach the queues.
        if (cmdEnum == SETLOCKPOS)
        {
            if (server.arg("lat").toDouble() == 0.0 || server.arg("lng").toDouble() == 0.0)
            {
                server.send(400, "text/plain", "SETLOCKPOS needs a non-zero lat and lng");
                return;
            }
        }

        // printf("Resolved cmdEnum: %d\r\n", cmdEnum);

        if (bid == 1) {
            RoboStruct msg = {};
            msg.IDs = 0x99; msg.IDr = mainData.mac;
            msg.cmd = (msg_t)cmdEnum; 
            if (cmdEnum == SETUPDATA) {
                if (server.hasArg("Kpr")) {
                    mainData.Kpr = server.arg("Kpr").toFloat();
                    mainData.Kir = server.arg("Kir").toFloat();
                    mainData.Kdr = server.arg("Kdr").toFloat();
                    mainData.Kps = server.arg("Kps").toFloat();
                    mainData.Kis = server.arg("Kis").toFloat();
                    mainData.Kds = server.arg("Kds").toFloat();
                    mainData.maxSpeed = server.arg("maxSpeed").toInt();
                    mainData.minSpeed = server.arg("minSpeed").toInt();
                    mainData.pivotSpeed = server.arg("pivotSpeed").toFloat();
                    mainData.compassOffset = server.arg("compassOffset").toFloat();
                    mainData.holdRad = server.arg("holdrad").toDouble();
                    if (mainData.holdRad < HOLD_RADIUS_MIN) mainData.holdRad = HOLD_RADIUS_MIN;
                    mainData.revBB = server.arg("revBB").toInt();
                    mainData.revSB = server.arg("revSB").toInt();
                    mainData.swap_BB_SB = server.arg("swap_BB_SB").toInt();
                    if (server.hasArg("compass_trim_enabled")) mainData.compass_trim_enabled = (server.arg("compass_trim_enabled").toInt() != 0);
                    // Owned by the Sub - the Top only relays it, so there is nothing to store here.
                    if (server.hasArg("dockAppDist")) mainData.dockApproachDist = server.arg("dockAppDist").toInt();
                    if (server.hasArg("dockAppDir")) mainData.dockApproachDir = server.arg("dockAppDir").toInt();
                    if (server.hasArg("dockToWP")) mainData.dockingToWaypoint = (server.arg("dockToWP").toInt() != 0);

                    // Only the docking approach is ours to keep. The PIDs, speed limits,
                    // compass offset and thruster wiring flags belong to the Sub; they reach it
                    // in the SETUPDATA frame built below and it stores them itself.
                    memDockApproach(&mainData, MEM_PUT);

                    msg = mainData;
                    msg.IDs = 0x99; msg.IDr = mainData.mac;
                    msg.cmd = (msg_t)cmdEnum;
                    msg.ack = SET;
                } else {
                    msg.ack = GET;
                }
            } else {
                msg = mainData; // For other commands, we might need existing state
                msg.IDs = 0x99; msg.IDr = mainData.mac;
                msg.cmd = (msg_t)cmdEnum;
                if (cmdEnum == DIRDIST) {
                    msg.tgDir = server.arg("dir").toFloat();
                    msg.tgDist = server.arg("dist").toFloat();
                    msg.ack = INF;
                } else if (cmdEnum == ADAPTIVE_TRIM) {
                    if (server.hasArg("compass_trim")) msg.compass_trim = server.arg("compass_trim").toFloat();
                    if (server.hasArg("compass_trim_enabled")) msg.compass_trim_enabled = (server.arg("compass_trim_enabled").toInt() != 0);
                    msg.ack = SET;
                } else if (cmdEnum == SETLOCKPOS) {
                    // handleRfData() case SETLOCKPOS reads exactly these two and then sets LOCKED
                    // itself, so nothing else here has to be filled in.
                    msg.tgLat = server.arg("lat").toDouble();
                    msg.tgLng = server.arg("lng").toDouble();
                    msg.ack = SET;
                } else {
                    msg.ack = INF;
                }
            }
            xQueueSend(udpIn, (void *)&msg, 10);
        } else {
            RoboStruct msg = buoyPara[bid-1]; 
            msg.IDs = 0x99;
            msg.IDr = buoyPara[bid-1].IDs;
            if (msg.IDr == 0) msg.IDr = BUOYIDALL;
            msg.cmd = (msg_t)cmdEnum;
            msg.ack = SET;

            if (cmdEnum == DIRDIST) {
                msg.tgDir = server.arg("dir").toFloat();
                msg.tgDist = server.arg("dist").toFloat();
            } else if (cmdEnum == PIDRUDDERSET) {
                msg.Kpr = server.arg("p").toFloat(); msg.Kir = server.arg("i").toFloat(); msg.Kdr = server.arg("d").toFloat();
            } else if (cmdEnum == PIDSPEEDSET) {
                msg.Kps = server.arg("p").toFloat(); msg.Kis = server.arg("i").toFloat(); msg.Kds = server.arg("d").toFloat();
            } else if (cmdEnum == MAXMINPWRSET) {
                msg.maxSpeed = server.arg("max").toInt(); msg.minSpeed = server.arg("min").toInt(); msg.pivotSpeed = server.arg("pivot").toFloat();
            } else if (cmdEnum == STORE_COMPASS_OFFSET) {
                msg.compassOffset = server.arg("offset").toFloat();
            } else if (cmdEnum == SETUPDATA) {
                if (server.hasArg("Kpr")) {
                    msg.Kpr = server.arg("Kpr").toFloat();
                    msg.Kir = server.arg("Kir").toFloat();
                    msg.Kdr = server.arg("Kdr").toFloat();
                    msg.Kps = server.arg("Kps").toFloat();
                    msg.Kis = server.arg("Kis").toFloat();
                    msg.Kds = server.arg("Kds").toFloat();
                    msg.maxSpeed = server.arg("maxSpeed").toInt();
                    msg.minSpeed = server.arg("minSpeed").toInt();
                    msg.pivotSpeed = server.arg("pivotSpeed").toFloat();
                    msg.compassOffset = server.arg("compassOffset").toFloat();
                    msg.holdRad = server.arg("holdrad").toDouble();
                    if (msg.holdRad < HOLD_RADIUS_MIN) msg.holdRad = HOLD_RADIUS_MIN;
                    msg.revBB = server.arg("revBB").toInt();
                    msg.revSB = server.arg("revSB").toInt();
                    msg.swap_BB_SB = server.arg("swap_BB_SB").toInt();
                    if (server.hasArg("compass_trim_enabled")) msg.compass_trim_enabled = (server.arg("compass_trim_enabled").toInt() != 0);
                    if (server.hasArg("dockAppDist")) msg.dockApproachDist = server.arg("dockAppDist").toInt();
                    if (server.hasArg("dockAppDir")) msg.dockApproachDir = server.arg("dockAppDir").toInt();
                    if (server.hasArg("dockToWP")) msg.dockingToWaypoint = (server.arg("dockToWP").toInt() != 0);
                    msg.ack = SET;
                } else {
                    msg.ack = GET;
                }
            } else if (cmdEnum == ADAPTIVE_TRIM) {
                if (server.hasArg("compass_trim")) msg.compass_trim = server.arg("compass_trim").toFloat();
                if (server.hasArg("compass_trim_enabled")) msg.compass_trim_enabled = (server.arg("compass_trim_enabled").toInt() != 0);
                msg.ack = SET;
            } else if (cmdEnum == SETLOCKPOS) {
                msg.tgLat = server.arg("lat").toDouble();
                msg.tgLng = server.arg("lng").toDouble();
            }

            // A command for another buoy has to leave this buoy over BOTH transports: the target
            // may be on WiFi, reachable only over LoRa, or both.
            // Do NOT route this via udpIn -- handleRfData() then sees from_udp == true and bridges
            // it to LoRa only, so a WiFi-connected buoy never receives it.
            xQueueSend(udpOut, (void *)&msg, 10);
            xQueueSend(loraOut, (void *)&msg, 10);
        }
        server.send(200, "text/plain", "OK");
    });

    // Anything we do not recognise goes to the dashboard. Together with the wildcard DNS this is
    // what makes the phone's captive-portal sheet open on our page the moment you join TOP_<id>;
    // on a real network it just means a stray URL still lands somewhere useful.
    server.onNotFound([]() {
        // Only hijack unknown URLs while we are the access point - that is what makes the phone's
        // captive-portal sheet open the dashboard. On a real network a 404 must stay a 404, or a
        // mistyped fetch quietly comes back as the whole dashboard instead of an error.
        if (apActive) {
            server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
            server.send(302, "text/plain", "");
        } else {
            server.send(404, "text/plain", "Not found");
        }
    });

    server.begin();
    if (udpOut != NULL) {
        xQueueReset(udpOut);
    }
    
    static unsigned long lastBackgroundConnectAttempt = 0;
    static int connectAttemptState = 0; // 0 = idle, 1 = trying NicE_WiFi, 2 = trying Robo_WiFi
    static unsigned long connectionStartedTime = 0;

    for (;;) {
        crumbAt(CRUMB_WIFI, 400);
        netLoopTicks++;
        server.handleClient();
        crumbAt(CRUMB_WIFI, 401);
        if (ota) ArduinoOTA.handle();
        
        // Serve the wildcard DNS that drives the captive portal. Cheap, and only while we are
        // actually the access point.
        if (apActive) {
            dnsServer.processNextRequest();
        }

        // Field-aware background hunt for a real network, running underneath our own AP.
        if (runBackgroundReconnection) {
            unsigned long current_time = millis();

            if (apActive && WiFi.softAPgetStationNum() > 0) {
                // Somebody is on our AP. Stop hunting: with one radio we would have to leave the
                // AP's channel to do it, and we are not knocking a phone off mid-session - being
                // reachable right now beats being reachable on a better network in a minute.
                if (connectAttemptState != 0) {
                    Serial.println("[WiFi] client on our AP - holding off, AP stability wins");
                    WiFi.disconnect();
                    connectAttemptState = 0;
                }
                lastBackgroundConnectAttempt = current_time;
            }
            else if (WiFi.status() != WL_CONNECTED) {
                if (connectAttemptState == 0) {
                    if (current_time - lastBackgroundConnectAttempt > 45000) {
                        lastBackgroundConnectAttempt = current_time;
                        connectAttemptState = 1;
                        connectionStartedTime = current_time;
                        Serial.printf("[WiFi] hunting '%s'...\r\n", HOME_SSID);
                        WiFi.begin(HOME_SSID, HOME_PASS);
                    }
                } else if (current_time - connectionStartedTime > 15000) {
                    if (connectAttemptState == 1) {
                        connectAttemptState = 2;
                        connectionStartedTime = current_time;
                        Serial.printf("[WiFi] no '%s', hunting '%s'...\r\n", HOME_SSID, FIELD_SSID);
                        WiFi.begin(FIELD_SSID, FIELD_PASS);
                    } else {
                        connectAttemptState = 0;
                        WiFi.disconnect();
                        // Neither network in reach. Out on the water that is simply normal, so we
                        // keep looking - but raise our own AP first, so you can still get at us.
                        if (!apActive) {
                            IPAddress apIp;
                            setup_wifi_ap(String(ownApSsid), String(OWN_AP_PASS), &apIp);
                        }
                    }
                }
            }
            else {
                if (connectAttemptState != 0) {
                    Serial.printf("[WiFi] joined '%s' as %s\r\n",
                                  WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
                    WiFi.setSleep(WIFI_PS_NONE);
                    connectAttemptState = 0;
                    start_mdns();
                }
                // On a real network with nobody using our AP: fold it away (see stop_own_ap).
                if (apActive && WiFi.softAPgetStationNum() == 0) {
                    stop_own_ap();
                }
            }
        }

        RoboStruct msgIdOut = {};
        if (xQueueReceive(udpOut, (void *)&msgIdOut, 1) == pdTRUE) {
            if (msgIdOut.IDs == 0) msgIdOut.IDs = espMac();
            if (msgIdOut.IDr == 0) msgIdOut.IDr = ROBOBASE;
            if (msgIdOut.cmd == TOPDATA || msgIdOut.cmd == DIRDIST || msgIdOut.cmd == LOCKED || msgIdOut.cmd == DOCKED) {
                if (msgIdOut.status == LOCKED || msgIdOut.status == LOCKING || msgIdOut.status == DOCKED || msgIdOut.status == DOCKING) {
                    msgIdOut.tgDist -= msgIdOut.holdRad;
                }
            }
            crumbAt(CRUMB_WIFI, 402);
            udp.broadcast(rfCode(&msgIdOut).c_str());
            crumbAt(CRUMB_WIFI, 403);
            netUdpTx++;
        }
        delay(1);
    }
}
