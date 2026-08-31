#include <WiFi.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <AsyncUDP.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include "main.h"
#include "topwifi.h"
#include "gpscalib.h"
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

void mancalNoteTable(const float *table, bool inEffect)
{
    // The Sub answers with the table that is IN EFFECT, not the one in NVS. With its correction
    // switched off that is the identity table, indistinguishable from a genuinely uncalibrated
    // buoy unless it says so - which it now does. Latching it would report a calibrated buoy as
    // having no corrections at all.
    if (!inEffect)
    {
        printf("MANCAL: ignoring table - the Sub reports its correction is OFF, so this is the "
               "identity table and not the stored one\r\n");
        return;
    }
    for (int i = 0; i < 8; i++) mancalTable[i] = table[i];
    mancalTableValid = true;
    printf("MANCAL: table from Sub:");
    for (int i = 0; i < 8; i++) printf(" %.1f", mancalTable[i]);
    printf("\r\n");
}

// Sends the Sub a SETUPDATA carrying everything we already hold for it, with only the harmonic
// flag changed. Dialing has to happen with the correction OFF or the table would be applied on top
// of the very measurement being taken; leaving the screen puts it back ON, because a Sub left with
// it off reports the identity table to everything that asks and its stored calibration goes
// invisible.
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

static void mancalSetHarmonic(bool on)
{
    RoboStruct msg = mainData;
    msg.IDs = 0x99;
    msg.IDr = mainData.mac;
    msg.cmd = SETUPDATA;
    msg.ack = SET;
    msg.interpEnabled = on;
    mainData.interpEnabled = on;
    xQueueSend(udpIn, (void *)&msg, 10);
}

// Asks the Sub for the table it is running. Must be sent while the correction is still ON: the Sub
// answers with the table IN EFFECT, so with it off the reply is the identity table and every
// stored correction reads as zero.
void mancalSessionService()
{
    if (!mancalActive) return;
    if (millis() - mancalLastPing <= MANCAL_TOP_TIMEOUT_MS) return;

    mancalActive = false;
    printf("MANCAL: no contact from the page for %lu ms - ending session, correction back on\r\n",
           MANCAL_TOP_TIMEOUT_MS);
    mancalSetHarmonic(true);
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
        json += "\"harmonicEnabled\":\"" + String(mainData.interpEnabled ? "true" : "false") + "\",";
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
        // Progress line for the GPS Fourier calibration; empty string whenever no run is active.
        json += "\"CalibMsg\":\"" + String(gpsCalibProgress()) + "\",";
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
            json += "\"harmonicEnabled\":\"" + String(buoyPara[i].interpEnabled ? "true" : "false") + "\",";
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
        // Set by the still-water variant of the GPS Fourier command below. Kept out of
        // the query string so the mode can never be half-specified.
        bool gpsCalStill = false;

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
                // Switch the correction ON first, then read. Unconditionally: the Sub answers
                // with the table that is IN EFFECT, so on a buoy whose correction is already off
                // the reply is the identity table and every stored correction reads as zero. The
                // table itself is untouched in the Sub's NVS either way - only the reporting of
                // it depends on the flag. Ordering holds because both frames take the same path
                // to the Sub and its serial handler processes them in turn.
                mancalSessionBegin();
                mancalTableValid = false;
                mancalSetHarmonic(true);
                mancalRequestTable();
                mainData.status = IDLING;
                server.send(200, "text/plain", "OK");
                return;
            }
            if (cmdStr == "MANCAL_HARMONIC_OFF")
            {
                mancalSessionPing();
                // Second half of entry, sent by the page once the table has come back.
                mancalSetHarmonic(false);
                server.send(200, "text/plain", "OK");
                return;
            }
            if (cmdStr == "MANCAL_POINT")
            {
                mancalSessionPing();
                int idx = server.arg("index").toInt();
                float corr = server.arg("corr").toFloat();
                if (idx < 0 || idx > 7) { server.send(400, "text/plain", "Invalid index"); return; }

                float target = (float)(idx * 45) - corr;
                while (target < 0.0f) target += 360.0f;
                while (target >= 360.0f) target -= 360.0f;
                mancalTable[idx] = target;

                // Pivot on the spot. handleTimerRoutines() repeats this to the Sub every 500 ms
                // for as long as we stay in REMOTE, so the order does not have to be resent here.
                mainData.tgDir = target;
                mainData.tgSpeed = 0;
                mainData.tgDist = 0;
                mainData.status = REMOTE;
                server.send(200, "application/json",
                            "{\"index\":" + String(idx) + ",\"target\":" + String(target, 1) + "}");
                return;
            }
            if (cmdStr == "MANCAL_NORTH")
            {
                mancalSessionPing();
                // The Sub owns compassOffset, and it already knows how to fold its current heading
                // into it (case SET_AS_NORTH). Doing it there rather than here means the arithmetic
                // uses the heading the Sub actually has, not our copy of it.
                RoboStruct msg = mainData;
                msg.IDs = 0x99;
                msg.IDr = mainData.mac;
                msg.cmd = (msg_t)SET_AS_NORTH;
                msg.ack = INF;
                xQueueSend(udpIn, (void *)&msg, 10);
                mancalTable[0] = 0.0f; // the North error now lives in the offset instead
                server.send(200, "text/plain", "OK");
                return;
            }
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
                mancalSessionEnd();
                bool save = (server.arg("save") == "1");
                if (save)
                {
                    RoboStruct msg = {};
                    msg.IDs = 0x99;
                    msg.IDr = mainData.mac;
                    msg.cmd = STORE_INTERPOLATION_TABLE;
                    msg.ack = SET;
                    for (int i = 0; i < 8; i++) msg.interpolationTable[i] = mancalTable[i];
                    xQueueSend(udpIn, (void *)&msg, 10);
                }
                // ON either way, saved or not. The Sub turns it on itself when it stores a table,
                // but not when we walk away without one.
                mancalSetHarmonic(true);
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
        else if (cmdStr == "CALIB_GPS_FOURIER") cmdEnum = GPS_FOURIER_CALIBRATE;
        else if (cmdStr == "CALIB_GPS_FOURIER_STILL") { cmdEnum = GPS_FOURIER_CALIBRATE; gpsCalStill = true; }
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
                    if (server.hasArg("harmonic")) mainData.interpEnabled = (server.arg("harmonic").toInt() != 0);
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
                } else if (cmdEnum == GPS_FOURIER_CALIBRATE) {
                    msg.gpsCalStillWater = gpsCalStill;
                    msg.ack = INF;
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
                    if (server.hasArg("harmonic")) msg.interpEnabled = (server.arg("harmonic").toInt() != 0);
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
