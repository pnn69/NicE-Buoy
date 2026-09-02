/**
 * @file subwifi.cpp
 * @brief Dashboard and WiFi Management for NicE-Buoy Sub.
 * 
 * This module is aligned directly with RoboTop's wifi implementation:
 * 1. Joins NicE_WiFi if it is in reach at boot, otherwise raises its own SUB_<id> AP and stays
 *    there - one attempt, at startup, and never again (see below).
 * 2. High-performance WebServer (Port 80) serving the dashboard.
 * 3. Lock-free, heap-allocation-free /data and /params JSON endpoints.
 * 4. Paced loop utilizing the precise RTOS delay structure.
 * 
 * Task Strategy: Runs on Core 0 to leave Core 1 free for navigation and sensors.
 */

#include <WiFi.h>
#include <AsyncUDP.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <RoboCompute.h>
#include "main.h"
#include "io_sub.h"
#include "compass.h"
#include "datastorage.h"
#include "pidrudspeed.h"
#include "subwifi.h"
#include "leds.h"
#include "buzzer.h"
#include "esc.h"

// Our identity on the network, filled in by build_identity() below. Declared up here because
// setup_OTA() needs the mDNS hostname and runs before any of the WiFi policy code.
static char ownApSsid[24] = "";  // SUB_b7a5099c
static char mdnsHost[24] = "";   // sub-b7a5099c  ->  http://sub-b7a5099c.local

// Global instances
WebServer subServer(80);
AsyncUDP udp;
extern float global_speed_bb;
extern float global_speed_sb;
extern bool icm_ready;
extern float global_hdg;
extern RoboStruct mainData;
extern SemaphoreHandle_t mainDataMutex;
extern QueueHandle_t compassIn;
extern float last_raw_x, last_raw_y, last_raw_z;
extern float hi_x, hi_y, hi_z;
extern float si_x, si_y, si_z;
extern volatile int icm_mode;
extern float pr_damping;
extern bool magRejected;
extern void updateUIHexFloat();
extern String global_cal_msg;
extern String global_cal_load, global_cal_ver;
extern uint32_t global_loop_cnt;
uint32_t global_params_rev = 0;

// Debug externs for detailed control-loop diagnostics
extern double rudderInput, rudderOutput, rampBb, rampSb, forward_ramp;
extern bool was_pure_pivot;

static RoboStruct subWifiOut;
static RoboStruct subWifiIn;
QueueHandle_t udpOut = NULL;
QueueHandle_t udpIn = NULL;
String global_mac_str = "";
static bool ota = false;
static LedData wifiCollorUtil;

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
//      Network policy: home, or our own AP - the Sub deliberately skips Robo_WiFi
//***************************************************************************************************
// The Top's list is NicE_WiFi -> Robo_WiFi -> own AP. The Sub's is one shorter:
//
//      NicE_WiFi  ->  our own SUB_<id> AP
//
// The Sub stays off Robo_WiFi on purpose, to keep the CYD's field AP for the Tops and a phone.
// It costs nothing operationally: the Sub has no LoRa at all, so its only link to the outside is
// the serial line to the Top, and the Top already forwards every command, setting and calibration
// down it. The Sub's UDP transmit has been switched off for a while too. All WiFi buys the Sub is
// its own dashboard and OTA - and its own AP serves both.
//
// Home is tried ONCE, at boot. The old build kept hunting for NicE_WiFi every 45 seconds from
// underneath the AP, and with one radio that is not free: WiFi.begin() takes the chip off the AP's
// channel for the whole association attempt, up to 15 seconds of it. From the outside the SUB_<id>
// AP simply vanished from the phone's WiFi list every three quarters of a minute and came back -
// exactly the kind of fault you cannot debug out on the water. So once we have fallen back to our
// own AP we stay on it until the next reboot: the AP is then rock steady, and getting back onto
// home is a power cycle away.
static const char *HOME_SSID   = "NicE_WiFi";
static const char *HOME_PASS   = "!Ni1001100110";
static const char *OWN_AP_PASS = "geenanker";

static DNSServer dnsServer;      // wildcard DNS, so joining our AP opens the dashboard by itself
static bool apActive = false;    // our own AP is currently up
// Short, readable identity. espMac() already folds the last four MAC bytes into the id the logs
// call b7a5099c, and a phone's WiFi list is far easier to search for SUB_b7a5099c than for the
// full twelve-hex MAC we used to advertise.
static void build_identity()
{
    snprintf(ownApSsid, sizeof(ownApSsid), "SUB_%08lx", (unsigned long)espMac());
    // mDNS labels may not contain '_', so the hostname is spelled with a hyphen instead.
    snprintf(mdnsHost, sizeof(mdnsHost), "sub-%08lx", (unsigned long)espMac());
}

// The captive portal only works while we are the access point - we will not hijack DNS on
// NicE_WiFi. mDNS covers the other half: http://sub-b7a5099c.local resolves in both modes, so
// there is always a way in that is not a memorised IP address.
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
// subServer.handleClient() and ArduinoOTA.handle() with it. begin() does its own short, targeted
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
 * @brief Home, or nothing. No scan - trying it directly gives the same answer without taking the
 *        radio off channel.
 */
bool connect_known_wifi(IPAddress *tmp)
{
    if (!apActive)
    {
        WiFi.mode(WIFI_STA);
    }
    if (try_connect(HOME_SSID, HOME_PASS, 8000))
    {
        *tmp = WiFi.localIP();
        return true;
    }
    Serial.println("[WiFi] NicE_WiFi not in reach");
    return false;
}

/**
 * @brief Brings up our own AP, plus the captive portal that makes it usable without an IP.
 */
void setup_wifi_ap(String ap, String ww, IPAddress *tmp)
{
    // AP_STA, not AP. WIFI_AP tears the station interface down, which is exactly why a buoy that
    // fell back to its own AP used to stay there forever: it had no radio left to look for home
    // with, and only a power cycle brought it back. Keeping STA alive is what lets the hunt carry
    // on underneath the AP - and underwater, hunting and finding nothing is the normal state.
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
 * @brief Settles on our own AP for good: no more association attempts, no more off-channel time.
 *
 * The station interface stays up (we remain in AP_STA) but is left idle and told not to
 * auto-reconnect, so nothing in the driver can pull the radio away from the AP's channel behind
 * our back. A reboot is what puts the Sub back on home.
 */
static void give_up_on_home()
{
    WiFi.setAutoReconnect(false);
    WiFi.disconnect();
    Serial.println("[WiFi] staying on our own AP until reboot - home is only tried at startup");
}

/**
 * @brief Retrieves the device's MAC address as an unsigned long.
 */
unsigned long espMac(void) {
    byte m[6]; WiFi.macAddress(m);
    unsigned long r=0; for(int i=2;i<6;i++) r=(r<<8)|m[i];
    return r;
}

/**
 * @brief Initializes the WiFi queues.
 */
void initwifi(void) {
    byte m[6]; WiFi.macAddress(m);
    char ms[25]; sprintf(ms, "SUB_%02X%02X%02X%02X%02X%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
    global_mac_str = String(ms);
    udpOut = xQueueCreate(10, sizeof(RoboStruct));
    udpIn = xQueueCreate(10, sizeof(RoboStruct));
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
 * @brief Core 0 Task: Manages WiFi, WebServer, and UDP communications.
 */
void WiFiTask(void *arg) {
    int wifiConfig = *(int *)arg;
    byte macarr[6];
    char macStr[20];
    WiFi.macAddress(macarr);
    sprintf(macStr, "%02x%02x%02x%02x%02x%02x", macarr[0], macarr[1], macarr[2], macarr[3], macarr[4], macarr[5]);
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
        wifiCollorUtil.blink = BLINK_SLOW;
        xQueueSend(ledStatus, (void *)&wifiCollorUtil, 10);
        
        if (!connect_known_wifi(&ip)) {
            // Nothing in reach - raise our own AP so there is still a way in out on the water,
            // and settle there. That was our one shot at home.
            setup_wifi_ap(String(ownApSsid), String(OWN_AP_PASS), &ip);
            give_up_on_home();
            runBackgroundReconnection = false;
        } else {
            start_mdns();
            // We did get on the network, and no AP of ours is up to be disturbed, so it is still
            // worth re-joining home if the link drops later. The moment that re-join fails and we
            // have to raise our own AP, the hunt stops for good - see the loop below.
            runBackgroundReconnection = true;
        }
    }
    
    wifiCollorUtil.color = CRGB::Black;
    wifiCollorUtil.blink = BLINK_OFF;
    xQueueSend(ledStatus, (void *)&wifiCollorUtil, 10);
    
    ota = setup_OTA();
    start_mdns();   // after OTA created the responder, not before
    udp_setup(1001);

    // Mount LittleFS
    if(!LittleFS.begin(true)){
        Serial.println("WiFiTask: LittleFS Mount Failed");
    } else {
        Serial.println("WiFiTask: LittleFS Mounted");
    }

    /*
     * Web Dashboard Router & Anti-Caching Strategy:
     * To prevent browsers from caching stale layouts or outdated JS dashboards, 
     * we issue HTTP/1.1 headers 'no-cache, no-store, must-revalidate' and set 'Expires: -1'.
     */
    subServer.on("/", HTTP_GET, [](){
        subServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        subServer.sendHeader("Pragma", "no-cache");
        subServer.sendHeader("Expires", "-1");
        if (LittleFS.exists("/index.html")) {
            File file = LittleFS.open("/index.html", "r");
            subServer.streamFile(file, "text/html");
            file.close();
        } else {
            subServer.send(404, "text/plain", "Index page not found on LittleFS");
        }
    });
    subServer.on("/savecal", HTTP_GET, [](){ 
        extern bool global_is_calibrating;
        global_is_calibrating = false;
        int c=34; 
        xQueueSend(compassIn,(void*)&c,10); 
        subServer.send(200,"text/plain","OK"); 
    });
    subServer.on("/save_cal", HTTP_GET, [](){
        if (subServer.hasArg("hx") && subServer.hasArg("hy") && subServer.hasArg("hz") &&
            subServer.hasArg("sx") && subServer.hasArg("sy") && subServer.hasArg("sz")) {

            float hi[3], si[3];
            hi[0] = subServer.arg("hx").toFloat();
            hi[1] = subServer.arg("hy").toFloat();
            hi[2] = subServer.arg("hz").toFloat();
            si[0] = fabs(subServer.arg("sx").toFloat());
            si[1] = fabs(subServer.arg("sy").toFloat());
            si[2] = fabs(subServer.arg("sz").toFloat());

            extern float si_matrix[3][3];
            // Handle optional full 3x3 matrix arguments
            if (subServer.hasArg("sxx") && subServer.hasArg("sxy") && subServer.hasArg("sxz") &&
                subServer.hasArg("syx") && subServer.hasArg("syy") && subServer.hasArg("syz") &&
                subServer.hasArg("szx") && subServer.hasArg("szy") && subServer.hasArg("szz")) {

                si_matrix[0][0] = fabs(subServer.arg("sxx").toFloat());
                si_matrix[0][1] = subServer.arg("sxy").toFloat();
                si_matrix[0][2] = subServer.arg("sxz").toFloat();
                si_matrix[1][0] = subServer.arg("syx").toFloat();
                si_matrix[1][1] = fabs(subServer.arg("syy").toFloat());
                si_matrix[1][2] = subServer.arg("syz").toFloat();
                si_matrix[2][0] = subServer.arg("szx").toFloat();
                si_matrix[2][1] = subServer.arg("szy").toFloat();
                si_matrix[2][2] = fabs(subServer.arg("szz").toFloat());
            } else {
                // Diagonal scale factor fallback
                si_matrix[0][0] = si[0];
                si_matrix[0][1] = 0.0f;
                si_matrix[0][2] = 0.0f;
                si_matrix[1][0] = 0.0f;
                si_matrix[1][1] = si[1];
                si_matrix[1][2] = 0.0f;
                si_matrix[2][0] = 0.0f;
                si_matrix[2][1] = 0.0f;
                si_matrix[2][2] = si[2];
            }

            // Write to NVS
            memIcmCalib(hi, si, false);

            // Update runtime variables
            hi_x = hi[0]; hi_y = hi[1]; hi_z = hi[2];
            si_x = si[0]; si_y = si[1]; si_z = si[2];

            // Re-generate user hex/text strings for telemetry
            updateUIHexFloat();

            extern bool firstHeadingRun;
            extern uint32_t lastInitTime;
            firstHeadingRun = true;
            lastInitTime = millis();

            extern bool global_is_calibrating;
            global_is_calibrating = false;

            // beep(-1) used to live here, which is the FAILURE tone - the same one played when a
            // start line cannot be computed. The comment claimed success while the buzzer said the
            // opposite. Ascending C major arpeggio instead, the same one a finished gyro
            // calibration plays, so a good save actually sounds like one.
            if (buzzer != NULL) {
                int notes[]     = {523, 659, 784, 1047};   // C5 E5 G5 C6
                int durations[] = {120, 120, 120, 350};
                int pauses[]    = {40,  40,  40,  200};
                for (int i = 0; i < 4; i++) {
                    Buzz note;
                    note.hz = notes[i];
                    note.duration = durations[i];
                    note.pause = pauses[i];
                    note.repeat = 0;
                    xQueueSend(buzzer, (void *)&note, 0);
                }
            }

            subServer.send(200, "text/plain", "OK");
        } else {
            subServer.send(400, "text/plain", "Err");
        }
    });
    subServer.on("/start_cal", HTTP_GET, [](){
        extern bool global_is_calibrating;
        global_is_calibrating = true;

        if (buzzer != NULL) {
            beep(3, buzzer); // Triple beep to signal start of calibration!
        }
        subServer.send(200, "text/plain", "OK");
    });
    subServer.on("/set_icm_mode", HTTP_GET, [](){
        if (subServer.hasArg("mode")) {
            icm_mode = subServer.arg("mode").toInt();
            // Not refused - being able to compare the modes is the whole point of this page. But
            // say so loudly: the compass table was measured on one of these variants and is being
            // applied to whichever one is selected now.
            if (!interpTableModeMatches()) {
                printf("WARNING: the compass table was measured in filtering mode %d and mode %d is "
                       "now selected. The correction is being applied to a different heading source "
                       "than it was measured on - recalibrate, or put the mode back." "\r\n",
                       interp_table_mode, icm_mode);
            }
            
            // Write to NVS
            float hi[3] = {hi_x, hi_y, hi_z};
            float si[3] = {si_x, si_y, si_z};
            memIcmCalib(hi, si, false);

            subServer.send(200, "text/plain", "OK");
        } else {
            subServer.send(400, "text/plain", "Err");
        }
    });
    subServer.on("/set-damping", HTTP_GET, [](){
        if (subServer.hasArg("sensor") && subServer.hasArg("val")) {
            String sensor = subServer.arg("sensor");
            float val = subServer.arg("val").toFloat();
            
            extern float damp_acc;
            extern float damp_gyro;
            extern float damp_mag;
            extern float damp_att;
            extern float pr_damping;

            if (sensor == "acc") {
                damp_acc = val;
            } else if (sensor == "gyro") {
                damp_gyro = val;
            } else if (sensor == "mag") {
                damp_mag = val;
            } else if (sensor == "att") {
                damp_att = val;
                // Sync attitude damping directly with pr_damping
                pr_damping = 1.0f - val;
                if (pr_damping < 0.0f) pr_damping = 0.0f;
                if (pr_damping > 0.99f) pr_damping = 0.99f;
                memPrDamping(&pr_damping, MEM_PUT);
            }

            memDampingFactors(&damp_acc, &damp_gyro, &damp_mag, &damp_att, MEM_PUT);
            Serial.printf("subServer: Saved damping coefficient: %s = %.2f\n\r", sensor.c_str(), val);
            subServer.send(200, "text/plain", "OK");
        } else {
            subServer.send(400, "text/plain", "Err");
        }
    });
    subServer.on("/set_interp_enabled", HTTP_GET, [](){
        if (subServer.hasArg("enabled")) {
            extern volatile bool interp_enabled;
            bool temp_enabled = (subServer.arg("enabled") == "1");
            interp_enabled = temp_enabled;
            
            // Save to Preferences NVS immediately
            extern void memInterpEnabled(bool *, bool);
            memInterpEnabled(&temp_enabled, MEM_PUT);
            
            subServer.send(200, "text/plain", "OK");
        } else {
            subServer.send(400, "text/plain", "Err");
        }
    });
    // /set_harmonic_point and /set_interpolation_point are gone. They wrote one table entry at a
    // time, straight into RAM, with no north rule and no ordering check, and a later save committed
    // whatever was left there. That is the free-order editing the guided run replaced - see
    // storeInterpolationTable() in compass.cpp, which is now the only way a table is written.
    // ---- Guided eight point calibration -----------------------------------------------------
    // Thin wrappers. Every rule lives in compass.cpp so the Sub's pages, the Top's page, the CYD
    // dashboard and the CYD touchscreen cannot drift apart on them.
    //
    // These were deleted by accident when the per-entry editors above were removed - they sat in
    // the same block - and the Sub answered 404 on every one of them until a live check caught it.
    subServer.on("/cal8_begin", HTTP_GET, [](){
        cal8Begin();
        subServer.send(200, "text/plain", "OK");
    });
    subServer.on("/cal8_set", HTTP_GET, [](){
        // Which direction this press is for. Defaults to the one the cursor is asking for, so a
        // page that only ever walks forwards need not pass it.
        int leg = subServer.hasArg("leg") ? subServer.arg("leg").toInt() : cal8_next;

        // Press serial 0: this page talks straight to the buoy with no lossy hop, so there is no
        // retry behind the press and nothing to de-duplicate. See cal8Set().
        int idx = cal8Set(leg, 0);
        if (idx < 0) {
            const char *why =
                (idx == CAL8_SET_NO_SESSION) ? "no calibration session is running" :
                (idx == CAL8_SET_BAD_LEG)    ? "that direction is not next, and is not captured yet" :
                (idx == CAL8_SET_DUPLICATE)  ? "repeated or out of step press" :
                (idx == CAL8_SET_OFF_ANCHOR) ? "Imag is not on zero - turn the hull until it is, then press N" :
                                               "capture refused";
            subServer.send(409, "text/plain", why);
            return;
        }
        char buf[48];
        snprintf(buf, sizeof(buf), "OK %d %.2f", idx, cal8_captured[idx]);
        subServer.send(200, "text/plain", buf);
    });
    subServer.on("/cal8_save", HTTP_GET, [](){
        if (!cal8Save()) { subServer.send(409, "text/plain", "not all eight directions captured yet"); return; }
        subServer.send(200, "text/plain", "OK");
    });
    subServer.on("/cal8_cancel", HTTP_GET, [](){
        cal8Cancel();
        subServer.send(200, "text/plain", "OK");
    });

    // Commit whatever table is in RAM, through the one door - which records the filtering mode it
    // was measured in and validates the ordering. It used to write NVS directly and skip both.
    subServer.on("/save_table", HTTP_GET, [](){
        extern float measured_angles[9];
        bool usable = storeInterpolationTable(measured_angles);
        subServer.send(200, "text/plain", usable ? "OK" : "STORED BUT NOT USABLE");
    });
    // Legacy aliases. /save_harmonic named a Fourier fit that no longer exists.
    subServer.on("/save_harmonic", HTTP_GET, [](){
        extern float measured_angles[9];
        bool usable = storeInterpolationTable(measured_angles);
        subServer.send(200, "text/plain", usable ? "OK" : "STORED BUT NOT USABLE");
    });
    subServer.on("/save_interpolation", HTTP_GET, [](){
        extern float measured_angles[9];
        storeInterpolationTable(measured_angles);
        subServer.send(200, "text/plain", "OK");
    });
    // Back to no correction at all. Goes through the same door as a real calibration, because a
    // second way of writing the table is a second set of rules waiting to drift.
    subServer.on("/reset_table", HTTP_GET, [](){
        float identity[8];
        for (int i = 0; i < 8; i++) identity[i] = i * 45.0f;
        storeInterpolationTable(identity);
        subServer.send(200, "text/plain", "OK");
    });
    // Legacy aliases
    subServer.on("/reset_harmonic", HTTP_GET, [](){
        float identity[8];
        for (int i = 0; i < 8; i++) identity[i] = i * 45.0f;
        storeInterpolationTable(identity);
        subServer.send(200, "text/plain", "OK");
    });
    subServer.on("/reset_interpolation", HTTP_GET, [](){
        float identity[8];
        for (int i = 0; i < 8; i++) identity[i] = i * 45.0f;
        storeInterpolationTable(identity);
        subServer.send(200, "text/plain", "OK");
    });
    // ---------------------------------------------------------------------------------------
    // MAN CAL - manual Fourier compass calibration, driven from this page.
    //
    // No GPS is involved anywhere in this. The buoy pivots on the spot under rudderPid() using
    // nothing but its own compass, so it works indoors, on a trailer or alongside a jetty. What it
    // DOES need is clear water or a stand, because the hull physically turns.
    //
    // The trick that keeps this simple: measured_angles[i] is by definition the heading the compass
    // reports when the hull physically points at i*45. So the heading we steer to IS
    // measured_angles[i] - dialing the correction and dialing the steer target are the same act,
    // and there is no second set of numbers to keep in step.
    //
    // The correction stays ON throughout, and has to. Captures read Imag, which sits before the
    // table, so the correction cannot contaminate them - and leaving it on is what lets the buoy
    // steer to a CORRECTED heading, which is how it puts itself roughly on each leg for you. The
    // page used to switch it off, and then every steer target was as wrong as the compass was.
    //
    // What the page does NOT do any more is keep its own copy of the table arithmetic. It presses
    // the same /cal8_* buttons as everything else; see the block comment in compass.cpp.
    // ---------------------------------------------------------------------------------------
    subServer.on("/mancal_enter", HTTP_GET, [](){
        // Hold PWRENABLE on and the serial watchdog off for the duration - without the Top feeding
        // them, both would otherwise interrupt the calibration. See main.h.
        mancalSessionBegin();
        if (mainDataMutex && xSemaphoreTake(mainDataMutex, pdMS_TO_TICKS(500))) {
            mainData.tgSpeed = 0;
            mainData.tgDist = 0;
            mainData.status = IDLING;
            xSemaphoreGive(mainDataMutex);
        }
        subServer.send(200, "text/plain", "OK");
    });

    // /cal8_steer used to live here: it pivoted the buoy towards leg * 45 as a CORRECTED heading,
    // so the operator could have the hull aim itself at each direction before pressing SET.
    //
    // It went with the change of what a leg means. The directions are no longer true bearings the
    // hull is aimed at - they are stops on a mechanical fixture, 45 degrees apart by construction,
    // indexed round from wherever the fixture was aligned with Imag = 0. Steering to a true heading
    // of leg * 45 would take the hull somewhere that has nothing to do with the mark the operator
    // is about to press, and it would look purposeful while doing it. Turning the buoy is the
    // fixture's job now.
    //
    // mancal_stop below is kept: it is how a session that did start the thrusters gets them
    // stopped again.
    subServer.on("/mancal_stop", HTTP_GET, [](){
        mancalSessionPing();
        if (mainDataMutex && xSemaphoreTake(mainDataMutex, pdMS_TO_TICKS(500))) {
            mainData.tgSpeed = 0;
            mainData.tgDist = 0;
            mainData.status = IDLING;
            xSemaphoreGive(mainDataMutex);
        }
        subServer.send(200, "text/plain", "OK");
    });

    // Release the motor hold and stop turning. Nothing about the table is decided here any more -
    // /cal8_save and /cal8_cancel are the only two things that touch it.
    //
    // A running calibration session is deliberately left alone. It lives on the buoy, not in the
    // page, so closing this tab or moving to the plain calibration page mid-run loses nothing: the
    // captures are still there and the next page picks up at the same step.
    subServer.on("/mancal_exit", HTTP_GET, [](){
        extern volatile bool interp_enabled;
        mancalSessionEnd(); // release PWRENABLE and the watchdog back to the normal timers

        // The correction stays on. A buoy left with it off reports the identity table to everything
        // that asks (see STORE_INTERPOLATION_TABLE in main.cpp), so its stored calibration becomes
        // invisible to the CYD and the dashboard even though it is still there.
        if (!interp_enabled) {
            bool enable = true;
            interp_enabled = true;
            memInterpEnabled(&enable, MEM_PUT);
            global_params_rev++;
        }

        if (mainDataMutex && xSemaphoreTake(mainDataMutex, pdMS_TO_TICKS(500))) {
            mainData.tgSpeed = 0;
            mainData.tgDist = 0;
            mainData.status = IDLING;
            xSemaphoreGive(mainDataMutex);
        }
        subServer.send(200, "text/plain", "OK");
    });

    subServer.on("/mancal", HTTP_GET, [](){
        subServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        subServer.sendHeader("Pragma", "no-cache");
        subServer.sendHeader("Expires", "-1");
        if (LittleFS.exists("/mancal.html")) {
            File file = LittleFS.open("/mancal.html", "r");
            subServer.streamFile(file, "text/html");
            file.close();
        } else {
            subServer.send(404, "text/plain", "mancal page not found on LittleFS");
        }
    });

    subServer.on("/eightpointcal", HTTP_GET, [](){
        subServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        subServer.sendHeader("Pragma", "no-cache");
        subServer.sendHeader("Expires", "-1");
        if (LittleFS.exists("/eightpointcal.html")) {
            File file = LittleFS.open("/eightpointcal.html", "r");
            subServer.streamFile(file, "text/html");
            file.close();
        } else {
            subServer.send(404, "text/plain", "eightpointcal page not found on LittleFS");
        }
    });
    // Old names kept as redirects, not deleted. /harmoniccorrection was the address for long enough
    // to be bookmarked and written down, and it described a Fourier fit that has not existed since
    // the piecewise interpolation replaced it - so the name had to go, but a dead link would just
    // look like a broken buoy.
    subServer.on("/harmoniccorrection", HTTP_GET, [](){
        subServer.sendHeader("Location", "/eightpointcal");
        subServer.send(302, "text/plain", "Redirecting...");
    });
    subServer.on("/linearinterpolation", HTTP_GET, [](){
        subServer.sendHeader("Location", "/eightpointcal");
        subServer.send(302, "text/plain", "Redirecting...");
    });
    subServer.on("/linerinterpolation", HTTP_GET, [](){
        subServer.sendHeader("Location", "/eightpointcal");
        subServer.send(302, "text/plain", "Redirecting...");
    });
    subServer.on("/calibration", HTTP_GET, [](){
        subServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        subServer.sendHeader("Pragma", "no-cache");
        subServer.sendHeader("Expires", "-1");
        if (LittleFS.exists("/calibration.html")) {
            File file = LittleFS.open("/calibration.html", "r");
            subServer.streamFile(file, "text/html");
            file.close();
        } else {
            subServer.send(404, "text/plain", "calibration page not found on LittleFS");
        }
    });
    subServer.on("/ShowActualData", HTTP_GET, [](){
        subServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        subServer.sendHeader("Pragma", "no-cache");
        subServer.sendHeader("Expires", "-1");
        if (LittleFS.exists("/showactualdata.html")) {
            File file = LittleFS.open("/showactualdata.html", "r");
            subServer.streamFile(file, "text/html");
            file.close();
        } else {
            subServer.send(404, "text/plain", "showactualdata page not found on LittleFS");
        }
    });
    subServer.on("/calibration", HTTP_GET, [](){
        subServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        subServer.sendHeader("Pragma", "no-cache");
        subServer.sendHeader("Expires", "-1");
        if (LittleFS.exists("/calibration.html")) {
            File file = LittleFS.open("/calibration.html", "r");
            subServer.streamFile(file, "text/html");
            file.close();
        } else {
            subServer.send(404, "text/plain", "calibration page not found on LittleFS");
        }
    });
    subServer.on("/set_north", HTTP_GET, [](){
        // Allowed at any time. This used to be refused while a calibration held the heading
        // reference, because the offset was applied before the table and moving it rotated the
        // whole deviation curve under the run. The offset is applied after the table now, so it
        // rotates the output and nothing else - see the pipeline in compass.cpp.
        double newOffset = 0;
        bool success = false;
        if(mainDataMutex && xSemaphoreTake(mainDataMutex, pdMS_TO_TICKS(500))){
            // Same exact solve the SET_AS_NORTH command uses, so this page and the handheld
            // cannot disagree about where north is.
            extern float computeSetAsNorthOffset(void);
            newOffset = computeSetAsNorthOffset();
            mainData.compassOffset = newOffset;
            success = true;
            xSemaphoreGive(mainDataMutex);
        }
        if (success) {
            CompasOffset(&mainData, MEM_PUT);
            global_params_rev++;
            subServer.send(200, "text/plain", "OK");
        } else {
            subServer.send(500, "text/plain", "Err");
        }
    });
    
    // Parameter Update API
    subServer.on("/setparam", HTTP_GET, [](){
        if(!subServer.hasArg("p")||!subServer.hasArg("v")){subServer.send(400,"text/plain","Err");return;}
        String p=subServer.arg("p"); float v=subServer.arg("v").toFloat();
        bool paramUpdated = false;
        
        if(mainDataMutex && xSemaphoreTake(mainDataMutex, pdMS_TO_TICKS(500))){
            if(p=="kpr"){mainData.Kpr=v; paramUpdated = true;}
            else if(p=="kir"){mainData.Kir=v; paramUpdated = true;}
            else if(p=="kdr"){mainData.Kdr=v; paramUpdated = true;}
            else if(p=="kps"){mainData.Kps=v; paramUpdated = true;}
            else if(p=="kis"){mainData.Kis=v; paramUpdated = true;}
            else if(p=="kds"){mainData.Kds=v; paramUpdated = true;}
            else if(p=="coff"){mainData.compassOffset=v; paramUpdated = true;}
            else if(p=="pvspd"){mainData.pivotSpeed=v; paramUpdated = true;}
            else if(p=="holdrad"){
                if(v < HOLD_RADIUS_MIN) v = HOLD_RADIUS_MIN;
                mainData.holdRad=v; paramUpdated = true;
            }
            else if(p=="minspd"){mainData.minSpeed=(int)v; paramUpdated = true;}
            else if(p=="maxspd"){mainData.maxSpeed=(int)v; paramUpdated = true;}
            else if(p=="revbb"){mainData.revBB=(v>0.5); paramUpdated = true;}
            else if(p=="revsb"){mainData.revSB=(v>0.5); paramUpdated = true;}
            else if(p=="tswap"){mainData.swap_BB_SB=(v>0.5); paramUpdated = true;}
            else if(p=="cavg"){
                extern int compass_avg_len;
                compass_avg_len = (int)v;
                if (compass_avg_len < 1) compass_avg_len = 1;
                if (compass_avg_len > 200) compass_avg_len = 200;
                paramUpdated = true;
            }
            else if(p=="ctrim"){
                mainData.compass_trim = v;
                if (mainData.compass_trim < -15.0f) mainData.compass_trim = -15.0f;
                if (mainData.compass_trim > 15.0f) mainData.compass_trim = 15.0f;
                paramUpdated = true;
            }
            else if(p=="ctrim_en"){
                mainData.compass_trim_enabled = (v > 0.5);
                paramUpdated = true;
            }
            else if(p=="ctrim_clr"){
                mainData.compass_trim = 0.0f;
                paramUpdated = true;
            }
            // ESC stop-pulse trim, microseconds. Takes effect on the next 20 ms pulse, so you can
            // watch a creeping thruster stop while you adjust it.
            else if(p=="nbb"){
                int t = (int)v;
                if (t < ESC_NEUTRAL_MIN_US) t = ESC_NEUTRAL_MIN_US;
                if (t > ESC_NEUTRAL_MAX_US) t = ESC_NEUTRAL_MAX_US;
                esc_neutral_bb = t; paramUpdated = true;
            }
            else if(p=="nsb"){
                int t = (int)v;
                if (t < ESC_NEUTRAL_MIN_US) t = ESC_NEUTRAL_MIN_US;
                if (t > ESC_NEUTRAL_MAX_US) t = ESC_NEUTRAL_MAX_US;
                esc_neutral_sb = t; paramUpdated = true;
            }
            else if(p=="prdamp"){
                pr_damping = v;
                if (pr_damping < 0.0f) pr_damping = 0.0f;
                if (pr_damping > 0.99f) pr_damping = 0.99f;
                paramUpdated = true;
            }
            
            if (paramUpdated) {
                global_params_rev++;
            }
            xSemaphoreGive(mainDataMutex);
        }
        
        if (paramUpdated) {
            if(p=="kpr" || p=="kir" || p=="kdr"){pidRudderParameters(&mainData,MEM_PUT);initRudPid(&mainData);}
            else if(p=="kps" || p=="kis" || p=="kds"){pidSpeedParameters(&mainData,MEM_PUT);initSpeedPid(&mainData);}
            else if(p=="coff"){CompasOffset(&mainData,MEM_PUT);}
            else if(p=="pvspd"){speedMaxMin(&mainData,MEM_PUT);initSpeedPid(&mainData);initRudPid(&mainData);}
            else if(p=="holdrad"){computeParameters(&mainData,MEM_PUT);initSpeedPid(&mainData);initRudPid(&mainData);}
            else if(p=="minspd" || p=="maxspd"){speedMaxMin(&mainData,MEM_PUT);initSpeedPid(&mainData);initRudPid(&mainData);}
            else if(p=="revbb" || p=="revsb"){thrusterInversion(&mainData,MEM_PUT);}
            else if(p=="tswap"){thrusterSwap(&mainData,MEM_PUT);}
            else if(p=="cavg"){
                extern int compass_avg_len;
                memCompassAvg(&compass_avg_len, MEM_PUT);
            }
            else if(p=="ctrim" || p=="ctrim_en" || p=="ctrim_clr"){
                float trim_val = (float)mainData.compass_trim;
                bool trim_en = mainData.compass_trim_enabled;
                memCompassTrim(&trim_val, &trim_en, MEM_PUT);
            }
            else if(p=="prdamp"){
                memPrDamping(&pr_damping, MEM_PUT);
            }
            else if(p=="nbb" || p=="nsb"){
                memEscNeutral(&esc_neutral_bb, &esc_neutral_sb, MEM_PUT);
            }
        }
        subServer.send(200,"text/plain","OK");
    });

    // Telemetry and Parameter Read API (fully lock-free, atomic reads of primitive fields)
    subServer.on("/params", HTTP_GET, [](){
        float kpr = mainData.Kpr;
        float kir = mainData.Kir;
        float kdr = mainData.Kdr;
        float kps = mainData.Kps;
        float kis = mainData.Kis;
        float kds = mainData.Kds;
        float coff = (float)mainData.compassOffset;
        int revbb = mainData.revBB ? 1 : 0;
        int revsb = mainData.revSB ? 1 : 0;
        int tswap = mainData.swap_BB_SB ? 1 : 0;
        float pvspd = (float)mainData.pivotSpeed;
        int minspd = mainData.minSpeed;
        int maxspd = mainData.maxSpeed;
        float holdrad = (float)mainData.holdRad;
        extern int compass_avg_len;
        int cavg = compass_avg_len;
        float ctrim = (float)mainData.compass_trim;
        int ctrim_en = mainData.compass_trim_enabled ? 1 : 0;
        
        extern float pr_damping;
        float prdamp = pr_damping;
        
        char buf[700];
        snprintf(buf, sizeof(buf), 
            "{\"kpr\":%.3f,\"kir\":%.3f,\"kdr\":%.3f,\"kps\":%.3f,\"kis\":%.3f,\"kds\":%.3f,\"coff\":%.1f,\"revbb\":%d,\"revsb\":%d,\"tswap\":%d,\"pvspd\":%.2f,\"minspd\":%d,\"maxspd\":%d,\"holdrad\":%.1f,\"cavg\":%d,\"ctrim\":%.3f,\"ctrim_en\":%d,\"prdamp\":%.3f,\"nbb\":%d,\"nsb\":%d,\"nbb_us\":%d,\"nsb_us\":%d}",
            kpr, kir, kdr, kps, kis, kds, coff, revbb, revsb, tswap, pvspd, minspd, maxspd, holdrad, cavg, ctrim, ctrim_en, prdamp,
            esc_neutral_bb, esc_neutral_sb, escActualPulseBb(), escActualPulseSb()
        );
        subServer.send(200, "application/json", buf);
    });

    /*
     * Real-Time Telemetry /data JSON Endpoint:
     * This API is polled by the dashboard (index.html) at high speed (e.g. 100-250ms).
     * To keep operations lock-free and lightweight, we copy primitive fields atomically 
     * without blocking other tasks on the main controller core.
     *
     * STATE VISUALIZATION MAPPING:
     * Maps the internal enum state integer to its corresponding human-readable string.
     * This is sent to the dashboard so that the operator has immediate feedback on whether
     * the buoy is idling, locking, locked, docking, docked, or in a calibration routine.
     */
    subServer.on("/data", HTTP_GET, [](){
        // The MAN CAL page polls this several times a second, so it doubles as the session
        // heartbeat: while someone is watching the readout the session stays alive, and when the
        // page goes away it expires on its own rather than pinning the buoy powered-on.
        mancalSessionPing();
        extern bool interp_table_usable;   // see computeFourierCoefficients() in compass.cpp
        subServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        subServer.sendHeader("Pragma", "no-cache");
        subServer.sendHeader("Expires", "-1");
        extern float damp_acc;
        extern float damp_gyro;
        extern float damp_mag;
        extern float damp_att;
        extern volatile float global_fusion_hdg;
        extern volatile float global_hdg_iron;
        extern float measured_angles[9];
        extern float getInterpolatedHeading(float);
        extern volatile bool interp_enabled;

        float icm = global_hdg;

        // The full pipeline as the compass task runs it: table, then the trim, then the mounting
        // offset last. Kept in the same order here, or this preview would disagree with the heading
        // the buoy actually steers by.
        float corrected_hdg = getInterpolatedHeading(global_hdg_iron);
        if (mainData.compass_trim_enabled) {
            corrected_hdg += mainData.compass_trim;
        }
        corrected_hdg += (float)mainData.compassOffset;
        while (corrected_hdg < 0.0f) corrected_hdg += 360.0f;
        while (corrected_hdg >= 360.0f) corrected_hdg -= 360.0f;

        int sbb = (int)mainData.speedBb; 
        int ssb = (int)mainData.speedSb;
        double ir = mainData.ir;
        double ip = mainData.ip;
        double pitch = mainData.pitch;
        double roll = mainData.roll;
        float vatt = mainData.subAccuV;
        float curr = mainData.subAccuI;
        int stat_val = mainData.status;

        const char* statusStr = "UNKNOWN";
        switch (stat_val) {
            case IDLE: statusStr = "IDLE"; break;
            case IDLING: statusStr = "IDLING"; break;
            case LOCKED: statusStr = "LOCKED"; break;
            case DOCKED: statusStr = "DOCKED"; break;
            case DIRDIST: statusStr = "DIRDIST"; break;
            case TGDIRSPEED: statusStr = "TGDIRSPEED"; break;
            case REMOTE: statusStr = "REMOTE"; break;
            case SPBBSPSB: statusStr = "SPBBSPSB"; break;
            case CALIBRATE_MAGNETIC_COMPASS: statusStr = "CAL_MAG"; break;
            case INFIELD_CALIBRATE: statusStr = "CAL_FIELD"; break;
        }

        /*
         * COORD REALIGNMENT & CALIBRATION CALCULATION:
         * To feed the high-performance 'ShowActualData' dashboard, we calculate raw, hard, and soft
         * headings dynamically inside this endpoint using the global magnetometer calibrations (NVS).
         * This isolates the calculations to this non-blocking HTTP thread, keeping the Core 1 CompassTask() 
         * lean and fast.
         */

        // Calculate raw uncompensated heading (matching prototype)
        float rawHeading = atan2(last_raw_x, last_raw_y) * 180.0 / M_PI;
        if (rawHeading < 0.0f) rawHeading += 360.0f;

        // Calculate Hard-iron compensated heading (matching prototype)
        float hx = last_raw_x - hi_x;
        float hy = last_raw_y - hi_y;
        float hardHeading = atan2(hx, hy) * 180.0 / M_PI;
        if (hardHeading < 0.0f) hardHeading += 360.0f;

        // Calculate Soft-iron compensated heading (matching prototype)
        float sx = last_raw_x * si_x;
        float sy = last_raw_y * si_y;
        float headingSoft = atan2(sx, sy) * 180.0 / M_PI;
        if (headingSoft < 0.0f) headingSoft += 360.0f;

        // Calculate Both Comp (Offsets + Scaling)
        float bx = hx * si_x;
        float by = hy * si_y;
        float softHeading = atan2(bx, by) * 180.0 / M_PI;
        if (softHeading < 0.0f) softHeading += 360.0f;

        // Calculate Option 3/5 heading: Analytical 3D tilt-compensated heading (matching prototype)
        // roll and pitch are raw/damped pitch & roll, we convert to radians
        float phi = roll * M_PI / 180.0f;
        float theta = pitch * M_PI / 180.0f;

        float cosRoll = cos(phi);
        float sinRoll = sin(phi);
        float cosPitch = cos(theta);
        float sinPitch = sin(theta);

        float cx_cal = hx * si_x;
        float cy_cal = hy * si_y;
        float cz_cal = (last_raw_z - hi_z) * si_z;

        float Xh = cx_cal * cosPitch + cy_cal * sinRoll * sinPitch - cz_cal * cosRoll * sinPitch;
        float Yh = cy_cal * cosRoll + cz_cal * sinRoll;

        float opt3Heading = atan2(Xh, Yh) * 180.0 / M_PI;
        if (opt3Heading < 0.0f) opt3Heading += 360.0f;

        // Align the calibrated magnetometer axes to match the accelerometer coordinate frame
        float mx_cal = bx;
        float my_cal = by;
        float mz_cal = cz_cal;

        // Align the raw magnetometer axes to match the native, unaligned sensor coordinate frame
        // This ensures calibration is calculated on the exact same coordinate system applied in CompassTask.
        float mx_raw_aligned = last_raw_x;
        float my_raw_aligned = last_raw_y;
        float mz_raw_aligned = last_raw_z;

        // Serialise the 9 measured angles for linear interpolation
        String measAngJson = "[";
        for (int i = 0; i < 9; i++) {
            if (i > 0) measAngJson += ",";
            measAngJson += String(measured_angles[i], 2);
        }
        measAngJson += "]";

        // The cal_ring buffer extraction has been removed since calibration data collection 
        // is now done directly via HTTP polling of mx_raw, my_raw, mz_raw.
        String pointsJson = "[]";

        String json = "{\"icm\":" + String(icm, 2) +
                      // Imag: iron corrected and nothing else. Named icm_no_offset until the
                      // offset moved out of the table's input, at which point the old name stopped
                      // being a description and started being a lie - it had included the offset
                      // since the day it was added.
                      ",\"imag\":" + String(global_hdg_iron, 2) +
                      // The finished heading: table, trim, then the mounting offset. Called
                      // harmonic_hdg until the Fourier fit it was named after stopped existing;
                      // the old key is still emitted below for anything not yet updated.
                      ",\"corrected_hdg\":" + String(corrected_hdg, 2) +
                      ",\"harmonic_hdg\":" + String(corrected_hdg, 2) +
                      ",\"fusion_no_offset\":" + String(global_fusion_hdg, 2) +
                      ",\"meas_ang\":" + measAngJson +
                      ",\"speed_bb\":" + String(sbb) +
                      ",\"speed_sb\":" + String(ssb) +
                      ",\"ir\":" + String(ir, 2) +
                      ",\"ip\":" + String(ip, 2) +
                      ",\"cal_load\":\"" + global_cal_load + "\"" +
                      ",\"cal_ver\":\"" + global_cal_ver + "\"" +
                      ",\"mac\":\"" + global_mac_str + "\"" +
                      ",\"cal_msg\":\"" + global_cal_msg + "\"" +
                      ",\"rev\":" + String(global_params_rev) +
                      ",\"vatt\":" + String(vatt, 1) +
                      ",\"curr\":" + String(curr, 2) +
                      ",\"status\":" + String(stat_val) +
                      ",\"status_str\":\"" + String(statusStr) + "\"" +
                      ",\"ri\":" + String(rudderInput, 2) +
                      ",\"ro\":" + String(rudderOutput, 2) +
                      ",\"rbb\":" + String(rampBb, 2) +
                      ",\"rsb\":" + String(rampSb, 2) +
                      ",\"framp\":" + String(forward_ramp, 2) +
                      // Was the LAST manoeuvre a pure pivot - a flag, not a speed. It sits in
                      // this run of PID diagnostics (ri, ro, rbb, rsb, framp) and that is all it
                      // has ever been, but calling it "pivot" made it read as the pivot SPEED
                      // setting stuck at 0 on a buoy actually configured for 0.20. Same name as
                      // the variable now, and the setting itself is right below.
                      ",\"pure_pivot\":" + String(was_pure_pivot ? 1 : 0) +
                      ",\"pvspd\":" + String(mainData.pivotSpeed, 2) +
                      ",\"mx_raw\":" + String(mx_raw_aligned, 2) +
                      ",\"my_raw\":" + String(my_raw_aligned, 2) +
                      ",\"mz_raw\":" + String(mz_raw_aligned, 2) +
                      ",\"icm_mode\":" + String(icm_mode) +
                      ",\"mag_rejected\":" + String(magRejected ? 1 : 0) +
                      ",\"raw\":" + String(rawHeading, 2) +
                      ",\"hard\":" + String(hardHeading, 2) +
                      ",\"hardSoft\":" + String(softHeading, 2) +
                      ",\"opt3\":" + String(opt3Heading, 2) +
                      ",\"mx_cal\":" + String(mx_cal, 2) +
                      ",\"my_cal\":" + String(my_cal, 2) +
                      ",\"mz_cal\":" + String(mz_cal, 2) +
                      ",\"ctrim\":" + String(mainData.compass_trim, 3) +
                      ",\"ctrim_en\":" + String(mainData.compass_trim_enabled ? 1 : 0) +
                      ",\"pitch\":" + String(pitch, 2) +
                      ",\"roll\":" + String(roll, 2) +
                      ",\"prdamp\":" + String(pr_damping, 3) +
                      ",\"damp_acc\":" + String(damp_acc, 3) +
                      ",\"damp_gyro\":" + String(damp_gyro, 3) +
                      ",\"damp_mag\":" + String(damp_mag, 3) +
                      ",\"damp_att\":" + String(damp_att, 3) +
                      ",\"harmonic_enabled\":" + String(interp_enabled ? 1 : 0) +
                      // Whether the 8 point table can actually be interpolated. Interpolation
                      // needs the measured angles to increase all the way round, and one
                      // mis-aimed direction breaks that. When this is 0 the compass is running
                      // UNCORRECTED however harmonic_enabled reads, so the pages can say so
                      // instead of leaving it to the serial log.
                      ",\"table_ok\":" + String(interp_table_usable ? 1 : 0) +
                      // Which filtering mode the table was measured in, and whether that is the one
                      // running. A mismatch means the correction is being applied to a different
                      // heading source than it was measured on - the table looks perfectly valid.
                      ",\"table_mode\":" + String(interp_table_mode) +
                      ",\"mode_match\":" + String(interpTableModeMatches() ? 1 : 0) +
                      // Guided calibration session, so every interface can render the same state
                      // rather than keeping its own idea of how far along the operator is.
                      ",\"cal8_active\":" + String(cal8_active ? 1 : 0) +
                      ",\"cal8_next\":" + String(cal8_next) +
                      // Which directions are actually in. The cursor cannot say: re-capturing an
                      // earlier direction leaves it where it was.
                      ",\"cal8_mask\":" + String((int)cal8_mask) +
                      ",\"cal8\":[" + String(cal8_captured[0],1) + "," + String(cal8_captured[1],1) + "," +
                        String(cal8_captured[2],1) + "," + String(cal8_captured[3],1) + "," +
                        String(cal8_captured[4],1) + "," + String(cal8_captured[5],1) + "," +
                        String(cal8_captured[6],1) + "," + String(cal8_captured[7],1) + "]" +
                      ",\"points\":" + pointsJson + "}";

        subServer.send(200, "application/json", json);
    });

    // Anything we do not recognise goes to the dashboard. Together with the wildcard DNS this is
    // what makes the phone's captive-portal sheet open on our page the moment you join SUB_<id>;
    // on a real network it just means a stray URL still lands somewhere useful.
    subServer.onNotFound([]() {
        // Only hijack unknown URLs while we are the access point - that is what makes the phone's
        // captive-portal sheet open the dashboard. On a real network a 404 must stay a 404, or a
        // mistyped fetch quietly comes back as the whole dashboard instead of an error.
        if (apActive) {
            subServer.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
            subServer.send(302, "text/plain", "");
        } else {
            subServer.send(404, "text/plain", "Not found");
        }
    });

    subServer.begin(); 
    Serial.println("WiFiTask: WebServer started");

    // Main Server Loop - Matching RoboTop's efficient for(;;) delay(1) loop
    uint32_t last_ota_time = 0;
    extern bool global_is_calibrating;
    
    static unsigned long lastBackgroundConnectAttempt = 0;
    static int connectAttemptState = 0; // 0 = idle, 1 = trying NicE_WiFi
    static unsigned long connectionStartedTime = 0;

    for (;;) {
        subServer.handleClient();
        
        uint32_t now = millis();
        // Skip ArduinoOTA.handle() during active calibration to eliminate periodic 200ms network stalls
        if (ota && !global_is_calibrating && (now - last_ota_time >= 200)) {
            last_ota_time = now;
            ArduinoOTA.handle();
        }
        
        // Serve the wildcard DNS that drives the captive portal. Cheap, and only while we are
        // actually the access point.
        if (apActive) {
            dnsServer.processNextRequest();
        }

        // Re-join home if we lose it. This only ever runs on a Sub that was on home to begin
        // with; one that fell back to its own AP at boot has already given up on home, and one
        // that raises its AP below gives up at that moment. Robo_WiFi is not on the list: see the
        // note above connect_known_wifi().
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
                    connectAttemptState = 0;
                    // Home is gone. Raise our own AP so you can still get at us - and then stop
                    // hunting, because every further attempt would drag the radio off the AP's
                    // channel and make the AP flicker in and out of the phone's WiFi list.
                    if (!apActive) {
                        IPAddress apIp;
                        setup_wifi_ap(String(ownApSsid), String(OWN_AP_PASS), &apIp);
                    }
                    give_up_on_home();
                    runBackgroundReconnection = false;
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
        // Check queue instantly with 0-tick timeout to prevent any blocking in the server loop
        if (udpOut && xQueueReceive(udpOut, (void *)&msgIdOut, 0) == pdTRUE) {
            if (msgIdOut.IDs == 0) msgIdOut.IDs = espMac();
            // Temporarily comment out UDP broadcast to see if it eliminates periodic 1-second hangs
            // udp.broadcast(rfCode(&msgIdOut).c_str());
        }
        delay(5);
    }
}
