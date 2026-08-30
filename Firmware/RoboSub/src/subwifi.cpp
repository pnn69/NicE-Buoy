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

            // Success beep sequence!
            if (buzzer != NULL) {
                beep(-1, buzzer);
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
    subServer.on("/set_harmonic_point", HTTP_GET, [](){
        if (subServer.hasArg("index") && subServer.hasArg("measured")) {
            int idx = subServer.arg("index").toInt();
            float val = subServer.arg("measured").toFloat();
            
            extern float measured_angles[9];
            if (idx >= 0 && idx < 9) {
                measured_angles[idx] = val;
                extern void computeFourierCoefficients();
                computeFourierCoefficients();
                subServer.send(200, "text/plain", "OK");
            } else {
                subServer.send(400, "text/plain", "Invalid index");
            }
        } else {
            subServer.send(400, "text/plain", "Missing args");
        }
    });
    // Legacy alias
    subServer.on("/set_interpolation_point", HTTP_GET, [](){
        if (subServer.hasArg("index") && subServer.hasArg("measured")) {
            int idx = subServer.arg("index").toInt();
            float val = subServer.arg("measured").toFloat();
            extern float measured_angles[9];
            if (idx >= 0 && idx < 9) {
                measured_angles[idx] = val;
                extern void computeFourierCoefficients();
                computeFourierCoefficients();
                subServer.send(200, "text/plain", "OK");
            } else {
                subServer.send(400, "text/plain", "Invalid index");
            }
        } else {
            subServer.send(400, "text/plain", "Missing args");
        }
    });
    subServer.on("/save_harmonic", HTTP_GET, [](){
        extern float measured_angles[9];
        memInterpolationTable(measured_angles, MEM_PUT);
        subServer.send(200, "text/plain", "OK");
    });
    // Legacy alias
    subServer.on("/save_interpolation", HTTP_GET, [](){
        extern float measured_angles[9];
        memInterpolationTable(measured_angles, MEM_PUT);
        subServer.send(200, "text/plain", "OK");
    });
    subServer.on("/reset_harmonic", HTTP_GET, [](){
        extern float measured_angles[9];
        for (int i = 0; i < 9; i++) {
            measured_angles[i] = i * 45.0f;
        }
        memInterpolationTable(measured_angles, MEM_PUT);
        extern void computeFourierCoefficients();
        computeFourierCoefficients();
        subServer.send(200, "text/plain", "OK");
    });
    // Legacy alias
    subServer.on("/reset_interpolation", HTTP_GET, [](){
        extern float measured_angles[9];
        for (int i = 0; i < 9; i++) {
            measured_angles[i] = i * 45.0f;
        }
        memInterpolationTable(measured_angles, MEM_PUT);
        extern void computeFourierCoefficients();
        computeFourierCoefficients();
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
    // The harmonic correction has to be OFF while dialing, or the table would be applied on top of
    // the very measurement being taken. /mancal_enter switches it off in RAM only and never writes
    // that to NVS, so a reboot in the middle of a session comes back with the correction on rather
    // than leaving the buoy silently uncalibrated.
    // ---------------------------------------------------------------------------------------
    subServer.on("/mancal_enter", HTTP_GET, [](){
        extern volatile bool interp_enabled;
        interp_enabled = false; // RAM only - deliberately not persisted, see above
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

    subServer.on("/mancal_point", HTTP_GET, [](){
        if (!subServer.hasArg("index") || !subServer.hasArg("corr")) {
            subServer.send(400, "text/plain", "Missing args");
            return;
        }
        mancalSessionPing();
        int idx = subServer.arg("index").toInt();
        float corr = subServer.arg("corr").toFloat();
        if (idx < 0 || idx > 7) {
            subServer.send(400, "text/plain", "Invalid index");
            return;
        }

        extern float measured_angles[9];
        extern void computeFourierCoefficients();

        float target = (float)(idx * 45) - corr;
        while (target < 0.0f) target += 360.0f;
        while (target >= 360.0f) target -= 360.0f;
        measured_angles[idx] = target;
        // Entry 8 is the 360 degree wrap of entry 0, kept in step so the stored table stays valid.
        if (idx == 0) measured_angles[8] = measured_angles[0] + 360.0f;
        computeFourierCoefficients();

        // Steer there at zero speed - a pure pivot, no forward thrust.
        if (mainDataMutex && xSemaphoreTake(mainDataMutex, pdMS_TO_TICKS(500))) {
            mainData.tgDir = target;
            mainData.tgSpeed = 0;
            mainData.tgDist = 0;
            mainData.status = REMOTE;
            xSemaphoreGive(mainDataMutex);
        }
        subServer.send(200, "application/json",
                       "{\"index\":" + String(idx) + ",\"corr\":" + String(corr, 1) +
                       ",\"target\":" + String(target, 1) + "}");
    });

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

    subServer.on("/mancal_exit", HTTP_GET, [](){
        extern float measured_angles[9];
        extern void computeFourierCoefficients();
        extern volatile bool interp_enabled;
        bool save = (subServer.arg("save") == "1");
        mancalSessionEnd(); // release PWRENABLE and the watchdog back to the normal timers

        if (save) {
            measured_angles[8] = measured_angles[0] + 360.0f;
            memInterpolationTable(measured_angles, MEM_PUT);
        } else {
            // Throw the session away and go back to what is actually stored.
            memInterpolationTable(measured_angles, MEM_GET);
        }
        computeFourierCoefficients();

        // Leave with the correction ON either way. A buoy left with it off reports the identity
        // table to everything that asks (see STORE_INTERPOLATION_TABLE in main.cpp), so its stored
        // calibration becomes invisible to the CYD and the dashboard even though it is still there.
        bool enable = true;
        interp_enabled = true;
        memInterpEnabled(&enable, MEM_PUT);
        global_params_rev++;

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

    subServer.on("/harmoniccorrection", HTTP_GET, [](){
        subServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        subServer.sendHeader("Pragma", "no-cache");
        subServer.sendHeader("Expires", "-1");
        if (LittleFS.exists("/harmoniccorrection.html")) {
            File file = LittleFS.open("/harmoniccorrection.html", "r");
            subServer.streamFile(file, "text/html");
            file.close();
        } else {
            subServer.send(404, "text/plain", "harmoniccorrection page not found on LittleFS");
        }
    });
    subServer.on("/linearinterpolation", HTTP_GET, [](){
        subServer.sendHeader("Location", "/harmoniccorrection");
        subServer.send(302, "text/plain", "Redirecting...");
    });
    subServer.on("/linerinterpolation", HTTP_GET, [](){
        subServer.sendHeader("Location", "/harmoniccorrection");
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
        double newOffset = 0;
        bool success = false;
        if(mainDataMutex && xSemaphoreTake(mainDataMutex, pdMS_TO_TICKS(500))){
            newOffset = mainData.compassOffset - global_hdg;
            while (newOffset < -180.0) newOffset += 360.0;
            while (newOffset > 180.0) newOffset -= 360.0;
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
        subServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        subServer.sendHeader("Pragma", "no-cache");
        subServer.sendHeader("Expires", "-1");
        extern float damp_acc;
        extern float damp_gyro;
        extern float damp_mag;
        extern float damp_att;
        extern volatile float global_fusion_hdg;
        extern volatile float global_hdg_no_offset;
        extern float measured_angles[9];
        extern float getInterpolatedHeading(float);
        extern volatile bool interp_enabled;

        float icm = global_hdg;

        // Perform harmonic curve correction on global_hdg_no_offset (which already includes compassOffset)
        float harmonic_hdg = getInterpolatedHeading(global_hdg_no_offset);
        if (mainData.compass_trim_enabled) {
            harmonic_hdg += mainData.compass_trim;
        }
        while (harmonic_hdg < 0.0f) harmonic_hdg += 360.0f;
        while (harmonic_hdg >= 360.0f) harmonic_hdg -= 360.0f;

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
                      ",\"icm_no_offset\":" + String(global_hdg_no_offset, 2) +
                      ",\"harmonic_hdg\":" + String(harmonic_hdg, 2) +
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
                      ",\"pivot\":" + String(was_pure_pivot ? 1 : 0) +
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
