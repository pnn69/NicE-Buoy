/**
 * @file subwifi.cpp
 * @brief Dashboard and WiFi Management for NicE-Buoy Sub.
 * 
 * This module is aligned directly with RoboTop's wifi implementation:
 * 1. Manual WiFi scanning for 'NicE_WiFi' with AP fallback (eliminates WiFiManager overhead).
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
#include <LittleFS.h>
#include <Adafruit_Sensor.h>
#include <RoboCompute.h>
#include "main.h"
#include "io_sub.h"
#include "compass.h"
#include "datastorage.h"
#include "pidrudspeed.h"
#include "subwifi.h"
#include "index_html.h"
#include "leds.h"

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
extern uint8_t bno_cal_sys, bno_cal_gyro, bno_cal_accel, bno_cal_mag;
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
static String indexHtmlCache = "";
static bool ota = false;
static LedData wifiCollorUtil;

/**
 * @brief Sets up Over-The-Air (OTA) update functionality.
 */
bool setup_OTA()
{
    char buf[32];
    byte mac[6];
    WiFi.macAddress(mac);
    Serial.print("SETUP OTA...");
    sprintf(buf, "Sub_%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ArduinoOTA.setHostname(buf);
    ArduinoOTA.onStart([]() { Serial.println("OTA Start"); });
    ArduinoOTA.onEnd([]() { Serial.println("\nOTA End"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
    ArduinoOTA.onError([](ota_error_t error) { ESP.restart(); });
    ArduinoOTA.begin();
    Serial.println("READY");
    return true;
}

/**
 * @brief Scans for a specific Wi-Fi Access Point and connects to it.
 */
bool scan_for_wifi_ap(String ssipap, String ww, IPAddress *tmp)
{
    unsigned long timeout = millis();
    Serial.print("Scanning for ap: "); Serial.println(ssipap);

    while (millis() - timeout < 120000)
    {
        int n = WiFi.scanNetworks();
        if (n > 0)
        {
            for (int i = 0; i < n; ++i)
            {
                if (WiFi.SSID(i) == ssipap)
                {
                    Serial.print("AP found, connecting...");
                    WiFi.begin(ssipap.c_str(), ww.c_str());
                    unsigned long conn_timeout = millis();
                    while (WiFi.status() != WL_CONNECTED)
                    {
                        delay(500); Serial.print(".");
                        if (millis() - conn_timeout > 30000) break;
                    }
                    if (WiFi.status() == WL_CONNECTED) {
                        Serial.println("CONNECTED");
                        WiFi.setSleep(WIFI_PS_NONE); // Disable power-saving sleep for constant low-latency
                        *tmp = WiFi.localIP();
                        Serial.print("WiFi IP address: "); Serial.println(*tmp);
                        return true;
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return false;
}

/**
 * @brief Sets up a Wi-Fi Access Point with a static IP.
 */
void setup_wifi_ap(String ap, String ww, IPAddress *tmp)
{
    WiFi.mode(WIFI_AP);
    IPAddress local_IP(192, 168, 1, 85); // 192.168.1.85 to avoid collision with Top (192.168.1.84)
    IPAddress subnet(255, 255, 255, 0);
    IPAddress gateway(192, 168, 1, 5);
    WiFi.softAPConfig(local_IP, gateway, subnet);

    if (WiFi.softAP(ap.c_str(), ww.c_str()))
    {
        WiFi.setSleep(WIFI_PS_NONE); // Disable power-saving sleep in AP mode as well
        *tmp = WiFi.softAPIP();
        Serial.print("AP IP address: "); Serial.println(*tmp);
    }
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
    
    IPAddress ip;
    String ap = "";
    String apww = "";

    if (wifiConfig == 1) {
        ap = "PAIR_ME_"; ap += macStr;
        setup_wifi_ap(ap, apww, &ip);
    } else {
        wifiCollorUtil.color = CRGB::LightBlue;
        wifiCollorUtil.blink = BLINK_SLOW;
        xQueueSend(ledStatus, (void *)&wifiCollorUtil, 10);
        
        ap = "NicE_WiFi"; apww = "!Ni1001100110";
        if (!scan_for_wifi_ap(ap, apww, &ip)) {
            ap = "BUOY_SUB_"; ap += macStr;
            apww = "";
            setup_wifi_ap(ap, apww, &ip);
        }
    }
    
    wifiCollorUtil.color = CRGB::Black;
    wifiCollorUtil.blink = BLINK_OFF;
    xQueueSend(ledStatus, (void *)&wifiCollorUtil, 10);
    
    ota = setup_OTA();
    udp_setup(1001);

    // Mount LittleFS and cache index.html in RAM for fast execution
    if(!LittleFS.begin(true)){
        Serial.println("WiFiTask: LittleFS Mount Failed");
    } else {
        Serial.println("WiFiTask: LittleFS Mounted");
        File file = LittleFS.open("/index.html", "r");
        if(file) {
            indexHtmlCache = file.readString();
            file.close();
            Serial.println("WiFiTask: Cached index.html in RAM (" + String(indexHtmlCache.length()) + " bytes)");
        } else {
            Serial.println("WiFiTask: Failed to open /index.html from LittleFS, using compiled fallback");
        }
    }

    /*
     * Web Dashboard Router & Anti-Caching Strategy:
     * To prevent browsers from caching stale layouts or outdated JS dashboards, 
     * we issue HTTP/1.1 headers 'no-cache, no-store, must-revalidate' and set 'Expires: -1'.
     *
     * IMPORTANT DESIGN FIX:
     * We bypass indexHtmlCache and force subServer to serve INDEX_HTML from flash (send_P).
     * This avoids memory leak issues or potential desynchronization with filesystem changes, 
     * guaranteeing that the compiled UI changes are consistently served and rendered.
     */
    subServer.on("/", HTTP_GET, [](){
        subServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        subServer.sendHeader("Pragma", "no-cache");
        subServer.sendHeader("Expires", "-1");
        subServer.send_P(200, "text/html", INDEX_HTML);
    });
    subServer.on("/savecal", HTTP_GET, [](){ 
        int c=34; 
        xQueueSend(compassIn,(void*)&c,10); 
        subServer.send(200,"text/plain","OK"); 
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
                if(v < 1.5f) v = 1.5f;
                mainData.holdRad=v; paramUpdated = true;
            }
            else if(p=="minspd"){mainData.minSpeed=(int)v; paramUpdated = true;}
            else if(p=="maxspd"){mainData.maxSpeed=(int)v; paramUpdated = true;}
            else if(p=="revbb"){mainData.revBB=(v>0.5); paramUpdated = true;}
            else if(p=="revsb"){mainData.revSB=(v>0.5); paramUpdated = true;}
            else if(p=="tswap"){mainData.swap_BB_SB=(v>0.5); paramUpdated = true;}
            
            if (paramUpdated) {
                global_params_rev++;
            }
            xSemaphoreGive(mainDataMutex);
        }
        
        if (paramUpdated) {
            if(p=="kpr" || p=="kir" || p=="kdr"){pidRudderParameters(&mainData,SET);initRudPid(&mainData);}
            else if(p=="kps" || p=="kis" || p=="kds"){pidSpeedParameters(&mainData,SET);initSpeedPid(&mainData);}
            else if(p=="coff"){CompasOffset(&mainData,SET);}
            else if(p=="pvspd"){speedMaxMin(&mainData,SET);initSpeedPid(&mainData);initRudPid(&mainData);}
            else if(p=="holdrad"){computeParameters(&mainData,SET);initSpeedPid(&mainData);initRudPid(&mainData);}
            else if(p=="minspd" || p=="maxspd"){speedMaxMin(&mainData,SET);initSpeedPid(&mainData);initRudPid(&mainData);}
            else if(p=="revbb" || p=="revsb"){thrusterInversion(&mainData,SET);}
            else if(p=="tswap"){thrusterSwap(&mainData,SET);}
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
        
        char buf[500];
        snprintf(buf, sizeof(buf), 
            "{\"kpr\":%.3f,\"kir\":%.3f,\"kdr\":%.3f,\"kps\":%.3f,\"kis\":%.3f,\"kds\":%.3f,\"coff\":%.1f,\"revbb\":%d,\"revsb\":%d,\"tswap\":%d,\"pvspd\":%.2f,\"minspd\":%d,\"maxspd\":%d,\"holdrad\":%.1f}",
            kpr, kir, kdr, kps, kis, kds, coff, revbb, revsb, tswap, pvspd, minspd, maxspd, holdrad
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
        float icm = global_hdg;
        int sbb = (int)mainData.speedBb; 
        int ssb = (int)mainData.speedSb;
        double ir = mainData.ir;
        double ip = mainData.ip;
        float vatt = mainData.subAccuV;
        float curr = mainData.subAccuI;
        int stat_val = mainData.status;

        const char* statusStr = "UNKNOWN";
        switch (stat_val) {
            case IDLE: statusStr = "IDLE"; break;
            case IDELING: statusStr = "IDELING"; break;
            case LOCKED: statusStr = "LOCKED"; break;
            case DOCKED: statusStr = "DOCKED"; break;
            case DIRDIST: statusStr = "DIRDIST"; break;
            case TGDIRSPEED: statusStr = "TGDIRSPEED"; break;
            case REMOTE: statusStr = "REMOTE"; break;
            case SPBBSPSB: statusStr = "SPBBSPSB"; break;
            case CALIBRATE_MAGNETIC_COMPASS: statusStr = "CAL_MAG"; break;
            case INFIELD_CALIBRATE: statusStr = "CAL_FIELD"; break;
        }

        char buf[1024];
        snprintf(buf, sizeof(buf),
            "{\"icm\":%.2f,\"speed_bb\":%d,\"speed_sb\":%d,\"ir\":%.2f,\"ip\":%.2f,\"cal_load\":\"%s\",\"cal_ver\":\"%s\",\"mac\":\"%s\",\"cal_levels\":[%d,%d,%d,%d],\"cal_msg\":\"%s\",\"rev\":%u,\"vatt\":%.1f,\"curr\":%.2f,\"status\":%d,\"status_str\":\"%s\",\"ri\":%.2f,\"ro\":%.2f,\"rbb\":%.2f,\"rsb\":%.2f,\"framp\":%.2f,\"pivot\":%d}",
            icm, sbb, ssb, ir, ip, 
            global_cal_load.c_str(), global_cal_ver.c_str(), global_mac_str.c_str(),
            bno_cal_sys, bno_cal_gyro, bno_cal_accel, bno_cal_mag,
            global_cal_msg.c_str(), global_params_rev, vatt, curr,
            stat_val, statusStr,
            rudderInput, rudderOutput, rampBb, rampSb, forward_ramp, was_pure_pivot ? 1 : 0
        );
        subServer.send(200, "application/json", buf);
    });

    subServer.begin(); 
    Serial.println("WiFiTask: WebServer started");

    // Main Server Loop - Matching RoboTop's efficient for(;;) delay(1) loop
    uint32_t last_ota_time = 0;
    for (;;) {
        subServer.handleClient();
        
        uint32_t now = millis();
        if (ota && (now - last_ota_time >= 200)) {
            last_ota_time = now;
            ArduinoOTA.handle();
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
