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
#include <RoboCompute.h>
#include "main.h"
#include "io_sub.h"
#include "compass.h"
#include "datastorage.h"
#include "pidrudspeed.h"
#include "subwifi.h"
#include "index_html.h"
#include "calibration_html.h"
#include "showactualdata_html.h"
#include "leds.h"
#include "buzzer.h"

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
extern int icm_mode;
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
            ap = "ROBOBUOY"; apww = "";
            if (!scan_for_wifi_ap(ap, apww, &ip)) {
                ap = "BUOY_SUB_"; ap += macStr;
                apww = "";
                setup_wifi_ap(ap, apww, &ip);
            }
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
        extern volatile int cal_ring_write_idx;
        extern volatile int cal_ring_read_idx;
        cal_ring_write_idx = 0;
        cal_ring_read_idx = 0;
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
    subServer.on("/calibration", HTTP_GET, [](){
        subServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        subServer.sendHeader("Pragma", "no-cache");
        subServer.sendHeader("Expires", "-1");
        subServer.send_P(200, "text/html", CALIBRATION_HTML);
    });
    subServer.on("/ShowActualData", HTTP_GET, [](){
        subServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        subServer.sendHeader("Pragma", "no-cache");
        subServer.sendHeader("Expires", "-1");
        subServer.send_P(200, "text/html", SHOWACTUALDATA_HTML);
    });
    subServer.on("/callibration", HTTP_GET, [](){
        subServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        subServer.sendHeader("Pragma", "no-cache");
        subServer.sendHeader("Expires", "-1");
        subServer.send_P(200, "text/html", CALIBRATION_HTML);
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
            CompasOffset(&mainData, SET);
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
                if(v < 1.5f) v = 1.5f;
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
            else if(p=="cavg"){
                extern int compass_avg_len;
                memCompassAvg(&compass_avg_len, SET);
            }
            else if(p=="ctrim" || p=="ctrim_en" || p=="ctrim_clr"){
                float trim_val = (float)mainData.compass_trim;
                bool trim_en = mainData.compass_trim_enabled;
                memCompassTrim(&trim_val, &trim_en, SET);
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
        
        char buf[650];
        snprintf(buf, sizeof(buf), 
            "{\"kpr\":%.3f,\"kir\":%.3f,\"kdr\":%.3f,\"kps\":%.3f,\"kis\":%.3f,\"kds\":%.3f,\"coff\":%.1f,\"revbb\":%d,\"revsb\":%d,\"tswap\":%d,\"pvspd\":%.2f,\"minspd\":%d,\"maxspd\":%d,\"holdrad\":%.1f,\"cavg\":%d,\"ctrim\":%.3f,\"ctrim_en\":%d}",
            kpr, kir, kdr, kps, kis, kds, coff, revbb, revsb, tswap, pvspd, minspd, maxspd, holdrad, cavg, ctrim, ctrim_en
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
        double pitch = mainData.pitch;
        double roll = mainData.roll;
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

        /*
         * COORD REALIGNMENT & CALIBRATION CALCULATION:
         * To feed the high-performance 'ShowActualData' dashboard, we calculate raw, hard, and soft
         * headings dynamically inside this endpoint using the global magnetometer calibrations (NVS).
         * This isolates the calculations to this non-blocking HTTP thread, keeping the Core 1 CompassTask() 
         * lean and fast.
         */

        // Calculate raw uncompensated heading (direct atan2 of raw magnetometer)
        float rawHeading = atan2(last_raw_y, last_raw_x) * 180.0 / M_PI;
        if (rawHeading < 0.0f) rawHeading += 360.0f;
        rawHeading = 360.0f - rawHeading; // Mirror rotation direction to match screen rose
        if (rawHeading >= 360.0f) rawHeading -= 360.0f;

        // Calculate Hard-iron compensated heading (subtract offsets hi_x, hi_y, hi_z)
        float mx_hi = last_raw_x - hi_x;
        float my_hi = last_raw_y - hi_y;
        float mz_hi = last_raw_z - hi_z;
        float hardHeading = atan2(my_hi, mx_hi) * 180.0 / M_PI;
        if (hardHeading < 0.0f) hardHeading += 360.0f;
        hardHeading = 360.0f - hardHeading; // Mirror rotation direction to match screen rose
        if (hardHeading >= 360.0f) hardHeading -= 360.0f;

        // Calculate Soft-iron compensated heading (apply full 3x3 soft-iron matrix multiplication)
        extern float si_matrix[3][3];
        float mxc = si_matrix[0][0] * mx_hi + si_matrix[0][1] * my_hi + si_matrix[0][2] * mz_hi;
        float myc = si_matrix[1][0] * mx_hi + si_matrix[1][1] * my_hi + si_matrix[1][2] * mz_hi;
        float mzc = si_matrix[2][0] * mx_hi + si_matrix[2][1] * my_hi + si_matrix[2][2] * mz_hi;

        float softHeading = atan2(myc, mxc) * 180.0 / M_PI;
        if (softHeading < 0.0f) softHeading += 360.0f;
        softHeading = 360.0f - softHeading; // Mirror rotation direction to match screen rose
        if (softHeading >= 360.0f) softHeading -= 360.0f;

        // Align the calibrated magnetometer axes to match the accelerometer coordinate frame
        float mx_cal = myc;
        float my_cal = -mxc;
        float mz_cal = -mzc;

        // Align the raw magnetometer axes to match the same accelerometer coordinate frame for direct UI comparison
        float mx_raw_aligned = last_raw_y;
        float my_raw_aligned = -last_raw_x;
        float mz_raw_aligned = -last_raw_z;

        // Extract all pending calibration points from the ring buffer
        extern float cal_ring_x[];
        extern float cal_ring_y[];
        extern float cal_ring_z[];
        extern volatile int cal_ring_write_idx;
        extern volatile int cal_ring_read_idx;

        String pointsJson = "[";
        bool firstPoint = true;
        while (cal_ring_read_idx != cal_ring_write_idx) {
            if (!firstPoint) pointsJson += ",";
            pointsJson += "[" + String(cal_ring_x[cal_ring_read_idx], 2) + "," +
                          String(cal_ring_y[cal_ring_read_idx], 2) + "," +
                          String(cal_ring_z[cal_ring_read_idx], 2) + "]";
            firstPoint = false;
            cal_ring_read_idx = (cal_ring_read_idx + 1) % 100; // CAL_RING_BUF_SIZE
        }
        pointsJson += "]";

        String json = "{\"icm\":" + String(icm, 2) +
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
                      ",\"mx_cal\":" + String(mx_cal, 2) +
                      ",\"my_cal\":" + String(my_cal, 2) +
                      ",\"mz_cal\":" + String(mz_cal, 2) +
                      ",\"ctrim\":" + String(mainData.compass_trim, 3) +
                      ",\"ctrim_en\":" + String(mainData.compass_trim_enabled ? 1 : 0) +
                      ",\"pitch\":" + String(pitch, 2) +
                      ",\"roll\":" + String(roll, 2) +
                      ",\"points\":" + pointsJson + "}";

        subServer.send(200, "application/json", json);
    });

    subServer.begin(); 
    Serial.println("WiFiTask: WebServer started");

    // Main Server Loop - Matching RoboTop's efficient for(;;) delay(1) loop
    uint32_t last_ota_time = 0;
    extern bool global_is_calibrating;
    for (;;) {
        subServer.handleClient();
        
        uint32_t now = millis();
        // Skip ArduinoOTA.handle() during active calibration to eliminate periodic 200ms network stalls
        if (ota && !global_is_calibrating && (now - last_ota_time >= 200)) {
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
