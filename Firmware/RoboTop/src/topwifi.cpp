#include <WiFi.h>
#include <SPIFFS.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <AsyncUDP.h>
#include "main.h"
#include "topwifi.h"
#include "leds.h"
#include "datastorage.h"
#include "buzzer.h"
#include "loratop.h"
#include "sercom.h"

static int statik = IDLE;
static RoboStruct msgIdOut;
static RoboStruct topWifiIn;
static RoboStruct udpBuffer;
static RoboStruct udpBufferRecieved;
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

WebServer server(80);

/**
 * @brief Sets up Over-The-Air (OTA) update functionality.
 */
bool setup_OTA()
{
    char buf[32];
    byte mac[6];
    WiFi.macAddress(mac);
    Serial.print("SETUP OTA...");
    sprintf(buf, "Top_%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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
    IPAddress local_IP(192, 168, 1, 84);
    IPAddress subnet(255, 255, 255, 0);
    IPAddress gateway(192, 168, 1, 5);
    WiFi.softAPConfig(local_IP, gateway, subnet);

    if (WiFi.softAP(ap.c_str(), ww.c_str()))
    {
        *tmp = WiFi.softAPIP();
        Serial.print("AP IP address: "); Serial.println(*tmp);
    }
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
void WiFiTask(void *arg)
{
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
        wifiCollorUtil.blink = FADE_ON;
        wifiCollorUtil.fadeAmount = 5;
        xQueueSend(ledStatus, (void *)&wifiCollorUtil, 10);
        
        ap = "NicE_WiFi"; apww = "!Ni1001100110";
        if (!scan_for_wifi_ap(ap, apww, &ip)) {
            ap = "ROBOWIFI"; apww = "";
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
    SPIFFS.begin(true);

    static String indexHtmlCache = "";
    File file = SPIFFS.open("/index.html", "r");
    if(file) {
        indexHtmlCache = file.readString();
        file.close();
        Serial.println("WiFiTask: Cached index.html in RAM (" + String(indexHtmlCache.length()) + " bytes)");
    } else {
        Serial.println("WiFiTask: Failed to open /index.html for caching");
    }

    server.on("/", HTTP_GET, [](){
        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        server.sendHeader("Pragma", "no-cache");
        server.sendHeader("Expires", "-1");
        if(indexHtmlCache.length() > 0) {
            server.send(200, "text/html", indexHtmlCache);
        } else {
            server.send(404, "text/plain", "File not found in RAM cache. Please check SPIFFS filesystem.");
        }
    });

    server.on("/data", HTTP_GET, []()
              {
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
        json += "\"TgDist\":\"" + String(mainData.tgDist, 2) + "\",";
        json += "\"GpsDir\":\"" + String(mainData.gpsDir) + "\",";
        json += "\"WDir\":\"" + String(mainData.wDir, 2) + "\",";
        json += "\"WStd\":\"" + String(mainData.wStd, 2) + "\",";
        json += "\"SubVolt\":\"" + String(mainData.subAccuV, 2) + "\",";
        json += "\"SubCurr\":\"" + String(mainData.subAccuI, 2) + "\",";
        json += "\"SubPerc\":\"" + String(mainData.subAccuP) + "\",";
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
        json += "\"mechanicCorrection\":\"" + String(mainData.mechanicCorrection, 2) + "\",";
        json += "\"compass_trim_enabled\":\"" + String(mainData.compass_trim_enabled ? "true" : "false") + "\",";
        json += "\"compass_trim\":\"" + String(mainData.compass_trim, 3) + "\",";
        json += "\"Lat\":\"" + String(mainData.lat, 6) + "\",";
        json += "\"Lng\":\"" + String(mainData.lng, 6) + "\",";
        json += "\"GpsFix\":\"" + String(mainData.gpsFix ? "true" : "false") + "\"";
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
            json += "\"TgDist\":\"" + String(buoyPara[i].tgDist, 2) + "\",";
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
            json += "\"mechanicCorrection\":\"" + String(buoyPara[i].mechanicCorrection, 2) + "\",";
            json += "\"compass_trim_enabled\":\"" + String(buoyPara[i].compass_trim_enabled ? "true" : "false") + "\",";
            json += "\"compass_trim\":\"" + String(buoyPara[i].compass_trim, 3) + "\",";
            json += "\"Lat\":\"" + String(buoyPara[i].lat, 6) + "\",";
            json += "\"Lng\":\"" + String(buoyPara[i].lng, 6) + "\",";
            json += "\"GpsFix\":\"" + String(buoyPara[i].gpsFix ? "true" : "false") + "\"";
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
        // printf("Web Command received: bid=%d, cmd=%s\r\n", bid, cmdStr.c_str());

        if (cmdStr == "LOCK") {
            if (bid == 1) {
                if (mainData.status == LOCKED || mainData.status == LOCKING) { mainData.status = IDELING; cmdEnum = IDELING; }
                else { mainData.status = LOCKING; cmdEnum = LOCKING; }
            } else {
                if (buoyPara[bid-1].status == LOCKED || buoyPara[bid-1].status == LOCKING) cmdEnum = IDLE;
                else cmdEnum = LOCKING;
            }
        }
        else if (cmdStr == "DOCK") {
            if (bid == 1) {
                if (mainData.status == DOCKING || mainData.status == DOCKED) { mainData.status = IDELING; cmdEnum = IDELING; }
                else { mainData.status = DOCKING; cmdEnum = DOCKING; }
            } else {
                if (buoyPara[bid-1].status == DOCKING || buoyPara[bid-1].status == DOCKED) cmdEnum = IDLE;
                else cmdEnum = DOCKING;
            }
        }
        else if (cmdStr == "SETUP" || cmdStr == "SUBSETUP" || cmdStr == "SETUPDATA") cmdEnum = SETUPDATA;
        else if (cmdStr == "IDLE") { if (bid == 1) mainData.status = IDELING; cmdEnum = IDELING; }
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
        else if (cmdStr == "ADAPTIVE_TRIM") cmdEnum = ADAPTIVE_TRIM;
        else if (cmdStr == "COMPUTESTART") cmdEnum = COMPUTESTART;
        else if (cmdStr == "COMPUTETRACK") cmdEnum = COMPUTETRACK;

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
                    mainData.revBB = server.arg("revBB").toInt();
                    mainData.revSB = server.arg("revSB").toInt();
                    mainData.swap_BB_SB = server.arg("swap_BB_SB").toInt();
                    if (server.hasArg("compass_trim_enabled")) mainData.compass_trim_enabled = (server.arg("compass_trim_enabled").toInt() != 0);
                    if (server.hasArg("mech")) mainData.mechanicCorrection = server.arg("mech").toFloat();

                    pidRudderParameters(&mainData, SET);
                    pidSpeedParameters(&mainData, SET);
                    computeParameters(&mainData, SET);
                    int offset = (int)mainData.compassOffset;
                    CompassOffsetCorrection(&offset, SET);

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
                } else {
                    msg.ack = INF;
                }
            }
            // printf("Sending command %d to udpIn (local/forward to sub)\r\n", msg.cmd);
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
                if (server.hasArg("mech")) msg.mechanicCorrection = server.arg("mech").toFloat();
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
                    msg.revBB = server.arg("revBB").toInt();
                    msg.revSB = server.arg("revSB").toInt();
                    msg.swap_BB_SB = server.arg("swap_BB_SB").toInt();
                    if (server.hasArg("compass_trim_enabled")) msg.compass_trim_enabled = (server.arg("compass_trim_enabled").toInt() != 0);
                    if (server.hasArg("mech")) msg.mechanicCorrection = server.arg("mech").toFloat();
                    msg.ack = SET;
                } else {
                    msg.ack = GET;
                }
            } else if (cmdEnum == ADAPTIVE_TRIM) {
                if (server.hasArg("compass_trim")) msg.compass_trim = server.arg("compass_trim").toFloat();
                if (server.hasArg("compass_trim_enabled")) msg.compass_trim_enabled = (server.arg("compass_trim_enabled").toInt() != 0);
                msg.ack = SET;
            }

            xQueueSend(serOut, (void *)&msg, 10);
        }
        server.send(200, "text/plain", "OK"); 
    });

    server.begin();
    for (;;) {
        server.handleClient();
        if (ota) ArduinoOTA.handle();
        RoboStruct msgIdOut = {};
        if (xQueueReceive(udpOut, (void *)&msgIdOut, 1) == pdTRUE) {
            if (msgIdOut.IDs == 0) msgIdOut.IDs = espMac();
            if (msgIdOut.IDr == 0) msgIdOut.IDr = ROBOBASE;
            udp.broadcast(rfCode(&msgIdOut).c_str());
        }
        delay(1);
    }
}
