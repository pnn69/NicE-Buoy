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
 * @brief Scans for ROBOBUOY first, then falls back to NiCe_WiFi/NicE_WiFi.
 */
bool scan_and_connect_wifi(IPAddress *tmp)
{
    unsigned long timeout = millis();
    Serial.println("Starting WiFi scan and connect sequence...");

    // Set STA mode and disconnect to ensure reliable scanning
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    // Loop for up to 30 seconds
    while (millis() - timeout < 30000)
    {
        Serial.println("Scanning for networks...");
        int n = WiFi.scanNetworks();
        Serial.printf("Scan done: %d networks found\n", n);

        if (n > 0)
        {
            bool robobuoy_found = false;
            bool nice_wifi_found = false;
            String nice_wifi_ssid = "";

            for (int i = 0; i < n; ++i)
            {
                String ssid = WiFi.SSID(i);
                if (ssid == "ROBOBUOY")
                {
                    robobuoy_found = true;
                }
                else if (ssid == "NicE_WiFi")
                {
                    nice_wifi_found = true;
                    nice_wifi_ssid = ssid;
                }
            }

            if (robobuoy_found)
            {
                Serial.println("Found 'ROBOBUOY'. Attempting connection...");
                WiFi.begin("ROBOBUOY", "");
                unsigned long conn_timeout = millis();
                while (WiFi.status() != WL_CONNECTED)
                {
                    delay(500);
                    Serial.print(".");
                    if (millis() - conn_timeout > 15000)
                    {
                        Serial.println("\nConnection to 'ROBOBUOY' timed out.");
                        break;
                    }
                }
                if (WiFi.status() == WL_CONNECTED)
                {
                    Serial.println("\nCONNECTED to 'ROBOBUOY'");
                    WiFi.setSleep(WIFI_PS_NONE); // Disable power-saving sleep
                    *tmp = WiFi.localIP();
                    Serial.print("WiFi IP address: ");
                    Serial.println(*tmp);
                    return true;
                }
                WiFi.disconnect();
                delay(100);
            }
            else if (nice_wifi_found)
            {
                Serial.printf("Found '%s'. Attempting connection...\n", nice_wifi_ssid.c_str());
                WiFi.begin(nice_wifi_ssid.c_str(), "!Ni1001100110");
                unsigned long conn_timeout = millis();
                while (WiFi.status() != WL_CONNECTED)
                {
                    delay(500);
                    Serial.print(".");
                    if (millis() - conn_timeout > 15000)
                    {
                        Serial.println("\nConnection to NiCe_WiFi timed out.");
                        break;
                    }
                }
                if (WiFi.status() == WL_CONNECTED)
                {
                    Serial.printf("\nCONNECTED to %s\n", nice_wifi_ssid.c_str());
                    WiFi.setSleep(WIFI_PS_NONE); // Disable power-saving sleep
                    *tmp = WiFi.localIP();
                    Serial.print("WiFi IP address: ");
                    Serial.println(*tmp);
                    return true;
                }
                WiFi.disconnect();
                delay(100);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    Serial.println("WiFi connection sequence completed without success.");
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
        
        if (!scan_and_connect_wifi(&ip)) {
            // Setup Access Point for Top: TOP_(MAC)
            byte mac[6];
            char macUpper[20];
            WiFi.macAddress(mac);
            sprintf(macUpper, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
            ap = "TOP_";
            ap += macUpper;
            apww = "";
            setup_wifi_ap(ap, apww, &ip);
            runBackgroundReconnection = false;
        } else {
            runBackgroundReconnection = true;
        }
    }
    
    wifiCollorUtil.color = CRGB::Black;
    wifiCollorUtil.blink = BLINK_OFF;
    xQueueSend(ledStatus, (void *)&wifiCollorUtil, 10);
    
    ota = setup_OTA();
    udp_setup(1001);
    SPIFFS.begin(true);

    server.on("/", HTTP_GET, [](){
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
        json += "\"dockAppDist\":\"" + String(mainData.dockApproachDist) + "\",";
        json += "\"dockAppDir\":\"" + String(mainData.dockApproachDir) + "\",";
        json += "\"dockToWP\":\"" + String(mainData.dockingToWaypoint ? "true" : "false") + "\",";
        json += "\"TgLat\":\"" + String(mainData.tgLat, 6) + "\",";
        json += "\"TgLng\":\"" + String(mainData.tgLng, 6) + "\",";
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
            json += "\"mechanicCorrection\":\"" + String(buoyPara[i].mechanicCorrection, 2) + "\",";
            json += "\"compass_trim_enabled\":\"" + String(buoyPara[i].compass_trim_enabled ? "true" : "false") + "\",";
            json += "\"compass_trim\":\"" + String(buoyPara[i].compass_trim, 3) + "\",";
            json += "\"dockAppDist\":\"" + String(buoyPara[i].dockApproachDist) + "\",";
            json += "\"dockAppDir\":\"" + String(buoyPara[i].dockApproachDir) + "\",";
            json += "\"dockToWP\":\"" + String(buoyPara[i].dockingToWaypoint ? "true" : "false") + "\",";
            json += "\"TgLat\":\"" + String(buoyPara[i].tgLat, 6) + "\",";
            json += "\"TgLng\":\"" + String(buoyPara[i].tgLng, 6) + "\",";
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
                    if (server.hasArg("dockAppDist")) mainData.dockApproachDist = server.arg("dockAppDist").toInt();
                    if (server.hasArg("dockAppDir")) mainData.dockApproachDir = server.arg("dockAppDir").toInt();
                    if (server.hasArg("dockToWP")) mainData.dockingToWaypoint = (server.arg("dockToWP").toInt() != 0);

                    pidRudderParameters(&mainData, MEM_PUT);
                    pidSpeedParameters(&mainData, MEM_PUT);
                    computeParameters(&mainData, MEM_PUT);
                    memDockApproach(&mainData, MEM_PUT);
                    int offset = (int)mainData.compassOffset;
                    CompassOffsetCorrection(&offset, MEM_PUT);

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

    server.begin();
    if (udpOut != NULL) {
        xQueueReset(udpOut);
    }
    
    static unsigned long lastBackgroundConnectAttempt = 0;
    static int connectAttemptState = 0; // 0 = Idle, 1 = Connecting to ROBOBUOY, 2 = Connecting to NicE_WiFi
    static unsigned long connectionStartedTime = 0;

    for (;;) {
        server.handleClient();
        if (ota) ArduinoOTA.handle();
        
        // Field-Aware Background Wi-Fi reconnection state machine
        if (runBackgroundReconnection) {
            if (WiFi.softAPgetStationNum() > 0) {
                // If a client is connected to our AP, immediately stop any background scans to prevent channel-jumping
                if (connectAttemptState != 0) {
                    Serial.println("[WiFi Background] Client connected to AP. Aborting background connection attempts to ensure AP stability.");
                    WiFi.disconnect();
                    connectAttemptState = 0;
                }
            }
            else if (WiFi.status() != WL_CONNECTED) {
                unsigned long current_time = millis();
                if (connectAttemptState == 0) {
                    // If we are idle and 45 seconds have passed since the last attempt, start the cycle
                    if (current_time - lastBackgroundConnectAttempt > 45000) {
                        lastBackgroundConnectAttempt = current_time;
                        connectAttemptState = 1;
                        connectionStartedTime = current_time;
                        Serial.println("[WiFi Background] Attempting to connect to 'ROBOBUOY'...");
                        WiFi.begin("ROBOBUOY", "");
                    }
                } else {
                    // We are currently waiting for a connection to establish
                    if (current_time - connectionStartedTime > 15000) {
                        // Timeout after 15 seconds
                        if (connectAttemptState == 1) {
                            // Switch to NicE_WiFi
                            connectAttemptState = 2;
                            connectionStartedTime = current_time;
                            Serial.println("[WiFi Background] 'ROBOBUOY' timed out. Attempting 'NicE_WiFi'...");
                            WiFi.begin("NicE_WiFi", "!Ni1001100110");
                        } else {
                            // All attempts failed, go back to idle
                            connectAttemptState = 0;
                            Serial.println("[WiFi Background] All background Wi-Fi connection attempts timed out.");
                            WiFi.disconnect(); // Clear active attempt
                        }
                    }
                }
            } else {
                // We are connected! Reset the state machine
                if (connectAttemptState != 0) {
                    Serial.printf("[WiFi Background] Successfully connected to SSID: %s! IP: %s\n", 
                                  WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
                    WiFi.setSleep(WIFI_PS_NONE); // Disable power-saving sleep
                    connectAttemptState = 0;
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
            udp.broadcast(rfCode(&msgIdOut).c_str());
        }
        delay(1);
    }
}
