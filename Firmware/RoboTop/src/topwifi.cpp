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
// static unsigned long tstart, tstop;
static unsigned long lastUpdMsg = 0;

WebServer server(80);

/**
 * @brief Sets up Over-The-Air (OTA) update functionality.
 * 
 * Configures the OTA hostname based on the device's MAC address and
 * sets up callbacks for start, end, progress, and error events.
 * 
 * @return true if OTA was successfully initialized.
 */
bool setup_OTA()
{
    char ssidl[20];
    char buf[30];
    byte mac[6];
    WiFi.macAddress(mac);
    Serial.print("SETUP OTA...");
    sprintf(buf, "Top_%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ArduinoOTA.setHostname(buf);
    ArduinoOTA.onStart([]()
                       {
    /* switch off all processes here!!!!! */
    Serial.println();
    Serial.println("Receiving new firmware now!"); });
    ArduinoOTA.onEnd([]()
                     {
    /* do stuff after update here!! */
    Serial.println("Receiving done!");
    Serial.println("Storing in memory and reboot!");
    Serial.println(); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
    ArduinoOTA.onError([](ota_error_t error){ ESP.restart(); });
    /* setup the OTA server */
    ArduinoOTA.begin();
    Serial.println("...done!");
    Serial.print("OTA ID: ");
    Serial.println(buf);
    return true;
}

/**
 * @brief Scans for a specific Wi-Fi Access Point and connects to it.
 * 
 * @param ssipap The SSID of the target access point.
 * @param ww The password for the target access point.
 * @param tmp Pointer to store the local IP address if connection is successful.
 * @return true if the access point was found and connected to, false otherwise.
 */
bool scan_for_wifi_ap(String ssipap, String ww, IPAddress *tmp)
{
    unsigned long timeout = millis();
    Serial.print("scan for for ap:");
    Serial.println(ssipap);
    
    while (millis() - timeout < 120000)
    {
        int n = WiFi.scanNetworks();
        if (n > 0)
        {
            for (int i = 0; i < n; ++i)
            {
                if (WiFi.SSID(i) == ssipap.c_str())
                {
                    Serial.print("Access point found, logging in...");
                    WiFi.begin(ssipap.c_str(), ww.c_str());
                    while (WiFi.status() != WL_CONNECTED)
                    {
                        delay(50);
                        Serial.print(".");
                        if (timeout + 120 * 1000 < millis())
                        {
                            esp_restart();
                        }
                    }
                    Serial.print(".");
                    Serial.print("Logend in to SSID: ");
                    Serial.println(ssipap);
                    *tmp = (WiFi.localIP());
                    return true;
                }
            }
        }
        Serial.println("Access point not found, retrying...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    Serial.println("Access point not found after 2 minutes.");
    return false;
}

/**
 * @brief Sets up a Wi-Fi Access Point with a static IP.
 * 
 * @param ap The SSID for the new access point.
 * @param ww The password for the new access point.
 * @param tmp Pointer to store the access point's IP address.
 */
void setup_wifi_ap(String ap, String ww, IPAddress *tmp)
{
    WiFi.mode(WIFI_AP); // Set Wi-Fi mode to Access Point
    Serial.println("Setting up access point now");

    // Configure static IP
    IPAddress local_IP(192, 168, 1, 84); // Desired static IP address
    IPAddress subnet(255, 255, 255, 0);  // Subnet mask
    IPAddress gateway(192, 168, 1, 5);   // Gateway address
    IPAddress primaryDNS(0, 0, 0, 0);    // Primary DNS (optional)
    IPAddress secondaryDNS(0, 0, 0, 0);  // Secondary DNS (optional)

    // Set the static IP address if possible
    if (!WiFi.softAPConfig(local_IP, gateway, subnet))
    {
        Serial.println("Failed to configure static IP for AP");
    }

    // Start the access point with the given SSID and password
    if (WiFi.softAP(ap.c_str(), ww.c_str()))
    {
        Serial.print("AP SSID: ");
        Serial.println(ap);

        // Get the AP's IP address and store it in the pointer tmp
        *tmp = WiFi.softAPIP();
        Serial.print("AP IP address: ");
        Serial.println(*tmp);
    }
    else
    {
        Serial.println("Failed to start access point");
    }
}
/**
 * @brief Initializes the Async UDP listener on the specified port.
 * 
 * Sets up a callback for incoming UDP packets, decodes them, and
 * sends them to the udpIn queue if they are valid and not from the local device.
 * 
 * @param poort The UDP port to listen on.
 * @return true if the UDP listener was successfully started.
 */
bool udp_setup(int poort)
{
    if (udp.listen(poort))
    {
        Serial.print("Udp port: ");
        Serial.println(poort);
        udp.onPacket([](AsyncUDPPacket packet)
                     {
                        String stringUdpIn((const char *)packet.data(), packet.length());
                        RoboStruct udpDataIn;
                        rfDeCode(stringUdpIn,&udpDataIn);
                        if (udpDataIn.IDs != -1 && udpDataIn.IDs != mac) // ignore own messages
                        {
                            xQueueSend(udpIn, (void *)&udpDataIn, 10); // notify main there is new data
                            if (wifiCollorUtil.color != CRGB::DarkBlue)
                            {
                                wifiCollorUtil.blink = BLINK_SLOW;
                                wifiCollorUtil.color = CRGB::DarkBlue;
                                xQueueSend(ledUtil, (void *)&wifiCollorUtil, 0); // update GPS led
                            }
                            lastUpdMsg = millis();
                        }
                        else
                        {
                            Serial.println("crc error: " + stringUdpIn);
                        } });
        return true;
    }
    return false;
}

/**
 * @brief Broadcasts data over UDP.
 * 
 * @param data The string data to be broadcasted.
 */
void udpSend(String data)
{
    udp.broadcast(data.c_str());
}

/**
 * @brief Retrieves the device's MAC address as an unsigned long.
 * 
 * The MAC address is cached after the first retrieval.
 * 
 * @return The last 4 bytes of the MAC address as an unsigned long.
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
 * 
 * Also retrieves and returns the device's MAC address.
 * 
 * @return The device's MAC address (via espMac()).
 */
unsigned long initwifiqueue(void)
{
    udpOut = xQueueCreate(10, sizeof(RoboStruct));
    udpIn = xQueueCreate(10, sizeof(RoboStruct));
    return espMac();
}

/**
 * @brief Main Wi-Fi management task.
 * 
 * This task handles:
 * - Wi-Fi connection (scanning or setting up AP).
 * - Initializing OTA and UDP.
 * - Setting up the Web Server and its endpoints (/, /data, /command).
 * - Handling web client requests and UDP broadcasting in a loop.
 * 
 * @param arg Pointer to an integer representing the Wi-Fi configuration (1 for pairing mode).
 */
void WiFiTask(void *arg)
{
    byte mac[6];
    char macStr[20];
    WiFi.macAddress(mac);
    sprintf(macStr, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    int wifiConfig = *((int *)arg);
    unsigned long nwUpdate = millis();
    unsigned char numClients = 0;
    String ap = "";
    String apww = "";
    unsigned long nextSamp = millis();
    if (wifiConfig == 1)
    {
        ap = "PAIR_ME_";
        ap += macStr;
        setup_wifi_ap(ap, apww, &ipTop);
    }
    else // try to find accespoint NicE_WiFi. If no succes make a accecpoint BUOY_[MAC]
    {
        wifiCollorUtil.color = CRGB::LightBlue;
        wifiCollorUtil.blink = FADE_ON;
        wifiCollorUtil.fadeAmount = 5;
        xQueueSend(ledStatus, (void *)&wifiCollorUtil, 10); // update util led
        ap = "NicE_WiFi";
        apww = "!Ni1001100110";
        if (scan_for_wifi_ap(ap, apww, &ipTop) == false)
        {
            ap = "BUOY_";
            ap += macStr;
            apww = "";
            setup_wifi_ap(ap, apww, &ipTop);
        }
    }
    Serial.print("IP address: ");
    Serial.println(ipTop);
    ota = setup_OTA();
    udp_setup(1001);
    wifiCollorUtil.color = CRGB::Black;
    wifiCollorUtil.blink = BLINK_OFF;
    xQueueSend(ledStatus, (void *)&wifiCollorUtil, 10); // update util led

    if(!SPIFFS.begin(true)){
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    server.on("/", HTTP_GET, [](){
        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        server.sendHeader("Pragma", "no-cache");
        server.sendHeader("Expires", "-1");
        File file = SPIFFS.open("/index.html", "r");
        if(!file){
            server.send(404, "text/plain", "File not found");
            return;
        }
        server.streamFile(file, "text/html");
        file.close();
    });

    server.on("/data", HTTP_GET, []() {
        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        String json = "{\"buoys\":[";
        
        // Buoy 1 (Top Buoy local data)
        json += "{";
        json += "\"ID\":\"" + String(espMac(), HEX) + "\",";
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
        json += "\"SubPerc\":\"" + String(mainData.subAccuP) + "\",";
        json += "\"PIDI\":\"" + String(mainData.ip, 2) + "\",";
        json += "\"PIDR\":\"" + String(mainData.ir, 2) + "\",";
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
        json += "\"icmCompassOffset\":\"" + String(mainData.icmCompassOffset, 2) + "\",";
        json += "\"Lat\":\"" + String(mainData.lat, 6) + "\",";
        json += "\"Lng\":\"" + String(mainData.lng, 6) + "\",";
        json += "\"GpsFix\":\"" + String(mainData.gpsFix) + "\"";
        json += "},";

        // Buoy 2 and 3 (from buoyPara placeholders)
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
            json += "\"SubPerc\":\"" + String(buoyPara[i].subAccuP) + "\",";
            json += "\"PIDI\":\"" + String(buoyPara[i].ip, 2) + "\",";
            json += "\"PIDR\":\"" + String(buoyPara[i].ir, 2) + "\",";
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
            json += "\"Lat\":\"" + String(mainData.lat, 6) + "\",";
            json += "\"Lng\":\"" + String(mainData.lng, 6) + "\",";
            json += "\"GpsFix\":\"" + String(buoyPara[i].gpsFix) + "\"";
            json += "}";
            if (i < 2) json += ",";
        }
        json += "]}";
        server.send(200, "application/json", json);
    });

    server.on("/command", HTTP_GET, []() {
        if (!server.hasArg("bid") || !server.hasArg("cmd")) {
            server.send(400, "text/plain", "Missing bid or cmd");
            return;
        }

        int bid = server.arg("bid").toInt();
        String cmdStr = server.arg("cmd");
        int cmdEnum = NOCMD;
        int statusEnum = -1;

        if (cmdStr == "LOCK") {
            if (bid == 1 && (mainData.status == LOCKED || mainData.status == LOCKING)) { cmdEnum = IDELING; statusEnum = IDLE; }
            else if (bid != 1 && (buoyPara[bid-1].status == LOCKED || buoyPara[bid-1].status == LOCKING)) cmdEnum = IDLE;
            else { cmdEnum = LOCKING; statusEnum = IDLE; }
        }
        else if (cmdStr == "DOCK") {
            if (bid == 1 && (mainData.status == DOCKING || mainData.status == DOCKED)) { cmdEnum = IDELING; statusEnum = IDLE; }
            else if (bid != 1 && (buoyPara[bid-1].status == DOCKING || buoyPara[bid-1].status == DOCKED)) cmdEnum = IDLE;
            else { cmdEnum = DOCKING; statusEnum = IDLE; }
        }
        else if (cmdStr == "SETUP") cmdEnum = SETUPDATA;
        else if (cmdStr == "IDLE") { cmdEnum = IDELING; statusEnum = IDLE; }
        else if (cmdStr == "DIRDIST") { cmdEnum = DIRDIST; statusEnum = IDLE; }
        else if (cmdStr == "MAP") cmdEnum = NEWBUOYPOS;
        else if (cmdStr == "PIDRUDDER") cmdEnum = PIDRUDDERSET;
        else if (cmdStr == "PIDSPEED") cmdEnum = PIDSPEEDSET;
        else if (cmdStr == "LIMITS") cmdEnum = MAXMINPWRSET;
        else if (cmdStr == "COMPASSOFFSET") cmdEnum = STORE_COMPASS_OFFSET;
        else if (cmdStr == "CALIB_COMPASS") cmdEnum = INFIELD_CALIBRATE;
        else if (cmdStr == "MANUAL_CALIB") cmdEnum = CALIBRATE_MAGNETIC_COMPASS;
        else if (cmdStr == "CALIB_OFFSET") cmdEnum = INFIELD_OFFSET_CALIBRATE;
        else if (cmdStr == "COMPUTESTART") cmdEnum = COMPUTESTART;
        else if (cmdStr == "COMPUTETRACK") cmdEnum = COMPUTETRACK;

        if (bid == 1) {
            // For movement commands, we inject an RF-like message into udpIn so main.cpp handles coordinate math
            if (cmdEnum == DIRDIST || cmdEnum == LOCKING || cmdEnum == DOCKING || cmdEnum == IDELING) {
                RoboStruct msg;
                msg.IDs = 0x99; // Simulate an incoming command from Python UI
                msg.IDr = espMac();
                msg.cmd = (msg_t)cmdEnum;
                msg.status = (msg_t)statusEnum;
                msg.ack = LORAGETACK; // Python uses LORAGETACK(3) for these movement states, except DIRDIST which uses LORAINF(6)
                if (cmdEnum == DIRDIST) msg.ack = LORAINF;
                
                if (cmdEnum == DIRDIST) {
                    msg.tgDir = server.arg("dir").toFloat();
                    msg.tgDist = server.arg("dist").toFloat();
                }
                
                xQueueSend(udpIn, (void *)&msg, 10);
            }
            
            // For setup commands, we still modify mainData directly and send to Sub
            if (cmdEnum == PIDRUDDERSET) {
                mainData.Kpr = server.arg("p").toFloat();
                mainData.Kir = server.arg("i").toFloat();
                mainData.Kdr = server.arg("d").toFloat();
                mainData.ack = LORASET;
            }
            else if (cmdEnum == PIDSPEEDSET) {
                mainData.Kps = server.arg("p").toFloat();
                mainData.Kis = server.arg("i").toFloat();
                mainData.Kds = server.arg("d").toFloat();
                mainData.ack = LORASET;
            }
            else if (cmdEnum == MAXMINPWRSET) {
                mainData.maxSpeed = server.arg("max").toInt();
                mainData.minSpeed = server.arg("min").toInt();
                mainData.pivotSpeed = server.arg("pivot").toFloat();
                mainData.ack = LORASET;
            }
            else if (cmdEnum == STORE_COMPASS_OFFSET) {
                mainData.compassOffset = server.arg("offset").toFloat();
                mainData.icmCompassOffset = server.arg("icmoffset").toFloat();
                mainData.ack = LORASET;
            }
            
            if (cmdEnum == SETUPDATA || cmdEnum == PIDRUDDERSET || cmdEnum == PIDSPEEDSET ||
                cmdEnum == MAXMINPWRSET || cmdEnum == STORE_COMPASS_OFFSET ||
                cmdEnum == INFIELD_CALIBRATE || cmdEnum == INFIELD_OFFSET_CALIBRATE || cmdEnum == CALIBRATE_MAGNETIC_COMPASS) {

                mainData.cmd = (msg_t)cmdEnum;                
                if (cmdEnum == SETUPDATA) {
                    mainData.ack = LORAGET;
                    mainData.IDr = BUOYIDALL;
                    mainData.IDs = espMac();
                } else if (cmdEnum == CALIBRATE_MAGNETIC_COMPASS || cmdEnum == INFIELD_CALIBRATE || cmdEnum == INFIELD_OFFSET_CALIBRATE) {
                    mainData.ack = LORAGETACK;
                }
                xQueueSend(serOut, (void *)&mainData, 10);
            }
        } else if (bid >= 2 && bid <= 3) {
            RoboStruct msg;
            msg.IDs = espMac();
            msg.IDr = buoyPara[bid - 1].IDs;
            msg.cmd = (cmdEnum != NOCMD) ? cmdEnum : statusEnum;
            
            if (cmdEnum == DIRDIST) {
                msg.tgDir = server.arg("dir").toFloat();
                msg.tgDist = server.arg("dist").toFloat();
            }
            else if (cmdEnum == PIDRUDDERSET) {
                msg.Kpr = server.arg("p").toFloat();
                msg.Kir = server.arg("i").toFloat();
                msg.Kdr = server.arg("d").toFloat();
            }
            else if (cmdEnum == PIDSPEEDSET) {
                msg.Kps = server.arg("p").toFloat();
                msg.Kis = server.arg("i").toFloat();
                msg.Kds = server.arg("d").toFloat();
            }
            else if (cmdEnum == MAXMINPWRSET) {
                msg.maxSpeed = server.arg("max").toInt();
                msg.minSpeed = server.arg("min").toInt();
                msg.pivotSpeed = server.arg("pivot").toFloat();
            }
            else if (cmdEnum == STORE_COMPASS_OFFSET) {
                msg.compassOffset = server.arg("offset").toFloat();
            }

            if (msg.IDr != 0) {
                xQueueSend(udpOut, (void *)&msg, 10);
                xQueueSend(loraOut, (void *)&msg, 10);
            }
        }

        server.send(200, "text/plain", "OK");
    });

    server.onNotFound([](){
        server.send(404, "text/plain", "File not found");
    });

    server.begin();
    printf("WiFI task running!\r\n");
    /*
        WiFI loop
    */
    for (;;)
    {
        server.handleClient();
        if (ota == true)
        {
            ArduinoOTA.handle();
        }

        if (WiFi.softAPgetStationNum() != numClients)
        {
            numClients = WiFi.softAPgetStationNum();
            Serial.print("You found me!\r\n");
            if (ap.indexOf("PAIR_ME_") != -1) // reboot if in pairing mode only
            {
                Serial.println("Rebooting in 5 seconds");
                delay(5000);
                esp_restart();
            }
        }

        if (xQueueReceive(udpOut, (void *)&msgIdOut, 1) == pdTRUE)
        {
            if (msgIdOut.IDs == 0) msgIdOut.IDs = espMac();
            if (msgIdOut.IDr == 0) msgIdOut.IDr = ROBOBASE; 
            String out = rfCode(&msgIdOut);
            udp.broadcast(out.c_str());
        }
        delay(1);
    }
}