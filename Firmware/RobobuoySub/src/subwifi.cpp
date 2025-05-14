#include <WiFi.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <AsyncUDP.h>
#include <ArduinoOTA.h>
#include <RoboCompute.h>
#include "main.h"
#include "io_sub.h"
#include "datastorage.h"
#include "subwifi.h"

RoboStruct subwifiData;
// UdpMsg udpData;
static RoboStruct udpData;
static RoboStruct subWifiIn;
static RoboStruct subWifiOut;
AsyncUDP udp;
// static UdpData udpBuffer;
QueueHandle_t udpIn;
QueueHandle_t udpOut;

//***************************************************************************************************
//      Setup OTA
//***************************************************************************************************
bool setup_OTA()
{
    char ssidl[20];
    char buf[30];
    byte mac[6];
    WiFi.macAddress(mac);
    Serial.print("SETUP OTA...");
    sprintf(buf, "Sub_%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ArduinoOTA.setHostname(buf);
    ArduinoOTA.onStart([]()
                       {
    /* switch off all processes here!!!!! */
    Serial.println();
    Serial.println("Recieving new firmware now!"); });
    ArduinoOTA.onEnd([]()
                     {
    /* do stuff after update here!! */
    Serial.println("\nRecieving done!");
    Serial.println("Storing in memory and reboot!");
    Serial.println(); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); 
                            digitalWrite(PWRENABLE, 1); // enable powersupply
                        });
    ArduinoOTA.onError([](ota_error_t error)
                       { ESP.restart(); });
    /* setup the OTA server */
    ArduinoOTA.begin();
    Serial.println("...done!");
    Serial.print("OTA ID: ");
    Serial.println(buf);
    return true;
}

//***************************************************************************************************
//      Searce for WiFi ap
//***************************************************************************************************
bool scan_for_wifi_ap(String *ssidap, String ww)
{
    unsigned long timeout = millis();
    Serial.print("scan for for ap:");
    Serial.println(*ssidap);
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    Serial.print(n);
    Serial.println(" networks found");
    if (n == 0)
    {
        return false;
    }
    else
    {
        for (int i = 0; i < n; ++i)
        {
            String ssid = WiFi.SSID(i);
            if (ssid.indexOf(*ssidap) != -1)
            {
                Serial.print("Acces point foud, logging in...");
                WiFi.begin(ssid.c_str(), ww.c_str());
                while (WiFi.status() != WL_CONNECTED)
                {
                    delay(50);
                    Serial.print(".");
                    if (timeout + 60 * 1000 < millis())
                    {
                        esp_restart();
                    }
                }
                Serial.print(".\r\n");
                Serial.print("Loggend in to SSIS: ");
                Serial.println(*ssidap);
                Serial.print("IP address: ");
                Serial.println(WiFi.localIP());
                if (ssid.indexOf("PAIR_ME_") != -1)
                {
                    ssid.replace("PAIR_ME_", "BUOY_");
                    apParameters(&ssid, &ww, false); // store parameters
                    Serial.println("Network parameters stored:" + ssid);
                    Serial.println("Rebooting in 5 seconds");
                    delay(5000);
                    esp_restart();
                }
                return true;
            }
        }
    }
    return false;
}

//***************************************************************************************************
//      Setup Async UDP
//***************************************************************************************************
void setupudp(void)
{
    if (udp.listen(1001))
    {
        Serial.print("UDP server na IP: ");
        Serial.print(WiFi.localIP());
        Serial.println(" port: 1001");
        udp.onPacket([](AsyncUDPPacket packet)
                     {
                         String stringUdpIn = (const char *)packet.data();
                         RoboStruct udpDataIn = rfDeCode(stringUdpIn);
                        if (udpDataIn.IDs != -1 )
                        {
                            xQueueSend(udpIn, (void *)&udpDataIn, 10); // notify main there is new data
                        }
                        else
                        {
                            Serial.println("crc error>" + stringUdpIn + "<");
                        } });
    }
}

//***************************************************************************************************
//  Setup WiFi queue
//***************************************************************************************************
unsigned long initwifi(void)
{
    udpOut = xQueueCreate(10, sizeof(RoboStruct));
    udpIn = xQueueCreate(10, sizeof(RoboStruct));
    byte mac[6];
    WiFi.macAddress(mac);
    unsigned long tmp = 0;
    for (int i = 2; i < 6; i++)
    {
        tmp = (tmp << 8) | mac[i];
    }
    Serial.printf("ESP32-mac: %02x%02x%02x%02x%02x%02x\r\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return tmp;
}

/*
    Scan for accespoindts
*/
bool scan_for_wifi_ap(String ssipap)
{
    unsigned long timeout = millis();
    Serial.print("scan for for ap \"");
    Serial.print(ssipap);
    Serial.println("\"");
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i)
    {
        if (WiFi.SSID(i) == ssipap.c_str())
        {
            Serial.println("Acces point foud!");
            return true;
        }
    }
    Serial.println("Acces point not foud.....");
    return false;
}

uint8_t* getMacId(uint8_t* mac)
{
    WiFi.macAddress(mac);
    return mac;
}

//***************************************************************************************************
//  WiFi task
//***************************************************************************************************
void WiFiTask(void *arg)
{
    int wifiConfig = *((int *)arg);
    WiFiManager wm;
    String ssid = "";
    String ww = "";
    WiFi.mode(WIFI_STA);
    byte mac[6];
    char macStr[25];
    WiFi.macAddress(mac);
    sprintf(macStr, "RoboSub_%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    apParameters(&ssid, &ww, GET);
    bool ap = false; // indicator accespoint found
    if (ssid != "")
    {
        ap = scan_for_wifi_ap(ssid);
    }
    if (ap == false)
    {
        //wm.resetSettings();
        wm.autoConnect(macStr);
        // wm.setConfigPortalTimeout(60 * 10);
        // wm.setConfigPortalBlocking(true);
        // wm.startConfigPortal("RoboSubConfigMe");
        ssid = wm.getWiFiSSID();
        ww = wm.getWiFiPass();
        Serial.println("SSID:" + ssid + " WW:" + ww);
        apParameters(&ssid, &ww, SET);
        delay(100);
    }
    else
    {
        wm.autoConnect(ssid.c_str(), ww.c_str());
    }
    Serial.print("Logged in to AP \"");
    Serial.print(ssid);
    Serial.println("\"");
    setup_OTA();
    setupudp();
    Serial.print("WiFI task running!\r\n");
    //***************************************************************************************************
    //  WiFi main loop
    //***************************************************************************************************
    for (;;)
    {
        ArduinoOTA.handle();
        if (xQueueReceive(udpOut, (void *)&subwifiData, 0) == pdTRUE)
        {
            subwifiData.IDr = subwifiData.mac;
            subwifiData.IDs = subwifiData.mac;
            String out = rfCode(subwifiData);
            udp.broadcast(out.c_str());
        }
        delay(100);
    }
}