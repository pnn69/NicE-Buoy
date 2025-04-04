#include <WiFi.h>
#include <AsyncUDP.h>
#include <ArduinoOTA.h>
#include <RoboCompute.h>
#include "main.h"
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
                          { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
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
unsigned long initwifiqueue(void)
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
    return tmp;
}

//***************************************************************************************************
//  WiFi task
//***************************************************************************************************
void WiFiTask(void *arg)
{
    int msg = -1;
    int wifiConfig = *((int *)arg);
    String ssid = "";
    String ww = "";
    bool apswitch = false;
    unsigned long wifiTimeOut = millis();
    // apParameters(&ssid, &ww, true);
    // if (wifiConfig == 1)
    // {
    //     wifiPwrData.bb = CRGB::DarkBlue;
    //     wifiPwrData.sb = CRGB::DarkBlue;
    //     wifiPwrData.blinkBb = BLINK_FAST;
    //     wifiPwrData.blinkSb = BLINK_SLOW;
    //     xQueueSend(ledPwr, (void *)&wifiPwrData, 10); // update util led
    //     ssid = "PAIR_ME_";
    //     ww = "";
    //     while (scan_for_wifi_ap(&ssid, ww) == false)
    //     {
    //         Serial.println("Try again with ssid: " + String(ssid));
    //     }
    //     wifiPwrData.bb = CRGB::DarkBlue;
    //     wifiPwrData.sb = CRGB::DarkBlue;
    //     wifiPwrData.blinkBb = BLINK_OFF;
    //     wifiPwrData.blinkSb = BLINK_SLOW;
    //     xQueueSend(ledPwr, (void *)&wifiPwrData, 10); // update util led
    // }
    // else
    // {
    //     wifiPwrData.bb = CRGB::DarkGreen;
    //     wifiPwrData.sb = CRGB::DarkGreen;
    //     wifiPwrData.blinkBb = BLINK_FAST;
    //     wifiPwrData.blinkSb = BLINK_SLOW;
    //     xQueueSend(ledPwr, (void *)&wifiPwrData, 10); // update util led
    //     apParameters(&ssid, &ww, true);               // get ap parameters

    ssid = "NicE_WiFi";
    ww = "!Ni1001100110";
    while (scan_for_wifi_ap(&ssid, ww) == false)
    {
        if (apswitch == false)
        {
            apParameters(&ssid, &ww, true); // get ap parameters
        }
        else
        {
            ssid = "NicE_WiFi";
            ww = "!Ni1001100110";
        }
        Serial.println("Try again with ssid: " + String(ssid));
        apswitch = !apswitch;
        if (wifiTimeOut + 10 * 1000 < millis())
        {
            esp_restart();
        }
    }
    Serial.print("Logged in to AP:");
    Serial.println(ssid);
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
        delay(1);
    }
}