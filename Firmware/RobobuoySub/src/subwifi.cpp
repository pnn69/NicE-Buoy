#include <WiFi.h>
#include <AsyncUDP.h>
#include <ArduinoOTA.h>
#include <RoboCalc.h>
#include "main.h"
#include "datastorage.h"
#include "leds.h"
#include "subwifi.h"
#include "RoboCodeDecode.h"

RoboStruct subwifiData;
static int8_t buoyId;
AsyncUDP udp;
static UdpData udpBuffer;
QueueHandle_t udpIn;
QueueHandle_t udpOut;
static LedData wifiLedStatus;
static PwrData wifiPwrData;
static unsigned long lastPing = millis();

bool ota = false;

/*
    Setup OTA
*/
bool setup_OTA()
{
    char ssidl[20];
    char buf[30];
    byte mac[6];
    WiFi.macAddress(mac);
    memBuoyId(&buoyId, true);
    Serial.print("SETUP OTA...");
    sprintf(buf, "Buoy_%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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

bool scan_for_wifi_ap(String *ssidap, String ww)
{
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

/*
    Setup Async UDP
*/
void setupudp(void)
{
    if (udp.listen(1001))
    {
        Serial.print("UDP server na IP: ");
        Serial.print(WiFi.localIP());
        Serial.println(" port: 1001");
        udp.onPacket([](AsyncUDPPacket packet)
                     {
                         String stringUdp = (const char *)packet.data();
                         if (stringUdp.startsWith("ping"))
                         {
                             packet.printf("pong"); //
                             wifiLedStatus.color = CRGB::DarkBlue;
                             wifiLedStatus.blink = BLINK_FAST;
                             xQueueSend(ledStatus, (void *)&wifiLedStatus, 10); // update util led
                             lastPing = millis();
                         }
                         else
                         {
                            RoboDecode(stringUdp,subwifiData);
                            xQueueSend(udpIn, (void *)&subwifiData, 10); // update WiFi
                         } });
    }
}

bool initwifiqueue(void)
{
    udpOut = xQueueCreate(10, sizeof(RoboStruct));
    if (udpOut == NULL)
    {
        printf("Queue udpOut could not be created. %p\\r\n", udpOut);
        return false;
    }
    else
    {
        printf("Queue udpOut created.\r\n");
    }
    udpIn = xQueueCreate(10, sizeof(RoboStruct));
    if (udpIn == NULL)
    {
        printf("Queue udpOut could not be created. %p\\r\n", udpOut);
        return false;
    }
    else
    {
        printf("Queue udpOut created.\r\n");
    }
    return true;
}

/*
    WiFi task
*/
void WiFiTask(void *arg)
{
    int msg = -1;
    int wifiConfig = *((int *)arg);
    String ssid = "";
    String ww = "";
    bool apswitch = false;
    unsigned long nextSamp = millis();
    apParameters(&ssid, &ww, true);
    if (wifiConfig == 1)
    {
        wifiPwrData.bb = CRGB::DarkBlue;
        wifiPwrData.sb = CRGB::DarkBlue;
        wifiPwrData.blinkBb = BLINK_FAST;
        wifiPwrData.blinkSb = BLINK_SLOW;
        xQueueSend(ledPwr, (void *)&wifiPwrData, 10); // update util led
        ssid = "PAIR_ME_";
        ww = "";
        while (scan_for_wifi_ap(&ssid, ww) == false)
        {
            Serial.println("Try again with ssid: " + String(ssid));
        }
        wifiPwrData.bb = CRGB::DarkBlue;
        wifiPwrData.sb = CRGB::DarkBlue;
        wifiPwrData.blinkBb = BLINK_OFF;
        wifiPwrData.blinkSb = BLINK_SLOW;
        xQueueSend(ledPwr, (void *)&wifiPwrData, 10); // update util led
    }
    else
    {
        wifiPwrData.bb = CRGB::DarkGreen;
        wifiPwrData.sb = CRGB::DarkGreen;
        wifiPwrData.blinkBb = BLINK_FAST;
        wifiPwrData.blinkSb = BLINK_SLOW;
        xQueueSend(ledPwr, (void *)&wifiPwrData, 10); // update util led
        apParameters(&ssid, &ww, true);               // get ap parameters
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
        }
        wifiPwrData.bb = CRGB::DarkGreen;
        wifiPwrData.sb = CRGB::DarkGreen;
        wifiPwrData.blinkBb = BLINK_OFF;
        wifiPwrData.blinkSb = BLINK_SLOW;
        xQueueSend(ledPwr, (void *)&wifiPwrData, 10); // update util led
    }
    Serial.print("Logged in to AP:");
    Serial.println(ssid);
    ota = setup_OTA();
    setupudp();
    udp.broadcast("Anyone here?");
    Serial.print("WiFI task running!\r\n");
    /*
        WiFi main loop
    */
    for (;;)
    {
        if (xQueueReceive(udpOut, (void *)&subwifiData, 0) == pdTRUE)
        {
            String out = RoboCode(subwifiData);
            addCRCToString(out);
            udp.broadcast(out.c_str());
        }

        if (ota == true)
        {
            ArduinoOTA.handle();
        }
        if (nextSamp < millis())
        {
            nextSamp = 1000 + millis();
        }
        delay(10);
    }
}