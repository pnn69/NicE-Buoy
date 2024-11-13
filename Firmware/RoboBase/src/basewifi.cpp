#include <WiFi.h>
#include <AsyncUDP.h>
#include <ArduinoOTA.h>
#include <RoboCompute.h>
#include "main.h"
#include "basewifi.h"

RoboStruct subwifiData;
// UdpMsg udpData;
static char udpData[MAXSTRINGLENG];
static char udpDataOut[MAXSTRINGLENG];
static RoboStruct subWifiIn;
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
    sprintf(buf, "Base_%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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
bool scan_for_wifi_ap(String ssipap, String ww, IPAddress *tmp)
{
    unsigned long timeout = millis();
    Serial.print("scan for for ap:");
    Serial.println(ssipap);
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
            if (WiFi.SSID(i) == ssipap.c_str())
            {
                Serial.print("Acces point foud, logging in...");
                WiFi.begin(ssipap.c_str(), ww.c_str());
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
                Serial.println(ssipap);
                *tmp = (WiFi.localIP());
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
                         if(stringUdpIn.length() < MAXSTRINGLENG)
                         {
                            if (verifyCRC(stringUdpIn))
                            {
                                stringUdpIn.toCharArray(udpData,stringUdpIn.length() + 1);
                                xQueueSend(udpIn, (void *)&udpData, 10); // notify main there is new data
                            }
                            else
                            {
                                Serial.println("crc error>" + stringUdpIn + "<");
                            }
                        } });
    }
}

//***************************************************************************************************
//  Setup WiFi queue
//***************************************************************************************************
unsigned long initwifiqueue(void)
{
    udpOut = xQueueCreate(10,  MAXSTRINGLENG);
    udpIn = xQueueCreate(10, MAXSTRINGLENG);
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
    String ssid = "";
    String ww = "";
    bool apswitch = false;
    IPAddress ipAddr;
    ssid = "NicE_WiFi";
    ww = "!Ni1001100110";
    if (scan_for_wifi_ap(ssid, ww, &ipAddr) == false)
    {
        esp_restart();
    }
    Serial.print("Logged in to AP:");
    Serial.print(ssid);
    Serial.print(" Ip adress:");
    Serial.println(ipAddr);
    setup_OTA();
    setupudp();
    Serial.print("WiFI task running!\r\n");
    //***************************************************************************************************
    //  WiFi main loop
    //***************************************************************************************************
    for (;;)
    {
        ArduinoOTA.handle();
        if (xQueueReceive(udpOut, (void *)&udpDataOut, 0) == pdTRUE)
        {
            String out = addCRCToString(String(udpDataOut));
            udp.broadcast(out.c_str());
        }
        delay(1);
    }
}