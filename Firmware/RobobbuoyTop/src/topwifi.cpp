#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <AsyncUDP.h>
#include "topwifi.h"
#include "leds.h"
#include "datastorage.h"
static UdpData udpBuffer;
static LedData wifiCollorUtil;
static bool ota = false;
static int8_t id = 0;
static char gpsDataIn[100];
static IPAddress ipTop;
AsyncUDP udp;
QueueHandle_t udpOut;
static unsigned long tstart, tstop;
static unsigned long lastPong = millis();

/*
    Setup OTA
*/
bool setup_OTA()
{
    char ssidl[20];
    char buf[30];
    byte mac[6];
    WiFi.macAddress(mac);
    memBuoyId(&id, true);
    sprintf(ssidl, "Buoy_%03d", id);
    Serial.print("SETUP OTA...");
    sprintf(buf, "%s_%02x%02x%02x%02x%02x%02x", ssidl, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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

/*
    Scan for networks
*/
bool scan_for_wifi_ap(String ssipap, String ww)
{
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
                }
                Serial.print(".\r\n");
                Serial.print("Loggend in to SSIS: ");
                Serial.println(ssipap);
                Serial.print("IP address: ");
                Serial.println(WiFi.localIP());
                return true;
            }
        }
    }
    return false;
}

/*
    Setup accecpoint
*/
void setup_wifi_ap(char set, IPAddress *tmp)
{
    WiFi.mode(WIFI_AP);
    Serial.println("Setting up AP now");
    char buf[20];
    delay(1000);
    Serial.println("Accespoint setup");
    IPAddress local_IP(192, 168, 4, 1); // Your Desired Static IP Address
    IPAddress subnet(255, 255, 255, 0);
    IPAddress gateway(192, 168, 1, 5);
    IPAddress primaryDNS(0, 0, 0, 0);   // Not Mandatory
    IPAddress secondaryDNS(0, 0, 0, 0); // Not Mandatory
    memBuoyId(&id, true);
    if (set == 0)
    {
        sprintf(buf, "BUOY_%d", id);
    }
    else if (set == 1)
    {
        sprintf(buf, "PAIR_ME_%d", id);
    }
    WiFi.softAPsetHostname("BUOY_1");
    // WiFi.softAPsetHostname(buf);
    WiFi.softAP("BUOY_1");
    *tmp = WiFi.softAPIP();
}

/*
    Setup Async UDP
*/
bool udp_setup(int poort)
{
    if (udp.listen(poort))
    {
        Serial.print("Udp port: ");
        Serial.println(poort);
        udp.onPacket([](AsyncUDPPacket packet)
                     {
                         String myString = (const char *)packet.data();
                         if (myString.startsWith("pong"))
                         {
                            tstart = millis()-tstart;
                            digitalWrite(2,!digitalRead(2));
                            wifiCollorUtil.color = CRGB::DarkBlue;
                            wifiCollorUtil.blink = BLINK_FAST;
                            xQueueSend(ledUtil, (void *)&wifiCollorUtil, 10);     // update util led
                            //printf("flight time:%d msec\r\n",tstart);
                            lastPong = millis();
                         }else{ 
                            printf("%s\r\n",packet.data());
                         } });
        return true;
    }
    return false;
}

/*
    init WiFi que
*/
bool initwifiqueue(void)
{
    udpOut = xQueueCreate(10, sizeof(UdpMsg));
    if (udpOut == NULL)
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
    pinMode(2, OUTPUT);
    unsigned long nwUpdate = millis();
    unsigned char numClients = 0;
    String ap = "NicE_WiFi";
    String apww = "!Ni1001100110";
    unsigned long nextSamp = millis();
    if (scan_for_wifi_ap(ap, apww) == false)
    {
        setup_wifi_ap(0, &ipTop);
    }
    Serial.print("IP address: ");
    Serial.println(ipTop);
    ota = setup_OTA();
    udp_setup(1001);
    wifiCollorUtil.color = CRGB::Black;
    wifiCollorUtil.blink = BLINK_OFF;
    xQueueSend(ledStatus, (void *)&wifiCollorUtil, 10); // update util led
    printf("WiFI task running!\r\n");
    for (;;)
    {
        if (xQueueReceive(udpOut, (void *)&udpBuffer, 0) == pdTRUE)
        {
            xQueueSend(ledUtil, (void *)&wifiCollorUtil, 10); // update util led
            tstart = millis();
            if (udpBuffer.port == 0)
            {
                udp.broadcast(udpBuffer.msg);
            }
            else
            {
                udp.broadcastTo(udpBuffer.msg, udpBuffer.port);
            }
            // printf("ping out\n\r");
        }
        if (WiFi.softAPgetStationNum() != numClients)
        {
            numClients = WiFi.softAPgetStationNum();
            Serial.print("Num of Connected Clients : ");
            Serial.println(WiFi.softAPgetStationNum());
        }
        if (ota == true)
        {
            ArduinoOTA.handle();
        }
        if (lastPong + 1000 < millis())
        {
            lastPong += 100000;
            wifiCollorUtil.color = CRGB::OrangeRed;
            wifiCollorUtil.blink = 0;
            xQueueSend(ledUtil, (void *)&wifiCollorUtil, 10); // update util led
        }

        delay(100);
    }
}