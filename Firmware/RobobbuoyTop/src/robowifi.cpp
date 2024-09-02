#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <AsyncUDP.h>
#include "robowifi.h"
#include "leds.h"
#include "datastorage.h"
static UdpData udpBuffer;
static LedData gpsCollorUtil;
static bool ap = false;
static bool ota = false;
static int8_t id = 0;
static char gpsDataIn[100];
const char *ssid = "NicE_WiFi"; // Change this to your WiFi SSID
const char *password = "!Ni1001100110";
// const char *ssidap = "Buoy_4";
char ssidap[] = "Buoy_4";
IPAddress ipTop;
AsyncUDP udp;
QueueHandle_t udpOut;

/*
    Scan for networks
*/
bool scan_for_wifi_ap(void)
{
    Serial.println("scan for AP");
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    if (n == 0)
    {
        return false;
    }
    else
    {
        for (int i = 0; i < n; ++i)
        {
            if (WiFi.SSID(i) == "NicE_WiFi")
            {
                WiFi.mode(WIFI_STA);
                Serial.print("AP NicE_WiFi foud, logging in...");
                WiFi.begin("NicE_WiFi", password);
                while (WiFi.status() != WL_CONNECTED)
                {
                    delay(50);
                    Serial.print(".");
                }
                Serial.print(".\r\n");
                Serial.print("Loggend in to SSIS: ");
                Serial.println(ssid);
                Serial.print("IP address: ");
                Serial.println(WiFi.localIP());
                return true;
            }
        }
        delay(10);
    }
    return false;
}

/*
    Setup accecpoint
*/
bool setup_wifi_ap(char set)
{
    char buf[20];
    WiFi.mode(WIFI_MODE_NULL);
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
    Serial.println("\r\nscan for AP");
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    for (int i = 0; i < n; ++i)
    {
        if (WiFi.SSID(i) == "NicE_WiFi_tralal")
        {
            ap = false;
            WiFi.mode(WIFI_STA);
            Serial.print("AP NicE_WiFi foud, logging in...");
            WiFi.begin("NicE_WiFi", "!Ni1001100110");
            while (WiFi.status() != WL_CONNECTED)
            {
                delay(50);
                Serial.print(".");
            }
            Serial.print(".\r\n");
            Serial.print("Loggend in to SSIS: ");
            Serial.println(ssid);
            Serial.print("IP address: ");
            Serial.println(WiFi.localIP());
            ipTop = WiFi.localIP();
        }
    }
    if (ap == false)
    {
        WiFi.mode(WIFI_AP);
        WiFi.softAP("BUOY_1", "");
        Serial.print("Accespoint setu as: BUOY_1\r\n");
        ipTop = WiFi.softAPIP();
        ap = true;
    }
    return ap;
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
                            gpsCollorUtil.color = CRGB::Green;
                            gpsCollorUtil.blink = false;
                            xQueueSend(ledUtil, (void *)&gpsCollorUtil, 10);     // update util led
                            printf("pong in\n\r");
                         } });
        return true;
    }
    return false;
}

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
    sprintf(ssidl, "RoboBuoy%d", id);
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
    Serial.println("\r\nscan for AP");
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    for (int i = 0; i < n; ++i)
    {
        if (WiFi.SSID(i) == "NicE_WiFi_tralal")
        {
            ap = false;
            WiFi.mode(WIFI_STA);
            Serial.print("AP NicE_WiFi foud, logging in...");
            WiFi.begin("NicE_WiFi", "!Ni1001100110");
            while (WiFi.status() != WL_CONNECTED)
            {
                delay(50);
                Serial.print(".");
            }
            Serial.print(".\r\n");
            Serial.print("Loggend in to SSIS: ");
            Serial.println(ssid);
            ipTop = WiFi.localIP();
        }
    }
    if (ap == false)
    {
        Serial.println("Setting up AP now");
        WiFi.mode(WIFI_AP);
        WiFi.softAP("BUOY_1", "");
        Serial.print("AP Name SSID: ");
        Serial.println(WiFi.softAPSSID());
        Serial.print("AP IP Address: ");
        Serial.println(WiFi.softAPIP());
        Serial.print("AP Hostname: ");
        Serial.println(WiFi.softAPgetHostname());
        ipTop = WiFi.softAPIP();
        ap = true;
    }

    Serial.print("IP address: ");
    Serial.println(ipTop);
    udp_setup(1001);

    // if (udp.listen(1001))
    // {
    //     Serial.println("Udp port: 1001");
    //     udp.onPacket([](AsyncUDPPacket packet)
    //                  {
    //                      String myString = (const char *)packet.data();
    //                      if (myString.startsWith("pong"))
    //                      {
    //                         gpsCollorUtil.color = CRGB::Green;
    //                         gpsCollorUtil.blink = false;
    //                         xQueueSend(ledUtil, (void *)&gpsCollorUtil, 10);     // update util led
    //                         printf("pong in\n\r");
    //                      } });
    // }

    printf("WiFI task running!\r\n");
    for (;;)
    {

        if (xQueueReceive(udpOut, (void *)&udpBuffer, 0) == pdTRUE)
        {
            gpsCollorUtil.color = CRGB::Red;
            gpsCollorUtil.blink = false;
            xQueueSend(ledUtil, (void *)&gpsCollorUtil, 10); // update util led
            udp.broadcast(udpBuffer.msg);
            printf("ping out\n\r");
        }

        // ArduinoOTA.handle();
        delay(1);
    }
}