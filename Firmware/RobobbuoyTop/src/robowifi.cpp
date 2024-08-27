#include <WiFi.h>
#include <ArduinoOTA.h>
#include <AsyncUDP.h>
#include "robowifi.h"
#include "topgeneral.h"
#include "leds.h"
#include "datastorage.h"
static LedData collorWiFi;
static UdpData udpOutBuffer;
static UdpData udpBuffer;
static LedData wiFiCollorStatus;
static bool web = false;
static bool ota = false;
static int8_t id = 0;
static char gpsDataIn[100];
const char *ssid = "NicE_WiFi"; // Change this to your WiFi SSID
const char *password = "!Ni1001100110";
//const char *ssidap = "Buoy_4";  
char ssidap[] = "Buoy_4";  

AsyncUDP udp;
QueueHandle_t udpOut;
WiFiServer server(80);

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
                return true;
            }
        }
        delay(10);
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
    server.begin();
    return true;
}

/*
    Setup accecpoint
*/
bool wifi_ap(char set)
{
    String ssidap;
    WiFi.mode(WIFI_MODE_NULL);
    delay(1000);
    WiFi.mode(WIFI_AP);
    Serial.println("Accespoint setup");
    IPAddress local_IP(192, 168, 4, 1); // Your Desired Static IP Address
    IPAddress subnet(255, 255, 255, 0);
    IPAddress gateway(192, 168, 1, 5);
    IPAddress primaryDNS(0, 0, 0, 0);   // Not Mandatory
    IPAddress secondaryDNS(0, 0, 0, 0); // Not Mandatory

    WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    memBuoyId(&id, true);
    if (set == 0)
    {
        ssidap =  "Buoy_" + String(id);
        //strcpy(ssidap, "Buoy_1");
    }
    else if (set == 1)
    {
        ssidap = "Pair_Me_" + String(id);
        //strcpy(ssidap, "Pair_Me_1");
    }
    Serial.print("Seting up AP: ");
    Serial.println(ssidap);
    //WiFi.softAPsetHostname(ssidap);
    WiFi.softAP(ssidap.c_str());
    Serial.print("Mode = ");
    Serial.println(WiFi.getMode());
    IPAddress IP = WiFi.softAPIP();
    web = true;
    return web;
}

/*
    Start WiFi
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

bool initwifi(void)
{
    // if (key_1 != pressed)
    {
        if (scan_for_wifi_ap())
        {
            WiFi.mode(WIFI_STA);
            Serial.print("AP NicE_WiFi foud, logging in...");
            WiFi.begin(ssid, password);
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
        else
        {
            return (wifi_ap(1));
        }
    }
    // else
    // {
    //        return (wifi_ap(1));
    // }
    return false;
}

/*
    Send udp msg
*/
void udpsend(char *msg, int_fast8_t port)
{
    udp.broadcastTo(msg, port);
}

/*
    Setup Async UDP
*/
void setupudp(void)
{
    memBuoyId(&id, true);
    if (udp.listen(1000))
    // if (udp.listen(1000 + id))
    {
        Serial.print("UDP Listening on: ");
        Serial.print(WiFi.localIP());
        Serial.print(":");
        Serial.println(1000);
        // Serial.println(1000 + id);

        udp.onPacket([](AsyncUDPPacket packet)
                     {
            Serial.print("UDP Packet Type: ");
            Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
            Serial.print(", From: ");
            Serial.print(packet.remoteIP());
            Serial.print(":");
            Serial.print(packet.remotePort());
            Serial.print(", To: ");
            Serial.print(packet.localIP());
            Serial.print(":");
            Serial.print(packet.localPort());
            Serial.print(", Length: ");
            Serial.print(packet.length());
            Serial.print(", Data: ");
            Serial.write(packet.data(), packet.length());
            Serial.println();
            //reply to the client
            char* tmpStr = (char*) malloc(packet.length() + 1);
            tmpStr[packet.length()] = '\0'; // ensure null termination
            memcpy(tmpStr, packet.data(), packet.length());
            if(strstr(tmpStr,"pong")){
                wiFiCollorStatus.color = CRGB ::Green;
                wiFiCollorStatus.blink = false;
                xQueueSend(ledStatus, (void *)&wiFiCollorStatus, 10); // update util led
                printf("got pong!\r\n");
            }
            free(tmpStr); // Strign(char*) creates a copy so we can delete our one
            packet.printf("Got %u bytes of data", packet.length()); });
    }
}

/*
    WiFi task
*/
void WiFiTask(void *arg)
{
    // web = 1;
    // web = wifi_ap(1);
    
    web = initwifi();
    if (web)
    {
        ota = setup_OTA();
    }
    else
    {
        while (1)
        {
            delay(10000);
        }
    }
    setupudp();
    // udp.broadcast("Anyone here?");
    Serial.print("WiFI task running!\r\n");
    for (;;)
    {
        if (xQueueReceive(udpOut, (void *)&udpBuffer, 0) == pdTRUE)
        {
            udp.broadcastTo(udpBuffer.msg, udpBuffer.port);
            printf(">%s< send!\r\n", udpBuffer.msg);
        }
        ArduinoOTA.handle();
        delay(1);
    }
}