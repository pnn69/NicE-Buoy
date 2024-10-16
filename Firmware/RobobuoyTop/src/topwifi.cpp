#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <AsyncUDP.h>
#include <RoboCalc.h>
#include <RoboCodeDecode.h>
#include "main.h"
#include "topwifi.h"
#include "leds.h"
#include "datastorage.h"
#include "buzzer.h"

static int statik = IDLE;
static RoboStruct msgIdOut;
static RoboStruct topWifiIn;
static UdpData udpBuffer;
static UdpData udpBufferRecieved;
static LedData wifiCollorUtil;
static Buzz wifiBuzzerData;
static bool ota = false;
static int8_t id = 0;
static char gpsDataIn[100];
static IPAddress ipTop;
AsyncUDP udp;
QueueHandle_t udpOut;
QueueHandle_t udpIn;
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

/*
    Scan for networks
*/
bool scan_for_wifi_ap(String ssipap, String ww, IPAddress *tmp)
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
                *tmp = (WiFi.localIP());
                return true;
            }
        }
    }
    return false;
}

/*
    Setup accecpoint
*/
void setup_wifi_ap(String ap, String ww, IPAddress *tmp)
{
    WiFi.mode(WIFI_AP); // Set Wi-Fi mode to Access Point
    Serial.println("Setting up access point now");

    // Configure static IP
    IPAddress local_IP(192, 168, 4, 1); // Desired static IP address
    IPAddress subnet(255, 255, 255, 0); // Subnet mask
    IPAddress gateway(192, 168, 1, 5);  // Gateway address
    IPAddress primaryDNS(0, 0, 0, 0);   // Primary DNS (optional)
    IPAddress secondaryDNS(0, 0, 0, 0); // Secondary DNS (optional)

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
                         String stringUdpIn = (const char *)packet.data();
                         //  strncpy(udpBufferRecieved.msg, stringUdpIn.c_str(), sizeof(udpBufferRecieved.msg) - 1);
                         //  udpBufferRecieved.msg[sizeof(udpBufferRecieved.msg) - 1] = '\0'; // Ensure null termination
                         //  if (verifyCRC(String(udpBufferRecieved.msg)))
                         Serial.print("New incomming UPD message: ");
                         Serial.println(stringUdpIn);
                         if (verifyCRC(stringUdpIn))
                         {
                             int msg = RoboDecode(stringUdpIn, topWifiIn);
                             xQueueSend(udpIn, (void *)&topWifiIn, 10); // notify main there is new data
                         }
                         else
                         {
                             Serial.println("crc error");
                         }
                         // send data to main
                     });
        return true;
    }
    return false;
}

/*
    init WiFi que
*/
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
        printf("Queue udpIn could not be created. %p\\r\n", udpIn);
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
    int wifiConfig = *((int *)arg);
    unsigned long nwUpdate = millis();
    unsigned char numClients = 0;
    String ap = "";
    String apww = "";
    byte mac[6];
    char macStr[20];
    WiFi.macAddress(mac);
    sprintf(macStr, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    unsigned long nextSamp = millis();
    if (wifiConfig == 1)
    {
        ap = "PAIR_ME_";
        ap += macStr;
        setup_wifi_ap(ap, apww, &ipTop);
    }
    else // try to find accespoint NicE_WiFi. If no succes make a accecpoint BUOY_[MAC]
    {
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
    printf("WiFI task running!\r\n");
    /*
        WiFI loop
    */
    for (;;)
    {
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
        if (ota == true)
        {
            ArduinoOTA.handle();
        }

        if (xQueueReceive(udpOut, (void *)&msgIdOut, 0) == pdTRUE)
        {
            tstart = millis();
            String out = RoboCode(msgIdOut);
            addCRCToString(String(out));
            udp.broadcast(out.c_str());
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