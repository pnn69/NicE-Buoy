#include <ArduinoOTA.h>
#include <WiFi.h>
#include "general.h"
#include "AsyncUDP.h"

// Replace with your network credentials
const char *ssid = "NicE_Engineering_UPC";
const char *password = "1001100110";
const char *ssidwsop = "wsopwsop";
const char *passwordwsop = "wsopwsop";

const char *host = "NicE-Buoy";
static bool OTA = false;
static bool WEBok = false;
AsyncUDP udp;

/* ****************************************************************************/
/* * Over the air setup */
/* * If timers are used kill the processes befor going to dwonload the new
 * firmware */
/* * To realise that kill them in the function ArduinoOTA.onStart */
/* ***************************************************************************
 */
void setup_OTA()
{
    char ssidl[20];
    char buf[30];
    byte mac[6];
    WiFi.macAddress(mac);
    sprintf(ssidl, "NicE_Buoy_%d", buoyID);
    Serial.print("[SETUP] OTA...");
    sprintf(buf, "%s-%02x:%02x:%02x:%02x:%02x:%02x", ssidl, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ArduinoOTA.setHostname(buf);
    ArduinoOTA.onStart([]()
                       {
    /* switch off all processes here!!!!! */
    Serial.println();
    Serial.println("Receiving new firmware now!"); });
    ArduinoOTA.onEnd([]()
                     {
    /* do stuff after update here!! */
    Serial.println("\nReceiving done!");
    Serial.println("Storing in memory and reboot!");
    Serial.println(); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
    ArduinoOTA.onError([](ota_error_t error)
                       { ESP.restart(); });
    /* setup the OTA server */
    ArduinoOTA.begin();
    Serial.println("...done!");
    OTA = true;
    Serial.print("OTA ID: ");
    Serial.println(buf);
}

void websetup()
{
    // String nwk = "";
    // int n = WiFi.scanNetworks();
    // if (n == 0)
    // {
    //     Serial.println("no networks found");
    // }
    // else
    // {
    //     Serial.print(n);
    //     Serial.println(" networks found");
    //     for (int i = 0; i < n; ++i)
    //     {
    //         // Print SSID and RSSI for each network found
    //         Serial.print(i + 1);
    //         Serial.print(": ");
    //         Serial.print(WiFi.SSID(i));
    //         if (WiFi.SSID(i) == "NicE_Engineering_UPC")
    //         {
    //             nwk = WiFi.SSID(i);
    //         }
    //         if (WiFi.SSID(i) == "wvop")
    //         {
    //             nwk = WiFi.SSID(i);
    //         }
    //         Serial.print(" (");
    //         Serial.print(WiFi.RSSI(i));
    //         Serial.print(")");
    //         Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
    //         delay(10);
    //     }
    // }
    // Serial.println("");

    // if (nwk == "NicE_Engineering_UPC")
    // {
    //     int se = 0;
    //     WiFi.mode(WIFI_STA);
    //     WiFi.begin(ssid, password);
    //     Serial.print("Connecting to WiFi ..");
    //     while (WiFi.status() != WL_CONNECTED && se < 10)
    //     {
    //         Serial.print('.');
    //         delay(1000);
    //         se++;
    //         if (se > 10)
    //             break;
    //     }
    //     if (se >= 10)
    //     {
    //         nwk == "";
    //     }
    // }

    // if (nwk == "wsop")
    // {
    //     int se = 0;
    //     WiFi.mode(WIFI_STA);
    //     WiFi.begin(ssidwsop, passwordwsop);
    //     Serial.print("Connecting to WiFi ..");
    //     while (WiFi.status() != WL_CONNECTED && se < 10)
    //     {
    //         Serial.print('.');
    //         delay(1000);
    //         se++;
    //         if (se > 10)
    //             break;
    //     }
    //     if (se >= 10)
    //     {
    //         nwk == "";
    //     }
    // }
    // if (nwk == "")
    // {
    Serial.println("Accespoint setup!");
    IPAddress local_IP(192, 168, 4, 1); // Your Desired Static IP Address
    IPAddress subnet(255, 255, 255, 0);
    IPAddress gateway(192, 168, 1, 1);
    IPAddress primaryDNS(0, 0, 0, 0);   // Not Mandatory
    IPAddress secondaryDNS(0, 0, 0, 0); // Not Mandatory
    // Configures Static IP Address

    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
    {
        Serial.println("Configuration Failed!");
    }

    char ssidl[20];
    sprintf(ssidl, "NicE_Buoy_%d", buoyID);
    // Serial.println("No WiFi network found!");
    Serial.print("Seting up AP: ");
    Serial.println(ssidl);
    WiFi.softAP(ssidl);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    // }

    // else
    // {
    //     Serial.print("WiFi network found!\r\nIP: ");
    //     Serial.println(WiFi.localIP());
    // }
    setup_OTA();
    WEBok = true;
}

void udpsend(char *dataout)
{
    udp.broadcastTo(dataout, 1001);
}

unsigned int udpmsgtimer = 0;
void webloop()
{
    if (WEBok)
    {
        if (OTA)
        {
            ArduinoOTA.handle();
        }
    }
}