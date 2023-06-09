#include <ArduinoOTA.h>
#include <WiFi.h>
#include "general.h"

// Replace with your network credentials
const char *ssid = "NicE_Engineering_UPC";
const char *password = "1001100110";
const char *host = "NicE-Buoy-Remote";
static bool OTA = false;

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
    OTA = true;
    Serial.print("OTA ID: ");
    Serial.println(buf);
}

void websetup()
{
    // Connect to Wi-Fi
    int se = 0;
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi ..");
    while (WiFi.status() != WL_CONNECTED && se < 10)
    {
        Serial.print('.');
        delay(1000);
        se++;
        if (se > 10)
            break;
    }
    Serial.println(WiFi.localIP());

    if (se >= 10)
    {
        char ssidl[20];
        sprintf(ssidl, "NicE_Buoy_%d", buoyID);
        Serial.println("No WiFi network found!");
        Serial.print("Seting up AP: ");
        Serial.println(ssidl);
        WiFi.softAP(ssidl);
        IPAddress IP = WiFi.softAPIP();
        Serial.print("AP IP address: ");
        Serial.println(IP);
    }
    else
    {
        Serial.println("WiFi network found!");
        Serial.println(WiFi.localIP());
    }
    // setup_OTA();
}

void webloop()
{
    if (OTA)
    {
        ArduinoOTA.handle();
    }
}