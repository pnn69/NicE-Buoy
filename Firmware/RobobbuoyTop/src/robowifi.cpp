#include <WiFi.h>
#include <ArduinoOTA.h>
#include "robowifi.h"
#include "topgeneral.h"

static bool web = false;
static bool ota = false;

bool setup_OTA()
{
    char ssidl[20];
    char buf[30];
    byte mac[6];
    WiFi.macAddress(mac);
    sprintf(ssidl, "RoboBuoy_%d", buoyId);
    Serial.print("SETUP OTA...");
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
    Serial.print("OTA ID: ");
    Serial.println(buf);
    return true;
}

bool initwifi(void)
{
    Serial.println("Accespoint setup");
    IPAddress local_IP(192, 168, 4, 1); // Your Desired Static IP Address
    IPAddress subnet(255, 255, 255, 0);
    IPAddress gateway(192, 168, 1, 1);
    IPAddress primaryDNS(0, 0, 0, 0);   // Not Mandatory
    IPAddress secondaryDNS(0, 0, 0, 0); // Not Mandatory

    if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS))
    {
        Serial.println("Configuration Failed!");
        return false;
    }

    char ssidl[20];
    sprintf(ssidl, "RoboBuoy_%d", buoyId);
    // Serial.println("No WiFi network found!");
    Serial.print("Seting up AP: ");
    Serial.println(ssidl);
    WiFi.softAP(ssidl);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    return true;
}

void WiFiTask(void *arg)
{
    web = initwifi();
    if (web)
    {
        ota = setup_OTA();
        Serial.println("WiFI task running!");
    }
    while (1)
    {
        delay(100);
    }
}