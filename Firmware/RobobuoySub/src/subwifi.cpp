#include <WiFi.h>
#include <ArduinoOTA.h>

bool OTA = false;

void setup_OTA()
{
    char ssidl[20];
    char buf[30];
    byte mac[6];
    char buoyID = 1;
    WiFi.macAddress(mac);
    sprintf(ssidl, "Robobuoy_sub_%d", buoyID);
    Serial.print("[SETUP] OTA...");
    sprintf(buf, "%s_%02x:%02x:%02x:%02x:%02x:%02x", ssidl, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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
