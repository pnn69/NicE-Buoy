#include <WiFi.h>
#include <AsyncUDP.h>
#include <ArduinoOTA.h>

AsyncUDP udp;

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

bool scan_for_wifi_ap(void)
{
    Serial.println("scan for AP");
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
            if (WiFi.SSID(i) == "BUOY_1")
            {
                // WiFi.mode(WIFI_STA);
                Serial.print("Acces point foud, logging in...");
                WiFi.begin("BUOY_1");
                while (WiFi.status() != WL_CONNECTED)
                {
                    delay(50);
                    Serial.print(".");
                }
                Serial.print(".\r\n");
                Serial.print("Loggend in to SSIS: ");
                Serial.println("BUOY_1");
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
                         String myString = (const char *)packet.data();
                         if (myString.startsWith("ping"))
                         {
                             packet.printf("pong"); //
                            printf("Ping recieved!\r\n");
                         } });
    }
}

/*
    WiFi task
*/
void WiFiTask(void *arg)
{
    scan_for_wifi_ap();
    setupudp();
    // udp.broadcast("Anyone here?");
    Serial.print("WiFI task running!\r\n");
    for (;;)
    {
        delay(1000);
        // Send broadcast
    }
}