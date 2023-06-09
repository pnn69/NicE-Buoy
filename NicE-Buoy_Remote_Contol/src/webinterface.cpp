// Import required libraries
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include "SPIFFS.h"
#include "general.h"
#include "calculate.h"
#include "LiLlora.h"

// Replace with your network credentials
const char *ssid = "NicE_Engineering_UPC";
const char *password = "1001100110";
const char *host = "NicE-Buoy-Remote";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool ledState = 0;
static bool OTA = false;
int notify = 0;
int radiobutton[NR_BUOYS];

/* ****************************************************************************/
/* * Over the air setup */
/* * If timers are used kill the processes befor going to dwonload the new
 * firmware */
/* * To realise that kill them in the function ArduinoOTA.onStart */
/* ***************************************************************************
 */
void setup_OTA()
{
    char ssidl[26];
    char buf[40];
    byte mac[6];
    WiFi.macAddress(mac);
    strcpy(ssidl, "NicE_Buoy_Control");
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

void notifyClients(int buoy_nr)
{
    char buffer[500];
    DynamicJsonDocument doc(500);

    String n = String(buoy_nr);
    doc["STATUS" + n] = radiobutton[buoy_nr];
    doc["mdir" + n] = buoy[buoy_nr].mdir;
    doc["tgdistance" + n] = buoy[buoy_nr].tgdistance;
    doc["speed" + n] = buoy[buoy_nr].speed;
    doc["speedbb" + n] = buoy[buoy_nr].speedbb;
    doc["speedsb" + n] = buoy[buoy_nr].speedsb;
    doc["rssi" + n] = buoy[buoy_nr].rssi;

    if (buoy[buoy_nr].status == LOCKED)
    {
        if (determineDirection(buoy[1].tgdir, buoy[1].mdir))
        {
            doc["ddir" + n] = smallestAngle(buoy[buoy_nr].tgdir, buoy[buoy_nr].mdir);
            doc["tgdir" + n] = buoy[buoy_nr].tgdir;
        }
        else
        {
            doc["ddir" + n] = -1 * smallestAngle(buoy[buoy_nr].tgdir, buoy[buoy_nr].mdir);
            doc["tgdir" + n] = buoy[buoy_nr].tgdir;
        }

        doc["speed" + n] = buoy[buoy_nr].speed;
    }
    else if (buoy[buoy_nr].status == REMOTE)
    {
        doc["ddir" + n] = buoy[buoy_nr].cdir;
    }
    else if (buoy[buoy_nr].status == IDLE)
    {
        doc["ddir" + n] = 0;
        doc["speed" + n] = 0;
        doc["tgdir" + n] = 0;
        doc["tgdistance" + n] = 0;
        doc["speedbb" + n] = 0;
        doc["speedsb" + n] = 0;
    }
    else
    {
        if (determineDirection(buoy[buoy_nr].tgdir, buoy[buoy_nr].mdir))
        {
            doc["ddir" + n] = smallestAngle(buoy[buoy_nr].tgdir, buoy[buoy_nr].mdir);
            doc["tgdir" + n] = buoy[buoy_nr].tgdir;
        }
        else
        {
            doc["ddir" + n] = -1 * smallestAngle(buoy[buoy_nr].tgdir, buoy[buoy_nr].mdir);
            doc["tgdir" + n] = buoy[buoy_nr].tgdir;
        }
        doc["speed" + n] = buoy[buoy_nr].speed;
    }
    serializeJson(doc, buffer);
    ws.textAll(buffer);
    // Serial.printf("Websocket data out: %s <-Len%d\n", (char *)buffer, strlen(buffer));
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
    {
        data[len] = 0;
        Serial.printf("Websocket data in: %s\n", (char *)data);
        DynamicJsonDocument doc(len * 2);
        DeserializationError err = deserializeJson(doc, data);
        if (err)
        {
            Serial.print(F("deserializeJson() failed with code "));
            Serial.println(err.c_str());
            return;
        }
        JsonObject obj = doc.as<JsonObject>();
        if (doc.containsKey("GETSTATUS"))
        {
            char buffer[500];
            DynamicJsonDocument doc1(500);
            doc1["STATUS1"] = buoy[1].status;
            serializeJson(doc1, buffer);
            ws.textAll(buffer);
        }
        if (doc.containsKey("Lock_Buoy_1"))
        {
            radiobutton[1] = 2;
            notify = true;
        }
        if (doc.containsKey("Remote_Buoy_1"))
        {
            radiobutton[1] = 6;
            notify = true;
        }
        if (doc.containsKey("Doc_Buoy_1"))
        {
            radiobutton[1] = 7;
            notify = true;
        }
        if (doc.containsKey("Idle_1"))
        {
            radiobutton[1] = 1;
            notify = true;
        }

        if (doc.containsKey("Rudder1"))
        {
            if (buoy[1].status != LOCKED)
            {
                buoy[1].cdir = doc["Rudder1"];
            }
            // notify = 1;
        }

        if (doc.containsKey("Speed1"))
        {
            if (buoy[1].status != LOCKED)
            {
                buoy[1].cspeed = doc["Speed1"];
            }
            // notify = 1;
        }
        if (doc.containsKey("LOCKEDBUOY1"))
        {
            buoy[1].cdir = 0;
            buoy[1].cspeed = 0;
            buoy[1].speed = 0;
            buoy[1].speedbb = 0;
            buoy[1].speedsb = 0;

            buoy[1].status = LOCKING;
            buoy[1].cmnd = TARGET_POSITION;
            buoy[1].ackOK = false;
            buoy[1].gsa = SET;

            // notify = 1;
        }
        if (doc.containsKey("REMOTEBUOY1"))
        {
            buoy[1].cdir = 0;
            buoy[1].cspeed = 0;
            buoy[1].speed = 0;
            buoy[1].speedbb = 0;
            buoy[1].speedsb = 0;
            buoy[1].status = REMOTE;
            buoy[1].cmnd = SAIL_DIR_SPEED;
            buoy[1].ackOK = false;
            buoy[1].gsa = SET;
        }
        if (doc.containsKey("DOCBUOY1"))
        {
            buoy[1].tgdir = 0;
            buoy[1].speed = 0;
            buoy[1].status = DOC;
            buoy[1].cmnd = GOTO_DOC_POSITION;
            buoy[1].ackOK = false;
            buoy[1].gsa = SET;
        }
        if (doc.containsKey("IDLEBUOY1"))
        {
            buoy[1].tgdir = 0;
            buoy[1].speed = 0;
            buoy[1].status = IDLE;
            buoy[1].cmnd = BUOY_MODE_IDLE;
            buoy[1].ackOK = false;
            buoy[1].gsa = SET;
        }
        if (buoy[1].status == REMOTE)
        {
            buoy[1].status = REMOTE;
            buoy[1].cmnd = SAIL_DIR_SPEED;
            buoy[1].ackOK = false;
            buoy[1].gsa = SET;
        }
        notify = 1;
        // Serial.printf("Status %d\r\n", buoy[1].status);
    }
}
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    switch (type)
    {
    case WS_EVT_CONNECT:
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        notify = true;
        break;
    case WS_EVT_DISCONNECT:
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
        break;
    case WS_EVT_DATA:
        handleWebSocketMessage(arg, data, len);
        break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
        break;
    }
}

/*
 * End websocket stuff
 */

void initWebSocket()
{
    ws.onEvent(onEvent);
    server.addHandler(&ws);
}

void websetup()
{
    if (!SPIFFS.begin())
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }
    else
    {
        Serial.println("Mounted SPIFFS");
    }

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
        Serial.println("No WiFi network found!");
        Serial.print("Seting up AP: ");
        Serial.println("NicE_Buoy_Control");
        WiFi.softAP("NicE_Buoy_Control_192.168.4.1");
        IPAddress IP = WiFi.softAPIP();
        Serial.print("AP IP address: ");
        Serial.println(IP);
    }
    else
    {
        Serial.println("WiFi network found!");
        Serial.println(WiFi.localIP());
    }
    initWebSocket();

    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/index.html", "text/html"); });
    //              { request->send(200, "/index.html", "text/html"); });
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/style.css", "text/css"); });
    //              { request->send(200, "/style.css", "text/css"); });
    server.on("/index.js", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/index.js", "text/js"); });
    // Start server
    server.begin();
    // setup_OTA();
}

void webloop()
{
    if (OTA)
    {
        ArduinoOTA.handle();
    }
    ws.cleanupClients();
    if (notify)
    {
        notifyClients(notify);
        notify = 0;
    }
    // digitalWrite(ledPin, ledState);
}