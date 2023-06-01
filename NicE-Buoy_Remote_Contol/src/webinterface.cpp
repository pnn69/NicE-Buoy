// Import required libraries
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>;
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include "SPIFFS.h"
#include "general.h"
#include "menu.h"
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
int notify = 0;

void notifyClients(int buoy_nr)
{
    char buffer[500];
    DynamicJsonDocument doc(500);

    String n = String(buoy_nr);
    doc["mdir" + n] = buoy[buoy_nr].mdir;
    doc["tgdistance" + n] = buoy[buoy_nr].tgdistance;
    doc["speedsb" + n] = buoy[buoy_nr].speedsb;
    doc["speedbb" + n] = buoy[buoy_nr].speedbb;
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
        doc["speed" + n] = buoy[buoy_nr].cspeed;
        doc["tgdir" + n] = buoy[buoy_nr].tgdir;
    }
    else if (buoy[buoy_nr].status == IDLE)
    {
        doc["ddir" + n] = 0;
        doc["speed" + n] = 0;
        doc["tgdir" + n] = 0;
        doc["tgdistance" + n] = 0;
        doc["speedsb" + n] = 0;
        doc["speedbb" + n] = 0;
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
        Serial.printf("Websocket data in: %s <-Len %d\n", (char *)data, len);
        DynamicJsonDocument doc(len * 2);
        DeserializationError err = deserializeJson(doc, data);
        if (err)
        {
            Serial.print(F("deserializeJson() failed with code "));
            Serial.println(err.c_str());
            return;
        }
        Serial.println("New data form websocket");
        JsonObject obj = doc.as<JsonObject>();
        if (doc.containsKey("GETSTATUS"))
        {
            char buffer[500];
            DynamicJsonDocument doc1(500);
            doc1["STATUS1"] = buoy[1].status;
            serializeJson(doc1, buffer);
            ws.textAll(buffer);
        }
        if (doc.containsKey("Rudder1"))
        {
            if (buoy[1].status != LOCKED)
            {
                buoy[1].cdir = doc["Rudder1"];
                Serial.print("Rudder1 found: ");
                Serial.printf("%d\r\n", buoy[1].cdir);
            }
            // notify = 1;
        }

        if (doc.containsKey("Speed1"))
        {
            if (buoy[1].status != LOCKED)
            {
                buoy[1].cspeed = doc["Speed1"];
                Serial.print("Speed1 found: ");
                Serial.printf("%d\r\n", buoy[1].cspeed);
            }
            // notify = 1;
        }
        if (doc.containsKey("LOCKEDBUOY1"))
        {
            menu(SET_TARGET_POSITION, 1);
            buoy[1].speed = 0;
            buoy[1].status = LOCKING;
            // notify = 1;
        }
        if (doc.containsKey("REMOTEBUOY1"))
        {
            menu(SET_SAIL_DIR_SPEED, 1);
            buoy[1].cdir = 0;
            buoy[1].cspeed = 0;
            buoy[1].status = REMOTE;
            // notify = 1;
        }
        if (doc.containsKey("DOCBUOY1"))
        {
            menu(GOTO_DOC_POSITION, 1);
            buoy[1].tgdir = 0;
            buoy[1].speed = 0;
            buoy[1].status = DOC;
            // notify = 1;
        }
        if (doc.containsKey("IDLEBUOY1"))
        {
            menu(SET_BUOY_MODE_IDLE, 1);
            buoy[1].tgdir = 0;
            buoy[1].speed = 0;
            buoy[1].status = IDLE;
            // notify = 1;
        }
        if (buoy[1].status == REMOTE)
        {
            sendLoraSetSailDirSpeed(1, buoy[1].cdir, buoy[1].cspeed);
        }
        // Serial.printf("Status %d\r\n", buoy[1].status);
    }
}
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    switch (type)
    {
    case WS_EVT_CONNECT:
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
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
    AsyncElegantOTA.begin(&server);
    server.begin();
}

void webloop()
{
    ws.cleanupClients();
    if (notify)
    {
        notifyClients(notify);
        notify = 0;
    }
    // digitalWrite(ledPin, ledState);
}