// Import required libraries
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>;
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include "SPIFFS.h"
#include "general.h"

// Replace with your network credentials
const char *ssid = "NicE_Engineering_UPC";
const char *password = "1001100110";
const char *host = "NicE-Buoy-Remote";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool ledState = 0;
bool notify = false;

void notifyClients()
{
    char buffer[100];
    DynamicJsonDocument doc(200);
    doc["Speed1"] = (buoy[1].speedbb + buoy[1].speedsb) / 2;
    doc["Dir1"] = buoy[1].mdir;
    serializeJson(doc, buffer);
    ws.textAll(buffer);
    Serial.printf("Websocket data out: %s <-Len%d\n", (char *)buffer, strlen(buffer));
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
        JsonObject obj = doc.as<JsonObject>();
        const char *error;
        error = obj["toggle"];
        // Is there an error after all?
        if (error != nullptr)
        {
            ledState = !ledState;
        }
        if (doc.containsKey("Speed1"))
        {
            {
                buoy[1].speedbb = doc["Speed1"];
                buoy[1].speedsb = doc["Speed1"];
                Serial.print("Speed found: ");
                Serial.printf("%d\r\n", buoy[1].speedbb);
            }
            notify = true;
        }

        if (doc.containsKey("Rudder1"))
        {
            {
                buoy[1].mdir = doc["Rudder1"];
                Serial.print("Rudder1 found: ");
                Serial.printf("%d\r\n", buoy[1].mdir);
            }
            notify = true;
        }
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

String processor(const String &var)
{
    Serial.println(var);
    if (var == "STATE")
    {
        if (ledState)
        {
            return "ON";
        }
        else
        {
            return "OFF";
        }
    }
    return String();
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
        WiFi.softAP("NicE_Buoy_Control", password);
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
    // Start server
    AsyncElegantOTA.begin(&server);
    server.begin();
}

void webloop()
{
    ws.cleanupClients();
    if (notify)
    {
        notify = false;
        notifyClients();
    }
    // digitalWrite(ledPin, ledState);
}