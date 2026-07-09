#include <WiFi.h>

#include <WebServer.h>
#include <ArduinoOTA.h>
#include <AsyncUDP.h>
#include <SPIFFS.h>
#include <WebSocketsServer.h>
#include <DNSServer.h>
#include "main.h"
#include "controlwifi.h"

// Instantiate the global WebServer running on standard HTTP port 80 and WebSockets on port 81
WebServer server(80);
WebSocketsServer webSocket(81);

// Captive Portal DNS Server instance to intercept and redirect network traffic.
// All DNS queries (on standard DNS UDP Port 53) will be resolved to the ESP32's local Access Point IP address.
// This forces operating systems (iOS, Android, Windows) to open the local landing page automatically.
DNSServer dnsServer;
const byte DNS_PORT = 53;

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
            Serial.printf("[%u] Connected!\n", num);
            break;
        }
        case WStype_TEXT: {
            // Safe copy of payload since it is not guaranteed to be null-terminated
            String message = "";
            message.reserve(length);
            for (size_t i = 0; i < length; i++) {
                message += (char)payload[i];
            }
            message.trim();

            Serial.print("WebSocket RX: ");
            Serial.println(message);

            RoboStruct wsDataIn = {};
            rfDeCode(message, &wsDataIn);
            Serial.printf("Decoded WebSocket RX -> IDr: %08x, IDs: %08x, ack: %d, cmd: %d, status: %d\n", wsDataIn.IDr, wsDataIn.IDs, wsDataIn.ack, wsDataIn.cmd, wsDataIn.status);

            if (wsDataIn.IDs != -1 && wsDataIn.IDs != 0) {
                // Queue the decoded structure safely to webDataIn and set the hasWebCommand flag
                // This lets the main loop safely inject the webpage command right before dispatching,
                // making it 100% immune to being overwritten by background telemetry packets!
                extern RoboStruct webDataIn;
                extern bool hasWebCommand;
                webDataIn = wsDataIn;
                hasWebCommand = true;
                
                // Route to udpIn for thread-safe main task signaling
                xQueueSend(udpIn, (void *)&wsDataIn, 10);
            }
            break;
        }
        default:
            break;
    }
}

static int statik = IDLE;
static RoboStruct msgIdOut;
static RoboStruct topWifiIn;
static RoboStruct udpBuffer;
static RoboStruct udpBufferRecieved;

static bool ota = false;
static int8_t id = 0;
static char udpDataIn[MAXSTRINGLENG];
static IPAddress ipTop;
AsyncUDP udp;
QueueHandle_t udpOut;
QueueHandle_t udpIn;
QueueHandle_t wsOutQueue;
// static unsigned long tstart, tstop;
static unsigned long lastUpdMsg = 0;

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
    sprintf(buf, "Buoy_LORA_%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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
    unsigned long timeout = millis();
    Serial.print("scan for for ap:");
    Serial.println(ssipap);
    
    // Explicitly set STA mode and disconnect to ensure reliable scanning
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    Serial.print(n);
    Serial.println(" networks found");
    if (n <= 0)
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
                    delay(500);
                    Serial.print(".");
                    // Reduce timeout to 15 seconds to allow graceful fallback instead of blocking for 2 minutes
                    if (timeout + 15 * 1000 < millis())
                    {
                        Serial.println("\nConnection timeout! Falling back...");
                        WiFi.disconnect();
                        return false; // Return false to trigger AP fallback instead of rebooting
                    }
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
    IPAddress local_IP(192, 168, 1, 84); // Desired static IP address
    IPAddress subnet(255, 255, 255, 0);  // Subnet mask
    IPAddress gateway(192, 168, 1, 5);   // Gateway address
    IPAddress primaryDNS(0, 0, 0, 0);    // Primary DNS (optional)
    IPAddress secondaryDNS(0, 0, 0, 0);  // Secondary DNS (optional)

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

        // Start DNS Server redirecting every domain to the ESP32 IP
        dnsServer.start(DNS_PORT, "*", *tmp);
        Serial.println("DNS Server started for Captive Portal");
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
                         String stringUdpIn = String((const char *)packet.data(), packet.length());
                         
                         // Push incoming UDP packets to wsOutQueue for thread-safe WebSocket broadcast
                         if (wsOutQueue != NULL) {
                             String ipStr = packet.remoteIP().toString();
                             String wsMsg = "UDP:" + ipStr + ":" + stringUdpIn;
                             char packetBuf[160];
                             memset(packetBuf, 0, sizeof(packetBuf));
                             // Safe copy with length limit to fit inside packetBuf (159 chars + null terminator)
                             wsMsg.substring(0, 159).toCharArray(packetBuf, sizeof(packetBuf));
                             xQueueSend(wsOutQueue, (void *)packetBuf, 10);
                         }

                         RoboStruct udpDataIn = {};
                         rfDeCode(stringUdpIn, &udpDataIn);
                         if (udpDataIn.IDs != -1 && udpDataIn.IDs != 0)
                         {
                             // Capture the actual UDP sender remote IP address last octet
                             udpDataIn.ip = packet.remoteIP()[3];
                             
                             xQueueSend(udpIn, (void *)&udpDataIn, 10); // notify main there is new data
                             lastUpdMsg = millis();
                         }
                      });        return true;
    }
    return false;
}

void udpSend(String data)
{
    data = addCRCToString(data);
    udp.broadcast(data.c_str());
}

/*
    init WiFi que
*/
unsigned long espMac(void)
{
    byte macarr[6];
    WiFi.macAddress(macarr);
    unsigned long mac = 0;
    for (int i = 2; i < 6; i++)
    {
        mac = (mac << 8) | macarr[i];
    }
    return mac;
}

/*
    init WiFi que
*/
unsigned long initwifiqueue(void)
{
    udpOut = xQueueCreate(10, sizeof(RoboStruct));
    udpIn = xQueueCreate(10, sizeof(RoboStruct));
    wsOutQueue = xQueueCreate(15, 160 * sizeof(char));
    return espMac();
}

/*
    WiFi task
*/
void WiFiTask(void *arg)
{
    byte mac[6];
    char macStr[20];
    WiFi.macAddress(mac);
    sprintf(macStr, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    int wifiConfig = *((int *)arg);
    unsigned long nwUpdate = millis();
    unsigned char numClients = 0;
    String ap = "";
    String apww = "";
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
            ap = "BUOY_LORA";
            ap += macStr;
            apww = "";
            setup_wifi_ap(ap, apww, &ipTop);
        }
    }
    Serial.print("IP address: ");
    Serial.println(ipTop);
    ota = setup_OTA();
    udp_setup(1001);

    // Mount SPIFFS filesystem
    if (!SPIFFS.begin(true)) {
        Serial.println("An Error has occurred while mounting SPIFFS");
    } else {
        Serial.println("SPIFFS mounted successfully!");
    }

    // Configure WebServer routes
    server.on("/", []() {
        File file = SPIFFS.open("/index.html", "r");
        if (file) {
            server.streamFile(file, "text/html");
            file.close();
        } else {
            server.send(404, "text/plain", "index.html not found");
        }
    });

    server.on("/index.js", []() {
        File file = SPIFFS.open("/index.js", "r");
        if (file) {
            server.streamFile(file, "application/javascript");
            file.close();
        } else {
            server.send(404, "text/plain", "index.js not found");
        }
    });

    server.on("/style.css", []() {
        File file = SPIFFS.open("/style.css", "r");
        if (file) {
            server.streamFile(file, "text/css");
            file.close();
        } else {
            server.send(404, "text/plain", "style.css not found");
        }
    });

    // Unrecognized URL redirect for Captive Portal support
    server.onNotFound([]() {
        server.sendHeader("Location", "http://192.168.1.84/", true);
        server.send(302, "text/plain", "");
    });

    server.begin();
    Serial.println("HTTP WebServer started on port 80!");

    // Start WebSocket Server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket Server started on port 81!");

    printf("WiFI task running!\r\n");
    /*
        WiFI loop
    */
    for (;;)
    {
        dnsServer.processNextRequest();
        
        if (ota == true)
        {
            ArduinoOTA.handle();
        }

        server.handleClient();
        webSocket.loop();

        // Thread-safe WebSocket transmitter queue polling
        char wsMsg[160];
        if (wsOutQueue != NULL && xQueueReceive(wsOutQueue, (void *)wsMsg, 1) == pdTRUE)
        {
            String wsMsgStr = String(wsMsg);
            webSocket.broadcastTXT(wsMsgStr);
        }

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

        if (xQueueReceive(udpOut, (void *)&msgIdOut, 1) == pdTRUE)
        {
            msgIdOut.IDs = 0x99; // Set sender ID to 99 to match Python UI logic
            String out = rfCode(&msgIdOut);
            Serial.print("UDP BROADCAST: ");
            Serial.println(out);
            udp.broadcast(out.c_str());
            
            // Broadcast the compiled message string to all websocket clients with UDP prefix
            String wsMsg = "UDP:" + out;
            webSocket.broadcastTXT(wsMsg.c_str());
        }
        delay(1);
    }
}