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
    Scan for networks and connect according to priority: ROBOBUOY first, then NiCe_WiFi/NicE_WiFi
*/
bool scan_and_connect_wifi(IPAddress *tmp)
{
    unsigned long timeout = millis();
    Serial.println("Starting WiFi scan and connect sequence...");

    // Set STA mode and disconnect to ensure reliable scanning
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    // Loop for up to 30 seconds
    while (millis() - timeout < 30000)
    {
        Serial.println("Scanning for networks...");
        int n = WiFi.scanNetworks();
        Serial.printf("Scan done: %d networks found\n", n);

        if (n > 0)
        {
            bool robobuoy_found = false;
            bool nice_wifi_found = false;
            String nice_wifi_ssid = "";

            for (int i = 0; i < n; ++i)
            {
                String ssid = WiFi.SSID(i);
                if (ssid == "ROBOBUOY")
                {
                    robobuoy_found = true;
                }
                else if (ssid == "NicE_WiFi")
                {
                    nice_wifi_found = true;
                    nice_wifi_ssid = ssid;
                }
            }

            if (robobuoy_found)
            {
                Serial.println("Found 'ROBOBUOY'. Attempting connection...");
                WiFi.begin("ROBOBUOY", "");
                unsigned long conn_timeout = millis();
                while (WiFi.status() != WL_CONNECTED)
                {
                    delay(500);
                    Serial.print(".");
                    if (millis() - conn_timeout > 15000)
                    {
                        Serial.println("\nConnection to 'ROBOBUOY' timed out.");
                        break;
                    }
                }
                if (WiFi.status() == WL_CONNECTED)
                {
                    Serial.println("\nCONNECTED to 'ROBOBUOY'");
                    WiFi.setSleep(WIFI_PS_NONE); // Disable power-saving sleep
                    *tmp = WiFi.localIP();
                    Serial.print("WiFi IP address: ");
                    Serial.println(*tmp);
                    return true;
                }
                WiFi.disconnect();
                delay(100);
            }
            else if (nice_wifi_found)
            {
                Serial.printf("Found '%s'. Attempting connection...\n", nice_wifi_ssid.c_str());
                WiFi.begin(nice_wifi_ssid.c_str(), "!Ni1001100110");
                unsigned long conn_timeout = millis();
                while (WiFi.status() != WL_CONNECTED)
                {
                    delay(500);
                    Serial.print(".");
                    if (millis() - conn_timeout > 15000)
                    {
                        Serial.println("\nConnection to NiCe_WiFi timed out.");
                        break;
                    }
                }
                if (WiFi.status() == WL_CONNECTED)
                {
                    Serial.printf("\nCONNECTED to %s\n", nice_wifi_ssid.c_str());
                    WiFi.setSleep(WIFI_PS_NONE); // Disable power-saving sleep
                    *tmp = WiFi.localIP();
                    Serial.print("WiFi IP address: ");
                    Serial.println(*tmp);
                    return true;
                }
                WiFi.disconnect();
                delay(100);
            }
        }
        delay(1000);
    }

    Serial.println("WiFi connection sequence completed without success.");
    return false;
}

/*
    Setup accecpoint
*/
void setup_wifi_ap(String ap, String ww, IPAddress *tmp)
{
    WiFi.mode(WIFI_AP_STA); // Set Wi-Fi mode to Access Point + Station
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
                             int rssi = WiFi.RSSI();
                             String wsMsg = "UDP:" + String(rssi) + ":" + ipStr + ":" + stringUdpIn;
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
    static bool runBackgroundReconnection = false;

    if (wifiConfig == 1)
    {
        ap = "PAIR_ME_";
        ap += macStr;
        setup_wifi_ap(ap, apww, &ipTop);
        runBackgroundReconnection = false;
    }
    else
    {
        if (scan_and_connect_wifi(&ipTop) == false)
        {
            ap = "ROBOBUOY";
            apww = "";
            setup_wifi_ap(ap, apww, &ipTop);
            runBackgroundReconnection = false;
        }
        else
        {
            runBackgroundReconnection = true;
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
        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        File file = SPIFFS.open("/index.html", "r");
        if (file) {
            server.streamFile(file, "text/html");
            file.close();
        } else {
            server.send(404, "text/plain", "index.html not found");
        }
    });

    server.on("/index.js", []() {
        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        File file = SPIFFS.open("/index.js", "r");
        if (file) {
            server.streamFile(file, "application/javascript");
            file.close();
        } else {
            server.send(404, "text/plain", "index.js not found");
        }
    });

    server.on("/style.css", []() {
        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
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
    static unsigned long lastBackgroundConnectAttempt = 0;
    static int connectAttemptState = 0; // 0 = Idle, 1 = Connecting to ROBOBUOY, 2 = Connecting to NicE_WiFi
    static unsigned long connectionStartedTime = 0;
    static int backgroundConnectRetryCount = 0;

    for (;;)
    {
        dnsServer.processNextRequest();
        
        if (ota == true)
        {
            ArduinoOTA.handle();
        }

        server.handleClient();
        webSocket.loop();

        // Field-Aware Background Wi-Fi reconnection state machine
        if (runBackgroundReconnection) {
            if (WiFi.softAPgetStationNum() > 0) {
                // If a client is connected to our AP, immediately stop any background scans to prevent channel-jumping
                if (connectAttemptState != 0) {
                    Serial.println("[WiFi Background] Client connected to AP. Aborting background connection attempts to ensure AP stability.");
                    WiFi.disconnect();
                    connectAttemptState = 0;
                }
            }
            else if (WiFi.status() != WL_CONNECTED) {
                unsigned long current_time = millis();
                if (connectAttemptState == 0) {
                    // If we are idle and 45 seconds have passed since the last attempt, start the cycle
                    if (current_time - lastBackgroundConnectAttempt > 45000) {
                        lastBackgroundConnectAttempt = current_time;
                        connectAttemptState = 1;
                        connectionStartedTime = current_time;
                        Serial.println("[WiFi Background] Attempting to connect to 'ROBOBUOY'...");
                        WiFi.begin("ROBOBUOY", "");
                    }
                } else {
                    // We are currently waiting for a connection to establish
                    if (current_time - connectionStartedTime > 15000) {
                        // Timeout after 15 seconds
                        if (connectAttemptState == 1) {
                            // Switch to NicE_WiFi
                            connectAttemptState = 2;
                            connectionStartedTime = current_time;
                            Serial.println("[WiFi Background] 'ROBOBUOY' timed out. Attempting 'NicE_WiFi'...");
                            WiFi.begin("NicE_WiFi", "!Ni1001100110");
                        } else {
                            // All attempts failed, go back to idle
                            connectAttemptState = 0;
                            Serial.println("[WiFi Background] All background Wi-Fi connection attempts timed out.");
                            WiFi.disconnect(); // Clear active attempt
                        }
                    }
                }
            } else {
                // We are connected! Reset the state machine
                if (connectAttemptState != 0) {
                    Serial.printf("[WiFi Background] Successfully connected to SSID: %s! IP: %s\n", 
                                  WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
                    WiFi.setSleep(WIFI_PS_NONE); // Disable power-saving sleep
                    connectAttemptState = 0;
                }
            }
        }

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