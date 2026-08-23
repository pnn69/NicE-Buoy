#include <WiFi.h>

#include <WebServer.h>
#include <ArduinoOTA.h>
#include <AsyncUDP.h>
#include <SPIFFS.h>
#include <WebSocketsServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
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

//***************************************************************************************************
//      Network policy: RoboLora is a client, never the field AP
//***************************************************************************************************
// Same priority list as the Tops:
//
//      NicE_WiFi  ->  Robo_WiFi  ->  our own LORA_<id> AP
//
// RoboLora used to raise the field AP itself, under the same name the CYD used. Two devices
// broadcasting one SSID on one subnet is two separate networks wearing the same badge, and the
// fleet would split between them. The CYD is the field AP now; we join it like everything else.
static const char *HOME_SSID   = "NicE_WiFi";
static const char *HOME_PASS   = "!Ni1001100110";
static const char *FIELD_SSID  = "Robo_WiFi";
static const char *FIELD_PASS  = "geenanker";
static const char *OWN_AP_PASS = "geenanker";

static bool apActive = false;    // our own AP is currently up
static char ownApSsid[24] = "";  // LORA_73798c
static char mdnsHost[24] = "";   // lora-73798c  ->  http://lora-73798c.local

// Short, readable identity - espMac() folds the last four MAC bytes into the id the logs use, and
// a phone's WiFi list is far easier to search for LORA_xxxxxxxx than for a twelve-hex MAC.
static void build_identity()
{
    snprintf(ownApSsid, sizeof(ownApSsid), "LORA_%08lx", (unsigned long)espMac());
    // mDNS labels may not contain '_', so the hostname is spelled with a hyphen instead.
    snprintf(mdnsHost, sizeof(mdnsHost), "lora-%08lx", (unsigned long)espMac());
}

// The captive portal only works while we are the access point - we will not hijack DNS on someone
// else's network. mDNS covers the other half: http://lora-xxxxxxxx.local resolves in both modes.
static void start_mdns()
{
    MDNS.end();
    if (MDNS.begin(mdnsHost))
    {
        MDNS.addService("http", "tcp", 80);
        // ArduinoOTA.begin() calls MDNS.begin() with its own hostname, so the two must agree or
        // whichever runs last wins. Re-advertising the OTA service matters too: every reconnect
        // tears mDNS down and back up, which would otherwise drop OTA off the air.
        MDNS.enableArduino(3232);
        Serial.printf("[WiFi] reachable as http://%s.local\r\n", mdnsHost);
    }
}

// Try one network, without scanning for it first. WiFi.scanNetworks() is the blocking form: it
// walks all thirteen channels and parks this task for seconds, and with a softAP up that drags the
// single radio off our own channel long enough for a connected client to give up on us. begin()
// does its own short, targeted probe and fails on timeout when the AP is not there.
static bool try_connect(const char *ssid, const char *pass, uint32_t timeoutMs)
{
    Serial.printf("[WiFi] trying '%s' ... ", ssid);
    WiFi.begin(ssid, pass);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - start > timeoutMs)
        {
            Serial.println("not there");
            WiFi.disconnect();
            return false;
        }
        delay(100);
    }

    // Associated is not the same as usable: starting mDNS or OTA while the lease is still 0.0.0.0
    // takes the lwIP stack - and the chip - down with it.
    while (WiFi.localIP() == IPAddress(0, 0, 0, 0))
    {
        if (millis() - start > timeoutMs + 6000)
        {
            Serial.println("no DHCP lease");
            WiFi.disconnect();
            return false;
        }
        delay(100);
    }

    WiFi.setSleep(WIFI_PS_NONE);
    Serial.printf("joined, IP %s\r\n", WiFi.localIP().toString().c_str());
    return true;
}

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
static RoboStruct udpBufferReceived;

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
    // Same name mDNS advertises - see start_mdns(). Also drops the underscore, not legal in a
    // hostname label.
    sprintf(buf, "%s", mdnsHost);
    ArduinoOTA.setHostname(mdnsHost);
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
    Home first, then the CYD's field AP. No scan anywhere - trying them in order gives the same
    answer as scanning and then deciding, without taking the radio off channel.
*/
bool scan_and_connect_wifi(IPAddress *tmp)
{
    if (!apActive)
    {
        WiFi.mode(WIFI_STA);
    }
    if (try_connect(HOME_SSID, HOME_PASS, 8000) || try_connect(FIELD_SSID, FIELD_PASS, 8000))
    {
        *tmp = WiFi.localIP();
        return true;
    }
    Serial.println("[WiFi] neither NicE_WiFi nor Robo_WiFi in reach");
    return false;
}

/*
    Setup our own access point, plus the captive portal that makes it usable without an IP.
*/
void setup_wifi_ap(String ap, String ww, IPAddress *tmp)
{
    // AP_STA so the station side stays alive and can keep hunting underneath the AP.
    WiFi.mode(WIFI_AP_STA);
    Serial.println("Setting up access point now");

    // The AP is its own gateway. We used to advertise 192.168.1.5, where nothing answers, and
    // Android reacts to a gateway that never replies by deciding the network is broken.
    IPAddress apIP(192, 168, 4, 1);
    if (!WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0)))
    {
        Serial.println("Failed to configure static IP for AP");
    }

    // max_connection defaults to 4 in the Arduino wrapper; the ESP32 silicon allows 15.
    if (WiFi.softAP(ap.c_str(), ww.length() ? ww.c_str() : NULL, 1, 0, 8))
    {
        WiFi.setSleep(WIFI_PS_NONE);
        *tmp = WiFi.softAPIP();
        apActive = true;
        Serial.printf("[WiFi] AP '%s' up on %s\r\n", ap.c_str(), tmp->toString().c_str());

        // Wildcard DNS so the phone's own connectivity probe lands on our web server.
        dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
        dnsServer.start(DNS_PORT, "*", apIP);
        Serial.println("DNS Server started for Captive Portal");
    }
    else
    {
        Serial.println("Failed to start access point");
    }
    start_mdns();
}

/*
    Fold our own AP away once we are on a real network again.
*/
static void stop_own_ap()
{
    if (!apActive)
    {
        return;
    }
    dnsServer.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    apActive = false;
    Serial.println("[WiFi] own AP folded away, we are on a real network now");
    start_mdns();
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
    build_identity();
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
            // Our own AP, not the field AP - that job belongs to the CYD now.
            ap = ownApSsid;
            apww = OWN_AP_PASS;
            setup_wifi_ap(ap, apww, &ipTop);
        }
        else
        {
            start_mdns();
        }
        // Always on now, in both branches: sitting on our own AP we still have to keep hunting,
        // and having joined a network we still have to cope with losing it.
        runBackgroundReconnection = true;
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

    server.onNotFound([]() {
        // Only hijack unknown URLs while we are the access point - that is what opens the phone's
        // captive-portal sheet. On a real network a 404 must stay a 404. This used to redirect to
        // a hardcoded 192.168.1.84, which was the Top's old AP address, not ours.
        if (apActive) {
            server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
            server.send(302, "text/plain", "");
        } else {
            server.send(404, "text/plain", "Not found");
        }
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
    static int connectAttemptState = 0; // 0 = idle, 1 = trying NicE_WiFi, 2 = trying Robo_WiFi
    static unsigned long connectionStartedTime = 0;

    for (;;)
    {
        if (apActive) {
            dnsServer.processNextRequest();
        }
        
        if (ota == true)
        {
            ArduinoOTA.handle();
        }

        server.handleClient();
        webSocket.loop();

        // Field-aware background hunt for a real network, running underneath our own AP.
        if (runBackgroundReconnection) {
            unsigned long current_time = millis();

            if (apActive && WiFi.softAPgetStationNum() > 0) {
                // Somebody is on our AP. Stop hunting: with one radio we would have to leave the
                // AP's channel to do it, and we are not knocking a client off mid-session.
                if (connectAttemptState != 0) {
                    Serial.println("[WiFi] client on our AP - holding off, AP stability wins");
                    WiFi.disconnect();
                    connectAttemptState = 0;
                }
                lastBackgroundConnectAttempt = current_time;
            }
            else if (WiFi.status() != WL_CONNECTED) {
                if (connectAttemptState == 0) {
                    if (current_time - lastBackgroundConnectAttempt > 45000) {
                        lastBackgroundConnectAttempt = current_time;
                        connectAttemptState = 1;
                        connectionStartedTime = current_time;
                        Serial.printf("[WiFi] hunting '%s'...\r\n", HOME_SSID);
                        WiFi.begin(HOME_SSID, HOME_PASS);
                    }
                } else if (current_time - connectionStartedTime > 15000) {
                    if (connectAttemptState == 1) {
                        connectAttemptState = 2;
                        connectionStartedTime = current_time;
                        Serial.printf("[WiFi] no '%s', hunting '%s'...\r\n", HOME_SSID, FIELD_SSID);
                        WiFi.begin(FIELD_SSID, FIELD_PASS);
                    } else {
                        connectAttemptState = 0;
                        WiFi.disconnect();
                        // Neither in reach - raise our own AP so there is still a way in.
                        if (!apActive) {
                            IPAddress apIp;
                            setup_wifi_ap(String(ownApSsid), String(OWN_AP_PASS), &apIp);
                        }
                    }
                }
            }
            else {
                if (connectAttemptState != 0) {
                    Serial.printf("[WiFi] joined '%s' as %s\r\n",
                                  WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
                    WiFi.setSleep(WIFI_PS_NONE);
                    connectAttemptState = 0;
                    start_mdns();
                }
                // On a real network with nobody using our AP: fold it away.
                if (apActive && WiFi.softAPgetStationNum() == 0) {
                    stop_own_ap();
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