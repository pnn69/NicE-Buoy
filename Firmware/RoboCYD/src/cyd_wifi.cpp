#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <AsyncUDP.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <SPIFFS.h>
#include "cyd_wifi.h"
#include "cyd_display.h"
#include "boot_screen.h"
#include "buoy_data.h"
#include "cyd_lora.h"

AsyncUDP udp;

// Instantiate the global WebServer running on standard HTTP port 80 and WebSockets on port 81
WebServer server(80);
WebSocketsServer webSocket(81);

//***************************************************************************************************
//      Network policy: at home we are a client, in the field we are the network
//***************************************************************************************************
// The CYD decides once, at boot: if NicE_WiFi is in reach we join it like everything else; if it
// is not, we become Robo_WiFi and the buoys come to us. That is the whole of the field network -
// the Tops join us, the Sub deliberately does not (it keeps to its own SUB_<id> AP so the client
// slots stay free for the Tops and a phone).
//
// Deciding once, rather than continuously, is on purpose: migrating home mid-session would tear
// Robo_WiFi out from under every Top at once. We do re-check in the background, but only while
// nobody is connected to us - see cyd_wifi_background() at the bottom of this file.
static const char *HOME_SSID = "NicE_WiFi";
static const char *HOME_PASS = "!Ni1001100110";
static const char *FIELD_SSID = "Robo_WiFi";
static const char *FIELD_PASS = "geenanker";
static const char *MDNS_HOST = "robocyd"; // http://robocyd.local, in either mode

static DNSServer dnsServer;   // wildcard DNS, so joining Robo_WiFi opens the dashboard by itself
static bool apActive = false; // we are currently Robo_WiFi

// mDNS gives us a name that works in both modes. The captive portal below cannot do that: we will
// not hijack DNS on NicE_WiFi, only on our own network.
static void start_mdns()
{
    // Do NOT call MDNS.begin() or MDNS.end() here. ArduinoOTA.begin() already brings the mDNS
    // responder up under this same hostname, and standing a second one next to it - or tearing
    // it down while OTA still holds it - is exactly what the CYD file already warned about:
    // "dual-allocation heap responder collisions and panics". We only add our own HTTP record
    // on top of the responder OTA created, once.
    static bool advertised = false;
    if (advertised)
    {
        return;
    }
    if (MDNS.addService("http", "tcp", 80))
    {
        advertised = true;
        Serial.printf("[WiFi] reachable as http://%s.local\r\n", MDNS_HOST);
    }
}

// Try one network, without scanning for it first.
//
// WiFi.scanNetworks() is the blocking form: it walks all thirteen channels and stalls everything
// for seconds. Once we are also running a softAP that drags the single radio off our own channel
// long enough for a connected Top or phone to give up on us. begin() does its own short, targeted
// probe and simply fails on timeout when the AP is not there - the only thing the scan ever told
// us that we actually needed.
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
        // Association is the longest blocking stretch of the whole boot. Ticking the splash from
        // inside the wait is what keeps it from looking hung. Inert once boot is over, which
        // matters because handle_wifi_clients() calls us again every 45 s in the field.
        boot_screen_tick();
        delay(100);
    }

    // Associated is not the same as usable. WL_CONNECTED fires on association, but starting mDNS
    // or OTA while the lease is still 0.0.0.0 takes the lwIP stack - and the chip - down with it.
    while (WiFi.localIP() == IPAddress(0, 0, 0, 0))
    {
        if (millis() - start > timeoutMs + 6000)
        {
            Serial.println("no DHCP lease");
            WiFi.disconnect();
            return false;
        }
        boot_screen_tick();
        delay(100);
    }

    WiFi.setSleep(WIFI_PS_NONE);
    Serial.printf("joined, IP %s\r\n", WiFi.localIP().toString().c_str());
    return true;
}

// Become Robo_WiFi: the field network the Tops look for.
static void start_field_ap()
{
    WiFi.mode(WIFI_AP);

    // Same subnet as home on purpose, so the field network and the living room look identical
    // from a browser's point of view, and the AP is its own gateway - handing out a gateway that
    // never answers is what makes Android decide a network is broken and drop it.
    IPAddress apIP(192, 168, 4, 1);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));

    // max_connection defaults to 4 in the Arduino wrapper, which is exactly the budget of three
    // Tops plus a phone and leaves nothing for a laptop. The ESP32 silicon allows 15; 8 is plenty.
    if (WiFi.softAP(FIELD_SSID, FIELD_PASS, 1, 0, 8))
    // if (WiFi.softAP(FIELD_SSID, FIELD_PASS))
    {
        Serial.printf("AP IP = %s\n", WiFi.softAPIP().toString().c_str());
        WiFi.setSleep(WIFI_PS_NONE);
        apActive = true;
        String url = "http://" + WiFi.softAPIP().toString();
        boot_screen_status(url.c_str());

        // Wildcard DNS: every lookup resolves to us, so a phone's own connectivity probe lands on
        // our web server, gets the redirect from onNotFound instead of the 204 it expected, and
        // opens the "sign in to network" sheet on the dashboard with nobody typing an address.
        dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
        dnsServer.start(53, "*", apIP);

        Serial.printf("[WiFi] AP '%s' up on %s\r\n", FIELD_SSID, WiFi.softAPIP().toString().c_str());
        boot_screen_wifi(BOOT_WIFI_AP);
        boot_screen_status(WiFi.softAPIP().toString().c_str());
    }
    else
    {
        Serial.println("[WiFi] softAP failed!");
        boot_screen_wifi(BOOT_WIFI_DOWN);
        boot_screen_status("AP FAILED");
    }
    start_mdns();
}

bool scan_and_connect_wifi()
{
    // No fillScreen here any more: the splash is already up and reports progress through its own
    // status band. Clearing the panel would throw the logo away a few hundred ms after drawing it.
    WiFi.mode(WIFI_STA);
    boot_screen_status("NicE_WiFi");

    if (try_connect(HOME_SSID, HOME_PASS, 8000))
    {
        apActive = false;
        start_mdns();
        boot_screen_wifi(BOOT_WIFI_CLIENT);
        boot_screen_status(WiFi.localIP().toString().c_str());
        return true;
    }

    start_field_ap();
    return false;
}

void setup_Arduino_OTA()
{
    // Same name mDNS advertises - see the note in start_mdns(). Also drops the underscore, which
    // is not legal in a hostname label.
    ArduinoOTA.setHostname(MDNS_HOST);

    ArduinoOTA.onStart([]()
                       {
        Serial.println("OTA Start");
        tft.fillScreen(TFT_BLACK);
        
        int w = tft.width();
        int h = tft.height();
        
        // Draw initial white boundary border dynamically
        tft.drawRect(0, 0, w, h, TFT_WHITE);
        
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextSize(3);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("OTA Update...", w / 2, h / 2 - 30); });

    ArduinoOTA.onEnd([]()
                     {
        Serial.println("\nOTA End");
        tft.fillScreen(TFT_BLACK);
        tft.drawString("Restarting...", tft.width() / 2, tft.height() / 2); });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          {
        static int lastPercent = -1;
        int percent = progress / (total / 100);
        
        if (percent != lastPercent) {
            lastPercent = percent;
            
            Serial.printf("Progress: %u%%\r", percent);
            
            int w = tft.width();
            int h = tft.height();
            
            // Draw progress text (only clearing text area dynamically to avoid flicker)
            tft.fillRect(20, h / 2 + 10, w - 40, 40, TFT_BLACK);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setTextSize(3);
            tft.setTextDatum(MC_DATUM);
            
            char buf[32];
            sprintf(buf, "Progress: %d%%", percent);
            tft.drawString(buf, w / 2, h / 2 + 30);

            // Dynamic progress bar wrapping clockwise around screen border (filling red on white)
            int w_idx = w - 1;
            int h_idx = h - 1;
            
            if (percent <= 25) {
                int x_end = map(percent, 0, 25, 0, w_idx);
                tft.drawLine(0, 0, x_end, 0, TFT_RED);
            } else {
                tft.drawLine(0, 0, w_idx, 0, TFT_RED); // Fully draw top line
                if (percent <= 50) {
                    int y_end = map(percent, 25, 50, 0, h_idx);
                    tft.drawLine(w_idx, 0, w_idx, y_end, TFT_RED);
                } else {
                    tft.drawLine(w_idx, 0, w_idx, h_idx, TFT_RED); // Fully draw right line
                    if (percent <= 75) {
                        int x_end = map(percent, 50, 75, w_idx, 0);
                        tft.drawLine(w_idx, h_idx, x_end, h_idx, TFT_RED);
                    } else {
                        tft.drawLine(w_idx, h_idx, 0, h_idx, TFT_RED); // Fully draw bottom line
                        int y_end = map(percent, 75, 100, h_idx, 0);
                        tft.drawLine(0, h_idx, 0, y_end, TFT_RED);
                    }
                }
            }
        } });

    ArduinoOTA.onError([](ota_error_t error)
                       {
        Serial.printf("Error[%u]\n", error);
        ESP.restart(); });

    ArduinoOTA.begin();
    // Redundant mDNS initialization removed (ArduinoOTA.begin() natively handles mDNS registration automatically,
    // thereby avoiding dual-allocation heap responder collisions and panics!).
    Serial.printf("OTA Service initialized with hostname: %s.local\n", MDNS_HOST);
}

// WebSocket server event callback
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case WStype_DISCONNECTED:
        Serial.printf("[%u] Web Client Disconnected!\n", num);
        break;
    case WStype_CONNECTED:
    {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Web Client Connected from %s\n", num, ip.toString().c_str());
        // Send welcome message to newly connected webpage clients
        webSocket.sendTXT(num, "UDP:Welcome to RoboCYD Dashboard");
        break;
    }
    case WStype_TEXT:
    {
        String message = "";
        message.reserve(length);
        for (size_t i = 0; i < length; i++)
        {
            message += (char)payload[i];
        }

        // Ensure message is trimmed and has proper line endings before transmission to hardware receivers
        String trimmed = message;
        trimmed.trim();
        String finalMsg = trimmed + "\r\n";

        Serial.print("WebSocket RX Command: ");
        Serial.println(trimmed);

        // Forward the command directly over both communication channels with proper line endings!
        parse_buoy_packet(trimmed, "UDP"); // Register the change locally on screen
        send_lora_packet(finalMsg);        // Send to LoRa radio channel WITH \r\n
        udp_broadcast(finalMsg);           // Send to UDP network channel WITH \r\n
        break;
    }
    default:
        break;
    }
}

void init_wifi_and_ota()
{
    // CRITICAL FIRST STEP: Mount SPIFFS filesystem FIRST, before connecting to WiFi or starting servers.
    // This isolates the filesystem mount from active network interrupts and thread allocations,
    // preventing watchdog timeouts and memory leaks.
    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
    }
    else
    {
        Serial.println("SPIFFS mounted successfully!");
    }

    // Either we joined home or we are now Robo_WiFi; both are networks OTA can be reached over,
    // and in the field being the AP is the only way it ever will be.
    scan_and_connect_wifi();
    setup_Arduino_OTA();
    start_mdns(); // after OTA created the responder, not before

    // Configure WebServer HTTP standard Port 80 routes
    server.on("/", []()
              {
        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        File file = SPIFFS.open("/index.html", "r");
        if (file && !file.isDirectory()) {
            server.streamFile(file, "text/html");
            file.close();
        } else {
            server.send(404, "text/plain", "index.html not found");
        } });

    server.on("/index.js", []()
              {
        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        File file = SPIFFS.open("/index.js", "r");
        if (file && !file.isDirectory()) {
            server.streamFile(file, "application/javascript");
            file.close();
        } else {
            server.send(404, "text/plain", "index.js not found");
        } });

    server.on("/style.css", []()
              {
        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        File file = SPIFFS.open("/style.css", "r");
        if (file && !file.isDirectory()) {
            server.streamFile(file, "text/css");
            file.close();
        } else {
            server.send(404, "text/plain", "style.css not found");
        } });

    // Captive portal. A relative "/" is not enough for the phone's connectivity probe to follow -
    // it has to be pointed at us by absolute address. Combined with the wildcard DNS above, this
    // is what makes the "sign in to network" sheet open the dashboard when you join Robo_WiFi.
    server.onNotFound([]()
                      {
        // Only hijack unknown URLs while we are the access point - that is what makes the phone's
        // captive-portal sheet open the dashboard. On a real network a 404 must stay a 404, or a
        // mistyped fetch quietly comes back as the whole dashboard instead of an error.
        if (apActive) {
            server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
            server.send(302, "text/plain", "");
        } else {
            server.send(404, "text/plain", "Not found");
        } });

    server.begin();
    Serial.println("HTTP WebServer started on port 80!");

    // Start WebSocket Server on Port 81
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket Server started on port 81!");

    // Set up AsyncUDP listener on port 1001 for buoy broadcasts
    if (udp.listen(1001))
    {
        Serial.println("Listening on UDP port 1001 for Buoy telemetry...");
        udp.onPacket([](AsyncUDPPacket packet)
                     {
            String stringUdpIn = String((const char *)packet.data(), packet.length());
            
            // Parse locally
            parse_buoy_packet(stringUdpIn, "UDP:" + packet.remoteIP().toString());
            
            // Broadcast over WebSockets to webpage clients using dynamic RSSI and IP
            int rssi = WiFi.RSSI();
            String senderIp = packet.remoteIP().toString();
            broadcast_websocket_udp(stringUdpIn, rssi, senderIp); });
    }
    else
    {
        Serial.println("Failed to bind UDP port 1001!");
    }
}

void handle_ota()
{
    ArduinoOTA.handle();
}

void udp_broadcast(const String &message)
{
    // Turn the on-screen UDP indicator dot RED for the duration of the transmit blink
    last_udp_tx_ms = millis();

    // Explicitly broadcast to all devices on port 1001, matching RoboLora controlwifi
    udp.broadcastTo(message.c_str(), 1001);
}

// A log line on the same UDP port the Subs and Tops use (1002). The handheld had no logging of any
// kind - no serial cable in the field, no diagnostic endpoint - so when something it does silently
// stops happening there is nothing to look at but the screen. Everything else on this network can
// be watched from the PC; this makes the CYD the same.
void cyd_log(const char *fmt, ...)
{
    char body[180];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);
    char line[220];
    snprintf(line, sizeof(line), "CYD %lu %s\n", (unsigned long)millis(), body);
    Serial.print(line);
    udp.broadcastTo(line, 1002);
}

void broadcast_websocket_udp(const String &payload, int rssi, const String &ip)
{
    // RoboLora protocol: UDP:RSSI:IP:$Payload
    String wsMsg = "UDP:" + String(rssi) + ":" + ip + ":" + payload;
    webSocket.broadcastTXT(wsMsg.c_str());
}

void broadcast_websocket_lora(const String &payload, int rssi)
{
    // RoboLora protocol: LORA:RSSI:$Payload
    String wsMsg = "LORA:" + String(rssi) + ":" + payload;
    webSocket.broadcastTXT(wsMsg.c_str());
}

void handle_wifi_clients()
{
    server.handleClient();
    webSocket.loop();

    if (!apActive)
    {
        return;
    }

    // Serve the wildcard DNS that drives the captive portal.
    dnsServer.processNextRequest();

    // Re-check for home in the background, but only while nobody is connected to us. With a Top
    // or a phone attached, migrating would tear Robo_WiFi out from under them - and on the CYD
    // clients are the normal state, so this only ever fires when the fleet is off or out of
    // range. It is the "carried everything indoors" case, and it saves a power cycle.
    static unsigned long lastHomeCheck = 0;
    unsigned long now = millis();
    if (WiFi.softAPgetStationNum() > 0)
    {
        lastHomeCheck = now;
        return;
    }
    if (now - lastHomeCheck < 45000)
    {
        return;
    }
    lastHomeCheck = now;

    Serial.println("[WiFi] nobody on Robo_WiFi - checking whether we are home yet");
    WiFi.mode(WIFI_AP_STA);
    if (try_connect(HOME_SSID, HOME_PASS, 8000))
    {
        dnsServer.stop();
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_STA);
        apActive = false;
        Serial.println("[WiFi] home found - Robo_WiFi folded away");
        start_mdns();
    }
    else
    {
        WiFi.mode(WIFI_AP);
    }
}
