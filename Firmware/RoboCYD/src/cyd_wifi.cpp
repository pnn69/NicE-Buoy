#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <AsyncUDP.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <SPIFFS.h>
#include "cyd_wifi.h"
#include "cyd_display.h"
#include "buoy_data.h"
#include "cyd_lora.h"

AsyncUDP udp;

// Instantiate the global WebServer running on standard HTTP port 80 and WebSockets on port 81
WebServer server(80);
WebSocketsServer webSocket(81);

bool scan_and_connect_wifi() {
    Serial.println("Starting WiFi scan...");
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.drawString("WiFi Scan...", 20, 20);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    int n = WiFi.scanNetworks();
    Serial.printf("Scan done: %d networks found\n", n);

    bool robobuoy_found = false;
    bool nice_wifi_found = false;

    for (int i = 0; i < n; ++i) {
        String ssid = WiFi.SSID(i);
        if (ssid == "ROBOBUOY") {
            robobuoy_found = true;
        } else if (ssid == "NicE_WiFi") {
            nice_wifi_found = true;
        }
    }

    // PRIORITIZE "NicE_WiFi" to join same subnet as RoboLora module (192.168.1.x)
    if (nice_wifi_found) {
        Serial.println("Found 'NicE_WiFi'. Connecting...");
        tft.drawString("Connecting to NicE_WiFi...", 20, 50);
        WiFi.begin("NicE_WiFi", "!Ni1001100110");
    } else if (robobuoy_found) {
        Serial.println("Found 'ROBOBUOY'. Connecting...");
        tft.drawString("Connecting to ROBOBUOY...", 20, 50);
        WiFi.begin("ROBOBUOY", "");
    } else {
        Serial.println("No configured WiFi found. AP mode fallback...");
        tft.drawString("WiFi not found!", 20, 50);
        tft.drawString("Starting AP: RoboCYD", 20, 80);
        WiFi.softAP("RoboCYD", "");
        return false;
    }

    unsigned long conn_timeout = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - conn_timeout > 15000) {
            Serial.println("\nConnection timed out. Starting AP RoboCYD...");
            tft.drawString("WiFi Timeout!", 20, 80);
            tft.drawString("Starting AP: RoboCYD", 20, 110);
            WiFi.softAP("RoboCYD", "");
            return false;
        }
    }

    // CRITICAL: Explicitly wait for DHCP to assign a valid IP address before starting mDNS/OTA!
    // WL_CONNECTED occurs on AP association, but initializing network responders while local IP is still 0.0.0.0
    // causes the lwIP network stack to instantly crash and reboot the ESP32.
    Serial.println("\nWiFi Associated. Waiting for DHCP IP...");
    tft.drawString("Acquiring IP...", 20, 80);
    
    unsigned long ip_timeout = millis();
    while (WiFi.localIP() == IPAddress(0, 0, 0, 0)) {
        delay(100);
        Serial.print("o");
        if (millis() - ip_timeout > 6000) { // 6 seconds timeout for DHCP IP lease
            Serial.println("\nFailed to acquire DHCP IP. AP fallback...");
            tft.drawString("IP Lease Failed!", 20, 110);
            WiFi.softAP("RoboCYD", "");
            return false;
        }
    }

    Serial.println("\nWiFi CONNECTED");
    WiFi.setSleep(WIFI_PS_NONE); // Disable sleep to ensure fast OTA
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    char buf[64];
    sprintf(buf, "IP: %s", ip.toString().c_str());
    tft.drawString("Connected!", 20, 80);
    tft.drawString(buf, 20, 110);
    delay(1000);
    return true;
}

void setup_Arduino_OTA() {
    byte mac[6];
    WiFi.macAddress(mac);
    
    // Declared static to ensure pointer string lives forever in global heap memory,
    // avoiding stack deallocation pointers crashes inside ArduinoOTA/MDNS loops!
    static char hostname[32];
    sprintf(hostname, "CYD_%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    ArduinoOTA.setHostname(hostname);
    
    ArduinoOTA.onStart([]() {
        Serial.println("OTA Start");
        tft.fillScreen(TFT_BLACK);
        
        int w = tft.width();
        int h = tft.height();
        
        // Draw initial white boundary border dynamically
        tft.drawRect(0, 0, w, h, TFT_WHITE);
        
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextSize(3);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("OTA Update...", w / 2, h / 2 - 30);
    });
    
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA End");
        tft.fillScreen(TFT_BLACK);
        tft.drawString("Restarting...", tft.width() / 2, tft.height() / 2);
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
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
        }
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]\n", error);
        ESP.restart();
    });
    
    ArduinoOTA.begin();
    // Redundant mDNS initialization removed (ArduinoOTA.begin() natively handles mDNS registration automatically,
    // thereby avoiding dual-allocation heap responder collisions and panics!).
    Serial.printf("OTA Service initialized with hostname: %s.local\n", hostname);
}

// WebSocket server event callback
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Web Client Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Web Client Connected from %s\n", num, ip.toString().c_str());
            // Send welcome message to newly connected webpage clients
            webSocket.sendTXT(num, "UDP:Welcome to RoboCYD Dashboard");
            break;
        }
        case WStype_TEXT: {
            String message = "";
            message.reserve(length);
            for (size_t i = 0; i < length; i++) {
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
            send_lora_packet(finalMsg);         // Send to LoRa radio channel WITH \r\n
            udp_broadcast(finalMsg);            // Send to UDP network channel WITH \r\n
            break;
        }
        default:
            break;
    }
}

void init_wifi_and_ota() {
    // CRITICAL FIRST STEP: Mount SPIFFS filesystem FIRST, before connecting to WiFi or starting servers.
    // This isolates the filesystem mount from active network interrupts and thread allocations,
    // preventing watchdog timeouts and memory leaks.
    if (!SPIFFS.begin(true)) {
        Serial.println("An Error has occurred while mounting SPIFFS");
    } else {
        Serial.println("SPIFFS mounted successfully!");
    }

    bool connected = scan_and_connect_wifi();
    
    // Only initialize station-dependent network systems if connection was successful
    if (connected) {
        setup_Arduino_OTA();
    }

    // Configure WebServer HTTP standard Port 80 routes
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

    // Fallback/Captive Portal support
    server.onNotFound([]() {
        server.sendHeader("Location", "/", true);
        server.send(302, "text/plain", "");
    });

    server.begin();
    Serial.println("HTTP WebServer started on port 80!");

    // Start WebSocket Server on Port 81
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket Server started on port 81!");

    // Set up AsyncUDP listener on port 1001 for buoy broadcasts
    if (udp.listen(1001)) {
        Serial.println("Listening on UDP port 1001 for Buoy telemetry...");
        udp.onPacket([](AsyncUDPPacket packet) {
            String stringUdpIn = String((const char *)packet.data(), packet.length());
            
            // Parse locally
            parse_buoy_packet(stringUdpIn, "UDP:" + packet.remoteIP().toString());
            
            // Broadcast over WebSockets to webpage clients using dynamic RSSI and IP
            int rssi = WiFi.RSSI();
            String senderIp = packet.remoteIP().toString();
            broadcast_websocket_udp(stringUdpIn, rssi, senderIp);
        });
    } else {
        Serial.println("Failed to bind UDP port 1001!");
    }
}

void handle_ota() {
    ArduinoOTA.handle();
}

void udp_broadcast(const String &message) {
    // Turn the on-screen UDP indicator dot RED for the duration of the transmit blink
    last_udp_tx_ms = millis();

    // Explicitly broadcast to all devices on port 1001, matching RoboLora controlwifi
    udp.broadcastTo(message.c_str(), 1001);
}

void broadcast_websocket_udp(const String &payload, int rssi, const String &ip) {
    // RoboLora protocol: UDP:RSSI:IP:$Payload
    String wsMsg = "UDP:" + String(rssi) + ":" + ip + ":" + payload;
    webSocket.broadcastTXT(wsMsg.c_str());
}

void broadcast_websocket_lora(const String &payload, int rssi) {
    // RoboLora protocol: LORA:RSSI:$Payload
    String wsMsg = "LORA:" + String(rssi) + ":" + payload;
    webSocket.broadcastTXT(wsMsg.c_str());
}

void handle_wifi_clients() {
    server.handleClient();
    webSocket.loop();
}
