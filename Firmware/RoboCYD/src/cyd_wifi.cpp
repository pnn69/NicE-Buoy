#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "cyd_wifi.h"
#include "cyd_display.h"

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

    if (robobuoy_found) {
        Serial.println("Found 'ROBOBUOY'. Connecting...");
        tft.drawString("Connecting to ROBOBUOY...", 20, 50);
        WiFi.begin("ROBOBUOY", "");
    } else if (nice_wifi_found) {
        Serial.println("Found 'NicE_WiFi'. Connecting...");
        tft.drawString("Connecting to NicE_WiFi...", 20, 50);
        WiFi.begin("NicE_WiFi", "!Ni1001100110");
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

    Serial.println("\nWiFi CONNECTED");
    WiFi.setSleep(WIFI_PS_NONE); // Disable sleep to ensure fast OTA
    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    char buf[64];
    sprintf(buf, "IP: %s", ip.toString().c_str());
    tft.drawString("Connected!", 20, 80);
    tft.drawString(buf, 20, 110);
    delay(2000);
    return true;
}

void setup_Arduino_OTA() {
    byte mac[6];
    WiFi.macAddress(mac);
    char hostname[32];
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
    MDNS.begin(hostname);
    Serial.printf("OTA hostname set to: %s\n", hostname);
}

void init_wifi_and_ota() {
    scan_and_connect_wifi();
    setup_Arduino_OTA();
}

void handle_ota() {
    ArduinoOTA.handle();
}
