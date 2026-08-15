/**
 * ICM-20948 9DoF IMU Telemetry & OTA Web Server
 * 
 * Features:
 * - High-speed real-time updates via Server-Sent Events (SSE)
 * - Custom responsive HTML5 dashboard loaded from SPIFFS (standalone, zero CDN dependencies)
 * - Seamless ArduinoOTA firmware updates (Hostname: "ICM20948")
 * - Smart WiFi handling: attempts connection to STA, falls back to Access Point (AP) mode
 * - Robust sensor initialization with automatic AD0 I2C address detection (0x69 or 0x68)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "ICM_20948.h"
#include <Preferences.h>

// Global damping parameters stored in NVS
float damp_acc = 0.15f;
float damp_gyro = 0.15f;
float damp_mag = 0.15f;
float damp_att = 0.15f;

// ==========================================
// WIFI CONFIGURATION
// ==========================================
// Replace with your local WiFi credentials:
#define WIFI_SSID "NicE_WiFi"
#define WIFI_PASSWORD "!Ni1001100110"

// Fallback Access Point (AP) credentials (used if STA connection fails):
#define AP_SSID "ICM20948_AP"
#define AP_PASSWORD "password123"

// ==========================================
// TELEMETRY SETTINGS
// ==========================================
// Update interval in milliseconds (100ms = 10Hz, provides high-speed updates)
const unsigned long TELEMETRY_INTERVAL_MS = 100; 

// ==========================================
// GLOBAL OBJECTS & STATE
// ==========================================
AsyncWebServer server(80);
AsyncEventSource events("/events");
ICM_20948_I2C myICM;

bool isAPMode = false;
bool sensor_ok = false;
unsigned long last_telemetry_time = 0;

// ==========================================
// SENSOR SETUP & READ FUNCTIONS
// ==========================================
void initSensor() {
    Serial.println("Initializing ICM-20948 sensor...");
    
    // The sensor can reside at I2C address 0x69 (AD0 pin high = 1) or 0x68 (AD0 pin low = 0).
    // Let's attempt to connect to 0x69 first (SparkFun default), then fallback to 0x68.
    myICM.begin(Wire, 1);
    
    if (myICM.status != ICM_20948_Stat_Ok) {
        Serial.print("Sensor not found at AD0=1 (0x69). Error: ");
        Serial.println(myICM.statusString());
        Serial.println("Attempting fallback to AD0=0 (0x68)...");
        
        myICM.begin(Wire, 0);
    }
    
    if (myICM.status == ICM_20948_Stat_Ok) {
        sensor_ok = true;
        Serial.println("ICM-20948 sensor initialized successfully!");
        
        // Configure Digital Low Pass Filter (DLPF) to stabilize readings and filter noise
        ICM_20948_dlpcfg_t dlpcfg;
        dlpcfg.a = acc_d111bw4_n136bw;  // Accelerometer low-pass filter: 111.4Hz bandwidth
        dlpcfg.g = gyr_d119bw5_n154bw3; // Gyroscope low-pass filter: 119.5Hz bandwidth
        myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), dlpcfg);
        
        // Enable the DLPF
        myICM.enableDLPF(ICM_20948_Internal_Acc, true);
        myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
        Serial.println("DLPF configured and enabled successfully.");
    } else {
        sensor_ok = false;
        Serial.print("ICM-20948 initialization failed! Error: ");
        Serial.println(myICM.statusString());
        Serial.println("Please check I2C wiring (SDA=GPIO21, SCL=GPIO22, VCC=3.3V, GND) and pullups.");
    }
}

// ==========================================
// NETWORK SETUP
// ==========================================
void setupWiFi() {
    Serial.println("Configuring Network...");
    
    // Check if user has entered custom credentials
    if (strcmp(WIFI_SSID, "YOUR_WIFI_SSID") != 0 && strlen(WIFI_SSID) > 0) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        Serial.printf("Connecting to WiFi SSID: %s ", WIFI_SSID);
        
        // Wait for connection with a 10-second timeout
        int timeout_counter = 0;
        while (WiFi.status() != WL_CONNECTED && timeout_counter < 20) {
            delay(500);
            Serial.print(".");
            timeout_counter++;
        }
        Serial.println();
    }
    
    // Fallback to AP Mode if STA is unconfigured or failed to connect
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\r\nWiFi connection failed or unconfigured.");
        Serial.printf("Starting Fallback Access Point (AP) mode...\r\nSSID: %s\r\nPassword: %s\r\n", AP_SSID, AP_PASSWORD);
        
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID, AP_PASSWORD);
        isAPMode = true;
        
        Serial.print("AP Active. Server IP address: ");
        Serial.println(WiFi.softAPIP());
    } else {
        isAPMode = false;
        Serial.print("\r\nWiFi Connected successfully! IP Address: ");
        Serial.println(WiFi.localIP());
    }
}

// ==========================================
// ARDUINO OTA SETUP
// ==========================================
void setupOTA() {
    // Set Hostname as requested: "ICM20948"
    ArduinoOTA.setHostname("ICM20948");
    
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_SPIFFS
            type = "filesystem";
            // SPIFFS must be unmounted before updating the filesystem
            SPIFFS.end();
        }
        Serial.println("Start OTA Update: updating " + type);
    });
    
    ArduinoOTA.onEnd([]() {
        Serial.println("\r\nOTA Update Completed!");
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Authentication Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connection Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    
    ArduinoOTA.begin();
    Serial.println("ArduinoOTA Service Started (mDNS: ICM20948.local)");
}

// ==========================================
// MAIN SETUP
// ==========================================
void setup() {
    // Initialize serial console
    Serial.begin(115200);
    delay(1000);
    Serial.println("\r\n=============================================");
    Serial.println("       ICM-20948 9DoF IMU Web Telemetry      ");
    Serial.println("=============================================");

    // Initialize SPIFFS with auto-format on failure
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS mount failed! (An auto-format was attempted)");
    } else {
        Serial.println("SPIFFS mounted successfully.");
    }

    // Initialize I2C interface
    Wire.begin();
    Wire.setClock(100000); // 100kHz Standard Mode I2C (Highly stable, prevents bit-flip errors)
    
    // Initialize Sensor
    initSensor();

    // Set up network (STA with fallback to AP)
    setupWiFi();

    // Set up OTA uploads
    setupOTA();

    // Configure and launch AsyncWebServer routes
    
    // Test Read Route: Verify index.html can be read
    server.on("/test-read", HTTP_GET, [](AsyncWebServerRequest *request){
        File file = SPIFFS.open("/index.html", "r");
        if (!file) {
            file = SPIFFS.open("index.html", "r");
        }

        if (file) {
            String content = "File found! First 100 bytes:\n";
            for (int i = 0; i < 100 && file.available(); i++) {
                content += (char)file.read();
            }
            file.close();
            request->send(200, "text/plain", content);
        } else {
            request->send(404, "text/plain", "File not found!");
        }
    });

    // Calibration Route: Serves the calibration page
    server.on("/calibration", HTTP_GET, [](AsyncWebServerRequest *request){
        if (SPIFFS.exists("/calibration.html")) {
            request->send(SPIFFS, "/calibration.html", "text/html");
        } else if (SPIFFS.exists("calibration.html")) {
            request->send(SPIFFS, "calibration.html", "text/html");
        } else {
            request->send(404, "text/plain", "calibration.html not found on SPIFFS!");
        }
    });

    // Initialize Preferences to load damping factors from NVS
    Preferences preferences;
    preferences.begin("damping", true); // read-only mode
    damp_acc = preferences.getFloat("acc", 0.15f);
    damp_gyro = preferences.getFloat("gyro", 0.15f);
    damp_mag = preferences.getFloat("mag", 0.15f);
    damp_att = preferences.getFloat("att", 0.15f);
    preferences.end();
    
    Serial.println("\r\nDamping factors loaded from NVS:");
    Serial.printf("  Accel: %.2f\r\n", damp_acc);
    Serial.printf("  Gyro: %.2f\r\n", damp_gyro);
    Serial.printf("  Mag: %.2f\r\n", damp_mag);
    Serial.printf("  Att: %.2f\r\n", damp_att);

    // Route to save dynamic damping factor in NVS
    server.on("/set-damping", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("sensor") && request->hasParam("val")) {
            String sensor = request->getParam("sensor")->value();
            float val = request->getParam("val")->value().toFloat();
            
            Preferences prefs;
            prefs.begin("damping", false); // read/write
            
            if (sensor == "acc") {
                damp_acc = val;
                prefs.putFloat("acc", val);
            } else if (sensor == "gyro") {
                damp_gyro = val;
                prefs.putFloat("gyro", val);
            } else if (sensor == "mag") {
                damp_mag = val;
                prefs.putFloat("mag", val);
            } else if (sensor == "att") {
                damp_att = val;
                prefs.putFloat("att", val);
            }
            
            prefs.end();
            Serial.printf("Saved new damping factor in NVS: %s = %.2f\r\n", sensor.c_str(), val);
            request->send(200, "text/plain", "OK");
        } else {
            request->send(400, "text/plain", "Missing parameter sensor or val");
        }
    });

    // Serve static files from SPIFFS root (highly optimized, handles chunking and caching)
    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

    // Diagnostic Route: List all files on SPIFFS
    server.on("/list-files", HTTP_GET, [](AsyncWebServerRequest *request){
        String output = "SPIFFS File System Diagnostics:\n";
        output += "--------------------------------\n";
        
        File root = SPIFFS.open("/");
        if (!root) {
            output += "Failed to open SPIFFS root directory!\n";
        } else if (!root.isDirectory()) {
            output += "Root path '/' is not a directory!\n";
        } else {
            File file = root.openNextFile();
            if (!file) {
                output += "The file system is empty. No files found!\n";
            } else {
                while (file) {
                    output += "File: " + String(file.name()) + " - Size: " + String(file.size()) + " bytes\n";
                    file = root.openNextFile();
                }
            }
        }
        request->send(200, "text/plain", output);
    });

    // Event Stream Route
    server.addHandler(&events);

    // Launch server
    server.begin();
    Serial.println("HTTP AsyncWebServer started on Port 80.");
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
    // Handle outstanding OTA packets
    ArduinoOTA.handle();

    // Handle high-speed sensor data polling & broadcasting
    unsigned long current_time = millis();
    if (current_time - last_telemetry_time >= TELEMETRY_INTERVAL_MS) {
        last_telemetry_time = current_time;

        // Only send updates if there are connected web browsers to preserve bandwidth/CPU
        if (events.count() > 0) {
            // Retain previous readings across loops to avoid spikes or zeros if a transaction fails
            static float ax = 0, ay = 0, az = 0;
            static float gx = 0, gy = 0, gz = 0;
            static float mx = 0, my = 0, mz = 0;
            static float temp = 0;
            static String last_read_status = "Unknown";

            // Check if sensor is working and has new data
            if (sensor_ok && myICM.dataReady()) {
                myICM.getAGMT(); // Poll all data fields (accel, gyro, mag, temp)
                
                if (myICM.status == ICM_20948_Stat_Ok) {
                    ax = myICM.accX();
                    ay = myICM.accY();
                    az = myICM.accZ();
                    
                    gx = myICM.gyrX();
                    gy = myICM.gyrY();
                    gz = myICM.gyrZ();
                    
                    mx = myICM.magX();
                    my = myICM.magY();
                    mz = myICM.magZ();
                    
                    // Retrieve raw 16-bit temperature from the sensor structure
                    int16_t raw_temp = myICM.agmt.tmp.val;
                    
                    // Correct the known ICM-20948 hardware register anomaly (bit 13/0x2000 occasionally flips high)
                    if (raw_temp & 0x2000) {
                        raw_temp &= ~0x2000;
                    }
                    
                    // Convert corrected raw value to Celsius using InvenSense datasheet formula
                    float new_temp = (((float)raw_temp - 21) / 333.87) + 21;
                    
                    // Apply Exponential Moving Average (EMA) to smooth out minor thermal noise
                    if (temp == 0.0f) {
                        temp = new_temp;
                    } else {
                        temp = 0.9f * temp + 0.1f * new_temp;
                    }
                    
                    last_read_status = "Success";
                } else {
                    last_read_status = String(myICM.statusString());
                    Serial.print("Sensor read error: ");
                    Serial.println(last_read_status);
                }
            } else if (sensor_ok && !myICM.dataReady()) {
                // Keep last values but skip updating if no new data is ready
            } else {
                // If sensor connection failed during boot, try re-initializing periodically
                static unsigned long last_reinit_attempt = 0;
                if (current_time - last_reinit_attempt > 5000) { // every 5 seconds
                    last_reinit_attempt = current_time;
                    Serial.println("Attempting to reconnect/re-initialize sensor...");
                    initSensor();
                }
            }

            // Construct JSON telemetry payload
            StaticJsonDocument<512> doc;
            doc["sensor_ok"] = sensor_ok;
            doc["read_status"] = last_read_status;
            doc["damp_acc"] = damp_acc;
            doc["damp_gyro"] = damp_gyro;
            doc["damp_mag"] = damp_mag;
            doc["damp_att"] = damp_att;
            doc["ax"] = ax;
            doc["ay"] = ay;
            doc["az"] = az;
            doc["gx"] = gx;
            doc["gy"] = gy;
            doc["gz"] = gz;
            doc["mx"] = mx;
            doc["my"] = my;
            doc["mz"] = mz;
            doc["temp"] = temp;
            
            // Diagnostics metadata
            doc["heap"] = ESP.getFreeHeap();
            doc["uptime"] = current_time / 1000;
            
            if (isAPMode) {
                doc["ip"] = WiFi.softAPIP().toString();
                doc["ssid"] = AP_SSID;
                doc["rssi"] = 0; // Local client connecting to AP
            } else {
                doc["ip"] = WiFi.localIP().toString();
                doc["ssid"] = WIFI_SSID;
                doc["rssi"] = WiFi.RSSI();
            }

            // Serialize and stream
            String jsonOutput;
            serializeJson(doc, jsonOutput);
            
            // Broadcast the Event to all connected browser sessions
            events.send(jsonOutput.c_str(), "sensor_data", current_time);
        }
    }
}
