#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <Wire.h>
#include <LSM303.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>

#define I2C_SDA      21
#define I2C_SCL      22

const char* ssid = "NicE_WiFi";
const char* password = "!Ni1001100110";

#if defined(COMPASS_STANDALONE)
WebServer server(80);
#endif
WebSocketsServer ws(81);

// Renamed to compass_sensor to resolve conflicting global definition with QueueHandle_t compass!
LSM303 compass_sensor;

float declination = 2.6f;
int avg = 10;

float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;

float scaleX = 1;
float scaleY = 1;
float scaleZ = 1;

// Leveling/Accelerometer alignment variables (Default is perfectly flat)
float levelX = 0;
float levelY = 0;
float levelZ = 1;

float headingRaw = 0;
float headingHard = 0;
float headingHardSoft = 0;
float headingTilt = 0;
float pitchDeg = 0;
float rollDeg = 0;
float rawX = 0, rawY = 0, rawZ = 0;
float calX = 0, calY = 0, calZ = 0;

float global_hdg = 0;
int compass_avg_samples = 10;
int icm_mode = 4;

unsigned long lastBroadcast = 0;
bool isLIS2MDL = false;

// Independent low-pass filtering for compass vs. tilt dampening
// Highly responsive Option A (Snappy Mode) coefficients
float magAlpha = 0.45f;    // Highly responsive magnetometer for Option A (45% new data)
float accelAlpha = 0.25f; // Highly responsive accelerometer for Option A (45% new data)
bool firstRead = true;

float smoothed_ax = 0;
float smoothed_ay = 0;
float smoothed_az = 0;

float smoothed_mx = 0;
float smoothed_my = 0;
float smoothed_mz = 0;

void saveCalibration() {
    Preferences prefs;
    prefs.begin("compass", false);
    prefs.putFloat("ox", offsetX);
    prefs.putFloat("oy", offsetY);
    prefs.putFloat("oz", offsetZ);
    prefs.putFloat("sx", scaleX);
    prefs.putFloat("sy", scaleY);
    prefs.putFloat("sz", scaleZ);
    prefs.putInt("mode", icm_mode);
    prefs.end();
}

void saveLevelCalibration() {
    Preferences prefs;
    prefs.begin("compass", false);
    prefs.putFloat("lx", levelX);
    prefs.putFloat("ly", levelY);
    prefs.putFloat("lz", levelZ);
    prefs.end();
}

void loadCalibration() {
    Preferences prefs;
    prefs.begin("compass", true);
    offsetX = prefs.getFloat("ox", 0);
    offsetY = prefs.getFloat("oy", 0);
    offsetZ = prefs.getFloat("oz", 0);
    scaleX = prefs.getFloat("sx", 1);
    scaleY = prefs.getFloat("sy", 1);
    scaleZ = prefs.getFloat("sz", 1);

    levelX = prefs.getFloat("lx", 0);
    levelY = prefs.getFloat("ly", 0);
    levelZ = prefs.getFloat("lz", 1);

    icm_mode = prefs.getInt("mode", 4);
    if (icm_mode < 1 || icm_mode > 4) icm_mode = 4;
    prefs.end();

    // Guard against uninitialized/bad values from NVM (like 0, NaN or Inf)
    if (isnan(offsetX) || isinf(offsetX)) offsetX = 0;
    if (isnan(offsetY) || isinf(offsetY)) offsetY = 0;
    if (isnan(offsetZ) || isinf(offsetZ)) offsetZ = 0;

    if (isnan(scaleX) || isinf(scaleX) || scaleX == 0) scaleX = 1.0f;
    if (isnan(scaleY) || isinf(scaleY) || scaleY == 0) scaleY = 1.0f;
    if (isnan(scaleZ) || isinf(scaleZ) || scaleZ == 0) scaleZ = 1.0f;

    if (isnan(levelX) || isinf(levelX)) levelX = 0;
    if (isnan(levelY) || isinf(levelY)) levelY = 0;
    if (isnan(levelZ) || isinf(levelZ) || levelZ == 0) levelZ = 1.0f;
}

void initMagnetometer() {
    Wire.beginTransmission(0x1E);
    Wire.write(0x60); 
    Wire.write(0x80); 
    byte error = Wire.endTransmission();

    if (error == 0) {
        isLIS2MDL = true;
        Serial.println("LIS2MDL/LSM303AGR magnetometer detected and configured successfully!");
    } else {
        isLIS2MDL = false;
        Serial.println("Classic LSM303DLHC/DLM magnetometer selected.");
    }
}

bool readMagnetometer(float &mx, float &my, float &mz) {
    if (isLIS2MDL) {
        Wire.beginTransmission(0x1E);
        Wire.write(0x68); 
        byte error = Wire.endTransmission(false);
        if (error == 0) {
            Wire.requestFrom(0x1E, 6);
            if (Wire.available() == 6) {
                int16_t x = Wire.read() | (Wire.read() << 8);
                int16_t y = Wire.read() | (Wire.read() << 8);
                int16_t z = Wire.read() | (Wire.read() << 8);
                mx = x;
                my = y;
                mz = z;
                return true;
            }
        }
    }

    mx = compass_sensor.m.x;
    my = compass_sensor.m.y;
    mz = compass_sensor.m.z;
    return (mx != 0 || my != 0 || mz != 0);
}

// Helper function implementing the LSM303DLH Application Note (AN3192) Equation (13) quadrant logic
float calculateHeading(float X, float Y) {
    if (X > 0.0f && Y >= 0.0f) {
        return atan(Y / X) * 180.0f / PI;
    } else if (X > 0.0f && Y < 0.0f) {
        return atan(Y / X) * 180.0f / PI + 360.0f;
    } else if (X < 0.0f) {
        return atan(Y / X) * 180.0f / PI + 180.0f;
    } else if (X == 0.0f && Y > 0.0f) {
        return 90.0f;
    } else if (X == 0.0f && Y < 0.0f) {
        return 270.0f;
    }
    return 0.0f; // Default/fallback for X == 0, Y == 0
}

void updateCompass() {
    compass_sensor.read();

    float mx = 0, my = 0, mz = 0;
    readMagnetometer(mx, my, mz);
    
    // Invert the Y-axis of the magnetometer to correct the inverted rotation direction
    my = -my;

    float ax = compass_sensor.a.x;
    float ay = compass_sensor.a.y;
    float az = compass_sensor.a.z;

    // Apply constant, rock-solid, ultra-slow dual-rate filtering
    if (firstRead) {
        smoothed_ax = ax;
        smoothed_ay = ay;
        smoothed_az = az;
        smoothed_mx = mx;
        smoothed_my = my;
        smoothed_mz = mz;
        firstRead = false;
    } else {
        // Smooth the raw accelerometer with our constant ultra-slow dampening
        smoothed_ax = accelAlpha * ax + (1.0f - accelAlpha) * smoothed_ax;
        smoothed_ay = accelAlpha * ay + (1.0f - accelAlpha) * smoothed_ay;
        smoothed_az = accelAlpha * az + (1.0f - accelAlpha) * smoothed_az;
        
        // Snappy but filtered magnetometer (magAlpha = 0.15)
        smoothed_mx = magAlpha * mx + (1.0f - magAlpha) * smoothed_mx;
        smoothed_my = magAlpha * my + (1.0f - magAlpha) * smoothed_my;
        smoothed_mz = magAlpha * mz + (1.0f - magAlpha) * smoothed_mz;
    }

    rawX = smoothed_mx; rawY = smoothed_my; rawZ = smoothed_mz;

    // 1. Raw Heading
    headingRaw = calculateHeading(smoothed_mx, smoothed_my);

    // 2. Hard Iron
    float hmx = smoothed_mx - offsetX;
    float hmy = smoothed_my - offsetY;
    headingHard = calculateHeading(hmx, hmy) + declination;
    if (headingHard < 0) headingHard += 360;
    if (headingHard >= 360) headingHard -= 360;

    // 3. Hard + Soft Iron
    float cmx = hmx * scaleX;
    float cmy = hmy * scaleY;
    float cmz = (smoothed_mz - offsetZ) * scaleZ;
    calX = cmx; calY = cmy; calZ = cmz;

    headingHardSoft = calculateHeading(cmx, cmy) + declination;
    if (headingHardSoft < 0) headingHardSoft += 360;
    if (headingHardSoft >= 360) headingHard -= 360;

    // 4. Tilt Compensated (Hard + Soft + Pitch/Roll)
    // As per LSM303DLH App Note (AN3192), normalize the accelerometer vector:
    float normA = sqrt(smoothed_ax * smoothed_ax + smoothed_ay * smoothed_ay + smoothed_az * smoothed_az);
    float ax1 = 0.0f, ay1 = 0.0f, az1 = 1.0f;
    if (normA > 0.0f) {
        ax1 = smoothed_ax / normA;
        ay1 = smoothed_ay / normA;
        az1 = smoothed_az / normA;
    }

    // Apply Leveling Calibration (aligning misaligned sensor axis to housing G-axis)
    float ax_aligned = ax1;
    float ay_aligned = ay1;
    float az_aligned = az1;

    float s2 = levelX * levelX + levelY * levelY;
    if (s2 > 1e-6f) {
        float k = (1.0f - levelZ) / s2;
        float v_dot_p = levelY * ax1 - levelX * ay1;
        ax_aligned = ax1 * levelZ - levelX * az1 + k * v_dot_p * levelY;
        ay_aligned = ay1 * levelZ - levelY * az1 - k * v_dot_p * levelX;
        az_aligned = az1 * levelZ + (levelY * ay1 + levelX * ax1);
    }

    // Pitch (rho) = arcsin(-ax_aligned)
    float pitch = asin(constrain(-ax_aligned, -1.0f, 1.0f));
    
    // Roll (gamma) = arcsin(ay_aligned / cos(pitch))
    float cosPitch = cos(pitch);
    float roll = 0.0f;
    if (fabs(cosPitch) > 1e-4f) {
        roll = asin(constrain(ay_aligned / cosPitch, -1.0f, 1.0f));
    } else {
        roll = 0.0f; // Prevent division by zero / singularity at pitch = +-90 degrees
    }

    // Direct angle filtering using a constant, balanced coefficient (0.450f)
    static float smoothed_pitch = 0.0f;
    static float smoothed_roll = 0.0f;
    static bool firstAngleFilter = true;
    const float pitchRollAlpha = 0.450f; // Highly responsive filter for Option A (45% new data)

    if (firstAngleFilter) {
        smoothed_pitch = pitch;
        smoothed_roll = roll;
        firstAngleFilter = false;
    } else {
        smoothed_pitch = pitchRollAlpha * pitch + (1.0f - pitchRollAlpha) * smoothed_pitch;
        smoothed_roll = pitchRollAlpha * roll + (1.0f - pitchRollAlpha) * smoothed_roll;
    }

    rollDeg = smoothed_roll * 180.0 / PI;
    pitchDeg = smoothed_pitch * 180.0 / PI;

    // Invert the roll angle sign strictly inside the tilt compensation equations to align it with the inverted magnetometer Y-axis orientation!
    float roll_comp = -smoothed_roll;

    // Tilt Compensation Equations using the filtered pitch and roll angles:
    float Xh = cmx * cos(smoothed_pitch) + cmz * sin(smoothed_pitch);
    float Yh = cmx * sin(roll_comp) * sin(smoothed_pitch) + cmy * cos(roll_comp) - cmz * sin(roll_comp) * cos(smoothed_pitch);

    headingTilt = calculateHeading(Xh, Yh) + declination;
    if (headingTilt < 0) headingTilt += 360;
    if (headingTilt >= 360) headingTilt -= 360;
}

void broadcastData() {
    JsonDocument doc;
    doc["raw"] = headingRaw;
    doc["hard"] = headingHard;
    doc["hardSoft"] = headingHardSoft;
    doc["tilt"] = headingTilt;
    doc["heading"] = global_hdg;
    doc["pitch"] = pitchDeg;
    doc["roll"] = rollDeg;
    doc["mx_raw"] = rawX;
    doc["my_raw"] = rawY;
    doc["mz_raw"] = rawZ;
    doc["mx_cal"] = calX;
    doc["my_cal"] = calY;
    doc["mz_cal"] = calZ;
    
    String json;
    serializeJson(doc, json);
    ws.broadcastTXT(json);
}

void onWsEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    if (type == WStype_TEXT) {
        JsonDocument doc;
        DeserializationError error = deserializeJson(doc, payload);
        if (!error) {
            if (doc["command"] == "save_cal") {
                offsetX = doc["hi_x"];
                offsetY = doc["hi_y"];
                offsetZ = doc["hi_z"];
                scaleX = doc["si_x"];
                scaleY = doc["si_y"];
                scaleZ = doc["si_z"];
                saveCalibration();
                Serial.println("Calibration saved!");
            } else if (doc["command"] == "calibrate_level") {
                // Read the current smoothed accelerometer vector
                float normA = sqrt(smoothed_ax * smoothed_ax + smoothed_ay * smoothed_ay + smoothed_az * smoothed_az);
                if (normA > 0.0f) {
                    levelX = smoothed_ax / normA;
                    levelY = smoothed_ay / normA;
                    levelZ = smoothed_az / normA;
                    saveLevelCalibration();
                    Serial.printf("Level calibration saved: lx=%f, ly=%f, lz=%f\n", levelX, levelY, levelZ);
                }
            }
        }
    }
}

void scanI2CBus() {
    Serial.println("\n--- I2C Bus Scan ---");
    byte error, address;
    int nDevices = 0;
    for (address = 1; address < 127; address++) {
        Wire.beginTransmission(address);
        error = Wire.endTransmission();
        if (error == 0) {
            Serial.print("Found I2C device at 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
            nDevices++;
        } else if (error == 4) {
            Serial.print("Unknown I2C error at 0x");
            if (address < 16) Serial.print("0");
            Serial.println(address, HEX);
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found.");
    } else {
        Serial.printf("Scan complete. Found %d active devices.\n", nDevices);
    }
    Serial.println("--------------------\n");
}

#if defined(COMPASS_STANDALONE)
void setup() {
    Serial.begin(115200);
    delay(1000); 

    Wire.begin(I2C_SDA, I2C_SCL, 100000);
    scanI2CBus();

    if (!compass_sensor.init()) {
        Serial.println("LSM303 init failed! Accelerometer not detected.");
    } else {
        Serial.println("LSM303 Accelerometer detected successfully!");
        compass_sensor.enableDefault();
    }

    initMagnetometer();
    loadCalibration();

    if (!LittleFS.begin(true)) {
        Serial.println("LittleFS mount failed");
        return;
    }

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    ArduinoOTA.setHostname("CompassTest");
    ArduinoOTA.begin();

    server.on("/", []() {
        File file = LittleFS.open("/index.html", "r");
        if (file) {
            server.streamFile(file, "text/html");
            file.close();
        } else {
            server.send(404, "text/plain", "index.html not found");
        }
    });

    server.serveStatic("/", LittleFS, "/");
    server.begin();
    
    ws.onEvent(onWsEvent);
    ws.begin();

    Serial.println("Web server started");
}

void loop() {
    ArduinoOTA.handle();
    updateCompass();
    server.handleClient();
    ws.loop();

    if (millis() - lastBroadcast > 100) {
        lastBroadcast = millis();
        broadcastData();
    }
}
#endif


// ============================================================================
// ROBOSUB COMPATIBILITY LAYER
// ============================================================================

#include "main.h"
#include "io_sub.h"
#include "esc.h"
#include "datastorage.h"

#define NUM_DIRECTIONS 30

extern RoboStruct mainData;
extern Message escOut;
extern SemaphoreHandle_t mainDataMutex;

// Defined the global queue handles used by RoboSub
QueueHandle_t compass = NULL;
QueueHandle_t compassIn = NULL;
bool icm_ready = false;

void updateUIHexFloat(void);

float last_raw_x = 0, last_raw_y = 0, last_raw_z = 0;
float last_raw_ax = 0, last_raw_ay = 0, last_raw_az = 0;
uint32_t global_loop_cnt = 0;
String global_cal_msg = "LSM Active";
String global_cal_load = "";
String global_cal_ver = "LSM303 Active";

bool InitCompass(void) {
    Serial.println("\r\nInitializing LSM303/LIS2MDL Compass...");
    
    // Check if LSM303 is connected
    bool ok = compass_sensor.init();
    if (!ok) {
        Serial.println("LSM303: Init failed! Accelerometer/Compass not detected.");
    } else {
        Serial.println("LSM303: Accelerometer/Compass detected successfully!");
        compass_sensor.enableDefault();
    }

    initMagnetometer();
    loadCalibration();
    updateUIHexFloat();

    // Load remaining persistent parameters
    CompassOffsetCorrection(&mainData.compassOffset, true);
    CompasOffset(&mainData, true);
    MechanicalCorrection(&mainData.mechanicCorrection, true);

    icm_ready = ok;
    return ok;
}

void updateUIHexFloat() {
    char hex[150];
    snprintf(hex, sizeof(hex), "HI: [%.1f,%.1f,%.1f] SI: [%.2f,%.2f,%.2f] LVL: [%.2f,%.2f,%.2f]", 
             offsetX, offsetY, offsetZ, scaleX, scaleY, scaleZ, levelX, levelY, levelZ);
    global_cal_load = String(hex);
}

float CompassAverage(float in) {
    static float directions_x[200] = {0};
    static float directions_y[200] = {0};
    static int cbufpointer = 0;

    if (isnan(in)) return 0.0f;

    int limit = compass_avg_samples;
    if (limit < 1) limit = 1;
    if (limit > 200) limit = 200;

    float new_x = cos(in * M_PI / 180.0);
    float new_y = sin(in * M_PI / 180.0);

    directions_x[cbufpointer] = new_x;
    directions_y[cbufpointer] = new_y;

    cbufpointer = (cbufpointer + 1) % limit;

    float sum_x = 0;
    float sum_y = 0;
    for (int i = 0; i < limit; i++) {
        sum_x += directions_x[i];
        sum_y += directions_y[i];
    }

    float res = atan2(sum_y, sum_x) * 180.0 / M_PI;
    if (res < 0) res += 360.0;
    return res;
}

void initcompassQueue(void) {
    compass = xQueueCreate(1, sizeof(float));
    compassIn = xQueueCreate(10, sizeof(int));
}

void CompassTask(void *arg) {
    // Safe initial delay of 3 seconds to let the WiFi stack and LwIP initialize fully on Core 0
    vTaskDelay(pdMS_TO_TICKS(3000));

    // Start WebSockets server safely on Core 1
    ws.onEvent(onWsEvent);
    ws.begin();

    while (1) {
        global_loop_cnt++;
        int cmd = 0;

        if (compassIn && xQueueReceive(compassIn, &cmd, 0) == pdTRUE) {
            if (cmd == 34) {
                saveCalibration();
                updateUIHexFloat();
            }
        }

        // Always run WebSockets task loop to allow connection even if sensor is initializing/offline
        ws.loop();

        // Broadcast real-time telemetry over WebSockets every 100ms
        static uint32_t lastBroadcast = 0;
        if (millis() - lastBroadcast > 100) {
            lastBroadcast = millis();
            broadcastData();
        }

        if (icm_ready) {
            updateCompass();

            last_raw_x = smoothed_mx;
            last_raw_y = smoothed_my;
            last_raw_z = smoothed_mz;
            last_raw_ax = smoothed_ax;
            last_raw_ay = smoothed_ay;
            last_raw_az = smoothed_az;

            float heading = headingHard;
            if (icm_mode == 3 || icm_mode == 4) {
                heading = headingTilt;
            } else if (icm_mode == 2) {
                heading = headingHardSoft;
            }

            heading = CompassAverage(heading);

            // Output to Queue & Globals directly with user offset
            if (mainDataMutex && xSemaphoreTake(mainDataMutex, portMAX_DELAY)) {
                heading += mainData.compassOffset;

                while (heading < 0) heading += 360.0f;
                while (heading >= 360.0f) heading -= 360.0f;

                global_hdg = heading;
                mainData.dirMag = heading;

                if (compass) xQueueOverwrite(compass, (void *)&heading);

                xSemaphoreGive(mainDataMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

float GetHeading(void) { return global_hdg; }
float GetHeadingRaw(void) { return global_hdg; }
int linMagCalib(int *corr) { return 0; }
bool CalibrateCompass(void) { return true; }
int get_cal_point_count() { return 3; }
bool global_is_calibrating = false;
