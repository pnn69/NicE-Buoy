#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoOTA.h>
#include <SPIFFS.h>
#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>
#include <ICM_20948.h>
#include <MadgwickAHRS.h>

// Pins
#define LED_PIN 2
#define I2C_SDA 21
#define I2C_SCL 22

ICM_20948_I2C icm;
Madgwick filter;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences preferences;

// Calibration parameters (Stored in Preferences NVM)
float hi_x = 0.0, hi_y = 0.0, hi_z = 0.0;
float si_x = 1.0, si_y = 1.0, si_z = 1.0;

// Gyroscope bias calibration parameters
float gyro_bias_x = 0.0;
float gyro_bias_y = 0.0;
float gyro_bias_z = 0.0;

// Shared sensor data structure
struct SensorData {
    float headingA;
    float headingB;
    float yaw;       // Madgwick heading
    float roll;      // Madgwick roll
    float pitch;     // Madgwick pitch
    float headingRaw; // Uncompensated 2D magnetometer heading
    float mx_raw, my_raw, mz_raw;
    float mx_cal, my_cal, mz_cal;
};

SensorData sharedData;
portMUX_TYPE sharedDataMutex = portMUX_INITIALIZER_UNLOCKED;
volatile bool dataReady = false;
TaskHandle_t SensorTaskHandle = NULL;

// Local variable tracking for serial logging
float headingA = 0.0;
float headingB = 0.0;
float yawVal = 0.0;
float rollVal = 0.0;
float pitchVal = 0.0;
float headingRawVal = 0.0;
float mx_raw = 0.0, my_raw = 0.0, mz_raw = 0.0;
float mx_cal = 0.0, my_cal = 0.0, mz_cal = 0.0;

unsigned long lastDisplayUpdate = 0;
bool ledState = false;
bool clientCalibrating = false;

// Helpers to load/save calibration
void loadCalibration() {
    preferences.begin("calibration", true);
    hi_x = preferences.getFloat("hi_x", 0.0);
    hi_y = preferences.getFloat("hi_y", 0.0);
    hi_z = preferences.getFloat("hi_z", 0.0);
    si_x = preferences.getFloat("si_x", 1.0);
    si_y = preferences.getFloat("si_y", 1.0);
    si_z = preferences.getFloat("si_z", 1.0);
    preferences.end();
}

void saveCalibration(float hx, float hy, float hz, float sx, float sy, float sz) {
    preferences.begin("calibration", false);
    preferences.putFloat("hi_x", hx);
    preferences.putFloat("hi_y", hy);
    preferences.putFloat("hi_z", hz);
    preferences.putFloat("si_x", sx);
    preferences.putFloat("si_y", sy);
    preferences.putFloat("si_z", sz);
    preferences.end();

    portENTER_CRITICAL(&sharedDataMutex);
    hi_x = hx; hi_y = hy; hi_z = hz;
    si_x = sx; si_y = sy; si_z = sz;
    portEXIT_CRITICAL(&sharedDataMutex);
}

// Helper to parse simple JSON manually
float getValue(String data, String key) {
    int keyIdx = data.indexOf("\"" + key + "\":");
    if (keyIdx < 0) return 0.0;
    int valIdx = keyIdx + key.length() + 3;
    int endIdx = data.indexOf(",", valIdx);
    if (endIdx < 0) endIdx = data.indexOf("}", valIdx);
    if (endIdx < 0) return 0.0;
    return data.substring(valIdx, endIdx).toFloat();
}

// WebSocket Event Handler
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
               void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.println("WebSocket client connected.");
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.println("WebSocket client disconnected.");
        clientCalibrating = false;
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            data[len] = 0;
            String message = (char*)data;
            Serial.printf("WS received: %s\n", message.c_str());

            if (message.indexOf("start_cal") >= 0) {
                clientCalibrating = true;
                Serial.println("Client started calibration stream.");
            } else if (message.indexOf("save_cal") >= 0) {
                clientCalibrating = false;
                float hx = getValue(message, "hi_x");
                float hy = getValue(message, "hi_y");
                float hz = getValue(message, "hi_z");
                float sx = getValue(message, "si_x");
                float sy = getValue(message, "si_y");
                float sz = getValue(message, "si_z");
                saveCalibration(hx, hy, hz, sx, sy, sz);
                Serial.println("New calibration values saved successfully.");
            }
        }
    }
}

// Core 0 High-Priority Sensor Task
void sensorTaskCode(void * pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz
    xLastWakeTime = xTaskGetTickCount();

    static float ax_f = 0.0f, ay_f = 0.0f, az_f = 0.0f;
    static float mx_f = 0.0f, my_f = 0.0f, mz_f = 0.0f;
    static bool firstRun = true;
    const float alpha = 0.15f; // EMA low-pass filter coefficient

    for(;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        if (icm.dataReady()) {
            icm.getAGMT(); // Read all sensors

            float ax_raw = icm.accX();
            float ay_raw = icm.accY();
            float az_raw = icm.accZ();

            float mx_raw_val = icm.magX();
            float my_raw_val = icm.magY();
            float mz_raw_val = icm.magZ();

            float gx_raw = icm.gyrX() - gyro_bias_x;
            float gy_raw = icm.gyrY() - gyro_bias_y;
            float gz_raw = icm.gyrZ() - gyro_bias_z;

            if (firstRun) {
                ax_f = ax_raw; ay_f = ay_raw; az_f = az_raw;
                mx_f = mx_raw_val; my_f = my_raw_val; mz_f = mz_raw_val;
                firstRun = false;
            } else {
                ax_f = alpha * ax_raw + (1.0f - alpha) * ax_f;
                ay_f = alpha * ay_raw + (1.0f - alpha) * ay_f;
                az_f = alpha * az_raw + (1.0f - alpha) * az_f;
                mx_f = alpha * mx_raw_val + (1.0f - alpha) * mx_f;
                my_f = alpha * my_raw_val + (1.0f - alpha) * my_f;
                mz_f = alpha * mz_raw_val + (1.0f - alpha) * mz_f;
            }

            float ax = ax_f;
            float ay = ay_f;
            float az = az_f;

            float mxr = mx_f;
            float myr = my_f;
            float mzr = mz_f;

            // Apply Hard Iron Offset
            portENTER_CRITICAL(&sharedDataMutex);
            float hx = hi_x;
            float hy = hi_y;
            float hz = hi_z;
            float sx = si_x;
            float sy = si_y;
            float sz = si_z;
            portEXIT_CRITICAL(&sharedDataMutex);

            float mx_hi = mxr - hx;
            float my_hi = myr - hy;
            float mz_hi = mzr - hz;

            // Apply Soft Iron Scaling
            float mxc = mx_hi * sx;
            float myc = my_hi * sy;
            float mzc = mz_hi * sz;

            // Align ICM magnetometer axes with the accelerometer/gyroscope coordinate frame (InvenSense Datasheet DS-000189):
            // X_aligned = raw_mag_y, Y_aligned = raw_mag_x, Z_aligned = raw_mag_z
            float mx_hi_aligned = my_hi;
            float my_hi_aligned = mx_hi;
            float mz_hi_aligned = mz_hi;

            float mx_cal_aligned = myc;
            float my_cal_aligned = mxc;
            float mz_cal_aligned = mzc;

            // Calculate Pitch and Roll (radians)
            float r = atan2(ay, az);
            float p = atan2(-ax, sqrt(ay * ay + az * az));

            float cosRoll = cos(r);
            float sinRoll = sin(r);
            float cosPitch = cos(p);
            float sinPitch = sin(p);

            // Apply Tilt Compensation and calculate headings using aligned axes
            float Xh_hi = mx_hi_aligned * cosPitch + my_hi_aligned * sinRoll * sinPitch + mz_hi_aligned * cosRoll * sinPitch;
            float Yh_hi = my_hi_aligned * cosRoll - mz_hi_aligned * sinRoll;
            float hA = atan2(Yh_hi, Xh_hi) * 180.0 / PI;
            if (hA < 0) hA += 360.0;

            float Xh_cal = mx_cal_aligned * cosPitch + my_cal_aligned * sinRoll * sinPitch + mz_cal_aligned * cosRoll * sinPitch;
            float Yh_cal = my_cal_aligned * cosRoll - mz_cal_aligned * sinRoll;
            float heading_cal = atan2(Yh_cal, Xh_cal) * 180.0 / PI;
            if (heading_cal < 0) heading_cal += 360.0;

            // Calculate uncompensated 2D heading (magenta needle) using aligned, calibrated magnetometer
            float hRaw = atan2(my_cal_aligned, mx_cal_aligned) * 180.0 / PI;
            if (hRaw < 0) hRaw += 360.0;

            // Update Madgwick filter using raw gyro (deg/s), accelerometer, and aligned calibrated magnetometer (inverting gz_raw to correct direction of rotation)
            filter.update(gx_raw, gy_raw, -gz_raw, ax, ay, az, mx_cal_aligned, my_cal_aligned, mz_cal_aligned);

            float mRoll = filter.getRoll();
            float mPitch = filter.getPitch();
            float mYaw = filter.getYaw();
            if (mYaw < 0) mYaw += 360.0;

            // Low-pass filter output angles to eliminate high-frequency MEMS noise and hand tremor
            static float roll_f = 0.0f;
            static float pitch_f = 0.0f;
            static float yaw_f = 0.0f;
            static bool firstAngleRun = true;

            if (firstAngleRun) {
                roll_f = mRoll;
                pitch_f = mPitch;
                yaw_f = mYaw;
                firstAngleRun = false;
            } else {
                const float angle_alpha = 0.12f; // Smooths sensor jitter completely while maintaining excellent responsiveness
                roll_f = angle_alpha * mRoll + (1.0f - angle_alpha) * roll_f;
                pitch_f = angle_alpha * mPitch + (1.0f - angle_alpha) * pitch_f;

                // Yaw unwrap-safe EMA filter
                float diff = mYaw - yaw_f;
                if (diff > 180.0f) diff -= 360.0f;
                else if (diff < -180.0f) diff += 360.0f;
                yaw_f += angle_alpha * diff;
                if (yaw_f < 0.0f) yaw_f += 360.0f;
                else if (yaw_f >= 360.0f) yaw_f -= 360.0f;
            }

            portENTER_CRITICAL(&sharedDataMutex);
            sharedData.headingA = hA;
            sharedData.headingB = hRaw; // Set the primary calibrated output to use raw 2D (no tilt) heading
            sharedData.roll = roll_f;
            sharedData.pitch = pitch_f;
            sharedData.yaw = yaw_f;
            sharedData.headingRaw = hRaw;
            sharedData.mx_raw = mxr;
            sharedData.my_raw = myr;
            sharedData.mz_raw = mzr;
            sharedData.mx_cal = mxc;
            sharedData.my_cal = myc;
            sharedData.mz_cal = mzc;
            dataReady = true;
            portEXIT_CRITICAL(&sharedDataMutex);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n--- ESPLipO ICM-20948 Starting Setup ---");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Wire.begin(I2C_SDA, I2C_SCL, 400000);

    // Initializing ICM-20948 with automatic address discovery
    bool sensorOk = false;
    Serial.println("Connecting to ICM-20948 at 0x69...");
    if (icm.begin(Wire, 0x69) == ICM_20948_Stat_Ok) {
        sensorOk = true;
        Serial.println("ICM-20948 initialized at 0x69.");
    } else {
        Serial.println("Failed at 0x69. Trying 0x68...");
        if (icm.begin(Wire, 0x68) == ICM_20948_Stat_Ok) {
            sensorOk = true;
            Serial.println("ICM-20948 initialized at 0x68.");
        }
    }

    if (!sensorOk) {
        Serial.println("Error: ICM-20948 sensor not found on the I2C bus!");
        while (1) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }

    loadCalibration();

    Serial.print("Connecting to Wi-Fi...");
    WiFi.begin("NicE_WiFi", "!Ni1001100110");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWi-Fi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP().toString());

    ArduinoOTA.setHostname("wemosicm");
    ArduinoOTA.onStart([]() {
        Serial.println("OTA Update Started...");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA Update Ended.");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]\n", error);
    });
    ArduinoOTA.begin();

    if (SPIFFS.begin(true)) {
        server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(SPIFFS, "/index.html", "text/html");
        });
        server.on("/callibration", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(SPIFFS, "/callibration.html", "text/html");
        });
        server.on("/calibration", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(SPIFFS, "/callibration.html", "text/html");
        });
        ws.onEvent(onWsEvent);
        server.addHandler(&ws);
        server.begin();
        Serial.println("Web Server & WebSocket started.");
    } else {
        Serial.println("Failed to mount SPIFFS.");
    }

    // Calibrate gyroscope bias (Zero-Rate calibration)
    Serial.println("Calibrating gyroscope bias... Please keep the device completely static!");
    float g_sum_x = 0, g_sum_y = 0, g_sum_z = 0;
    int samples = 200;
    int count = 0;
    // Blink LED fast while calibrating
    while (count < samples) {
        if (icm.dataReady()) {
            icm.getAGMT();
            g_sum_x += icm.gyrX();
            g_sum_y += icm.gyrY();
            g_sum_z += icm.gyrZ();
            count++;
            if (count % 10 == 0) {
                digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            }
        }
        delay(10);
    }
    gyro_bias_x = g_sum_x / samples;
    gyro_bias_y = g_sum_y / samples;
    gyro_bias_z = g_sum_z / samples;
    digitalWrite(LED_PIN, LOW); // Turn off after calibration
    Serial.printf("Gyroscope calibration complete. Offsets -> X: %.4f, Y: %.4f, Z: %.4f\n", gyro_bias_x, gyro_bias_y, gyro_bias_z);

    // Initialize Madgwick filter at 100Hz ODR
    filter.begin(100);

    xTaskCreatePinnedToCore(
        sensorTaskCode,
        "SensorTask",
        4096,
        NULL,
        10,
        &SensorTaskHandle,
        0
    );
    Serial.println("High-priority sensor fusion task started on Core 0.");
}

void loop() {
    ArduinoOTA.handle();
    ws.cleanupClients();

    if (dataReady) {
        SensorData localData;

        portENTER_CRITICAL(&sharedDataMutex);
        localData = sharedData;
        dataReady = false;
        portEXIT_CRITICAL(&sharedDataMutex);

        String json = "{";
        json += "\"headingA\":" + String(localData.headingA, 2) + ",";
        json += "\"headingB\":" + String(localData.headingB, 2) + ",";
        json += "\"yaw\":" + String(localData.yaw, 2) + ",";
        json += "\"roll\":" + String(localData.roll, 2) + ",";
        json += "\"pitch\":" + String(localData.pitch, 2) + ",";
        json += "\"headingRaw\":" + String(localData.headingRaw, 2) + ",";
        json += "\"mx_raw\":" + String(localData.mx_raw, 2) + ",";
        json += "\"my_raw\":" + String(localData.my_raw, 2) + ",";
        json += "\"mz_raw\":" + String(localData.mz_raw, 2) + ",";
        json += "\"mx_cal\":" + String(localData.mx_cal, 2) + ",";
        json += "\"my_cal\":" + String(localData.my_cal, 2) + ",";
        json += "\"mz_cal\":" + String(localData.mz_cal, 2);
        json += "}";

        for (auto const& client : ws.getClients()) {
            if (client.status() == WS_CONNECTED && client.queueLen() < 2) {
                const_cast<AsyncWebSocketClient&>(client).text(json);
            }
        }

        headingA = localData.headingA;
        headingB = localData.headingB;
        yawVal = localData.yaw;
        rollVal = localData.roll;
        pitchVal = localData.pitch;
        headingRawVal = localData.headingRaw;
        mx_raw = localData.mx_raw;
        my_raw = localData.my_raw;
        mz_raw = localData.mz_raw;
        mx_cal = localData.mx_cal;
        my_cal = localData.my_cal;
        mz_cal = localData.mz_cal;
    }

    unsigned long now = millis();
    unsigned long displayRefreshInterval = clientCalibrating ? 2000 : 300;
    if (now - lastDisplayUpdate >= displayRefreshInterval) {
        lastDisplayUpdate = now;

        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);

        Serial.printf("RAW:[%.1f,%.1f,%.1f] CAL:[%.1f,%.1f,%.1f] HeadA:%.1f HeadB:%.1f Yaw:%.1f Pitch:%.1f Roll:%.1f\n",
                      mx_raw, my_raw, mz_raw, mx_cal, my_cal, mz_cal, headingA, headingB, yawVal, pitchVal, rollVal);
    }
}