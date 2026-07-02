#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoOTA.h>
#include <SPIFFS.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <Preferences.h>

// Pins
#define LED_PIN 18
#define VEXT_PIN 3
#define BL_PIN 21
#define TFT_CS 38
#define TFT_RST 39
#define TFT_DC 40
#define TFT_SCLK 41
#define TFT_MOSI 42
#define I2C_SDA 4
#define I2C_SCL 5

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_ICM20948 icm;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Preferences preferences;

// Calibration parameters (Stored in Preferences NVM)
float hi_x = 0.0, hi_y = 0.0, hi_z = 0.0;
float si_x = 1.0, si_y = 1.0, si_z = 1.0;

// Shared sensor data structure
struct SensorData {
    float headingA;
    float headingB;
    float roll;
    float pitch;
    float mx_raw, my_raw, mz_raw;
    float mx_cal, my_cal, mz_cal;
};

SensorData sharedData;
portMUX_TYPE sharedDataMutex = portMUX_INITIALIZER_UNLOCKED;
volatile bool dataReady = false;
TaskHandle_t SensorTaskHandle = NULL;

// Local screen drawing variables
float headingA = 0.0;
float headingB = 0.0;
float roll = 0.0;
float pitch = 0.0;
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
                Serial.println("Client started calibration stream. Throttling TFT screen to 2s.");
            } else if (message.indexOf("save_cal") >= 0) {
                clientCalibrating = false;
                float hx = getValue(message, "hi_x");
                float hy = getValue(message, "hi_y");
                float hz = getValue(message, "hi_z");
                float sx = getValue(message, "si_x");
                float sy = getValue(message, "si_y");
                float sz = getValue(message, "si_z");
                saveCalibration(hx, hy, hz, sx, sy, sz);
            }
        }
    }
}

// Screen Base Layout
void drawScreenBase() {
    tft.fillScreen(ST77XX_BLACK);
    tft.fillRect(0, 0, 160, 14, ST77XX_BLUE);
    tft.setCursor(8, 3);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    tft.print("WEMOS ICM COMPASS");
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

        sensors_event_t accel_event;
        sensors_event_t mag_event;
        sensors_event_t temp_event;
        sensors_event_t gyro_event;
        icm.getEvent(&accel_event, &gyro_event, &temp_event, &mag_event);

        float ax_raw = accel_event.acceleration.x;
        float ay_raw = accel_event.acceleration.y;
        float az_raw = accel_event.acceleration.z;

        float mx_raw_val = mag_event.magnetic.x;
        float my_raw_val = mag_event.magnetic.y;
        float mz_raw_val = mag_event.magnetic.z;

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
        float mx_hi = mxr - hi_x;
        float my_hi = myr - hi_y;
        float mz_hi = mzr - hi_z;

        // Apply Soft Iron Scaling
        float mxc = mx_hi * si_x;
        float myc = my_hi * si_y;
        float mzc = mz_hi * si_z;

        // Align ICM magnetometer axes with the accelerometer coordinate frame before tilt compensation:
        float mx_hi_aligned = -my_hi;
        float my_hi_aligned = mx_hi;
        float mz_hi_aligned = mz_hi;

        float mx_cal_aligned = -myc;
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

        portENTER_CRITICAL(&sharedDataMutex);
        sharedData.headingA = hA;
        sharedData.headingB = heading_cal;
        sharedData.roll = r * 180.0 / PI;
        sharedData.pitch = p * 180.0 / PI;
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

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, HIGH);
    delay(100);
    pinMode(BL_PIN, OUTPUT);
    digitalWrite(BL_PIN, HIGH);

    SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
    tft.initR(INITR_MINI160x80);
    tft.setRotation(1);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);

    tft.setCursor(5, 5);
    tft.println("WemosICM Compass");
    tft.drawFastHLine(0, 18, 160, ST77XX_BLUE);

    Wire.begin(I2C_SDA, I2C_SCL, 400000);
    
    tft.setCursor(5, 25);
    if (!icm.begin_I2C(0x69, &Wire) && !icm.begin_I2C(0x68, &Wire)) {
        tft.setTextColor(ST77XX_RED);
        tft.println("ICM20948 Error!");
        Serial.println("ICM20948 Init Failed!");
        while(1) delay(10);
    }
    icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
    tft.println("Sensor: OK");

    loadCalibration();

    tft.setCursor(5, 45);
    tft.print("Wi-Fi Connecting...");
    WiFi.begin("NicE_WiFi", "!Ni1001100110");
    while (WiFi.status() != WL_CONNECTED) { delay(500); tft.print("."); }
    
    drawScreenBase();

    ArduinoOTA.setHostname("wemosicm");
    ArduinoOTA.onStart([]() {
        tft.fillScreen(ST77XX_BLACK);
        tft.fillRect(0, 0, 160, 14, ST77XX_RED);
        tft.setCursor(24, 3);
        tft.setTextColor(ST77XX_WHITE);
        tft.print("SYSTEM UPDATE");
        tft.drawRect(10, 35, 140, 15, ST77XX_WHITE);
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        int percent = progress / (total / 100);
        tft.fillRect(12, 37, (136 * percent) / 100, 11, ST77XX_GREEN);
    });
    ArduinoOTA.begin();

    if (SPIFFS.begin(true)) {
        server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(SPIFFS, "/index.html", "text/html");
        });
        ws.onEvent(onWsEvent);
        server.addHandler(&ws);
        server.begin();
    }

    xTaskCreatePinnedToCore(
        sensorTaskCode,
        "SensorTask",
        4096,
        NULL,
        10,
        &SensorTaskHandle,
        0
    );
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
        json += "\"roll\":" + String(localData.roll, 2) + ",";
        json += "\"pitch\":" + String(localData.pitch, 2) + ",";
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
        roll = localData.roll;
        pitch = localData.pitch;
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

        tft.setTextSize(1);

        tft.setCursor(5, 20);
        if (WiFi.status() == WL_CONNECTED) {
            tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
            tft.print("Web: http://");
            tft.print(WiFi.localIP().toString());
            tft.print("    ");
        } else {
            tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
            tft.print("Web Server: OFFLINE    ");
        }

        tft.setCursor(5, 32);
        tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
        tft.print("Heading A: ");
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.print(headingA, 1);
        tft.write(0xF7);
        tft.print("   ");

        tft.setCursor(5, 44);
        tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
        tft.print("Heading B: ");
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.print(headingB, 1);
        tft.write(0xF7);
        tft.print("   ");

        tft.setCursor(5, 56);
        tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
        tft.print("Pitch:     ");
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.print(pitch, 1);
        tft.write(0xF7);
        tft.print("   ");

        tft.setCursor(5, 68);
        tft.setTextColor(ST77XX_CYAN, ST77XX_BLACK);
        tft.print("Roll:      ");
        tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        tft.print(roll, 1);
        tft.write(0xF7);
        tft.print("   ");

        ledState = !ledState;
        if (ledState) {
            tft.fillRect(146, 3, 8, 8, ST77XX_GREEN);
        } else {
            tft.fillRect(146, 3, 8, 8, ST77XX_BLUE);
        }

        Serial.printf("RAW:[%.1f,%.1f,%.1f] CAL:[%.1f,%.1f,%.1f] HeadA:%.1f HeadB:%.1f Pitch:%.1f Roll:%.1f\n",
                      mx_raw, my_raw, mz_raw, mx_cal, my_cal, mz_cal, headingA, headingB, pitch, roll);
    }
}
