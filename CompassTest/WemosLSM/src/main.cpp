#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>
#include <SPIFFS.h>
#include <Preferences.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_Sensor.h>

// -----------------------------------------------------------------------------
// Hardware Pins & Config (from LSM_arduino.md)
// -----------------------------------------------------------------------------
#define VEXT_PIN     3   // Power rail enable (HIGH = power on TFT, GNSS, etc.)
#define BL_PIN       21  // Display Backlight (HIGH = backlight ON)
#define TFT_CS       38  // Chip Select
#define TFT_RST      39  // Reset
#define TFT_DC       40  // Data/Command (RS)
#define TFT_SCLK     41  // SPI Clock
#define TFT_MOSI     42  // SPI MOSI

#define I2C_SDA      4
#define I2C_SCL      5

#define LED_PIN      18 

// -----------------------------------------------------------------------------
// Constants and Variables
// -----------------------------------------------------------------------------
const char* ssid = "NicE_WiFi";
const char* password = "!Ni1001100110";

// TFT Display Instantiation (Hardware SPI for maximum speed)
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// Sensors Instantiation
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);

// Server & WebSockets
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Calibration Parameters (Stored in Preferences NVM)
Preferences preferences;
float hi_x = 0.0, hi_y = 0.0, hi_z = 0.0;
float si_x = 1.0, si_y = 1.0, si_z = 1.0;

// Global Sensor Values (Maintained for Display)
float headingA = 0.0;
float headingB = 0.0;
float roll = 0.0;
float pitch = 0.0;
float mx_raw = 0.0, my_raw = 0.0, mz_raw = 0.0;
float mx_cal = 0.0, my_cal = 0.0, mz_cal = 0.0;

// Dual-Core Multi-Threading Struct & Sync variables
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

// Timers and States
unsigned long lastDisplayUpdate = 0;
bool ledState = false;
bool clientCalibrating = false; // Throttles display when true to prioritize Core 1 network bandwidth

// -----------------------------------------------------------------------------
// Calibration Storage Functions
// -----------------------------------------------------------------------------
void loadCalibration() {
    preferences.begin("calibration", true); // read-only
    hi_x = preferences.getFloat("hi_x", 0.0);
    hi_y = preferences.getFloat("hi_y", 0.0);
    hi_z = preferences.getFloat("hi_z", 0.0);
    si_x = preferences.getFloat("si_x", 1.0);
    si_y = preferences.getFloat("si_y", 1.0);
    si_z = preferences.getFloat("si_z", 1.0);
    preferences.end();
    
    Serial.printf("Calibration Loaded: HI[%f, %f, %f] SI[%f, %f, %f]\n", 
                  hi_x, hi_y, hi_z, si_x, si_y, si_z);
}

void saveCalibration(float hx, float hy, float hz, float sx, float sy, float sz) {
    preferences.begin("calibration", false); // read-write
    preferences.putFloat("hi_x", hx);
    preferences.putFloat("hi_y", hy);
    preferences.putFloat("hi_z", hz);
    preferences.putFloat("si_x", sx);
    preferences.putFloat("si_y", sy);
    preferences.putFloat("si_z", sz);
    preferences.end();

    // Critical Section: update active calibration variables safely
    portENTER_CRITICAL(&sharedDataMutex);
    hi_x = hx; hi_y = hy; hi_z = hz;
    si_x = sx; si_y = sy; si_z = sz;
    portEXIT_CRITICAL(&sharedDataMutex);

    Serial.printf("New Calibration Stored & Applied: HI[%f, %f, %f] SI[%f, %f, %f]\n", 
                  hi_x, hi_y, hi_z, si_x, si_y, si_z);
}

// -----------------------------------------------------------------------------
// Helper to Parse Simple JSON WebSocket Payload manually
// -----------------------------------------------------------------------------
float getValue(String data, String key) {
    int keyIdx = data.indexOf("\"" + key + "\":");
    if (keyIdx < 0) return 0.0;
    int valIdx = keyIdx + key.length() + 3;
    int endIdx = data.indexOf(",", valIdx);
    if (endIdx < 0) endIdx = data.indexOf("}", valIdx);
    if (endIdx < 0) return 0.0;
    return data.substring(valIdx, endIdx).toFloat();
}

// -----------------------------------------------------------------------------
// WebSockets Callback Handler
// -----------------------------------------------------------------------------
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
               void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.println("WebSocket client connected.");
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.println("WebSocket client disconnected.");
        clientCalibrating = false; // Restore normal fast display refresh on disconnect
    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            data[len] = 0;
            String message = (char*)data;
            Serial.printf("WS received: %s\n", message.c_str());

            if (message.indexOf("start_cal") >= 0) {
                clientCalibrating = true; // Throttles display updates
                Serial.println("Client started calibration stream. Throttling TFT screen to 2s.");
            } else if (message.indexOf("save_cal") >= 0) {
                clientCalibrating = false; // Restore normal fast display refresh
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

// -----------------------------------------------------------------------------
// Screen Layout Maintenance (Flicker-Free Base)
// -----------------------------------------------------------------------------
void drawScreenBase() {
    tft.fillScreen(ST7735_BLACK);
    tft.fillRect(0, 0, 160, 14, ST7735_BLUE);
    tft.setCursor(8, 3);
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(1);
    tft.print("WEMOS LSM COMPASS");
}

// -----------------------------------------------------------------------------
// Initialization Functions
// -----------------------------------------------------------------------------
void initDisplay() {
    pinMode(VEXT_PIN, OUTPUT);
    digitalWrite(VEXT_PIN, HIGH); // Enable Vext power rail
    delay(100);

    pinMode(BL_PIN, OUTPUT);
    digitalWrite(BL_PIN, HIGH); // Backlight On

    // Start Hardware SPI on the specified pins (extremely fast 40MHz bus)
    SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);

    tft.initR(INITR_MINI160x80); // Init ST7735R 160x80 (Mini)
    tft.setRotation(1); // Landscape mode
    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(1);
    
    tft.setCursor(5, 5);
    tft.println("WemosLSM Compass");
    tft.drawFastHLine(0, 18, 160, ST7735_BLUE);
}

// Helper to write directly to the LSM303 Accel I2C control registers
void setLSM303ODR(uint8_t odr_code) {
    Wire.beginTransmission(0x19); // Default I2C address for LSM303 Accel
    Wire.write(0x20);             // CTRL_REG1_A register (Control Register 1)
    Wire.write((odr_code << 4) | 0x07); // Enable Xen, Yen, Zen axes (0x07) and apply ODR
    Wire.endTransmission();
}

void initSensors() {
    Wire.begin(I2C_SDA, I2C_SCL, 400000); // Start I2C at 400kHz (Fast Mode) on custom pins
    delay(50);

    tft.setCursor(5, 25);
    if (!accel.begin()) {
        tft.setTextColor(ST7735_RED);
        tft.println("LSM303 Accel Error!");
        Serial.println("LSM303 Accel initialization failed!");
        while(1);
    }
    
    // Set Accelerometer to high-speed 200Hz ODR (odr_code = 0x06) to eliminate tilt latency
    setLSM303ODR(0x06);
    tft.println("Accel: OK");

    if (!mag.begin()) {
        tft.setTextColor(ST7735_RED);
        tft.println("LIS2MDL Mag Error!");
        Serial.println("LIS2MDL Mag initialization failed!");
        while(1);
    }
    
    // Set Magnetometer output data rate to its maximum 100Hz hardware speed (default is 10Hz)
    mag.setDataRate(LIS2MDL_RATE_100_HZ);
    tft.println("Mag: OK");
}

void initWiFi() {
    tft.setCursor(5, 55);
    tft.print("Wi-Fi Connecting...");
    WiFi.begin(ssid, password);

    int count = 0;
    while (WiFi.status() != WL_CONNECTED && count < 20) {
        delay(500);
        tft.print(".");
        count++;
    }

    tft.fillRect(0, 55, 160, 25, ST7735_BLACK);
    tft.setCursor(5, 55);
    if (WiFi.status() == WL_CONNECTED) {
        tft.setTextColor(ST7735_GREEN);
        tft.println("Wi-Fi Connected!");
        tft.setTextColor(ST7735_WHITE);
        tft.print("IP: ");
        tft.println(WiFi.localIP());
        Serial.printf("Connected to WiFi. IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        tft.setTextColor(ST7735_RED);
        tft.println("Wi-Fi Timeout!");
        Serial.println("WiFi connection failed.");
    }
    delay(2000);
    drawScreenBase(); // Establish the operational static visual frame once WiFi is complete
}

void initOTA() {
    ArduinoOTA.setHostname("wemoslsm");
    ArduinoOTA.onStart([]() {
        tft.fillScreen(ST7735_BLACK);
        tft.fillRect(0, 0, 160, 14, ST7735_RED);
        tft.setCursor(24, 3);
        tft.setTextColor(ST7735_WHITE);
        tft.print("SYSTEM UPDATE");
        tft.drawRect(10, 35, 140, 15, ST7735_WHITE);
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        int percent = progress / (total / 100);
        tft.fillRect(12, 37, (136 * percent) / 100, 11, ST7735_GREEN);
    });
    ArduinoOTA.onEnd([]() {
        tft.fillScreen(ST7735_GREEN);
        tft.setCursor(5, 35);
        tft.setTextColor(ST7735_BLACK);
        tft.println("OTA Complete!");
        tft.println("Rebooting...");
    });
    ArduinoOTA.onError([](ota_error_t error) {
        tft.fillScreen(ST7735_RED);
        tft.setCursor(5, 35);
        tft.println("OTA Error!");
    });
    ArduinoOTA.begin();
}

void initWebServer() {
    if(!SPIFFS.begin(true)){
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    // Server root
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/index.html", "text/html");
    });

    // Handle WebSocket endpoint
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);

    server.begin();
    Serial.println("HTTP and WebSocket server started.");
}

// -----------------------------------------------------------------------------
// Core 0 High-Priority Sensor Task Code (100Hz Jitter-Free)
// -----------------------------------------------------------------------------
void sensorTaskCode(void * pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz
    xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        // Precise FreeRTOS task delay blocks execution for exactly 10ms
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Perform physical I2C sensor reads
        sensors_event_t accel_event;
        sensors_event_t mag_event;
        accel.getEvent(&accel_event);
        mag.getEvent(&mag_event);

        float ax = accel_event.acceleration.x;
        float ay = accel_event.acceleration.y;
        float az = accel_event.acceleration.z;

        float mxr = mag_event.magnetic.x;
        float myr = mag_event.magnetic.y;
        float mzr = mag_event.magnetic.z;

        // Apply Hard Iron Offset
        float mx_hi = mxr - hi_x;
        float my_hi = myr - hi_y;
        float mz_hi = mzr - hi_z;

        // Apply Soft Iron Scaling
        float mxc = mx_hi * si_x;
        float myc = my_hi * si_y;
        float mzc = mz_hi * si_z;

        // Align LSM magnetometer axes with the accelerometer coordinate frame before tilt compensation:
        // X_aligned = -Y_lsm, Y_aligned = X_lsm, Z_aligned = Z_lsm (Matching WemosCompas exactly!)
        float mx_hi_aligned = -my_hi;
        float my_hi_aligned = mx_hi;
        float mz_hi_aligned = mz_hi;

        float mx_cal_aligned = -my_cal;
        float my_cal_aligned = mx_cal;
        float mz_cal_aligned = mz_cal;

        // Calculate Pitch and Roll (radians)
        float r = atan2(ay, az);
        float p = atan2(-ax, sqrt(ay * ay + az * az));

        float cosRoll = cos(r);
        float sinRoll = sin(r);
        float cosPitch = cos(p);
        float sinPitch = sin(p);

        // Apply Tilt Compensation and calculate headings using aligned axes
        // Heading A: Hard-Iron Only tilt-compensation
        float Xh_hi = mx_hi_aligned * cosPitch + my_hi_aligned * sinRoll * sinPitch + mz_hi_aligned * cosRoll * sinPitch;
        float Yh_hi = my_hi_aligned * cosRoll - mz_hi_aligned * sinRoll;
        float hA = atan2(Yh_hi, Xh_hi) * 180.0 / PI; // Corrected sign of Yh_hi to match standard rotation direction
        if (hA < 0) hA += 360.0;

        // Heading B: Hard + Soft-Iron tilt-compensation
        float Xh_cal = mx_cal_aligned * cosPitch + my_cal_aligned * sinRoll * sinPitch + mz_cal_aligned * cosRoll * sinPitch;
        float Yh_cal = my_cal_aligned * cosRoll - mz_cal_aligned * sinRoll;
        float hB = atan2(Yh_cal, Xh_cal) * 180.0 / PI; // Corrected sign of Yh_cal to match standard rotation direction
        if (hB < 0) hB += 360.0;

        // Thread-Safe Shared Memory Write (atomic / critical section)
        portENTER_CRITICAL(&sharedDataMutex);
        sharedData.headingA = hA;
        sharedData.headingB = hB;
        sharedData.roll = r * 180.0 / PI; // store in degrees directly
        sharedData.pitch = p * 180.0 / PI; // store in degrees directly
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

// -----------------------------------------------------------------------------
// Arduino Setup
// -----------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);

    initDisplay();
    initSensors();
    loadCalibration();
    initWiFi();
    initWebServer();
    initOTA();

    // Create high-priority sensor task pinned to Core 0
    xTaskCreatePinnedToCore(
        sensorTaskCode,       /* Task function. */
        "SensorTask",         /* name of task. */
        4096,                 /* Stack size of task */
        NULL,                 /* parameter of the task */
        10,                   /* priority of the task (very high!) */
        &SensorTaskHandle,    /* Task handle to keep track of created task */
        0                     /* pin task to Core 0 */
    );
    Serial.println("FreeRTOS 100Hz Sensor Thread pinned to Core 0.");
}

// -----------------------------------------------------------------------------
// Core 1 Execution (Main loop)
// -----------------------------------------------------------------------------
void loop() {
    // 1. Core Services (OTA & WS client cleanup)
    ArduinoOTA.handle();
    ws.cleanupClients();

    // 2. Thread-Safe Telemetry Check & WebSocket Stream (Runs at Core 0's 100Hz pace)
    if (dataReady) {
        SensorData localData;

        // Critical Mutex Lock: extract shared sensor vectors safely
        portENTER_CRITICAL(&sharedDataMutex);
        localData = sharedData;
        dataReady = false;
        portEXIT_CRITICAL(&sharedDataMutex);

        // Format WebSocket package
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

        // Broadcast to clients safely with active congestion control (frame-skipping)
        // Only send if the client's output queue has fewer than 2 frames waiting.
        // This prevents the ESPAsyncWebServer from forcefully disconnecting clients due to queue overflow.
        for (auto const& client : ws.getClients()) {
            if (client.status() == WS_CONNECTED && client.queueLen() < 2) {
                const_cast<AsyncWebSocketClient&>(client).text(json);
            }
        }

        // Keep local copies of parameters for local screen drawing without blocking
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

    // 3. Timed Flicker-Free TFT Display updates
    unsigned long now = millis();
    unsigned long displayRefreshInterval = clientCalibrating ? 2000 : 300;
    if (now - lastDisplayUpdate >= displayRefreshInterval) {
        lastDisplayUpdate = now;

        tft.setTextSize(1);

        // Connection state & IP
        tft.setCursor(5, 20);
        if (WiFi.status() == WL_CONNECTED) {
            tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
            tft.print("Web: http://");
            tft.print(WiFi.localIP().toString());
            tft.print("    "); // trailing spaces overwrite any old text
        } else {
            tft.setTextColor(ST7735_RED, ST7735_BLACK);
            tft.print("Web Server: OFFLINE    ");
        }

        // Heading A: Hard-Iron Only
        tft.setCursor(5, 32);
        tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
        tft.print("Heading A: ");
        tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
        tft.print(headingA, 1);
        tft.write(0xF7); // degree symbol in ST7735 font
        tft.print("   ");

        // Heading B: Hard + Soft-Iron
        tft.setCursor(5, 44);
        tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
        tft.print("Heading B: ");
        tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
        tft.print(headingB, 1);
        tft.write(0xF7);
        tft.print("   ");

        // Pitch
        tft.setCursor(5, 56);
        tft.setTextColor(ST7735_CYAN, ST7735_BLACK);
        tft.print("Pitch:     ");
        tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
        tft.print(pitch, 1);
        tft.write(0xF7);
        tft.print("   ");

        // Roll
        tft.setCursor(5, 68);
        tft.setTextColor(ST7735_CYAN, ST7735_BLACK);
        tft.print("Roll:      ");
        tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
        tft.print(roll, 1);
        tft.write(0xF7);
        tft.print("   ");

        // Top bar heartbeat square (fits perfectly at 146, 3 on a 160px wide display)
        ledState = !ledState;
        if (ledState) {
            tft.fillRect(146, 3, 8, 8, ST7735_GREEN);
        } else {
            tft.fillRect(146, 3, 8, 8, ST7735_BLUE);
        }

        // Print raw variables in Serial plotter
        Serial.printf("RAW:[%.1f,%.1f,%.1f] CAL:[%.1f,%.1f,%.1f] HeadA:%.1f HeadB:%.1f Pitch:%.1f Roll:%.1f\n",
                      mx_raw, my_raw, mz_raw, mx_cal, my_cal, mz_cal, headingA, headingB, pitch, roll);
    }
}
