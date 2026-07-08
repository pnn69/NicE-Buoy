#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <Wire.h>
#include <LSM303.h>

#define SAMPLE_TARGET 500
#define I2C_SDA      4
#define I2C_SCL      5

const char* ssid = "NicE_WiFi";
const char* password = "!Ni1001100110";

WebServer server(80);
WebSocketsServer ws(81);
Preferences prefs;
LSM303 compass;

bool calibrating = false;
uint16_t sampleCount = 0;

float declination = 2.6f;

float offsetX = 0;
float offsetY = 0;
float offsetZ = 0;

float scaleX = 1;
float scaleY = 1;
float scaleZ = 1;

float rawHeading = 0;
float calHeading = 0;
float tiltHeading = 0;
float pitchDeg = 0;
float rollDeg = 0;

float xmin = 999999;
float xmax = -999999;
float ymin = 999999;
float ymax = -999999;
float zmin = 999999;
float zmax = -999999;

unsigned long lastBroadcast = 0;

void saveCalibration()
{
    prefs.begin("compass", false);

    prefs.putFloat("ox", offsetX);
    prefs.putFloat("oy", offsetY);
    prefs.putFloat("oz", offsetZ);

    prefs.putFloat("sx", scaleX);
    prefs.putFloat("sy", scaleY);
    prefs.putFloat("sz", scaleZ);

    prefs.end();
}

void loadCalibration()
{
    prefs.begin("compass", true);

    offsetX = prefs.getFloat("ox", 0);
    offsetY = prefs.getFloat("oy", 0);
    offsetZ = prefs.getFloat("oz", 0);

    scaleX = prefs.getFloat("sx", 1);
    scaleY = prefs.getFloat("sy", 1);
    scaleZ = prefs.getFloat("sz", 1);

    prefs.end();
}

void resetCalibration()
{
    offsetX = offsetY = offsetZ = 0;

    scaleX = scaleY = scaleZ = 1;

    saveCalibration();
}

void startCalibration()
{
    sampleCount = 0;

    xmin = ymin = zmin = 999999;
    xmax = ymax = zmax = -999999;

    calibrating = true;
}

void finishCalibration()
{
    offsetX = (xmax + xmin) * 0.5f;
    offsetY = (ymax + ymin) * 0.5f;
    offsetZ = (zmax + zmin) * 0.5f;

    float rx = (xmax - xmin) * 0.5f;
    float ry = (ymax - ymin) * 0.5f;
    float rz = (zmax - zmin) * 0.5f;

    float avg = (rx + ry + rz) / 3.0f;

    scaleX = avg / rx;
    scaleY = avg / ry;
    scaleZ = avg / rz;

    saveCalibration();

    calibrating = false;
}

void updateCompass()
{
    compass.read();

    float mx = compass.m.x;
    float my = compass.m.y;
    float mz = compass.m.z;

    float ax = compass.a.x;
    float ay = compass.a.y;
    float az = compass.a.z;

    if (calibrating)
    {
        xmin = min(xmin, mx);
        xmax = max(xmax, mx);

        ymin = min(ymin, my);
        ymax = max(ymax, my);

        zmin = min(zmin, mz);
        zmax = max(zmax, mz);

        sampleCount++;

        if (sampleCount >= SAMPLE_TARGET)
        {
            finishCalibration();
        }
    }

    rawHeading = atan2(my, mx) * 180.0f / PI;

    if (rawHeading < 0)
        rawHeading += 360.0f;

    float cmx = (mx - offsetX) * scaleX;
    float cmy = (my - offsetY) * scaleY;
    float cmz = (mz - offsetZ) * scaleZ;

    calHeading = atan2(cmy, cmx) * 180.0f / PI + declination;

    if (calHeading < 0)
        calHeading += 360;

    if (calHeading > 360)
        calHeading -= 360;

    float roll = atan2(ay, az);

    float pitch =
        atan2(
            -ax,
            sqrt(ay * ay + az * az)
        );

    rollDeg = roll * 180.0 / PI;
    pitchDeg = pitch * 180.0 / PI;

    float Xh =
        cmx * cos(pitch) +
        cmz * sin(pitch);

    float Yh =
        cmx * sin(roll) * sin(pitch)
        + cmy * cos(roll)
        - cmz * sin(roll) * cos(pitch);

    tiltHeading =
        atan2(Yh, Xh)
        * 180.0 / PI;

    tiltHeading += declination;

    if (tiltHeading < 0)
        tiltHeading += 360;

    if (tiltHeading > 360)
        tiltHeading -= 360;
}

void broadcastData()
{
    String json = "{";

    json += "\"raw\":" + String(rawHeading, 1) + ",";
    json += "\"cal\":" + String(calHeading, 1) + ",";
    json += "\"tilt\":" + String(tiltHeading, 1) + ",";
    json += "\"pitch\":" + String(pitchDeg, 1) + ",";
    json += "\"roll\":" + String(rollDeg, 1) + ",";
    json += "\"count\":" + String(sampleCount) + ",";
    json += "\"running\":" + String(calibrating ? "true" : "false");

    json += "}";

    ws.broadcastTXT(json);
}

void initWiFi()
{
    Serial.println("Connecting WiFi");

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
}



// void setup()
// {
//     Serial.begin(115200);

//     Wire.begin(I2C_SDA, I2C_SCL, 400000);

//     if (!compass.init())
//     {
//         Serial.println("LSM303 init failed");

//         while (true)
//         {
//             delay(1000);
//         }
//     }

//     compass.enableDefault();

//     loadCalibration();

//     if (!LittleFS.begin(true))
//     {
//         Serial.println("LittleFS mount failed");
//         return;
//     }

//     WiFi.begin(ssid, password);

//     while (WiFi.status() != WL_CONNECTED)
//     {
//         delay(500);
//         Serial.print(".");
//     }

//     Serial.println();
//     Serial.print("IP: ");
//     Serial.println(WiFi.localIP());


//     Serial.println();
//     Serial.print("IP: ");
//     Serial.println(WiFi.localIP());

//     server.serveStatic("/", LittleFS, "/");

//     server.on("/start", [] {
//       startCalibration();
//       server.send(200, "text/plain", "OK");
//     });

//     server.on("/reset", [] {
//       resetCalibration();
//       server.send(200, "text/plain", "OK");
//     });

//     server.begin();
//     ws.begin();

//     Serial.println("Web server started");
// }

void setup()
{
    Serial.begin(115200);

    delay(3000);

    Serial.println();
    Serial.println("================================");
    Serial.println("BOOT");
    Serial.println("================================");

    while(true)
    {
        Serial.println("ALIVE");
        delay(1000);
    }
}


void loop()
{
    updateCompass();

    server.handleClient();
    ws.loop();

    if (millis() - lastBroadcast > 100)
    {
        lastBroadcast = millis();
        broadcastData();
    }
}