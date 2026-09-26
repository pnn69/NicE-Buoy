#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <TFT_eSPI.h>

#include "migration_wifi_secrets.h"
#include "cyd_display.h"
#include "cyd_touch.h"

static bool otaStarted = false;
static bool wifiWasConnected = false;
static unsigned long lastWifiRetry = 0;
// Forward declaration.
// service_wifi_and_ota() calls start_ota() before its definition below.
static void start_ota();

// -----------------------------------------------------------------------------
// Wi-Fi
// -----------------------------------------------------------------------------

static void begin_wifi()
{
    Serial.printf("Connecting to %s\n", MIGRATION_WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);

    WiFi.begin(
        MIGRATION_WIFI_SSID,
        MIGRATION_WIFI_PASS
    );
}

static void service_wifi_and_ota()
{
    const bool connected =
        WiFi.status() == WL_CONNECTED;

    if (connected)
    {
        if (!wifiWasConnected)
        {
            Serial.println();
            Serial.print("WiFi connected, IP: ");
            Serial.println(WiFi.localIP());

            wifiWasConnected = true;
        }

        // If Wi-Fi returned after boot, this starts OTA.
        start_ota();

        if (otaStarted)
        {
            ArduinoOTA.handle();
        }

        return;
    }

    // We just lost the connection.
    if (wifiWasConnected)
    {
        Serial.println("WiFi connection lost");
        wifiWasConnected = false;
    }

    // Do not hammer the Wi-Fi stack.
    if (millis() - lastWifiRetry >= 10000)
    {
        lastWifiRetry = millis();

        Serial.println("Retrying WiFi...");
        WiFi.reconnect();
    }
}
// -----------------------------------------------------------------------------
// OTA
// -----------------------------------------------------------------------------
static void ota_error_callback(ota_error_t error)
{
    Serial.printf("\nOTA error [%u]\n", error);
}

static void start_ota()
{
    if (otaStarted)
    {
        return;
    }
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("OTA NOT started: WiFi unavailable");
        return;
    }

    ArduinoOTA.setHostname("robocyd");

    ArduinoOTA.onStart([]()
                       {
        Serial.println("OTA update started");

        tft.fillScreen(TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setTextSize(2);

        tft.drawString(
            "OTA UPDATE",
            tft.width() / 2,
            tft.height() / 2
        ); });

    ArduinoOTA.onEnd([]()
                     {
        Serial.println();
        Serial.println("OTA update complete"); });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          {
        if (total != 0)
        {
            const unsigned int percent =
                static_cast<unsigned int>(
                    (static_cast<uint64_t>(progress) * 100ULL) / total
                );

            Serial.printf("OTA: %u%%\r", percent);
        } });

    ArduinoOTA.onError(ota_error_callback);
    ArduinoOTA.begin();
    otaStarted = true;

    Serial.println("ArduinoOTA started");
    Serial.println("OTA hostname: robocyd");
}


// -----------------------------------------------------------------------------
// ESP-IDF entry point
// -----------------------------------------------------------------------------

extern "C" void app_main(void)
{
    // Start the Arduino compatibility layer.
    initArduino();

    Serial.begin(115200);
    delay(200);

    Serial.println();
    Serial.println("====================================");
    Serial.println(" RoboCYD ESP-IDF migration");
    Serial.println(" OTA self-update test #2");
    Serial.println("====================================");

    // OTA availability is our highest priority.
    begin_wifi();
    const unsigned long wifiWaitStart = millis();
    while (WiFi.status() != WL_CONNECTED &&
        millis() - wifiWaitStart < 10000)
    {
        delay(100);
    }

    // Initialise the display after Wi-Fi.
    init_display();
    init_touch();
    tft.fillScreen(TFT_BLACK);

    tft.setTextDatum(MC_DATUM);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(2);
    tft.drawString(
        "RoboCYD TOUCH",
        tft.width() / 2,
        40
    );

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.drawString(
        "ESP-IDF 5.5.5",
        tft.width() / 2,
        75
    );

    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString(
        "Touch anywhere",
        tft.width() / 2,
        105
    );

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString(
        "OTA READY",
        tft.width() / 2,
        135
    );




    // Start OTA after Wi-Fi and display are available.
    start_ota();

    unsigned long lastReport = 0;

    for (;;)
    {
        // CRITICAL:
        // RoboCYD is OTA-only, so this must continue to run frequently.
        service_wifi_and_ota();
        int touchX = 0;
        int touchY = 0;

        if (get_touch_point(touchX, touchY))
        {
            Serial.printf(
                "Touch at: X=%d, Y=%d\n",
                touchX,
                touchY
            );

            // Visual feedback using the real RoboCYD coordinate system.
            tft.fillCircle(
                touchX,
                touchY,
                5,
                TFT_YELLOW
            );
        }        

        if (millis() - lastReport >= 5000)
        {
            lastReport = millis();

            Serial.printf(
                "RoboCYD alive: %lu ms, WiFi=%d, OTA=%s, IP=%s\n",
                millis(),
                static_cast<int>(WiFi.status()),
                otaStarted ? "READY" : "DOWN",
                WiFi.localIP().toString().c_str()
            );
        }
        delay(2);
    }
}