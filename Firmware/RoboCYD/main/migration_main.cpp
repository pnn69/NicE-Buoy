#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <TFT_eSPI.h>

#include "migration_wifi_secrets.h"

static TFT_eSPI tft;

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
// Display
// -----------------------------------------------------------------------------

static void init_display()
{
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    const int w = tft.width();
    const int h = tft.height();

    // Screen boundary
    tft.drawRect(0, 0, w, h, TFT_WHITE);

    tft.setTextDatum(MC_DATUM);

    // Title
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.setTextSize(3);
    tft.drawString(
        "RoboCYD",
        w / 2,
        45);

    // Framework information
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);

    tft.drawString(
        "ESP-IDF 5.5.5",
        w / 2,
        88);

    tft.drawString(
        "TFT_eSPI OK",
        w / 2,
        116);

    // Network / OTA state
    if (WiFi.status() == WL_CONNECTED)
    {
        tft.setTextColor(TFT_CYAN, TFT_BLACK);

        tft.drawString(
            WiFi.localIP().toString(),
            w / 2,
            148);

        tft.setTextColor(TFT_GREEN, TFT_BLACK);

        tft.drawString(
            "OTA TEST 2",
            w / 2,
            178);
    }
    else
    {
        tft.setTextColor(TFT_RED, TFT_BLACK);

        tft.drawString(
            "NO WIFI - NO OTA",
            w / 2,
            178);
    }

    // Colour test bars
    const int barY = 210;
    const int barH = h - barY;
    const int barW = w / 4;

    tft.fillRect(
        0,
        barY,
        barW,
        barH,
        TFT_RED);

    tft.fillRect(
        barW,
        barY,
        barW,
        barH,
        TFT_GREEN);

    tft.fillRect(
        barW * 2,
        barY,
        barW,
        barH,
        TFT_BLUE);

    tft.fillRect(
        barW * 3,
        barY,
        w - (barW * 3),
        barH,
        TFT_YELLOW);

    Serial.printf(
        "TFT initialized: %d x %d\n",
        w,
        h);
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

    // Start OTA after Wi-Fi and display are available.
    start_ota();

    unsigned long lastReport = 0;

    for (;;)
    {
        // CRITICAL:
        // RoboCYD is OTA-only, so this must continue to run frequently.
        service_wifi_and_ota();

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