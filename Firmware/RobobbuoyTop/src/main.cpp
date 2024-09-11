#include <Arduino.h>
#include "../../RobobuoyDependency\RobobuoyVersion.h"
#include "../../RobobuoyDependency\RobobuoyMsg.h"
#include "../../RobobuoyDependency\RobobuoyDefaults.h"
#include "../../RobobuoyDependency\RobobuoyCalc.h"
#include "io_top.h"
#include "robolora.h"
#include "leds.h"
#include "topwifi.h"
#include "gps.h"
#include "datastorage.h"
#include "buzzer.h"

static LedData mainCollorStatus;
static LedData mainCollorUtil;
static LedData mainCollorGps;
static UdpData mainUdpOutMsg;
static UdpData mainUdpIn;
static Buzz mainBuzzerData;
static GpsDataType gpsStatus;
static int wifiConfig = 0;
int8_t buoyId;
unsigned long udpTimer = millis();

int buttonState = 0;              // Current state of the button
int lastButtonState = 0;          // Previous state of the button
unsigned long lastPressTime = 0;  // Time of the last press
unsigned long debounceDelay = 50; // Debounce time in milliseconds
int pressCount = 0;               // Count the number of button presses
bool debounce = false;            // Debouncing flag

int countKeyPressesWithTimeout()
{
    // Get the current time
    unsigned long currentTime = millis();
    buttonState = digitalRead(BUTTON_PIN);
    // Check if the button is pressed and it's a new press (debounce)
    if (buttonState == HIGH && lastButtonState == LOW && !debounce)
    {
        pressCount++;                // Increment the button press count
        debounce = true;             // Set debounce flag
        delay(debounceDelay);        // Simple debouncing by adding a delay
        lastPressTime = currentTime; // Record the time of the last press
    }
    // Reset debounce flag if the button is released
    if (buttonState == LOW)
    {
        debounce = false;
    }
    // If more than 2 seconds have passed without a press, return the count
    if ((currentTime - lastPressTime) > 500 && pressCount > 0)
    {
        int finalPressCount = pressCount; // Store the current press count
        pressCount = 0;                   // Reset the press count after returning
        return finalPressCount;           // Return the number of key presses
    }
    lastButtonState = buttonState; // Save the last button state
    return -1;                     // Return -1 if 0.5 seconds haven't passed yet
}

void setup()
{
    Serial.begin(115200);
    pinMode(BUTTON_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    delay(100);
    printf("\r\nSetup running!\r\n");
    printf("Robobuoy Top Version: %0.1f\r\n", TOPVERSION);
    initMemory();
    // buoyId = 1;
    // memBuoyId(&buoyId, false);
    initbuzzerqueue();
    initledqueue();
    initwifiqueue();
    initgpsqueue();
    xTaskCreatePinnedToCore(buzzerTask, "buzzTask", 1000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(LoraTask, "LoraTask", 4024, NULL, 20, NULL, 1);
    xTaskCreatePinnedToCore(GpsTask, "GpsTask", 2000, NULL, configMAX_PRIORITIES - 8, NULL, 1);
    memBuoyId(&buoyId, true);
    Serial.print("Buoy ID: ");
    Serial.println(buoyId);
    if (digitalRead(BUTTON_PIN) == true)
    {
        delay(100);
        if (digitalRead(BUTTON_PIN) == true)
        {
            wifiConfig = 1;
        }
    }
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 4000, &wifiConfig, configMAX_PRIORITIES - 5, NULL, 0);
    // mainCollorStatus.color = CRGB ::OrangeRed;
    // mainCollorStatus.blink = BLINK_OFF;
    // xQueueSend(ledStatus, (void *)&mainCollorStatus, 10); // update util led
    Serial.println("Main task running!");
}

void loop(void)
{
    mainBuzzerData.hz = 1000;
    mainBuzzerData.repeat = 0;
    mainBuzzerData.pause = 0;
    mainBuzzerData.duration = 100;
    xQueueSend(buzzer, (void *)&mainBuzzerData, 10); // update util led

    sprintf(mainUdpOutMsg.msg, "Hello world");
    mainUdpOutMsg.port = 0;
    xQueueSend(udpOut, (void *)&mainUdpOutMsg, 10); // update WiFi
    while (true)
    {
        // Call the function to check for key presses with timeout
        int presses = countKeyPressesWithTimeout();
        // If the function returns a valid number of key presses, print it
        if (presses >= 0)
        {
            Serial.print("Number of key presses: ");
            Serial.println(presses);
        }

        if (udpTimer < millis())
        {
            udpTimer = millis() + 500;
            if (udpOut != NULL && uxQueueSpacesAvailable(udpOut) > 0)
            {
                sprintf(mainUdpOutMsg.msg, "ping");
                mainUdpOutMsg.port = 1001;
                xQueueSend(udpOut, (void *)&mainUdpOutMsg, 20); // update WiFi
            }
            else
            {
                printf("ping not in buffer!\r\n");
            }
        }
        if (xQueueReceive(gpsQue, (void *)&gpsStatus, 0) == pdTRUE) // New gps data
        {
        }
        if (xQueueReceive(udpIn, (void *)&mainUdpIn, 0) == pdTRUE)
        {
            Serial.println(mainUdpIn.msg);
        }
    }
    delay(1);
}
