#include <Arduino.h>
#include "main.h"
#include "io_base.h"
#include "lorabase.h"

// #define BUFLENMHRG 60 // one sampel each sec so 60 sec for stabilisation

static extBuoyParameters extBuoyData[2]; // max 2 other buoys
static RoboWindStruct wind;
static lorabuf toLora;
static lorabuf fromLora;
static char loraTransmitt[MAXSTRINGLENG];
static RoboStruct mainData;
static unsigned int status = IDLE;
static int msg;
unsigned long loraTimer = millis();

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

int handelKeyPress(int stat)
{
    int presses = countKeyPressesWithTimeout();
    if (presses > 0)
    {
        switch (presses)
        {
        case 1: // lock / unlock
            break;
        case 3:
            // compute start line
            break;
        case 10:
            // Get doc position
            break;
        case 15:
            // store currend location as doc location
            break;
        default:
            break;
        }
    }
    return stat;
}

void setup()
{
    Serial.begin(115200);
    initloraqueue();
    xTaskCreatePinnedToCore(LoraTask, "LoraTask", 4000, NULL, configMAX_PRIORITIES - 2, NULL, 1);
    Serial.println("Main task running!");
}

void loop(void)
{
    int presses = -1;
    while (true)
    {
        if (xQueueReceive(loraIn, (void *)&fromLora, 10) == pdTRUE) // New lora data
        {
            Serial.println(String(fromLora.data));
        }
        vTaskDelay(10);
    }
}
