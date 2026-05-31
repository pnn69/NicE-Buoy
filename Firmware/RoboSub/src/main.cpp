/**
 * @file main.cpp
 * @brief Primary Entry Point and Navigation Loop for NicE-Buoy Sub.
 * 
 * This module orchestrates the initialization and execution of all sub-systems:
 * 1. Hardware abstraction and I2C discovery.
 * 2. Multi-core task distribution (Core 0: Network, Core 1: Navigation/Sensors).
 * 3. Main control loop running the Rudder and Speed PID algorithms.
 * 4. Inter-task communication via FreeRTOS Queues and Semaphores.
 */

#include <Arduino.h>
#include <Wire.h>
#include <RoboTone.h>
#include <RoboCompute.h>
#include "main.h"
#include "freertos/task.h"
#include "io_sub.h"
#include "subwifi.h"
#include "datastorage.h"
#include "leds.h"
#include "esc.h"
#include "compass.h"
#include "buzzer.h"
#include "adc.h"
#include "sercom.h"
#include "pidrudspeed.h"
#include "soc/rtc_cntl_reg.h"

#define HOST_NAME "RoboBuoySub"
TaskHandle_t compassTaskHandle = NULL;

// Global navigation data
RoboStruct mainData;
SemaphoreHandle_t mainDataMutex = NULL;
Message escOut;
int subStatus = IDLE;
bool global_thruster_swap = false;

// Local timing
unsigned long nextSamp = millis();
unsigned long logtimer = millis();

/**
 * @brief System Setup.
 * Configures hardware, initializes NVS, and spawns background tasks.
 */
void setup()
{
    // Initialize mutex for thread-safe access to mainData
    mainDataMutex = xSemaphoreCreateMutex();
    
    Serial.begin(115200);
    delay(100);
    Serial.println("\r\n\r\nNicE-Buoy Sub Booting...\r\n");

    // Enable power and initialize I2C
    pinMode(PWRENABLE, OUTPUT);
    digitalWrite(PWRENABLE, 1);
    delay(500); 
    Wire.begin(21, 22);
    Wire.setClock(100000); // 100kHz for BNO055 stability

    // Initialize Queues before tasks begin
    initledqueue();
    initbuzzerqueue();
    initcompassQueue();
    initserqueue();
    initwifi();

    // Load persistent data from NVS
    initMemory();
    
    mainData.mac = espMac();
    Serial.printf("Robobuoy ID: %08x\r\n", mainData.mac);

    // Initialize Sensor Fusion
    InitCompass();

    // Initialize Navigation Controllers
    initRudPid(&mainData);
    initSpeedPid(&mainData);
    speedMaxMin(&mainData, GET);
    thrusterInversion(&mainData, GET);
    thrusterSwap(&global_thruster_swap, GET);
    initescqueue();

    // Check for WiFi reset button trigger
    int wifiConfig = 0;
    if (digitalRead(BUTTON_PIN) == LOW) {
        delay(100);
        if (digitalRead(BUTTON_PIN) == LOW) wifiConfig = 1;
    }

    // Spawn Background Tasks
    // CORE 0: Network and Telemetry
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 8192, &wifiConfig, 1, NULL, 0);
    
    // CORE 1: Real-time Control and Sensors
    xTaskCreatePinnedToCore(buzzerTask, "buzzTask", 2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(EscTask, "EscTask", 2400, NULL, configMAX_PRIORITIES - 5, NULL, 1);
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(CompassTask, "CompassTask", 8192, NULL, configMAX_PRIORITIES - 1, &compassTaskHandle, 1);
    xTaskCreatePinnedToCore(SercomTask, "SerialTask", 8192, NULL, configMAX_PRIORITIES - 3, NULL, 1);

    Serial.println("Setup Complete!\r\n");
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout resets
}

/**
 * @brief Primary Navigation and Control Loop.
 * Runs on Core 1 (Main Arduino thread).
 */
void loop()
{
    // Initial state setup
    mainData.status = IDLE;
    beep(1, buzzer);

    while (true)
    {
        if (xSemaphoreTake(mainDataMutex, portMAX_DELAY)) 
        {
            // 1. Process Navigation PIDs
            rudderPid(&mainData);
            speedPid(&mainData);
            
            // 2. Handle System Status and Timers
            handelStatus(&mainData);
            handleTimerRoutines(&mainData);
            
            // 3. Update Heading from Compass Queue
            xQueueReceive(compass, (void *)&mainData.dirMag, 0);
            
            // 4. Handle External Communication (Serial/RF)
            handelSerandRfdata(&mainData);
            handelSerialTimeOut(&mainData);
            
            // 5. Periodic Telemetry Logging to COM51
            if (logtimer < millis())
            {
                logtimer = millis() + 1000;
                printf("V31 TD:%05.2f TgSpd:%05.2f C:%03.0f T:%03.0f A:%03.0f Rud:%02.2f bb:%03d Sb:%03d\r\n", 
                        mainData.tgDist - 2, mainData.tgSpeed, mainData.dirMag, mainData.tgDir, 
                        smallestAngle(mainData.tgDir, mainData.dirMag), rudderOutput, 
                        mainData.speedBb, mainData.speedSb);
            }
            
            xSemaphoreGive(mainDataMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz Control Loop
    }
}
