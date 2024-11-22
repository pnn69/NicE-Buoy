#include <Arduino.h>
#include <RoboTone.h>
#include <RoboCompute.h>
#include <PID_v1.h>
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
#define HOST_NAME "RoboBuoySub"

// pid subparameter;
static RoboStruct mainData;
static char udpInMain[MAXSTRINGLENG];
static Message esc;
static unsigned long buoyId;
static int subStatus = IDLE;
static LedData mainLedStatus;
static PwrData mainPwrData;
static Buzz mainBuzzerData;
static int wifiConfig = 0;

unsigned long nextSamp = millis();
unsigned long accuSamp = millis();
unsigned long logtimer = millis();
unsigned long udptimer = millis();

int buttonState = 0;              // Current state of the button
int lastButtonState = 0;          // Previous state of the button
unsigned long lastPressTime = 0;  // Time of the last press
unsigned long debounceDelay = 50; // Debounce time in milliseconds
int pressCount = 0;               // Count the number of button presses
bool debounce = false;            // Debouncing flag
int presses = 0;

//***************************************************************************************************
//  new pid stuff
//***************************************************************************************************
double Setpoint, Input, Output;
double Kp = 1, Ki = 0.1, Kd = 0.1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//***************************************************************************************************
// Setup
//***************************************************************************************************
void setup()
{
    Serial.begin(115200);
    Setpoint = 0;             // tg angle
    myPID.SetMode(AUTOMATIC); // turn the PID on
    myPID.SetOutputLimits(-180, 180);
    pinMode(BUTTON_PIN, INPUT);
    delay(10);
    printf("\r\nSetup running!\r\n");
    printf("Robobuoy Sub Version: %0.1f\r\n", SUBVERSION);
    buoyId = initwifiqueue();
    Serial.print("Buoy ID: ");
    Serial.println(buoyId);
    initbuzzerqueue();
    initledqueue();
    initescqueue();
    initMemory();
    initcompassQueue();
    InitCompass();
    xTaskCreatePinnedToCore(buzzerTask, "buzzTask", 1000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(EscTask, "EscTask", 2400, NULL, configMAX_PRIORITIES - 10, NULL, 1);
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(CompassTask, "CompasaTask", 2000, NULL, configMAX_PRIORITIES - 2, NULL, 0);
    if (digitalRead(BUTTON_PIN) == true)
    {
        delay(100);
        if (digitalRead(BUTTON_PIN) == true)
        {
            wifiConfig = 1;
        }
    }
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 8000, &wifiConfig, configMAX_PRIORITIES - 5, NULL, 0);
    /*
    Get sored settings
    */

    // mainData.ps = 20;
    // mainData.is = 0.1;
    // mainData.ds = 0;
    // pidSpeedParameters(mainData, SET);
    mainData.pr = 1;
    mainData.ir = 0.05;
    mainData.dr = 0.01;
    pidRudderParameters(mainData, SET);
    // mainData.minOfsetDist = 2;
    // mainData.maxOfsetDist = 8;
    // mainData.minSpeed = 0;
    // mainData.maxSpeed = 80;
    // computeParameters(mainData, SET);
    // mainData.minOfsetDist = 2;
    // mainData.maxOfsetDist = 8;
    // mainData.minSpeed = 0;
    // mainData.maxSpeed = 80;
    // computeParameters(mainData, SET);
    mainData = pidSpeedParameters(mainData, GET);
    mainData = pidRudderParameters(mainData, GET);
    // mainData.compassOffset = 0;
    // CompassOffsetCorrection(&mainData.compassOffset, SET);
    CompassOffsetCorrection(&mainData.compassOffset, GET);
    mainData = computeParameters(mainData, GET);
    // mainData.minOfsetDist = 1;
    // mainData.maxOfsetDist = 8;
    // mainData.minSpeed =  -80;
    // mainData.maxSpeed =  80;
    // mainData = computeParameters(mainData, SET);
    Serial.println("Compute Parameters Kpr:" + String(mainData.kps) +
                   " I:" + String(mainData.kis) +
                   " Rudder p:" + String(mainData.kpr) +
                   " I:" + String(mainData.kir) +
                   " MAxspeed:" + String(mainData.maxSpeed) +
                   " MINspeed:" + String(mainData.minSpeed) +
                   " Maxoffsetdistance:" + String(mainData.maxOfsetDist) +
                   " MINoffsetdistance:" + String(mainData.minOfsetDist));
    myPID.SetTunings(mainData.kpr, mainData.kir, mainData.kdr, DIRECT);
    Serial.println("Setup done!");
}

//***************************************************************************************************
//      keypress detection
//***************************************************************************************************
int countKeyPressesWithTimeoutAndLongPressDetecton()
{
    // Get the current time
    unsigned long currentTime = millis();
    if (currentTime < debounceDelay)
    {
        return -1; // debounce
    }
    buttonState = digitalRead(BUTTON_PIN);
    // Check if the button is pressed and it's a new press (debounce)
    if (buttonState == HIGH && lastButtonState == LOW && !debounce)
    {
        pressCount++;                     // Increment the button press count
        debounce = true;                  // Set debounce flag
        beep(10, buzzer);                 // short high pitch beep
        debounceDelay = currentTime + 50; // Simple debouncing by adding a delay
        lastPressTime = currentTime;      // Record the time of the last press
    }
    // Reset debounce flag if the button is released
    if (buttonState == LOW)
    {
        debounce = false;
    }
    // If more than 2 seconds have passed without a press, return the count
    if ((currentTime - lastPressTime) > 500 && pressCount > 0 && buttonState == LOW)
    {
        if (pressCount == 0x100) // previous detecion was a long press
        {
            pressCount = 0; // Reset the press count after returning
            return -1;      // Return -1 if 0.5 seconds haven't passed yet
        }
        int finalPressCount = pressCount; // Store the current press count
        pressCount = 0;                   // Reset the press count after returning
        return finalPressCount;           // Return the number of key presses
    }
    else if ((currentTime - lastPressTime) > 3000 && pressCount == 1 && buttonState == HIGH)
    {

        pressCount = 0x0100;
        return 0X100;
    }
    lastButtonState = buttonState; // Save the last button state
    return -1;                     // Return -1 if 0.5 seconds haven't passed yet
}

//***************************************************************************************************
// Setup
//***************************************************************************************************
RoboStruct handleTimerRoutines(RoboStruct in)
{
    if (in.status != IDLE && in.lastUdpIn + 5000 < millis()) // udp timeout detection
    {
        mainLedStatus.color = CRGB::LightGoldenrodYellow;
        mainLedStatus.blink = BLINK_FAST;
        xQueueSend(ledStatus, (void *)&mainLedStatus, 10); // update util led
        subStatus = IDELING;
    }

    if (nextSamp + 100 < millis())
    {
        nextSamp = millis();
        in.cmd = DIRSPEED;
        xQueueSend(udpOut, (void *)&in, 10); // update WiFi
    }

    if (accuSamp < millis())
    {
        accuSamp = 5000 + millis();
        in.cmd = SUBACCU;
        battVoltage(in.subAccuV, in.subAccuP);
        xQueueSend(udpOut, (void *)&in, 10); // update WiFi
    }

    if (logtimer < millis())
    {
        logtimer = millis() + 1000;
        if (subStatus == CALCRUDDER)
        {
            printf("hdg:%6.2f dir%6.2f dist%5.2f speed:%4d ", in.dirMag, in.tgDir, in.tgDist, in.speed);
            // printf(" buoy.iintergrater:%f pr%f ir%f err%f", in.iintergrater, in.pr, in.ir,in.lastErrr);
            printf("Dir:%6.1f Speed:%4d BB:%2d SB:%2d is:%5.1f ir:%5.1f\r\n", smallestAngle(in.dirMag, in.tgDir), in.speed, in.speedBb, in.speedSb, in.is, in.ir);
        }
        else
        {
            double ang = smallestAngle(in.dirSet, in.dirMag);

            printf("Hdg:%5.2f tgAngle:%5.1f Speed:%d Bb:%d Sb:%d ", in.dirMag, ang, in.speed, in.speedBb, in.speedSb);

            if (in.speedBb == in.speedSb)
            {
                printf("-\r\n");
            }
            if (in.speedBb < in.speedSb)
            {
                printf(">\r\n");
            }
            if (in.speedBb > in.speedSb)
            {
                printf("<\r\n");
            }
            logtimer = millis() + 500;
        }
    }
    return in;
}

//***************************************************************************************************
//      key press stuff
//***************************************************************************************************
int handelKeyPress(int key)
{
    int presses = countKeyPressesWithTimeoutAndLongPressDetecton();
    if (presses > 0)
    {
        Serial.print("Number of key presses: ");
        Serial.println(presses);
        if (presses == 5) // Calibrate compas north
        {
            beep(1, buzzer);
            calibrateNorthCompas();
            presses = -1;
        }
        if (presses == 10) // Calibrate compas
        {
            beep(1, buzzer);
            calibrateParametersCompas();
            presses = -1;
        }
        if (presses != -1)
        {
            beep(-1, buzzer);
        }
    }
    return key;
}

//***************************************************************************************************
//  RudderPID
//***************************************************************************************************
RoboStruct rudderPid(RoboStruct rud)
{
    Input = smallestAngle(rud.dirSet, rud.dirMag);
    Setpoint = 0;
    myPID.Compute();
    // Output += rud.compassOffset; // mecanical offset
    // if (Output < 0)
    // {
    //     Output += 360;
    // }
    // if (Output > 360)
    // {
    //     Output -= 360;
    // }
    double deltaPwr = (Output / 180) * 2;
    double bb = rud.speed * (1 - deltaPwr);
    double sb = rud.speed * (1 + deltaPwr);
    rud.speedSb = constrain(sb, rud.minSpeed, rud.maxSpeed);
    rud.speedBb = constrain(bb, rud.minSpeed, rud.maxSpeed);
    if (esc.speedbb != rud.speedBb || esc.speedsb != rud.speedSb)
    {
        esc.speedbb = rud.speedBb;
        esc.speedsb = rud.speedSb;
        xQueueSend(escspeed, (void *)&esc, 10);
    }
    return rud;
}

//***************************************************************************************************
//      status actions
//***************************************************************************************************
RoboStruct handelStatus(RoboStruct stat)
{
    if (stat.status == IDELING)
    {
        stat.speed = 0;
        stat.speedBb = 0;
        stat.speedSb = 0;
        stat.status = IDLE;
        esc.speedbb = 0;
        esc.speedsb = 0;
        xQueueSend(escspeed, (void *)&esc, 100);
    }
    return stat;
}

//***************************************************************************************************
//  Main loop
//***************************************************************************************************
void loop(void)
{
    String dataIn = "";
    String dataOut = "";
    int msg = -1;
    float vbat = 0;
    int speedbb = 50, speedsb = 0;
    unsigned long rudderTimer = 0;
    beep(1000, buzzer);
    delay(500);
    mainLedStatus.color = CRGB::LightPink;
    mainLedStatus.blink = BLINK_SLOW;
    xQueueSend(ledStatus, (void *)&mainLedStatus, 10); // update util led
    mainPwrData.bb = CRGB::Black;
    mainPwrData.sb = CRGB::Black;
    mainPwrData.blinkBb = BLINK_OFF;
    mainPwrData.blinkSb = BLINK_OFF;
    xQueueSend(ledPwr, (void *)&mainPwrData, 10); // update util led
    unsigned long pidtimer = millis();
    /*
        Main loop
    */
    while (true)
    {
        //***************************************************************************************************
        //      new PID gepruttel
        //***************************************************************************************************
        // if (pidtimer + 100 < millis())
        // {
        //     pidtimer = millis();
        //     if (mainData.status == LOCKED || mainData.status == DOCKED)
        //     {
        //         Input = smallestAngle(mainData.dirSet, mainData.dirMag);
        //         myPID.Compute();
        //         mainData.speed = 50;
        //         double deltaPwr = (Output / 180) * 2;
        //         double bb = mainData.speed * (1 + deltaPwr);
        //         double sb = mainData.speed * (1 - deltaPwr);
        //         mainData.speedSb = constrain(sb, mainData.minSpeed, mainData.maxSpeed);
        //         mainData.speedBb = constrain(bb, mainData.minSpeed, mainData.maxSpeed);
        //         // printf("Correction:%3.0f Angle:%6.2f SpeedBb:%4.1d SpeedSb:%4.1d tmp:%.2f\r\n", Output, Input, mainData.speedBb, mainData.speedSb, deltaPwr);
        //     }
        // }
        //***************************************************************************************************
        //      Timer routines
        //***************************************************************************************************
        mainData = handleTimerRoutines(mainData);
        //***************************************************************************************************
        //      Check front key
        //***************************************************************************************************
        presses = handelKeyPress(presses);
        //***************************************************************************************************
        //      Status change handeling
        //***************************************************************************************************
        mainData = handelStatus(mainData);
        //***************************************************************************************************
        //      New compass data
        //***************************************************************************************************
        if (xQueueReceive(compass, (void *)&mainData.dirMag, 2) == pdTRUE)
        {
            if (mainData.status == LOCKED || mainData.status == DOCKED)
            {
                if (rudderTimer + 250 < millis())
                {
                    rudderTimer = millis();
                    mainData = rudderPid(mainData);
                }
            }
        }
        //***************************************************************************************************
        //      new udp message
        //***************************************************************************************************
        if (mainData.lastUdpIn + 2000 < millis())
        {
            mainData.lastUdpIn = millis();
            mainLedStatus.color = CRGB::LightPink;
            mainLedStatus.blink = BLINK_SLOW;
            xQueueSend(ledStatus, (void *)&mainLedStatus, 10); // update util led
            mainData.status = IDELING;
        }
        if (xQueueReceive(udpIn, (void *)&udpInMain, 0) == pdTRUE)
        {
            mainData.lastUdpIn = millis();
            if (mainLedStatus.color != CRGB::DarkBlue)
            {
                mainLedStatus.color = CRGB::DarkBlue;
                mainLedStatus.blink = BLINK_SLOW;
                xQueueSend(ledStatus, (void *)&mainLedStatus, 10); // update util led
            }
            mainData = RoboDecode(udpInMain, mainData);
            switch (mainData.cmd)
            {
            case IDLE:
                mainData.status = IDELING;
                break;
            case DIRSPEED:
                mainData.speed = (int)mainData.speedSet;
                mainData.status = REMOTE;
                break;
            case REMOTE:
            case PIDRUDDERSET:
                pidRudderParameters(mainData, false);
                mainData.kir = 0;
                break;
            case PIDSPEEDSET:
                pidSpeedParameters(mainData, false);
                mainData.kis = 0;
                break;
            case STORE_CALIBRATE_OFFSET_MAGNETIC_COMPASS:
                mainData.compassOffset += mainData.tgDir;
                MechanicalCorrection(&mainData.compassOffset, SET);
                break;
            case PING:
                mainData.cmd = PONG;
                xQueueSend(udpOut, (void *)&mainData, 10); // update WiFi
                break;
            }
        }
    }
    vTaskDelay(1);
}
