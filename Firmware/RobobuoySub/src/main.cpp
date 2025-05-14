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
#include "sercom.h"
#define HOST_NAME "RoboBuoySub"

// pid subparameter;
static RoboStruct mainData;
static RoboStruct udpInMain;
static RoboStruct serDataIn;
static Message esc;
static unsigned long buoyId;
static unsigned long PwrOff;
static int subStatus = IDLE;
static LedData mainLedStatus;
static PwrData mainPwrData;
static Buzz mainBuzzerData;
static int wifiConfig = 0;

unsigned long nextSamp = millis();
unsigned long accuSamp = millis();
unsigned long logtimer = millis();
unsigned long udptimer = millis();
unsigned long pidtimer = millis();

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
double rudderSetpoint = 0, rudderInput, rudderOutput;
double Kp = 1, Ki = 0.1, Kd = 0.1;
PID rudderPID(&rudderInput, &rudderOutput, &rudderSetpoint, Kp, Ki, Kd, DIRECT);

//***************************************************************************************************
// Setup
//***************************************************************************************************
void setup()
{
    pinMode(PWRENABLE, OUTPUT);
    pinMode(BUTTON_PIN, INPUT);
    pinMode(ESC_SB_PWR_PIN, OUTPUT);
    pinMode(ESC_BB_PWR_PIN, OUTPUT);
    digitalWrite(ESC_SB_PWR_PIN, LOW);
    digitalWrite(ESC_BB_PWR_PIN, LOW);
    delay(100);
    digitalWrite(PWRENABLE, 1); // enable powersupply
    pinMode(BATTENABLE, OUTPUT);
    digitalWrite(BATTENABLE, 1); // enable batery sample signal
    Serial.begin(115200);
    // rudderPID.SetMode(AUTOMATIC); // turn the PID on
    // rudderPID.SetOutputLimits(-180, 180);

    digitalWrite(PWRENABLE, true);
    printf("\r\nSetup running!\r\n");
    printf("Robobuoy Sub Version: %0.1f\r\n", SUBVERSION);
    buoyId = initwifi(); // buoyID is mac adress esp32
    Serial.print("Buoy ID: ");
    Serial.println(buoyId);
    mainData.mac = buoyId;
    char mac[6];
    initMemory();
    InitCompass();
    initledqueue();
    initbuzzerqueue();
    initescqueue();
    initcompassQueue();
    initserqueue();
    xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 8000, &wifiConfig, configMAX_PRIORITIES - 5, NULL, 0);
    xTaskCreatePinnedToCore(buzzerTask, "buzzTask", 1000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(EscTask, "EscTask", 2400, NULL, configMAX_PRIORITIES - 10, NULL, 1);
    xTaskCreatePinnedToCore(LedTask, "LedTask", 2000, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(CompassTask, "CompasaTask", 2000, NULL, configMAX_PRIORITIES - 2, NULL, 0);
    xTaskCreatePinnedToCore(SercomTask, "SerialTask", 4000, NULL, configMAX_PRIORITIES - 2, NULL, 0);
    if (digitalRead(BUTTON_PIN) == true)
    {
        delay(100);
        if (digitalRead(BUTTON_PIN) == true)
        {
            wifiConfig = 1;
        }
    }
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
    rudderPID.SetTunings(mainData.kpr, mainData.kir, mainData.kdr, DIRECT);
    rudderPID.SetMode(AUTOMATIC);
    rudderPID.SetOutputLimits(-99, 99);
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
    if (buttonState == LOW && lastButtonState == HIGH && !debounce)
    {
        pressCount++;                     // Increment the button press count
        debounce = true;                  // Set debounce flag
        beep(10, buzzer);                 // short high pitch beep
        debounceDelay = currentTime + 50; // Simple debouncing by adding a delay
        lastPressTime = currentTime;      // Record the time of the last press
    }
    // Reset debounce flag if the button is released
    if (buttonState == HIGH)
    {
        debounce = false;
    }
    // If more than 2 seconds have passed without a press, return the count
    if ((currentTime - lastPressTime) > 500 && pressCount > 0 && buttonState == HIGH)
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
    else if ((currentTime - lastPressTime) > 3000 && pressCount == 1 && buttonState == LOW)
    {

        pressCount = 0x0100;
        return 0X100;
    }
    lastButtonState = buttonState; // Save the last button state
    return -1;                     // Return -1 if 0.5 seconds haven't passed yet
}

//***************************************************************************************************
// Timer routines
//***************************************************************************************************
RoboStruct handleTimerRoutines(RoboStruct in)
{
    if (nextSamp + 250 < millis())
    {
        printf("COG: %03.0f TG: %03f.0 Error: %03.0f PIDout: %03.0f\r\n", in.dirMag, in.tgDir, rudderOutput);
        nextSamp = millis();
    }

    if (accuSamp < millis())
    {
        accuSamp = 60 * 1010 + millis();
        battVoltage(in.subAccuV, in.subAccuP);
    }

    if (logtimer < millis())
    {
        logtimer = millis() + 1000;
        in.cmd = SUBACCU;
        xQueueSend(serOut, (void *)&in, 10); // Keep the watchdog in sub happy
        //        if (subStatus == CALCRUDDER)
        //        {
        // printf("hdg:%6.2f dir%6.2f dist%5.2f speed:%4d ", in.dirMag, in.tgDir, in.tgDist, in.speed);
        // printf("Dir:%6.1f Speed:%4d BB:%2d SB:%2d is:%5.1f ir:%5.1f\r\n", smallestAngle(in.dirMag, in.tgDir), in.speed, in.speedBb, in.speedSb, in.is, in.ir);
        printf("Hdg:%5.2f tgAngle:%5.1f Speed:%d Bb:%d Sb:%d \r\n", in.dirMag, in.tgDir, in.speed, in.speedBb, in.speedSb);
        // Serial.println("Distance: " + String(mainData.tgDist) + " Dir: " + String(mainData.tgDir));
        // }
        // else
        // {
        //     double ang = smallestAngle(in.dirSet, in.dirMag);

        //     // printf("Hdg:%5.2f tgAngle:%5.1f Speed:%d Bb:%d Sb:%d ", in.dirMag, ang, in.speed, in.speedBb, in.speedSb);

        //     if (in.speedBb == in.speedSb)
        //     {
        //         // printf("-\r\n");
        //     }
        //     if (in.speedBb < in.speedSb)
        //     {
        //         // printf(">\r\n");
        //     }
        //     if (in.speedBb > in.speedSb)
        //     {
        //         // printf("<\r\n");
        //     }
        //     logtimer = millis() + 500;
        // }
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
    rudderInput = smallestAngle(rud.tgDir, rud.dirMag);
    rudderSetpoint = 0;
    rudderPID.Compute();
    double bb = rud.tgSpeed - rudderOutput / 2;
    double sb = rud.tgSpeed + rudderOutput / 2;
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
    unsigned long lastSerIn = millis();
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
    PwrOff = millis();
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
        //         rudderPID.Compute();
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
        if (xQueueReceive(compass, (void *)&mainData.dirMag, 0) == pdTRUE)
        {
            if (mainData.status == LOCKED || mainData.status == DOCKED)
            {
                if (rudderTimer + 100 < millis())
                {
                    rudderTimer = millis();
                    mainData = rudderPid(mainData);
                }
            }
        }
        //***************************************************************************************************
        //      new udp message
        //***************************************************************************************************
        if (xQueueReceive(udpIn, (void *)&udpInMain, 0) == pdTRUE)
        {
            // mainData.lastSerIn = millis();
            switch (mainData.cmd)
            {
            case IDLE:
                mainData.status = IDELING;
                break;
            case DIRSPEED:
                mainData.speed = (int)udpInMain.speedSet;
                mainData.status = REMOTE;
                break;
            case DIRDISTSP:
                mainData.tgDir = (int)udpInMain.tgDir;
                mainData.tgDist = (int)udpInMain.tgDist;
                mainData.tgSpeed = (int)udpInMain.tgSpeed;
                mainData.status = LOCKED;
                break;
            case PIDRUDDERSET:
                pidRudderParameters(udpInMain, false);
                mainData.kir = 0;
                break;
            case PIDSPEEDSET:
                pidSpeedParameters(udpInMain, false);
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
        //***************************************************************************************************
        //      New serial data
        //***************************************************************************************************
        if (xQueueReceive(serIn, (void *)&serDataIn, 0) == pdTRUE)
        {
            mainData.lastSerIn = millis();
            if (serDataIn.cmd != PING)
            {
                PwrOff = millis();
            }
            if (mainLedStatus.color != CRGB::DarkBlue)
            {
                Serial.println("Set light to Blue");
                mainLedStatus.color = CRGB::DarkBlue;
                mainLedStatus.blink = BLINK_SLOW;
                xQueueSend(ledStatus, (void *)&mainLedStatus, 0); // update util led
            }
            switch (serDataIn.cmd)
            {
            case IDLE:
                mainData.status = IDELING;
                break;
            case DIRDISTSP:
                mainData.tgDir = (int)serDataIn.tgDir;
                mainData.tgDist = (int)serDataIn.tgDist;
                mainData.tgSpeed = (int)serDataIn.tgSpeed;
                mainData.status = LOCKED;
                break;
            case DIRSPEED:
                mainData.dirSet = (int)serDataIn.dirSet;
                mainData.speed = (int)serDataIn.speedSet;
                mainData.status = REMOTE;
                xQueueSend(serOut, (void *)&mainData, 10);
                break;
            case LOCKED:
            case DOCKED:
                mainData.tgDist = serDataIn.tgDist;
                mainData.tgDir = serDataIn.tgDir;
                mainData.status = serDataIn.cmd;
                break;
            case PIDRUDDERSET:
                mainData.kpr = serDataIn.kpr;
                mainData.kir = serDataIn.kir;
                mainData.kdr = serDataIn.kdr;
                pidRudderParameters(mainData, SET);
                mainData.kir = 0;
                break;
            case PIDSPEEDSET:
                mainData.kps = serDataIn.kps;
                mainData.kis = serDataIn.kis;
                mainData.kds = serDataIn.kds;
                pidSpeedParameters(mainData, SET);
                mainData.kis = 0;
                break;
            case STORE_CALIBRATE_OFFSET_MAGNETIC_COMPASS:
                mainData.compassOffset += mainData.tgDir;
                MechanicalCorrection(&mainData.compassOffset, SET);
                break;
            case PING:
                mainData.cmd = DIRSPEED;
                xQueueSend(serOut, (void *)&mainData, 10);
                break;
            default:
                // Serial.println(serDataIn.cmd);
                break;
            }
        }
        //***************************************************************************************************
        //      Serial watchdog
        //***************************************************************************************************
        // if (mainData.lastSerIn + 1000 * 5 < millis())
        if (mainData.lastSerIn + 1000 * 2 < millis())
        {
            if (mainLedStatus.color != CRGB::Red)
            {
                Serial.println("Set light to Red");
                mainData.lastSerIn = millis();
                mainLedStatus.color = CRGB::Red;
                mainLedStatus.blink = BLINK_SLOW;
                xQueueSend(ledStatus, (void *)&mainLedStatus, 10); // update util led
                beep(500, buzzer);
                mainData.speed = 0;
                mainData.status = IDELING;
            }
        }
        //***************************************************************************************************
        //      Shutdown the system
        //***************************************************************************************************
        // if (PwrOff + 1000 * 60 * 3 < millis())
        if (PwrOff + 1000 * 30 * 1 < millis())
        {
            beep(-1, buzzer);
            printf("Power off now!'r'n");
            delay(1000);
            digitalWrite(PWRENABLE, 0); // disable powersupply
            delay(1000);
            digitalWrite(PWRENABLE, 0); // disable powersupply
            delay(1000);
            PwrOff = millis();
        }
        vTaskDelay(1);
    }
}
