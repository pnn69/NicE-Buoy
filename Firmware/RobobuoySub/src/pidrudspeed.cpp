#include <pidrudspeed.h>
#include "esc.h"
#include "datastorage.h"
static Message esc;
static RoboStruct rudderData;
static RoboStruct speedData;

//***************************************************************************************************
//  new pid stuff
//***************************************************************************************************
double rudderSetpoint = 0, rudderInput = 0, rudderOutput;
double speedSetpoint = 0, speedInput = 0, speedOutput;
double pr = 0, ir = 0, dr = 0;
double ps = 0, is = 0, ds = 0;
PID rudderPID(&rudderInput, &rudderOutput, &rudderSetpoint, pr, ir, dr, DIRECT);
PID speedPID(&speedInput, &speedOutput, &speedSetpoint, ps, is, ds, DIRECT);

void initRudPid(RoboStruct* rud)
{
    speedMaxMin(rud,GET);
    pidRudderParameters(rud, GET);
    computeParameters(rud, GET);
    rudderPID.SetSampleTime(100); // milliseconds
    rudderPID.SetTunings(rud->pr, rud->ir, rud->dr, DIRECT);
    rudderPID.SetOutputLimits(-100, 100);
    rudderPID.SetMode(AUTOMATIC); // turn the PID on
    Serial.println("PID rudder pr:" + String(rud->pr, 2) +
                   " ir:" + String(rud->ir, 2) +
                   " dr:" + String(rud->dr, 2) +
                   " minSpeed:" + String(rud->minSpeed) +
                   " maxSpeed:" + String(rud->maxSpeed));
}

void initSpeedPid(RoboStruct* speed)
{
    speedMaxMin(speed,GET);
    speedPID.SetSampleTime(100); // milliseconds
    pidSpeedParameters(speed, GET);
    speedPID.SetTunings(speed->ps, speed->is, speed->ds, DIRECT);
    computeParameters(speed, GET);
    speedPID.SetOutputLimits(0, speed->maxSpeed);
    speedPID.SetMode(AUTOMATIC); // turn the PID on
    Serial.println("PID speed ps:" + String(speed->ps, 2) +
                   " is:" + String(speed->is, 2) +
                   " ds:" + String(speed->ds, 2) +
                   " minSpeed:" + String(speed->minSpeed) +
                   " maxSpeed:" + String(speed->maxSpeed));
}

//***************************************************************************************************
//  RudderPID
//***************************************************************************************************
void rudderPid(RoboStruct *rud)
{
    rudderInput = smallestAngle(rud->tgDir, rud->dirMag);
    rudderSetpoint = 0;
    rudderPID.Compute();

    double s = constrain(rud->tgSpeed, rud->minSpeed, rud->maxSpeed);
    s -=abs(rudderOutput);
    double sb = s - rudderOutput;
    double bb = s + rudderOutput;
    rud->speedSb = constrain(sb, rud->minSpeed, rud->maxSpeed);
    rud->speedBb = constrain(bb, rud->minSpeed, rud->maxSpeed);
    esc.speedbb = rud->speedBb;
    esc.speedsb = rud->speedSb;
    xQueueSend(escspeed, (void *)&esc, 10);

    //printf("TD:%05.2f  tgSpeed: %05.2f angle: %03.0f output: %07.2f  Sb: %3d Bb: %3d\r\n", rud->tgDist, rud->tgSpeed, rudderInput, rudderOutput, rud->speedSb, rud->speedBb);
}
//***************************************************************************************************
//  speedPID
//***************************************************************************************************
void speedPid(RoboStruct *dist)
{
    speedInput = -(dist->tgDist - 2);
    speedSetpoint = 0;
    speedPID.Compute();
    dist->tgSpeed = speedOutput;
}
