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
double Kpr = 0, Kir = 0, Kdr = 0;
double Kps = 0, Kis = 0, Kds = 0;
// add this in PID_v1.h>>> double GetITerm() { return outputSum; }
PID rudderPID(&rudderInput, &rudderOutput, &rudderSetpoint, Kpr, Kir, Kdr, DIRECT);
PID speedPID(&speedInput, &speedOutput, &speedSetpoint, Kps, Kis, Kds, DIRECT);

void initRudPid(RoboStruct *rud)
{
    speedMaxMin(rud, GET);
    pidRudderParameters(rud, GET);
    computeParameters(rud, GET);
    rudderPID.SetSampleTime(100); // milliseconds
    rudderPID.SetTunings(rud->Kpr, rud->Kir, rud->Kdr, DIRECT);
    rudderPID.SetOutputLimits(-100, 100);
    rudderPID.SetMode(AUTOMATIC); // turn the PID on
    Serial.println("PID rudder pr:" + String(rud->Kpr, 2) +
                   " ir:" + String(rud->Kir, 2) +
                   " dr:" + String(rud->Kdr, 2) +
                   " minSpeed:" + String(rud->minSpeed) +
                   " maxSpeed:" + String(rud->maxSpeed));
}

void initSpeedPid(RoboStruct *speed)
{
    speedMaxMin(speed, GET);
    speedPID.SetSampleTime(100); // milliseconds
    pidSpeedParameters(speed, GET);
    speedPID.SetTunings(speed->Kps, speed->Kis, speed->Kds, DIRECT);
    computeParameters(speed, GET);
    speedPID.SetOutputLimits(0, speed->maxSpeed);
    speedPID.SetMode(AUTOMATIC); // turn the PID on
    Serial.println("PID speed ps:" + String(speed->Kps, 2) +
                   " is:" + String(speed->Kis, 2) +
                   " ds:" + String(speed->Kds, 2) +
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
    int margin = abs(rudderOutput);
    if (s < rud->minSpeed - margin)
    {
        s = rud->minSpeed - margin; // or some appropriate minimum correction
    }
    else if (s > rud->maxSpeed + margin)
    {
        s = rud->maxSpeed + margin; // clamp to upper limit
    }
    double bb = s + rudderOutput;
    double sb = s - rudderOutput;
    rud->speedSb = constrain(sb, rud->minSpeed, rud->maxSpeed);
    rud->speedBb = constrain(bb, rud->minSpeed, rud->maxSpeed);
    esc.speedbb = rud->speedBb;
    esc.speedsb = rud->speedSb;
    xQueueSend(escspeed, (void *)&esc, 10);

    rud->ir = rudderPID.GetITerm();
    //Serial.println(rud->ir);

    // printf("TD:%05.2f  tgSpeed: %05.2f angle: %03.0f output: %07.2f  Sb: %3d Bb: %3d\r\n", rud->tgDist, rud->tgSpeed, rudderInput, rudderOutput, rud->speedSb, rud->speedBb);
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
    dist->is = speedPID.GetITerm();
    //Serial.println(dist->is);
}
