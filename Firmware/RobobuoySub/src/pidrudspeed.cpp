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
PID rudderPID(&rudderInput, &rudderOutput, &rudderSetpoint, Kpr, Kir, Kdr, P_ON_E);
PID speedPID(&speedInput, &speedOutput, &speedSetpoint, Kps, Kis, Kds, P_ON_E);

void initRudPid(RoboStruct *rud)
{
    speedMaxMin(rud, GET);
    pidRudderParameters(rud, GET);
    // computeParameters(rud, GET);
    rudderPID.SetSampleTime(100); // milliseconds
    rudderPID.SetTunings(rud->Kpr, rud->Kir, rud->Kdr, P_ON_E);
    rudderPID.SetOutputLimits(-100, 100);
    rudderPID.SetMode(MANUAL);
    rudderOutput = 0;
    rudderInput = 0;
    rudderSetpoint = 0;
    rudderOutput = 0;
    rudderPID.SetMode(AUTOMATIC); // turn the PID on
    rud->ir = rudderPID.GetITerm();
    Serial.println("PID rudder used for calculations> pr:" + String(rud->Kpr, 2) +
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
    speedPID.SetTunings(speed->Kps, speed->Kis, speed->Kds, P_ON_E);
    computeParameters(speed, GET);
    speedPID.SetOutputLimits(0, speed->maxSpeed);
    speedPID.SetMode(MANUAL);
    speedInput = 0;
    speedSetpoint = 0;
    speedOutput = 0;
    speedPID.SetMode(AUTOMATIC); // turn the PID on
    speed->ip = speedPID.GetITerm();
    Serial.println("PID speed  used for calculations> ps:" + String(speed->Kps, 2) +
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
    double bb = s - rudderOutput;
    double sb = s + rudderOutput;
    rud->speedSb = constrain(sb, rud->minSpeed, rud->maxSpeed);
    rud->speedBb = constrain(bb, rud->minSpeed, rud->maxSpeed);
    rud->ir = rudderPID.GetITerm();

    // printf("TD:%05.2f  tgSpeed: %05.2f angle: %03.0f output: %07.2f  Sb: %3d Bb: %3d\r\n", rud->tgDist, rud->tgSpeed, rudderInput, rudderOutput, rud->speedSb, rud->speedBb);
}
//***************************************************************************************************
//  speedPID
//***************************************************************************************************
void speedPid(RoboStruct *dist)
{
    // speedInput = -(dist->tgDist - 2);
    // speedSetpoint = 0;
    speedInput = dist->tgDist - 2; // error input
    speedSetpoint = 0;
    if (speedPID.Compute())
    {
        dist->tgSpeed = speedOutput;
        dist->ip = speedPID.GetITerm();
        // Serial.printf("input:%05.2f output: %5.2f i ip: %0.2f \r\n", abs(dist->tgDist - 2), speedOutput, dist->ip);
    }
}
