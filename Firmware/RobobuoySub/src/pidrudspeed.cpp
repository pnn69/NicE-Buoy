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
int sampleTime = 100; // milliseconds
PID rudderPID(&rudderInput, &rudderOutput, &rudderSetpoint, pr, ir, dr, DIRECT);
PID speedPID(&speedInput, &speedOutput, &speedSetpoint, ps, is, ds, DIRECT);

void initRudPid(void)
{
    static RoboStruct rud, t;
    rud = pidRudderParameters(rud, GET);
    t = computeParameters(t, GET);
    rudderPID.SetSampleTime(sampleTime);
    rud.maxSpeed = t.maxSpeed;
    rud.minSpeed = t.minSpeed;
    rudderPID.SetTunings(rud.pr, rud.ir, rud.dr, DIRECT);
    // rudderPID.SetOutputLimits(rud.minSpeed, rud.maxSpeed);
    rudderPID.SetOutputLimits(-100, 100);
    rudderPID.SetMode(AUTOMATIC); // turn the PID on
    Serial.println("PID rudder pr:" + String(rud.pr, 2) +
                   " ir:" + String(rud.ir, 2) +
                   " dr:" + String(rud.dr, 2) +
                   " minSpeed:" + String(rud.minSpeed) +
                   " maxSpeed:" + String(rud.maxSpeed));
}

void initSpeedPid(void)
{
    static RoboStruct speed, t;
    speedPID.SetSampleTime(sampleTime);
    speed = pidSpeedParameters(speed, GET);
    speedPID.SetTunings(speed.ps, speed.is, speed.ds, DIRECT);
    t = computeParameters(t, GET);
    speed.maxSpeed = t.maxSpeed;
    speed.minSpeed = t.minSpeed;
    speedPID.SetOutputLimits(speed.minSpeed, speed.maxSpeed);
    speedPID.SetMode(AUTOMATIC); // turn the PID on
    Serial.println("PID speed ps:" + String(speed.ps, 2) +
                   " is:" + String(speed.is, 2) +
                   " ds:" + String(speed.ds, 2) +
                   " minSpeed:" + String(speed.minSpeed) +
                   " maxSpeed:" + String(speed.maxSpeed));
}

//***************************************************************************************************
//  RudderPID
//***************************************************************************************************
void rudderPid(RoboStruct *rud)
{
    rudderInput = smallestAngle(rud->tgDir, rud->dirMag);
    rudderSetpoint = 0;
    rudderPID.Compute();
    rud->tgSpeed = constrain(rud->tgSpeed, rud->minSpeed, rud->maxSpeed);
    double bb = rud->tgSpeed - rudderOutput;
    double sb = rud->tgSpeed + rudderOutput;
    rud->speedSb = constrain(sb, rud->minSpeed, rud->maxSpeed);
    rud->speedBb = constrain(bb, rud->minSpeed, rud->maxSpeed);
    if (esc.speedbb != rud->speedBb || esc.speedsb != rud->speedSb)
    {
        esc.speedbb = rud->speedBb;
        esc.speedsb = rud->speedSb;
        xQueueSend(escspeed, (void *)&esc, 10);
    }

}
//***************************************************************************************************
//  speedPID
//***************************************************************************************************
double speedPid(double dist)
{
    speedInput = -dist + 2;
    speedSetpoint = 0;
    speedPID.Compute();
    // printf("tgDist: %05.2f tgSpeed: %05.2f     ", dist, speedOutput);
    // printf("ps:%0.2f is:%0.2f ds:%0.2f\r\n", speedPID.GetKp(), speedPID.GetKi(), speedPID.GetKd());
    return speedOutput;
}
