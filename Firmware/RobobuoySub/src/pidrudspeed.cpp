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

void resetRudPid()
{
    rudderPID.SetMode(MANUAL);
    rudderOutput = 0;
    rudderInput = 0;
    rudderSetpoint = 0;
    rudderPID.SetMode(AUTOMATIC);
}

void initRudPid(RoboStruct *rud)
{
    speedMaxMin(rud, GET);
    pidRudderParameters(rud, GET);
    rudderPID.SetSampleTime(100);
    rudderPID.SetTunings(rud->Kpr, rud->Kir, rud->Kdr, P_ON_E);
    rudderPID.SetOutputLimits(-100, 100);
    resetRudPid();
    rud->ir = rudderPID.GetITerm();
    Serial.println("PID rudder used for calculations> pr:" + String(rud->Kpr, 2) +
                   " ir:" + String(rud->Kir, 2) +
                   " dr:" + String(rud->Kdr, 2) +
                   " minSpeed:" + String(rud->minSpeed) +
                   " maxSpeed:" + String(rud->maxSpeed));
}

void resetSpeedPid()
{
    speedPID.SetMode(MANUAL);
    speedOutput = 0;
    speedInput = 0;
    speedSetpoint = 0;
    speedPID.SetMode(AUTOMATIC);
}
void initSpeedPid(RoboStruct *speed)
{
    speedMaxMin(speed, GET);
    speedPID.SetSampleTime(100);
    pidSpeedParameters(speed, GET);
    speedPID.SetTunings(speed->Kps, speed->Kis, speed->Kds, P_ON_E);
    computeParameters(speed, GET);
    speedPID.SetOutputLimits(0, speed->maxSpeed); // Use actual maxSpeed limit
    resetSpeedPid();
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

    // Constrain speed
    double s = constrain(rud->tgSpeed, rud->minSpeed, rud->maxSpeed);

    // Scale rudder effect based on speed
    const double scale = 50.0;
    const double baseGain = 50.0;
    const double minTurningAuthority = 10.0; // Ensure some turning even when stationary

    // Compute a dynamic gain that decreases at high speeds but has a floor
    double range = rud->maxSpeed - rud->minSpeed;
    double speedRatio = (range > 0) ? (s - rud->minSpeed) / range : 0.0;
    
    // Provide turning authority even if speed is 0
    double dynamicGain = baseGain * (0.3 + 0.7 * speedRatio); 

    double rudderAdj = tanh(rudderOutput / scale) * dynamicGain;

    // Apply the rudder adjustment (differential thrust)
    double bb = s - rudderAdj;
    double sb = s + rudderAdj;
    
    // Allow speed to slightly exceed min/max for turning if necessary, but keep safe
    rud->speedSb = constrain(sb, -rud->maxSpeed, rud->maxSpeed);
    rud->speedBb = constrain(bb, -rud->maxSpeed, rud->maxSpeed);
    rud->ir = rudderPID.GetITerm();
}

//***************************************************************************************************
//  speedPID
//***************************************************************************************************
void speedPid(RoboStruct *dist)
{
    speedInput = dist->tgDist; // Measured distance
    speedSetpoint = dist->minOfsetDist; // Use configured target distance
    if (speedPID.Compute())
    {
        // Prevent sailing backwards in speed PID mode
        if (speedOutput < 0)
            speedOutput = 0;

        dist->tgSpeed = speedOutput;
        dist->ip = speedPID.GetITerm();
    }
}