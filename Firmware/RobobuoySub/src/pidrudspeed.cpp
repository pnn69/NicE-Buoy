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
    // 3.2 Heading PID
    // heading_error = desired_heading - current_heading
    double heading_error = smallestAngle(rud->tgDir, rud->dirMag);

    // Wave Filtering (Low-pass) on heading error
    static double filtered_heading_error = 0;
    // Low pass filter, cutoff ~0.3-0.5 Hz (alpha ~0.1 at 50ms dt)
    filtered_heading_error = 0.90 * filtered_heading_error + 0.10 * heading_error;

    rudderInput = filtered_heading_error;
    rudderSetpoint = 0;
    rudderPID.Compute();

    double rotation_power = rudderOutput;

    // 3.3 Position PID (forward_power is calculated by speedPid and passed via tgSpeed)
    double forward_power = rud->tgSpeed;

    // 🚫 3.4 IMPORTANT: ±30° Heading Lockout
    // Prevents sideways drifting:
    if (abs(filtered_heading_error) > 30.0) {
        forward_power = 0;
        // Limit rotation speed during stationary turns to prevent aggressive spinning
        rotation_power = constrain(rotation_power, -rud->maxSpeed * rud->pivotSpeed, rud->maxSpeed * rud->pivotSpeed);
    }

    // 3.6 Thruster Mixer
    // left (bb) = forward_power + rotation_power
    // right (sb) = forward_power - rotation_power
    double left = forward_power + rotation_power;
    double right = forward_power - rotation_power;

    // Preserve turning moment if we overshoot max speed
    if (left > rud->maxSpeed) {
        double overshoot = left - rud->maxSpeed;
        left = rud->maxSpeed;
        right -= overshoot;
    } else if (left < -rud->maxSpeed) {
        double overshoot = -rud->maxSpeed - left;
        left = -rud->maxSpeed;
        right += overshoot;
    }

    if (right > rud->maxSpeed) {
        double overshoot = right - rud->maxSpeed;
        right = rud->maxSpeed;
        left -= overshoot;
    } else if (right < -rud->maxSpeed) {
        double overshoot = -rud->maxSpeed - right;
        right = -rud->maxSpeed;
        left += overshoot;
    }

    // Clamped to ±100% (or maxSpeed bounds)
    rud->speedBb = constrain(left, -rud->maxSpeed, rud->maxSpeed);
    rud->speedSb = constrain(right, -rud->maxSpeed, rud->maxSpeed);
    rud->ir = rudderPID.GetITerm();
}

//***************************************************************************************************
//  speedPID
//***************************************************************************************************
void speedPid(RoboStruct *dist)
{
    // Wave Filtering (Low-pass) on distance
    static double filtered_distance = 0;
    if (filtered_distance == 0) filtered_distance = dist->tgDist;
    filtered_distance = 0.90 * filtered_distance + 0.10 * dist->tgDist;

    speedInput = filtered_distance; // Measured distance
    speedSetpoint = dist->minOfsetDist; // Use configured target distance

    if (speedPID.Compute())
    {
        double forward_power = speedOutput;

        // Prevent sailing backwards in speed PID mode
        if (forward_power < 0) {
            forward_power = 0;
        }

        // 3.5 Dead Zones
        if (filtered_distance < 1.0) {
            forward_power = 0; // 0-1m -> no thrust
        }
        else if (filtered_distance < 3.0) {
            // 1-3 m -> soft corrections
            forward_power = constrain(forward_power, 0, (double)dist->maxSpeed * 0.3);
        }
        else {
            // > 3 m -> strong corrections
            forward_power = constrain(forward_power, 0, dist->maxSpeed);
        }

        dist->tgSpeed = forward_power;
        dist->ip = speedPID.GetITerm();
    }
}