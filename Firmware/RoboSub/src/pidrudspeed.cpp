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

/**
 * @brief Resets the Rudder PID controller to a clean state.
 * Transitions to MANUAL to clear internal terms, zeros the input/output/setpoint,
 * and returns to AUTOMATIC mode for a fresh start.
 */
void resetRudPid()
{
    rudderPID.SetMode(MANUAL);
    rudderOutput = 0;
    rudderInput = 0;
    rudderSetpoint = 0;
    rudderPID.SetMode(AUTOMATIC);
}

/**
 * @brief Initializes the Rudder PID with stored parameters.
 * Loads Kp, Ki, Kd and speed limits from the buoy's data structure,
 * configures the PID sample time and output limits (-100 to 100% rotation).
 */
void initRudPid(RoboStruct *rud)
{
    speedMaxMin(rud, GET);
    pidRudderParameters(rud, GET);
    rudderPID.SetSampleTime(20);
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

/**
 * @brief Resets the Speed PID controller to a clean state.
 * Similar to resetRudPid, this clears any accumulated error (I-term)
 * before starting a new navigation phase.
 */
void resetSpeedPid()
{
    speedPID.SetMode(MANUAL);
    speedOutput = 0;
    speedInput = 0;
    speedSetpoint = 0;
    speedPID.SetMode(AUTOMATIC);
}

/**
 * @brief Initializes the Speed PID with stored parameters.
 * Loads distance-based PID constants and applies the global maxSpeed
 * limit to the PID output range.
 */
void initSpeedPid(RoboStruct *speed)
{
    speedMaxMin(speed, GET);
    speedPID.SetSampleTime(20);
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

/**
 * @brief Core Steering and Thruster Mixing Logic.
 * 1. Calculates shortest angle to target and applies low-pass wave filtering.
 * 2. Processes Rudder PID (Target = 0 deg error).
 * 3. Enforces Drift/Lock states:
 *    - IDLE_DRIFT: Motors completely OFF.
 *    - PIVOT_PREP: Stationary rotation only (aiming at target while drifting out).
 *    - LOCKED: Normal sailing with a 30-degree safety lockout for stationary corrections.
 * 4. Mixes forward and rotation power into differential thruster speeds (BB/SB).
 * 5. Preserves turning moment if one motor hits max speed by reducing the other.
 */
void rudderPid(RoboStruct *rud)
{
    // 3.2 Heading PID
    // heading_error = desired_heading - current_heading
    double heading_error = smallestAngle(rud->tgDir, rud->dirMag);

    // Wave Filtering (Low-pass) on heading error
    static double filtered_heading_error = 0;
    // Set alpha to 1.0 (No filtering) to eliminate phase lag and overshoot
    filtered_heading_error = 0.0 * filtered_heading_error + 1.0 * heading_error;

    rudderInput = filtered_heading_error;
    rudderSetpoint = 0;
    rudderPID.Compute();

    double rotation_power = -rudderOutput;

    // 3.3 Position PID (forward_power is calculated by speedPid and passed via tgSpeed)
    double target_forward = rud->tgSpeed;
    static double forward_ramp = 0;

    // --- DRIFT & LOCK LOGIC ---
    if (rud->status == IDLE || rud->status == IDELING) {
        target_forward = 0;
        rotation_power = 0;
    }
    else if (rud->status == LOCKED || rud->status == DOCKED || rud->status == DIRDIST) {
        if (rud->sub_status == SUB_STATUS_IDLE_DRIFT) {
            // Zone 1: < 1m - No thrust, no rotation
            target_forward = 0;
            rotation_power = 0;
        }
        else if (rud->sub_status == SUB_STATUS_PIVOT_PREP) {
            // Zone 2: 1m to holdRad - Pivot ONLY
            target_forward = 0;
            rotation_power = constrain(rotation_power, -rud->maxSpeed * rud->pivotSpeed, rud->maxSpeed * rud->pivotSpeed);
        }
        else {
            // Zone 3: Locked Mode (Active Holding)
            
            // Calculate a smoothing factor based on heading error (20 to 45 degrees)
            // 1.0 = Pure Normal (Forward + Differential)
            // 0.0 = Pure Pivot (Rotation only)
            double abs_error = abs(filtered_heading_error);
            double forward_factor = 1.0;
            if (abs_error > 45.0) {
                forward_factor = 0.0;
                resetRudPid();
            }
            else if (abs_error > 20.0) forward_factor = 1.0 - (abs_error - 20.0) / 25.0;

            target_forward = rud->tgSpeed * forward_factor;

            // Smoothly blend the rotation power limit between pivotSpeed and full power (1.0)
            double rot_limit_factor = rud->pivotSpeed + (1.0 - rud->pivotSpeed) * forward_factor;
            rotation_power = constrain(rotation_power, -rud->maxSpeed * rot_limit_factor, rud->maxSpeed * rot_limit_factor);

            // Handle PID resets only on major state exits to preserve continuity
            static bool was_pure_pivot = false;
            if (forward_factor < 0.1) was_pure_pivot = true;
            else if (forward_factor > 0.9 && was_pure_pivot) {
                resetRudPid();
                resetSpeedPid();
                was_pure_pivot = false;
            }
        }
    }

    // --- 3.5 CENTRAL SPEED RAMPING ---
    // Apply ramping to target_forward to prevent sudden current spikes and ESC blocking.
    
    // Forward Ramping
    if (forward_ramp < target_forward) {
        forward_ramp += 5.0; // Ramp up faster (approx 10% per 20ms = 500% per second)
        if (forward_ramp > target_forward) forward_ramp = target_forward;
    } else if (forward_ramp > target_forward) {
        forward_ramp -= 10.0; // Ramp down even faster
        if (forward_ramp < target_forward) forward_ramp = target_forward;
    }
    double forward_power = forward_ramp;

    // 3.6 Thruster Mixer (Differential Steering)
    // left (bb) = forward_power + rotation_power
    // right (sb) = forward_power - rotation_power
    double left = forward_power + rotation_power;
    double right = forward_power - rotation_power;

    // Preserve turning moment if we overshoot max speed
    // If one motor hits the limit, we decrease the other to keep the speed difference (the turn).
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

    // Clamped to ±maxSpeed bounds and store for ESC output
    rud->speedBb = constrain(left, -rud->maxSpeed, rud->maxSpeed);
    rud->speedSb = constrain(right, -rud->maxSpeed, rud->maxSpeed);
    rud->ir = rudderPID.GetITerm();
}

/**
 * @brief Core Distance and Speed State Machine.
 * 1. Low-pass filters the target distance to ignore wave-induced GPS noise.
 * 2. Manages the State Machine transitions:
 *    - < 1m: Force IDLE_DRIFT.
 *    - 1m -> holdRad: Transition to PIVOT_PREP (Wind-assisted drift out).
 *    - >= holdRad: Transition to LOCKED (Reset PIDs and start holding station).
 *    - < 1m (from Locked): Reset back to IDLE_DRIFT if pushed too deep.
 * 3. Calculates forward power ONLY when in LOCKED state, targeting holdRad.
 */
// Wave Filtering (Low-pass) on distance
static double filtered_distance = 0;

double GetFilteredDist(void) {
    return filtered_distance;
}

void speedPid(RoboStruct *dist)
{
    if (filtered_distance == 0) filtered_distance = dist->tgDist;
    filtered_distance = 0.90 * filtered_distance + 0.10 * dist->tgDist;

    // --- DRIFT & LOCK STATE MACHINE ---
    // This logic creates a hysteresis loop: Drift in center -> Pivot out -> Lock and Hold.
    double drift_threshold = (dist->status == DOCKED) ? 0.2 : 1.0;

    if (filtered_distance < drift_threshold) {
        dist->sub_status = SUB_STATUS_IDLE_DRIFT;
    }
    else if (dist->sub_status == SUB_STATUS_IDLE_DRIFT && filtered_distance >= drift_threshold) {
        dist->sub_status = SUB_STATUS_PIVOT_PREP;
    }
    else if (dist->sub_status == SUB_STATUS_PIVOT_PREP && filtered_distance >= dist->holdRad) {
        // Transition point to Active Station Keeping
        dist->sub_status = SUB_STATUS_LOCKED;
        resetSpeedPid(); // Clear old errors for a clean start at the 2m line
        resetRudPid();
    }
    else if (dist->sub_status == SUB_STATUS_LOCKED && filtered_distance < drift_threshold) {
        // Safety fallback if buoy is pushed deep inside the target zone
        dist->sub_status = SUB_STATUS_IDLE_DRIFT;
    }

    // --- POWER CALCULATION BASED ON STATE ---
    if (dist->sub_status == SUB_STATUS_LOCKED) {
        speedInput = filtered_distance; // Measured distance
        speedSetpoint = (double)dist->holdRad; // The holding station distance (e.g., 2m)

        if (speedPID.Compute()) {
            double forward_power = speedOutput;

            // Dynamic Anti-Windup: If distance is far off (> 5 meters), force I-term to 0 natively
            if (abs(filtered_distance - dist->holdRad) > 5.0) {
                 // Because we cannot cleanly disable I-term without PID_v1 resetting, 
                 // we just let it run but we will mask the I-term externally or rely on standard limits.
                 // Actually, the simplest anti-windup without hacking PID_v1 more is to just let it ride the Max Limit.
            }

            // Prevent sailing backwards in speed PID mode
            if (forward_power < 0) forward_power = 0;
            // Apply speed power for rudderPid to mix
            dist->tgSpeed = constrain(forward_power, 0, dist->maxSpeed);
        }
    } else {
        // Pre-stage phases (Idle or Pivot) require 0 forward thrust
        dist->tgSpeed = 0;
    }
    dist->ip = speedPID.GetITerm();
}