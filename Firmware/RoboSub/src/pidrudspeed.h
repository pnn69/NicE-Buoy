#ifndef PIDRUDSPEED_H
#define PIDRUDSPEED_H
#include <PID_v1.h>
#include <RoboCompute.h>
// Rudder Speed PID Controller Header
extern PID rudderPID;
extern PID speedPID;
extern double rudderOutput;

void resetRudPid(void);
void resetSpeedPid(void);
void initRudPid(RoboStruct *rud);
void initSpeedPid(RoboStruct *speed);

void rudderPid(RoboStruct *rud);
void speedPid(RoboStruct *speed);

// Sub Status constants for drift-pivot-lock logic
#define SUB_STATUS_IDLE_DRIFT 0 // < 1m, motors off
#define SUB_STATUS_PIVOT_PREP 1 // 1m to minOfsetDist, pivot only
#define SUB_STATUS_LOCKED     2 // >= minOfsetDist, active holding

#endif // PIDRUDSPEED_H