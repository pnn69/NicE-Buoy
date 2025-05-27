#ifndef PIDRUDSPEED_H
#define PIDRUDSPEED_H
#include <PID_v1.h>
#include <RoboCompute.h>
// Rudder Speed PID Controller Header
extern PID rudderPID;
extern PID speedPID;
extern double rudderOutput;

void initRudPid(void);
void initSpeedPid(void);

void rudderPid(RoboStruct *rud);
void speedPid(RoboStruct *speed);

#endif // PIDRUDSPEED_H