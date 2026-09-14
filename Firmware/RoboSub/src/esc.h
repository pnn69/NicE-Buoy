#ifndef ESC_H_
#define ESC_H_
#include "fastled.h"

extern QueueHandle_t escspeed;
typedef struct Message
{
    int speedbb;
    int speedsb;
} Message;

// Nominal ESC stop pulse, and how far either thruster's trim may sit from it. See memEscNeutral().
#define ESC_NEUTRAL_NOMINAL_US 1500
#define ESC_NEUTRAL_MIN_US     1400
#define ESC_NEUTRAL_MAX_US     1600

// Live trim, loaded from NVS at boot. subwifi.cpp writes these from /setparam.
extern int esc_neutral_bb;
extern int esc_neutral_sb;

int escActualPulseBb(void);
int escActualPulseSb(void);

// ---------------------------------------------------------------------------------------------
// Thruster cleaning.
//
// Weed and plastic collect in the props on a long run, and a fouled thruster cannot hold station:
// it delivers less thrust than the PID asked for, in a direction that depends on what is wrapped
// round it. The cure is mechanical - run it hard astern to throw the debris off the blades, then
// hard ahead to clear what the reverse pulled in, then each side on its own so a single blocked
// prop cannot be masked by the other one pushing.
//
// Driven as a step machine from the main loop, NOT as a delay chain. The steps add up to about
// ten seconds and loop() is also feeding the compass, the telemetry and the serial watchdog in
// that time; blocking through it would drop the heading stream and hand the Top a buoy that had
// apparently stopped talking.
//
// The buoy reports CLEANING while this runs and goes back to the status it had when it started -
// see CLEAN_THRUSTERS in RoboCompute.h.
//
// Bring the ESCs up, without putting a speed in the queue to do it. EscTask wakes on RECEIVING a
// non-zero speed, and escspeed is ten deep while IDLE pushes a zero into it on every pass of
// loop() - so a wake that has to queue behind that arrives late or not at all. Anything that needs
// the thrusters powered before it has a speed to ask for calls this instead.
void escRequestWake(void);

// Waking the ESCs is part of the sequence, not the caller's problem. EscTask drops their supply
// after 30 s of stop and bringing it back takes about 3.5 s of blocking arming inside that task,
// during which every pulse written is neutral whatever is in the queue - so a run started on a
// buoy that has been sat still waits for them rather than counting its 2 s bursts against a
// thruster that is not listening yet.
// ---------------------------------------------------------------------------------------------
void cleanStart(void);
bool cleanActive(void);
void cleanAbort(void);
// Advance the sequence and write the thrusters. Returns true while it is still running, false on
// the pass that finishes it.
bool cleanService(int *speedBbOut, int *speedSbOut);

void initescqueue(void);
void startESC(void);
void beepESC(void);
void triggerESC(void);
void EscTask(void *arg);

#endif /* ESC_H_ */
