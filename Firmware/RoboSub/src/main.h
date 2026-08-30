#ifndef MAIN_H_
#define MAIN_H_

#include <RoboCompute.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

extern SemaphoreHandle_t mainDataMutex;

#include "../../RoboDependency\RobobuoyVersion.h"

extern RoboStruct mainData;

// ---------------------------------------------------------------------------------------------
// MAN CAL session (the Sub's own /mancal page).
//
// That page drives the buoy on its own, with no Top involved. Two of the Sub's safety timers are
// fed exclusively by serial traffic FROM the Top, so an untended Sub would fight the operator:
//   - the 5 s serial watchdog forces IDLING, stopping the pivot mid-measurement;
//   - PwrOff is only ever refreshed by an incoming serial frame, so after POWEROFFTIME the buoy
//     pulls PWRENABLE low and switches itself off partway through a calibration.
// While a session is alive both are held off and PWRENABLE is actively kept high.
//
// The session is kept alive by the page's own polling rather than latched on, so a browser that
// crashes or drops off the WiFi cannot pin the buoy powered-on and steerable indefinitely: it
// expires by itself, idles the thrusters and hands control back to the normal timers.
// ---------------------------------------------------------------------------------------------
void mancalSessionBegin();
void mancalSessionPing();
void mancalSessionEnd();
bool mancalSessionAlive();

// Harmonic hold watchdog: the buoy guarantees for itself that its correction comes back on after
// an abandoned calibration, so none of the four front ends has to be trusted to do it. Armed by
// answering a table GET, disarmed by a table SET. See the block comment in main.cpp.
void mancalHoldArm();
void mancalHoldDisarm();
void mancalHoldService();

#endif /* MAIN_H_ */