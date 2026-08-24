#ifndef GPSSIM_H_
#define GPSSIM_H_
#include "main.h"

// Bench simulator for the GPS Fourier calibration, see gpscalib.cpp.
//
// The calibration cannot be tested on a bench: six of its eight phases need the buoy to turn onto
// a heading and then travel 100 m, and with the hull indoors neither the compass nor the GPS ever
// changes. Every run therefore dies in GC_SETTLE waiting for a heading it will never reach, which
// tells you nothing about the arithmetic that follows.
//
// This module closes that loop entirely inside the Top. Once armed it takes over the four fields
// the state machine reads about the outside world - lat, lng, gpsFix and dirMag - and integrates
// them from the heading orders the calibration itself is issuing. The run then executes end to
// end, in minutes rather than half an hour, with a compass deviation curve you chose yourself, so
// the table it builds can be checked against a known answer instead of against the sea.
//
// Nothing reaches the water while it is armed: sendLegCommand() does not queue its TGDIRSPEED to
// the Sub, so the thrusters are never asked to turn, and sendTable() does not write the computed
// table to the Sub, so a bench run cannot overwrite a real calibration.

// Arms or disarms the simulator.
//   c0  - constant deviation, degrees. Survives pair averaging.
//   a1  - one-cycle (hard-iron-like) amplitude, degrees. Pair averaging is blind to this by
//         design, so it is what tells the two modes apart: still water should find it, current
//         mode should not.
//   a2  - two-cycle (soft-iron-like) amplitude, degrees. Both modes should recover this.
//   mps - metres per second at 100% thrust. The legs are 100 m, so this sets the run time:
//         0.15 gives 7.5 m/s at the calibration's 50% setting, i.e. about 13 s per leg.
// Passing on == false disarms and restores the real receiver and compass.
void gpsSimSet(bool on, double c0, double a1, double a2, double mps);

// True while the simulator owns the position and heading fields.
bool gpsSimActive(void);

// The deviation the simulated compass adds at a given true heading, in degrees. The calibration
// should recover the negative of this, so it is also the answer key for a finished run.
double gpsSimDeviation(double trueHeading);

// Called by sendLegCommand() with every heading order the calibration issues.
void gpsSimNoteCommand(double heading, int speedPct);

// Called once per main loop pass, after handleSerialData() has written the real compass heading,
// so the simulated one replaces it rather than the other way round.
void gpsSimUpdate(RoboStruct *d);

// One line for the dashboard: armed state, simulated true heading, and what the compass is
// reporting for it. Empty when disarmed.
const char *gpsSimReport(void);

#endif /* GPSSIM_H_ */
