#include <Arduino.h>
#include <math.h>
#include <RoboCompute.h>
#include "main.h"
#include "gpssim.h"

//***************************************************************************************************
//  Simulated boat
//***************************************************************************************************
// A buoy pivoting on the spot makes almost no headway, so forward speed is scaled by the cosine of
// the heading error. Without that the hull would sail a full leg's worth of distance during every
// settle phase as well, and eight of those add up past GC_MAX_HOME_DIST - the runaway guard would
// abort the run before the arithmetic under test ever ran.
#define SIM_TURN_RATE      15.0     // deg/s, roughly what the real hull manages on thrusters alone
#define SIM_MAX_RUNTIME    2700000UL // 45 min, then it disarms itself - see the note in gpsSimUpdate

static bool   active     = false;
static bool   needSeed   = true;
static double devC0 = 0.0, devA1 = 0.0, devA2 = 0.0;
static double mpsPerPct  = 0.15;

static double simTrue    = 0.0;  // heading the hull really points, degrees
static double simLat     = 0.0;
static double simLon     = 0.0;
static double cmdHdg     = 0.0;
static int    cmdSpeed   = 0;

static unsigned long lastTick = 0;
static unsigned long armedAt  = 0;
static char   report[112] = "";

static double wrap360(double a)
{
    while (a >= 360.0) a -= 360.0;
    while (a < 0.0)    a += 360.0;
    return a;
}

static double wrap180(double a)
{
    while (a >  180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}

double gpsSimDeviation(double trueHeading)
{
    double r = trueHeading * DEG_TO_RAD;
    return devC0 + devA1 * sin(r) + devA2 * sin(2.0 * r);
}

void gpsSimSet(bool on, double c0, double a1, double a2, double mps)
{
    if (on)
    {
        devC0 = c0;
        devA1 = a1;
        devA2 = a2;
        if (mps > 0.0) mpsPerPct = mps;
        needSeed = true;
        armedAt  = millis();
        lastTick = 0;
        cmdHdg   = 0.0;
        cmdSpeed = 0;
        active   = true;
        printf("#GPSSIM: ARMED - deviation %.2f + %.2f*sin(t) + %.2f*sin(2t) deg, %.2f m/s per %%\r\n",
               devC0, devA1, devA2, mpsPerPct);
        printf("#GPSSIM: the Sub will NOT be commanded and its table will NOT be written\r\n");
    }
    else if (active)
    {
        active = false;
        report[0] = '\0';
        printf("#GPSSIM: DISARMED - real GPS and compass are back in charge\r\n");
    }
}

bool gpsSimActive(void) { return active; }

const char *gpsSimReport(void) { return report; }

void gpsSimNoteCommand(double heading, int speedPct)
{
    cmdHdg   = wrap360(heading);
    cmdSpeed = speedPct;
}

void gpsSimUpdate(RoboStruct *d)
{
    if (!active || d == NULL) return;

    // A Top left in simulation would look completely healthy while reporting a position it made
    // up, which is the one failure mode of this module that could actually matter on the water.
    // It therefore has a hard expiry rather than relying on somebody remembering to disarm it.
    if (millis() - armedAt > SIM_MAX_RUNTIME)
    {
        printf("#GPSSIM: expired after 45 minutes, disarming\r\n");
        gpsSimSet(false, 0, 0, 0, 0);
        return;
    }

    unsigned long now = millis();

    if (needSeed)
    {
        // Start from wherever the buoy really is, so the plot and the runaway guard both behave
        // exactly as they would afloat. A Top that has never had a fix starts on its own doorstep
        // rather than at 0,0 - null island would put the first leg 5800 km from home.
        simLat = (d->gpsFix && d->lat != 0.0) ? d->lat : 52.320382;
        simLon = (d->gpsFix && d->lng != 0.0) ? d->lng : 4.965454;
        // The reported heading is what the rest of the firmware has been looking at, so hold it
        // steady across the handover by solving back to the true heading that produces it.
        simTrue  = wrap360(d->dirMag - gpsSimDeviation(d->dirMag));
        needSeed = false;
        lastTick = now;
        printf("#GPSSIM: seeded at %.8f, %.8f heading %.1f (compass reads %.1f)\r\n",
               simLat, simLon, simTrue, wrap360(simTrue + gpsSimDeviation(simTrue)));
    }

    double dt = (now - lastTick) / 1000.0;
    lastTick = now;
    if (dt <= 0.0) return;
    if (dt > 1.0) dt = 1.0;   // a long scheduling gap must not teleport the hull

    // The Sub steers by its compass, so it turns until the DEVIATED reading matches the order.
    // One fixed-point step is plenty: the deviation is a few degrees and its slope is far below 1.
    double trueTarget = wrap360(cmdHdg - gpsSimDeviation(cmdHdg));
    double diff = wrap180(trueTarget - simTrue);
    double maxStep = SIM_TURN_RATE * dt;
    if (fabs(diff) <= maxStep) simTrue = trueTarget;
    else                       simTrue = wrap360(simTrue + (diff > 0 ? maxStep : -maxStep));

    double headErr = wrap180(trueTarget - simTrue);
    double reach = cos(headErr * DEG_TO_RAD);
    if (reach < 0.0) reach = 0.0;
    double v = (double)cmdSpeed * mpsPerPct * reach;   // m/s

    double step = v * dt;
    if (step > 0.0)
    {
        double r = simTrue * DEG_TO_RAD;
        simLat += (step * cos(r)) / 111320.0;
        double coslat = cos(simLat * DEG_TO_RAD);
        if (fabs(coslat) > 1e-6) simLon += (step * sin(r)) / (111320.0 * coslat);
    }

    // Take over the four fields the calibration reads about the outside world. dirMag is written
    // here rather than in handleSerialData() so that the real Sub can keep reporting normally -
    // its frames still arrive and still prove the serial link is alive, they are simply overruled.
    double reported = wrap360(simTrue + gpsSimDeviation(simTrue));
    d->lat    = simLat;
    d->lng    = simLon;
    d->gpsFix = true;
    d->gpsSat = 12;
    d->dirMag = reported;

    snprintf(report, sizeof(report), "SIM cmd %03.0f true %05.1f compass %05.1f dev %+0.1f %0.1fm/s",
             cmdHdg, simTrue, reported, gpsSimDeviation(simTrue), v);
}
