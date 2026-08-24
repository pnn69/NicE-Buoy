#include <Arduino.h>
#include <RoboCompute.h>
#include "main.h"
#include "gpscalib.h"
#include "gps.h"
#include "sercom.h"
#include "topwifi.h"
#include "loratop.h"
#include "buzzer.h"
#include "gpssim.h"
#include "udplog.h"

//***************************************************************************************************
//  Tunables
//***************************************************************************************************
#define GC_SPEED              50      // thruster setting for every leg, same as the other in-field routines
#define GC_SETTLE_TOL         5.0     // deg, heading must be this close to the commanded one ...
#define GC_SETTLE_HOLD        20000UL // ... for this long before a measurement starts
#define GC_SETTLE_TIMEOUT     180000UL
#define GC_MIN_LEG_DIST       100.0   // m, minimum displacement per leg (spec recommends 100-150 m)
#define GC_LEG_TIMEOUT        420000UL
#define GC_RUN_TOL            20.0    // deg, heading excursion during a run that invalidates it ...
#define GC_RUN_TOL_HOLD       15000UL // ... if it lasts this long
#define GC_MAX_LEG_RETRIES    3
#define GC_CMD_INTERVAL       500     // ms between TGDIRSPEED refreshes
#define GC_TABLE_TIMEOUT      4000UL  // ms to wait for the Sub to answer about its table
#define GC_TABLE_TRIES        5
#define GC_MAX_HOME_DIST      1500.0  // m, runaway guard
#define GC_GPS_LOST_TIMEOUT   10000UL // ms without a fix that aborts the run
#define GC_HEADING_STUCK_TIMEOUT 60000UL // ms of a bit-identical dirMag that aborts the run
// Progress broadcast rates. UDP is cheap so the dashboard stays smooth; LoRa is shared, slow and
// already carrying BUOYPOS + TOPDATA every second while the buoy is not idle, so it gets the
// slower rate. A 30 minute run is ~360 extra LoRa frames at 5 s.
#define GC_REPORT_UDP_MS      1000UL
#define GC_REPORT_LORA_MS     5000UL
#define GC_REPORT_MIN_DIST    10.0    // m of displacement before a live leg bearing means anything

// Opposite headings back to back. DO NOT reorder this into a plain 0,45,90,...,315 sweep.
//
// The original reason was arithmetic - a steady current cancels within each pair - and that reason
// disappears in still-water mode, where each leg is used on its own. The order stays anyway,
// because the more important reason is geometric: every leg is immediately undone by its opposite,
// so the buoy runs out and straight back along the same line, four times.
//
// That traces a half star centred on the start position, with spokes at 0/45/90/135 only - the
// return legs retrace the outbound ones rather than adding new directions. Simulated with 20 m of
// settle travel plus a 105 m measured leg, the buoy never gets further than ~125 m from home,
// never crosses to the west of it, and finishes where it started:
//
//        N          outbound spokes 0, 45, 90, 135
//        |   /       return  legs  180, 225, 270, 315 retrace them
//        |  /  -     footprint: eastern half disc, radius ~125 m
//        | / /
//   -----H------ E
//        | \
//        |  \
//
// A 0,45,90,... sweep would instead walk the buoy around an arc, ending up far from the start and
// needing several times the clear water. The still-water table mapping is
// perHeading[GC_LEGS[i] / 45], which is order independent, so the ordering costs the maths nothing.
static const int GC_LEGS[8] = {0, 180, 45, 225, 90, 270, 135, 315};

// Deliberately valued from gpscal_step_t in RoboCompute.h rather than numbered here: these go out
// on the wire in every GPS_FOURIER_STATUS report, so the shared header is the single definition
// anything listening (RoboCYD) can be written against.
enum GpsCalStep
{
    GC_IDLE        = GPSCAL_IDLE,
    GC_FETCH_TABLE = GPSCAL_FETCH_TABLE, // ask the Sub which table is currently in effect
    GC_SETTLE      = GPSCAL_SETTLE,      // steer the commanded heading and wait for it to hold
    GC_RUN         = GPSCAL_RUN,         // measure the displacement
    GC_STORE_TABLE = GPSCAL_STORE,       // hand the new table to the Sub
    GC_GO_HOME     = GPSCAL_DONE
};

static GpsCalStep step = GC_IDLE;
static int legIdx = 0;
static int legRetries = 0;

static double homeLat = 0, homeLon = 0;
static double startLat = 0, startLon = 0;

static double legBearing[8] = {0};
static double legError[8] = {0};
static double legDist[8] = {0};

// The table the Sub is running right now. The new one is built on top of it, so a second
// calibration pass refines the first instead of throwing it away.
static float oldTable[8] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 225.0f, 270.0f, 315.0f};
static float newTable[8] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 225.0f, 270.0f, 315.0f};

static unsigned long stepStart = 0;
static unsigned long lastCmdSend = 0;
static unsigned long settleSince = 0;   // 0 = not currently within tolerance
static unsigned long offCourseSince = 0;
static unsigned long lastFixTime = 0;
static unsigned long lastDirMagChange = 0;
static double lastDirMag = 0;
static unsigned long tableSentAt = 0;
static int tableTries = 0;
static bool tableReplyValid = false;    // a STORE_INTERPOLATION_TABLE frame arrived from the Sub
static float tableReply[8] = {0};

static char progressMsg[64] = "";

// Latched from cal->gpsCalStillWater when the run starts, so a setup change mid-run cannot switch
// the interpretation half way through a set of legs.
static bool stillWater = false;

//***************************************************************************************************
//  Helpers
//***************************************************************************************************
/**
 * @brief Wraps an angle difference into (-180, 180].
 */
static double normalizeErr(double error)
{
    while (error > 180.0) error -= 360.0;
    while (error <= -180.0) error += 360.0;
    return error;
}

/**
 * @brief Broadcasts a GPS_FOURIER_STATUS progress report over UDP and LoRa.
 *
 * The run is otherwise a half-hour of silence: the buoy sails away, turns around eight times and
 * eventually comes back, and nothing outside the Top's serial console says which leg it is on or
 * whether it has already given up. This is the report anything on the network can follow.
 *
 * @param cal        current buoy state.
 * @param reportStep phase to report, a gpscal_step_t. Passed in rather than read from `step` so
 *                   the terminal DONE and ABORTED frames can be sent from the code paths that
 *                   have just left the state machine.
 * @param force      send now on both transports, ignoring the rate limits. Used for the terminal
 *                   frames, which must not be lost to a timer that happens not to be due.
 */
static void sendCalibStatus(RoboStruct *cal, int reportStep, bool force)
{
    static unsigned long nextUdp = 0;
    static unsigned long nextLora = 0;
    unsigned long now = millis();

    bool doUdp = force || (long)(now - nextUdp) >= 0;
    bool doLora = force || (long)(now - nextLora) >= 0;
    if (!doUdp && !doLora) return;

    RoboStruct msg = {};
    msg.cmd = GPS_FOURIER_STATUS;
    msg.IDs = cal->mac;
    msg.IDr = BUOYIDALL;
    msg.ack = INF;
    msg.status = cal->status;
    msg.gpsCalStep = reportStep;
    msg.gpsCalAbort = cal->gpsCalAbort;
    // Clamped: legIdx runs to 8 once the last leg is measured, and a report of "leg 9 of 8" during
    // the table transfer helps nobody. Seven is the truthful answer there - it is the leg the buoy
    // last sailed. The error index below deliberately uses the UNCLAMPED value, so the store and
    // done frames still carry leg 7's result rather than leg 6's.
    msg.gpsCalLeg = (legIdx > 7) ? 7 : legIdx;
    // Only meaningful while a leg is being steered; zero elsewhere so a listener does not draw a
    // stale heading over the table-transfer phases.
    msg.tgDir = (reportStep == GPSCAL_SETTLE || reportStep == GPSCAL_RUN) ? (double)GC_LEGS[legIdx] : 0.0;
    msg.dirMag = cal->dirMag;
    msg.gpsCalDist = (reportStep == GPSCAL_RUN && cal->gpsFix)
                         ? distanceBetween(startLat, startLon, cal->lat, cal->lng)
                         : 0.0;

    // Live error of the leg being sailed: the bearing achieved SO FAR against the heading that was
    // commanded. Computed exactly the way the final value is at the end of the leg, so it can be
    // watched converging instead of only appearing once the leg is over.
    //
    // Held at zero below GC_REPORT_MIN_DIST: over the first few metres the start and current fix
    // are within GPS noise of each other and the bearing between them is close to random, which
    // would show as the error swinging through +/-180 for the first half minute. A consumer can
    // tell the two cases apart without a sentinel - gpsCalDist is in the same frame.
    if (reportStep == GPSCAL_RUN && cal->gpsFix && msg.gpsCalDist >= GC_REPORT_MIN_DIST)
    {
        double liveBearing = calculateBearing(startLat, startLon, cal->lat, cal->lng);
        msg.gpsCalErr = normalizeErr(liveBearing - (double)GC_LEGS[legIdx]);
    }
    else if (reportStep == GPSCAL_STORE || reportStep == GPSCAL_DONE)
    {
        // Both report leg 7 in gpsCalLeg, so keep "gpsCalErr is the error of that leg" true: by
        // now it is measured, not live.
        msg.gpsCalErr = legError[7];
    }

    // The last finished leg, so a display still has a result to show through the 20 s settle phase
    // of the next one, when there is no live measurement yet.
    msg.gpsCalLastErr = (legIdx > 0) ? legError[legIdx - 1] : 0.0;

    if (doUdp)
    {
        nextUdp = now + GC_REPORT_UDP_MS;
        xQueueSend(udpOut, (void *)&msg, 0);
    }
    if (doLora)
    {
        nextLora = now + GC_REPORT_LORA_MS;
        xQueueSend(loraOut, (void *)&msg, 0);
    }
}

/**
 * @brief Puts the finished table on the network so the result is visible, not just its effect.
 */
static void broadcastFinalTable(RoboStruct *cal)
{
    RoboStruct msg = {};
    msg.cmd = STORE_INTERPOLATION_TABLE;
    msg.IDs = cal->mac;
    msg.IDr = BUOYIDALL;
    // INF, not SET: this is an announcement. Only the Sub stores tables, and it is told over the
    // serial link. A SET here would enter the LoRa retry table and be retransmitted five times.
    msg.ack = INF;
    msg.status = cal->status;
    for (int i = 0; i < 8; i++) msg.interpolationTable[i] = newTable[i];
    xQueueSend(udpOut, (void *)&msg, 0);
    xQueueSend(loraOut, (void *)&msg, 10);
}

/**
 * @brief Refreshes the heading/speed order the Sub is steering to.
 *
 * TGDIRSPEED is a live order, not a stored one - the Sub keeps the last value it was given, so it
 * has to be repeated. IDr is set explicitly: the Sub only accepts frames addressed to its own MAC
 * or to BUOYIDALL, and a default-constructed RoboStruct carries IDr 0, which it drops.
 */
static void sendLegCommand(RoboStruct *cal, int heading)
{
    RoboStruct cmdMsg = {};
    cmdMsg.cmd = TGDIRSPEED;
    cmdMsg.IDs = cal->mac;
    cmdMsg.IDr = BUOYIDALL;
    cmdMsg.ack = INF;
    cmdMsg.status = TGDIRSPEED;
    cmdMsg.tgDir = (double)heading;
    cmdMsg.speedSet = GC_SPEED;
    // On the bench the order drives the simulated hull instead of the real one. Sending it to the
    // Sub as well would spin the thrusters dry for the whole run.
    if (gpsSimActive())
    {
        gpsSimNoteCommand((double)heading, GC_SPEED);
        return;
    }
    xQueueSend(serOut, (void *)&cmdMsg, 0);
}

/**
 * @brief Cuts the thrusters and drops out of the calibration.
 */
static void abortRun(RoboStruct *cal, const char *reason, gpscal_abort_t why)
{
    printf("#GPSFOURIER: ABORTED - %s\r\n", reason);
    udpLog("GPSCAL ABORTED (%d) %s", (int)why, reason);
    // Put the reason on the wire, not just in progressMsg. progressMsg only ever reached the
    // Top's own web page; over LoRa the CYD could see THAT a run aborted but never why.
    cal->gpsCalAbort = (int)why;
    snprintf(progressMsg, sizeof(progressMsg), "ABORTED: %.40s", reason);
    step = GC_IDLE;
    cal->tgDir = 0;
    cal->tgDist = 0;
    cal->speedSet = 0;
    cal->status = IDLING;
    // Sent after cal->status is already IDLING, so the frame reads "the run stopped and I am now
    // idling" rather than leaving a listener on a calibrating status that never updates again.
    sendCalibStatus(cal, GPSCAL_ABORTED, true);
    beep(-1, buzzer);
}

/**
 * @brief Requests the interpolation table currently in effect on the Sub.
 */
static void requestTable(RoboStruct *cal)
{
    RoboStruct req = {};
    req.cmd = STORE_INTERPOLATION_TABLE;
    req.IDs = cal->mac;
    req.IDr = BUOYIDALL;
    req.ack = GET;
    xQueueSend(serOut, (void *)&req, 10);
    tableSentAt = millis();
    tableTries++;
}

/**
 * @brief Sends the freshly computed table to the Sub for permanent storage.
 */
static void sendTable(RoboStruct *cal)
{
    RoboStruct msg = {};
    msg.cmd = STORE_INTERPOLATION_TABLE;
    msg.IDs = cal->mac;
    msg.IDr = BUOYIDALL;
    msg.ack = SET; // SET also puts it in sercom's retransmit list until the Sub echoes it back
    for (int i = 0; i < 8; i++) msg.interpolationTable[i] = newTable[i];
    tableSentAt = millis();
    tableTries++;
    // A bench run must not overwrite the calibration the buoy actually sails on. Report what it
    // WOULD have stored and fake the Sub's echo, so the run still completes through GC_STORE.
    if (gpsSimActive())
    {
        udpLog("GPSSIM table NOT written: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
               newTable[0], newTable[1], newTable[2], newTable[3],
               newTable[4], newTable[5], newTable[6], newTable[7]);
        printf("#GPSSIM: table NOT written to the Sub. It would have been:");
        for (int i = 0; i < 8; i++) printf(" %.2f", newTable[i]);
        printf("\r\n");
        for (int i = 0; i < 8; i++) tableReply[i] = newTable[i];
        tableReplyValid = true;
        return;
    }
    xQueueSend(serOut, (void *)&msg, 10);
}

/**
 * @brief Turns the eight leg errors into a new interpolation table.
 *
 * Opposite legs are averaged first. A steady current biases the two runs of a pair by equal and
 * opposite amounts (a north run pushed 8 deg east is a south run pushed 8 deg west), so the average
 * keeps the compass deviation and drops the current.
 *
 * NOTE that this is also the method's main limitation: averaging opposite headings removes the
 * whole odd part of the deviation curve, the genuine one-cycle (hard-iron-like) term included.
 * What survives is the constant plus the two-cycle term - which is what installation offset and
 * soft iron actually look like, and what this routine is for.
 */
static void buildTable(void)
{
    double corr[4];
    for (int p = 0; p < 4; p++)
    {
        // GC_LEGS is ordered as pairs, so legs 2p and 2p+1 are always opposite headings.
        corr[p] = (legError[p * 2] + legError[p * 2 + 1]) / 2.0;
    }

    printf("#GPSFOURIER: pair errors  0/180=%+.2f  45/225=%+.2f  90/270=%+.2f  135/315=%+.2f\r\n",
           corr[0], corr[1], corr[2], corr[3]);

    // Diagnostic only - nothing below changes the table.
    //
    // Each pair splits into an even part, (e + e180)/2, which becomes the correction, and an odd
    // part, (e - e180)/2, which is silently discarded. A current produces exactly that odd shape,
    // which is why the averaging works - but so does a residual hard-iron deviation, and one set of
    // legs sailed once cannot tell the two apart. Printing it is the only way to know what this run
    // gave up: near zero means the pair averaging cost nothing, several degrees means the buoy is
    // still that far out on the individual legs afterwards.
    double odd[4];
    double a = 0.0, b = 0.0;
    for (int p = 0; p < 4; p++)
    {
        odd[p] = (legError[p * 2] - legError[p * 2 + 1]) / 2.0;
        a += odd[p] * cos(radians((double)GC_LEGS[p * 2]));
        b += odd[p] * sin(radians((double)GC_LEGS[p * 2]));
    }
    // odd(C) = K*sin(set - C) = a*cos(C) + b*sin(C), with a = K*sin(set) and b = -K*cos(set).
    // cos and sin are orthogonal over the four pair headings 0/45/90/135 and each has a sum of
    // squares of 2, so this is a plain projection rather than a least-squares solve.
    a /= 2.0;
    b /= 2.0;
    double oddAmp = sqrt(a * a + b * b);
    double oddDir = fmod(degrees(atan2(a, -b)) + 360.0, 360.0);

    // How well the four dropped values fit that single sinusoid. A steady current IS one, so a
    // near-zero residual says "this really does look like current". A large one says the odd part
    // is not a one-cycle effect at all - noisy legs, a shifting breeze, or a leg that was not
    // actually held - and the amplitude above is then not worth reading.
    double oddRes = 0.0;
    double oddMax = 0.0;
    for (int p = 0; p < 4; p++)
    {
        double fit = a * cos(radians((double)GC_LEGS[p * 2])) + b * sin(radians((double)GC_LEGS[p * 2]));
        oddRes += (odd[p] - fit) * (odd[p] - fit);
        if (fabs(odd[p]) > oddMax) oddMax = fabs(odd[p]);
    }
    oddRes = sqrt(oddRes / 4.0);

    printf("#GPSFOURIER: pair split - even part is the same at both headings, odd part is not\r\n");
    for (int p = 0; p < 4; p++)
    {
        printf("#GPSFOURIER:   %3d/%3d legs %+6.2f /%+6.2f  ->  even %+6.2f  odd %+6.2f\r\n",
               GC_LEGS[p * 2], GC_LEGS[p * 2 + 1], legError[p * 2], legError[p * 2 + 1],
               corr[p], odd[p]);
    }
    // oddMax, not oddAmp, for the practical statement: the fitted amplitude is the best single
    // sinusoid through the four odd values, so when they do NOT lie on one it understates the worst
    // leg badly. Odd values of [4,0,0,0] fit an amplitude of 2.0 while one leg is really 4.0 out.
    printf("#GPSFOURIER: one-cycle term: worst leg %.2f\xC2\xB0, fits %.2f\xC2\xB0 towards %.0f\xC2\xB0 "
           "(residual %.2f\xC2\xB0)\r\n", oddMax, oddAmp, oddDir, oddRes);
    if (stillWater)
    {
        printf("#GPSFOURIER:   KEPT as hard iron. If there was in fact a current, up to %.2f\xC2\xB0 of "
               "it has just been written into the compass table\r\n", oddMax);
    }
    else
    {
        printf("#GPSFOURIER:   DISCARDED as current. A low residual supports that; in still water "
               "this is hard iron and each leg stays up to %.2f\xC2\xB0 out\r\n", oddMax);
    }

    // The correction to apply at each of the eight table headings.
    //
    // Pair-averaged mode gives both headings of a pair the same value, so the one-cycle component
    // cancels out by construction. Still-water mode uses each leg's own error, which keeps it.
    double perHeading[8];
    if (stillWater)
    {
        // Legs are sailed as 0,180,45,225,... but the table is indexed by reference angle, so
        // leg i belongs at table slot GC_LEGS[i] / 45.
        for (int i = 0; i < 8; i++) perHeading[GC_LEGS[i] / 45] = legError[i];
        printf("#GPSFOURIER: STILL WATER mode - each leg corrected on its own measurement, the\r\n");
        printf("#GPSFOURIER:   one-cycle (hard iron) term is KEPT rather than treated as current\r\n");
    }
    else
    {
        for (int i = 0; i < 8; i++) perHeading[i] = corr[i % 4];
        printf("#GPSFOURIER: PAIR AVERAGED mode - opposite legs share a correction, so a steady\r\n");
        printf("#GPSFOURIER:   current cancels and the one-cycle term is discarded with it\r\n");
    }

    // Entry i of the table is the compass reading seen while the buoy really pointed at i*45 deg,
    // so the correction the Sub applies at that heading is (i*45 - table[i]). The measured residual
    // has to be ADDED to whatever correction is already in effect, which is the same as subtracting
    // it from the stored reading. With an identity table in effect and pair averaging on, this
    // reduces exactly to the spec's "measured_angles[i] = ref - corr".
    for (int i = 0; i < 8; i++)
    {
        double v = (double)oldTable[i] - perHeading[i];
        while (v < 0.0) v += 360.0;
        while (v >= 360.0) v -= 360.0;
        newTable[i] = (float)v;
    }

    printf("#GPSFOURIER: old table:");
    for (int i = 0; i < 8; i++) printf(" %.2f", oldTable[i]);
    printf("\r\n#GPSFOURIER: new table:");
    for (int i = 0; i < 8; i++) printf(" %.2f", newTable[i]);
    printf("\r\n");
}

//***************************************************************************************************
//  Public entry points
//***************************************************************************************************
bool gpsCalibActive(void) { return step != GC_IDLE; }

const char *gpsCalibProgress(void) { return progressMsg; }

/**
 * @brief Receives the Sub's answer to a STORE_INTERPOLATION_TABLE request.
 */
void gpsCalibTableReply(const RoboStruct *in)
{
    if (in == NULL) return;
    for (int i = 0; i < 8; i++) tableReply[i] = in->interpolationTable[i];
    tableReplyValid = true;
}

/**
 * @brief Runs the GPS-based Fourier compass calibration state machine.
 *
 * @param cal Pointer to the buoy's main RoboStruct.
 */
void handleGpsFourierCalibration(RoboStruct *cal)
{
    if (cal->status != GPS_FOURIER_CALIBRATE)
    {
        if (step != GC_IDLE)
        {
            // Someone locked, docked or idled us mid-run. Whatever they asked for owns the Sub
            // now, so do not send anything ourselves - just stop.
            printf("#GPSFOURIER: cancelled by a status change to %d\r\n", cal->status);
            progressMsg[0] = '\0';
            step = GC_IDLE;
        }
        return;
    }

    //-----------------------------------------------------------------------------------------------
    // Entry
    //-----------------------------------------------------------------------------------------------
    if (step == GC_IDLE)
    {
        // Cleared before the pre-flight checks, not after: abortRun() broadcasts a status frame,
        // and without this a "cannot start" abort would report the leg number the PREVIOUS run
        // happened to stop on.
        legIdx = 0;
        legRetries = 0;
        cal->gpsCalAbort = GPSCAL_ABORT_NONE;   // a fresh run must not inherit the last reason

        if (!cal->gpsFix)
        {
            abortRun(cal, "no GPS fix", GPSCAL_ABORT_NO_FIX);
            return;
        }
        if (!subSerialAlive())
        {
            // Every heading in this routine comes from the Sub's compass over the serial link.
            // Without it dirMag is a stale number that would happily satisfy the settle test.
            abortRun(cal, "Sub silent on the serial link", GPSCAL_ABORT_SUB_SILENT);
            return;
        }

        homeLat = cal->lat;
        homeLon = cal->lng;
        stillWater = cal->gpsCalStillWater;
        legIdx = 0;
        legRetries = 0;
        tableTries = 0;
        tableReplyValid = false;
        lastFixTime = millis();
        lastDirMag = cal->dirMag;
        lastDirMagChange = millis();
        for (int i = 0; i < 8; i++) { legBearing[i] = 0; legError[i] = 0; legDist[i] = 0; }

        printf("\r\n#GPSFOURIER: ================ START ================\r\n");
        printf("#GPSFOURIER: home %.8f, %.8f\r\n", homeLat, homeLon);
        printf("#GPSFOURIER: 8 legs, min %.0f m each, %d%% thrust\r\n", (double)GC_MIN_LEG_DIST, GC_SPEED);
        printf("#GPSFOURIER: mode: %s\r\n", stillWater ? "STILL WATER (per-leg, corrects hard iron too)"
                                                       : "PAIR AVERAGED (current tolerant)");
        beep(1, buzzer);

        requestTable(cal);
        step = GC_FETCH_TABLE;
        stepStart = millis();
        snprintf(progressMsg, sizeof(progressMsg), "reading current table");
        return;
    }

    //-----------------------------------------------------------------------------------------------
    // Guards that apply to every step
    //-----------------------------------------------------------------------------------------------
    if (cal->gpsFix) lastFixTime = millis();
    else if (millis() - lastFixTime > GC_GPS_LOST_TIMEOUT)
    {
        abortRun(cal, "GPS fix lost", GPSCAL_ABORT_FIX_LOST);
        return;
    }

    if (!subSerialAlive())
    {
        abortRun(cal, "lost the serial link to the Sub", GPSCAL_ABORT_LINK_LOST);
        return;
    }

    // subSerialAlive() alone is not proof that the compass is alive. The Top and Sub share one
    // half-duplex wire, so every TGDIRSPEED we send is echoed straight back into our own receive
    // path and refreshes the "last heard from the Sub" timestamp. A hung compass task would then
    // leave dirMag frozen at a value that satisfies the settle test forever.
    if (fabs(cal->dirMag - lastDirMag) > 0.001)
    {
        lastDirMag = cal->dirMag;
        lastDirMagChange = millis();
    }
    else if (millis() - lastDirMagChange > GC_HEADING_STUCK_TIMEOUT)
    {
        abortRun(cal, "compass heading frozen", GPSCAL_ABORT_HEADING_FROZEN);
        return;
    }

    if (cal->gpsFix && distanceBetween(homeLat, homeLon, cal->lat, cal->lng) > GC_MAX_HOME_DIST)
    {
        abortRun(cal, "drifted too far from the start position", GPSCAL_ABORT_DRIFTED);
        return;
    }

    // Report before running the step, so the phase and leg on the wire are the ones the buoy is
    // actually in. Rate limited internally.
    sendCalibStatus(cal, step, false);

    switch (step)
    {
    //-----------------------------------------------------------------------------------------------
    case GC_FETCH_TABLE:
    {
        if (tableReplyValid)
        {
            tableReplyValid = false;
            for (int i = 0; i < 8; i++) oldTable[i] = tableReply[i];
            printf("#GPSFOURIER: table in effect on the Sub:");
            for (int i = 0; i < 8; i++) printf(" %.2f", oldTable[i]);
            printf("\r\n");
            legIdx = 0;
            legRetries = 0;
            step = GC_SETTLE;
            stepStart = millis();
            settleSince = 0;
            lastCmdSend = 0;
            break;
        }
        if (millis() - tableSentAt > GC_TABLE_TIMEOUT)
        {
            if (tableTries >= GC_TABLE_TRIES)
            {
                abortRun(cal, "Sub never reported its interpolation table", GPSCAL_ABORT_NO_TABLE);
                return;
            }
            printf("#GPSFOURIER: no table from the Sub, retry %d/%d\r\n", tableTries + 1, GC_TABLE_TRIES);
            requestTable(cal);
        }
        break;
    }

    //-----------------------------------------------------------------------------------------------
    case GC_SETTLE:
    {
        int heading = GC_LEGS[legIdx];

        if (lastCmdSend + GC_CMD_INTERVAL < millis())
        {
            lastCmdSend = millis();
            sendLegCommand(cal, heading);
        }

        double err = fabs(smallestAngle((double)heading, cal->dirMag));
        if (err < GC_SETTLE_TOL)
        {
            if (settleSince == 0) settleSince = millis();
        }
        else
        {
            settleSince = 0;
        }

        snprintf(progressMsg, sizeof(progressMsg), "LEG %d/8 %d deg - settling (%.1f deg off)",
                 legIdx + 1, heading, err);

        if (settleSince != 0 && millis() - settleSince >= GC_SETTLE_HOLD)
        {
            // Never anchor a leg on a position we do not have.
            if (!cal->gpsFix) break;

            startLat = cal->lat;
            startLon = cal->lng;
            step = GC_RUN;
            stepStart = millis();
            offCourseSince = 0;
            printf("#GPSFOURIER: LEG %d CMD=%d\xC2\xB0 start %.8f, %.8f\r\n",
                   legIdx, heading, startLat, startLon);
            break;
        }

        if (millis() - stepStart > GC_SETTLE_TIMEOUT)
        {
            abortRun(cal, "buoy never settled on the commanded heading", GPSCAL_ABORT_NO_SETTLE);
            return;
        }
        break;
    }

    //-----------------------------------------------------------------------------------------------
    case GC_RUN:
    {
        int heading = GC_LEGS[legIdx];

        if (lastCmdSend + GC_CMD_INTERVAL < millis())
        {
            lastCmdSend = millis();
            sendLegCommand(cal, heading);
        }

        double dist = cal->gpsFix ? distanceBetween(startLat, startLon, cal->lat, cal->lng) : 0.0;
        snprintf(progressMsg, sizeof(progressMsg), "LEG %d/8 %d deg - running %.0fm/%.0fm",
                 legIdx + 1, heading, dist, (double)GC_MIN_LEG_DIST);

        // A leg that wandered is not a measurement of anything. Sail it again rather than feeding
        // the average a number that mostly describes a gust.
        double err = fabs(smallestAngle((double)heading, cal->dirMag));
        if (err > GC_RUN_TOL)
        {
            if (offCourseSince == 0)
            {
                offCourseSince = millis();
            }
            else if (millis() - offCourseSince > GC_RUN_TOL_HOLD)
            {
                legRetries++;
                printf("#GPSFOURIER: LEG %d off course by %.1f\xC2\xB0 - restarting (attempt %d/%d)\r\n",
                       legIdx, err, legRetries, GC_MAX_LEG_RETRIES);
                if (legRetries >= GC_MAX_LEG_RETRIES)
                {
                    abortRun(cal, "could not hold a leg long enough to measure it", GPSCAL_ABORT_LEG_UNSTABLE);
                    return;
                }
                step = GC_SETTLE;
                stepStart = millis();
                settleSince = 0;
                break;
            }
        }
        else
        {
            offCourseSince = 0;
        }

        if (dist >= GC_MIN_LEG_DIST)
        {
            double endLat = cal->lat;
            double endLon = cal->lng;

            // NOT calculateAngle(): despite its use in the older calibrate.cpp, that helper takes
            // two 2D VECTORS and returns the unsigned angle between them - it knows nothing about
            // latitude. calculateBearing() is the great-circle initial bearing, 0..360.
            legBearing[legIdx] = calculateBearing(startLat, startLon, endLat, endLon);
            legError[legIdx] = normalizeErr(legBearing[legIdx] - (double)heading);
            legDist[legIdx] = dist;

            printf("\r\n#GPSFOURIER: LEG %d\r\n", legIdx);
            printf("#GPSFOURIER:   CMD   = %d\xC2\xB0\r\n", heading);
            printf("#GPSFOURIER:   START = %.8f, %.8f\r\n", startLat, startLon);
            printf("#GPSFOURIER:   END   = %.8f, %.8f\r\n", endLat, endLon);
            printf("#GPSFOURIER:   DIST  = %.1fm\r\n", dist);
            printf("#GPSFOURIER:   GPS   = %.1f\xC2\xB0\r\n", legBearing[legIdx]);
            printf("#GPSFOURIER:   ERR   = %+.1f\xC2\xB0\r\n\r\n", legError[legIdx]);
            beep(2, buzzer);

            legIdx++;
            legRetries = 0;
            if (legIdx >= 8)
            {
                buildTable();
                tableTries = 0;
                tableReplyValid = false;
                sendTable(cal);
                step = GC_STORE_TABLE;
                stepStart = millis();
                lastCmdSend = 0;
                snprintf(progressMsg, sizeof(progressMsg), "storing new table");
            }
            else
            {
                step = GC_SETTLE;
                stepStart = millis();
                settleSince = 0;
            }
            break;
        }

        if (millis() - stepStart > GC_LEG_TIMEOUT)
        {
            abortRun(cal, "leg did not cover the minimum distance in time", GPSCAL_ABORT_LEG_TOO_SLOW);
            return;
        }
        break;
    }

    //-----------------------------------------------------------------------------------------------
    case GC_STORE_TABLE:
    {
        // Keep the thrusters quiet while the Sub writes to NVS.
        if (lastCmdSend + GC_CMD_INTERVAL < millis())
        {
            lastCmdSend = millis();
            RoboStruct stop = {};
            stop.cmd = TGDIRSPEED;
            stop.IDs = cal->mac;
            stop.IDr = BUOYIDALL;
            stop.ack = INF;
            stop.status = TGDIRSPEED;
            stop.tgDir = cal->dirMag;
            stop.speedSet = 0;
            xQueueSend(serOut, (void *)&stop, 0);
        }

        if (tableReplyValid)
        {
            tableReplyValid = false;
            bool ok = true;
            for (int i = 0; i < 8; i++)
            {
                if (fabs((double)tableReply[i] - (double)newTable[i]) > 0.05) ok = false;
            }
            if (ok)
            {
                printf("#GPSFOURIER: Sub confirmed the new table\r\n");
                step = GC_GO_HOME;
                break;
            }
            printf("#GPSFOURIER: Sub echoed a different table, resending\r\n");
            if (tableTries >= GC_TABLE_TRIES)
            {
                abortRun(cal, "Sub would not store the new table", GPSCAL_ABORT_TABLE_REFUSED);
                return;
            }
            sendTable(cal);
            break;
        }

        if (millis() - tableSentAt > GC_TABLE_TIMEOUT)
        {
            if (tableTries >= GC_TABLE_TRIES)
            {
                abortRun(cal, "Sub never confirmed the new table", GPSCAL_ABORT_NO_CONFIRM);
                return;
            }
            printf("#GPSFOURIER: no confirmation, resend %d/%d\r\n", tableTries + 1, GC_TABLE_TRIES);
            sendTable(cal);
        }
        break;
    }

    //-----------------------------------------------------------------------------------------------
    case GC_GO_HOME:
    {
        printf("#GPSFOURIER: ================ COMPLETE ================\r\n");
        for (int i = 0; i < 8; i++)
        {
            printf("#GPSFOURIER: leg %d CMD=%3d\xC2\xB0 DIST=%5.1fm GPS=%5.1f\xC2\xB0 ERR=%+5.1f\xC2\xB0\r\n",
                   i, GC_LEGS[i], legDist[i], legBearing[i], legError[i]);
        }
        printf("#GPSFOURIER: returning to %.8f, %.8f\r\n", homeLat, homeLon);
        // These only ever went to the USB port, which on a sealed hull or a buoy on the water is
        // the one place nobody can read them. Mirror them to the UDP log so a finished run can be
        // checked from the shore - or from a bench simulation.
        for (int i = 0; i < 8; i++)
        {
            udpLog("GPSCAL leg %d cmd=%3d dist=%5.1fm gps=%5.1f err=%+5.1f",
                   i, GC_LEGS[i], legDist[i], legBearing[i], legError[i]);
        }
        udpLog("GPSCAL table %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
               newTable[0], newTable[1], newTable[2], newTable[3],
               newTable[4], newTable[5], newTable[6], newTable[7]);

        cal->tgLat = homeLat;
        cal->tgLng = homeLon;
        cal->tgDist = 0;
        cal->tgDir = 0;

        // The Sub has been in TGDIRSPEED for the whole run, so its rudder and speed integrators are
        // wound up around a heading order rather than a waypoint.
        RoboStruct reset = {};
        reset.cmd = RESET_SPEED_RUD_PID;
        reset.IDs = cal->mac;
        reset.IDr = BUOYIDALL;
        reset.ack = INF;
        xQueueSend(serOut, (void *)&reset, 10);

        step = GC_IDLE;
        // Sailing home is real navigation: LOCKED makes handleTimerRoutines() send DIRDIST to the
        // Sub, and the Sub drives its thrusters on that. Under simulation the home position is a
        // fiction, so the leg suppression in sendLegCommand() is not enough on its own - this
        // would hand the real hull a course to a place it was never at. Finish idle instead.
        if (gpsSimActive())
        {
            snprintf(progressMsg, sizeof(progressMsg), "done - simulated, not returning home");
            cal->status = IDLING;
        }
        else
        {
            snprintf(progressMsg, sizeof(progressMsg), "done - returning home");
            cal->status = LOCKED;
            cal->lastSerOut = 0; // force an immediate DIRDIST so the Sub starts navigating home
        }

        // Terminal frames: the periodic report stops here, so these are the last word on how the
        // run ended and what it produced.
        sendCalibStatus(cal, GPSCAL_DONE, true);
        broadcastFinalTable(cal);

        beep(5, buzzer);
        break;
    }

    default:
        step = GC_IDLE;
        break;
    }
}
