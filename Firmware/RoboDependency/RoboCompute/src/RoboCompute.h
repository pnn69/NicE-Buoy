#ifndef ROBOCOMPUTE_H
#define ROBOCOMPUTE_H
#include <Arduino.h>

#define EARTH_MEAN_RADIUS 6372795.0
#define EARTH_RADIUS_KM 6371.0
#define BUOYIDALL 1
#define ROBOBASE 99
#define HEAD 1
#define PORT 2
#define STARBOARD 3
#define BAUDRATE 115200
#define LEVEL true
#define SAMPELS 30
#define MAXSTRINGLENG 150

typedef enum
{
    NOCMD = -1,
    NOP = 0,
    GET = 1,
    SET,
    GETACK,
    ACK,
    NAC,
    INF,
    IDLE,
    IDLING,
    PING,
    PONG,
    ERROR,
    LOCKING,
    LOCKED,
    LOCK_POS,
    DOCKING,
    DOCKED,
    DOC,
    STOREASDOC,
    BUOYPOS,
    SETLOCKPOS,
    LOCKPOS,
    SETDOCKPOS,
    DOCKPOS,
    UNLOCK,
    REMOTE,
    REMOTING,
    CALIBRATE_MAGNETIC_COMPASS,
    START_CALIBRATE_MAGNETIC_COMPASS,
    LINEAR_CALIBRATING,
    // RETIRED. Magnetic declination was stored, put on the wire and printed, but no code in any
    // firmware ever applied it to a heading, and the only thing that could set it was an uncalled
    // function in RoboTop/src/calibrate.cpp whose send was commented out. It is also redundant
    // with compassOffset: that is measured by pointing the buoy at a known bearing, which already
    // contains the local declination, so having both invites correcting for it twice.
    // The two enumerators STAY so the numbering of every command after them is unchanged - a node
    // that is not reflashed at the same moment must not start mis-routing commands.
    SET_DECLINATION,
    STORE_DECLINATION,
    DOCK_STORING,
    MUTE_ESC,
    BLINK_SLOW,
    BLINK_FAST,
    BLINK_OFF,
    FADE_ON,
    SUBDATA,
    SUBACCU,
    MDIR,
    GDIR,
    TDIR,
    SPEED,
    SUBSPEED,
    DIRSPEED,
    TGDIRSPEED,
    DIRDIST,
    TOPID,
    SUBID,
    REMOTEID,
    TOPDATA,
    SPBBSPSB,
    ROUTETOPOINT,
    CALCRUDDER,
    PIDRUDDER,
    PIDRUDDERSET,
    PIDSPEED,
    PIDSPEEDSET,
    SUBPWR,
    TOPPWR,
    SENDTRACK,
    COMPUTESTART,
    COMPUTETRACK,
    NEWBUOYPOS,
    TXT,
    ROBODEFAULTS,
    WINDDATA,
    MAXMINPWR,
    MAXMINPWRSET,
    DIRMDIRTGDIRG,
    SOFTIRONCALIBRATION,
    HARDIRONFACTORS,
    SOFTIRONFACTORS,
    RAWCOMPASSDATA,
    STORE_COMPASS_OFFSET,
    CALC_COMPASS_OFFSET,
    INFIELD_CALIBRATE,
    INFIELD_OFFSET_CALIBRATE,
    RESET_RUDDER_PID,
    RESET_SPEED_PID,
    RESET_SPEED_RUD_PID,
    WAKEUP,
    SETUPDATA,
    ADAPTIVE_TRIM,
    REBOOT,
    SET_DOCWP,
	SET_AS_NORTH,
    // Carries the Sub's 8-point Fourier interpolation table (RoboStruct::interpolationTable).
    // ack GET/GETACK asks the Sub for the table that is currently IN EFFECT; ack SET hands it a
    // new one to commit to NVS and recompute the Fourier coefficients from. Either way the Sub
    // answers with ack INF and the table it now holds.
    STORE_INTERPOLATION_TABLE,
    // Starts the Top's GPS-based Fourier compass calibration run (8 navigation legs).
    // Payload: fields[5] = gpsCalStillWater, see RoboStruct below. An older node that sends no
    // payload at all is read as 0, i.e. the current-tolerant pair-averaged mode.
    GPS_FOURIER_CALIBRATE,
    // Broadcast progress report while a GPS Fourier calibration is running, ack INF.
    // Payload order (and the field index a raw comma-splitter such as RoboCYD's sees, where
    // fields[0..4] are IDr, IDs, ack, cmd, status):
    //   fields[5]  gpsCalStep      phase, see gpscal_step_t below
    //   fields[6]  gpsCalLeg       leg being sailed, 0..7, indexes into the 0/180/45/225/... order
    //   fields[7]  tgDir           heading the buoy was told to steer for this leg
    //   fields[8]  dirMag          heading it is actually reporting right now
    //   fields[9]  gpsCalDist      metres covered on this leg so far (0 outside the measuring phase)
    //   fields[10] gpsCalErr       LIVE signed error of the leg named in fields[6], degrees - the
    //                              bearing from that leg's start fix to the current fix, minus the
    //                              commanded heading. This is the value that will be recorded when
    //                              the leg ends, so it can be watched settling. Valid only once
    //                              fields[9] has passed 10 m; below that the displacement is too
    //                              short for a bearing and this reads 0.
    //   fields[11] gpsCalLastErr   signed error of the last COMPLETED leg, degrees
    //   fields[12] gpsCalAbort     why the run stopped, see gpscal_abort_t. 0 while it is still
    //                              running or finished cleanly. Optional: a Top that predates
    //                              this field simply sends a shorter frame.
    // One extra frame is forced on completion (step DONE) and on abort (step ABORTED), so a
    // listener always sees how the run ended rather than just silence.
    GPS_FOURIER_STATUS,

    // Guided eight point compass calibration, driven remotely. The session itself lives on the
    // Sub - see the block comment in RoboSub/src/compass.cpp - and this is the only way for the
    // Top's page, the CYD dashboard or the CYD touchscreen to press its buttons. They do not get
    // their own copy of the arithmetic, which is exactly the point: five hand-written versions of
    // it is how a corrected heading once ended up stored in the table as if it were a raw one.
    //
    // SET carries an action in RoboStruct::cal8Action, see cal8_action_t:
    //   CAL8_BEGIN   arm a session, ask for north
    //   CAL8_SET     capture the direction being asked for, then advance 45 degrees
    //   CAL8_SAVE    write the offset and the table together, or nothing
    //   CAL8_CANCEL  discard
    // GET just asks for the state. Either way the Sub answers with the state:
    //   fields[6]  cal8Action     echo of what was asked for
    //   fields[7]  cal8Active     1 while a session is running
    //   fields[8]  cal8Next       which direction is being asked for, 0..7, or 8 when all are in
    //   fields[9]  .. fields[16]  the eight captured entries, degrees
    // All count-guarded, so a node that predates this sends a short frame and leaves the reader's
    // own copy alone rather than reading as an empty session.
    //
    // Nothing on the buoy changes until CAL8_SAVE, so a lost frame anywhere in a run costs at most
    // one repeated press - and because the state is the Sub's, two screens can drive the same run.
    CAL8_SESSION
} msg_t;

// What a CAL8_SESSION SET is asking the buoy to do. Carried in RoboStruct::cal8Action.
typedef enum
{
    CAL8_BEGIN = 0,
    CAL8_SET,
    CAL8_SAVE,
    CAL8_CANCEL
} cal8_action_t;

// Smallest holding radius the buoy will accept, metres. Below this the station-keeping zones in
// pidrudspeed.cpp overlap: SUB_STATUS_PIVOT_PREP owns 1 m out to holdRad, so a radius near 1 m
// leaves no pivot band at all and the buoy hunts. The Sub's web page has always enforced it; the
// CYD used to allow 0.5 and the SETUPDATA path enforced nothing, so the same setting had three
// different minima depending on where you typed it. One number, here, for all of them.
#define HOLD_RADIUS_MIN 1.5

// Phase reported in RoboStruct::gpsCalStep by GPS_FOURIER_STATUS.
typedef enum
{
    GPSCAL_IDLE = 0,
    GPSCAL_FETCH_TABLE = 1, // asking the Sub which interpolation table it is applying
    GPSCAL_SETTLE = 2,      // steering the leg heading, waiting for it to hold steady
    GPSCAL_RUN = 3,         // measuring the GPS displacement over the leg
    GPSCAL_STORE = 4,       // handing the finished table to the Sub
    GPSCAL_DONE = 5,        // finished, returning to the start position
    GPSCAL_ABORTED = 6      // stopped early - gpscal_abort_t below says why
} gpscal_step_t;

// Why a GPS Fourier run stopped, reported in RoboStruct::gpsCalAbort by GPS_FOURIER_STATUS.
//
// The reason used to exist only as a string in the Top's progressMsg, which reaches its own web
// page and nothing else. Over LoRa the CYD could therefore show that a run had aborted but never
// why, and after fifteen seconds even that aged out and the panel went back to "No calibration
// running" - indistinguishable from the command never having arrived. Every abort now carries a
// code, so the display can say what actually went wrong while standing on the shore.
typedef enum
{
    GPSCAL_ABORT_NONE = 0,
    GPSCAL_ABORT_NO_FIX = 1,          // no GPS fix when the run was asked for
    GPSCAL_ABORT_SUB_SILENT = 2,      // Sub not answering on the serial link at the start
    GPSCAL_ABORT_FIX_LOST = 3,        // fix went away mid-run
    GPSCAL_ABORT_LINK_LOST = 4,       // serial link to the Sub went away mid-run
    GPSCAL_ABORT_HEADING_FROZEN = 5,  // dirMag bit-identical for too long, compass task hung
    GPSCAL_ABORT_DRIFTED = 6,         // wandered past the runaway guard from the start position
    GPSCAL_ABORT_NO_TABLE = 7,        // Sub never reported the table it is applying
    GPSCAL_ABORT_NO_SETTLE = 8,       // never held the commanded heading long enough to start
    GPSCAL_ABORT_LEG_UNSTABLE = 9,    // could not hold a leg straight enough to measure it
    GPSCAL_ABORT_LEG_TOO_SLOW = 10,   // leg did not cover the minimum distance in time
    GPSCAL_ABORT_TABLE_REFUSED = 11,  // Sub echoed back a different table than it was sent
    GPSCAL_ABORT_NO_CONFIRM = 12      // Sub never confirmed the new table
} gpscal_abort_t;

struct RoboStruct
{
    unsigned long mac = 0;
    unsigned long IDs = 0;
    unsigned long IDr = 0;
    int cmd = 0;
    int ack = -1;
    int loralstmsg = 0;
    int status = 0;
    int sub_status = 0;
    double lat = 0;
    double lng = 0;
    double tgLat = 0;
    double tgLng = 0;
    int dockApproachDist = 0; // meters
    int dockApproachDir = 0; // degrees
    bool dockingToWaypoint = false;
    int gpsDir = 0;
    int gpsSat = 0;
    int dirSet = 0;
    bool gpsFix = false;
    uint32_t gpsFixAge = 0;
    double wDir = 0;
    double wStd = 0;
    double dirMag = 0;
    double tgDir = 0;
    double tgSpeed = 0;
    bool locked = false;
    int trackPos = 0;
    // The heading with the hard and soft iron correction and the mounting offset applied, but
    // BEFORE the eight point compass table and before the adaptive trim. Reported as "Imag" on the
    // wire, alongside dirMag.
    //
    // This is the value the compass table is indexed by, which makes it the one thing a calibration
    // actually needs. Publishing it means MAN CAL no longer has to switch the table off, wait for
    // the buoy to confirm, and switch it back on afterwards - a sequence that failed silently and
    // produced a table applied on top of the one already in effect. With Imag on the wire a
    // calibration reads a value that is always correct, changes nothing on the buoy, and cannot be
    // spoiled by the state of the correction or the trim.
    //
    // 0 means "not reported" - the field is count guarded, so a node that predates it simply sends
    // a shorter frame and every reader keeps its own value.
    double imag = 0;
    int speed = 0;
    int speedBb = 0;
    int speedSb = 0;
    double speedSet = 0;
    double tgDist = 0;
    float subAccuV = 0;
    float topAccuV = 0;
    float subAccuI = 0;
    float topAccuI = 0;
    int subAccuP = 0;
    int topAccuP = 0;
    unsigned long lastTimes = 0;
    double errSums = 0;
    double lastErrs = 0;
    double Kpr, Kir, Kdr;
    double Kps, Kis, Kds;
    double ip, ir;
    double pivotSpeed = 0.2;
    double holdRad = 2.0;

    int minSpeed = 0;
    int maxSpeed = 75;
    double compassOffset = 0;
    unsigned long buoyId = 0;
    unsigned long lastLoraIn = 0;
    unsigned long lastLoraOut = 0;
    unsigned long lastUdpOut = 0;
    unsigned long lastSerOut = 0;
    unsigned long lastSerIn = 0;
    unsigned long lastUdpIn = 0;
    unsigned char retry = 0;
    double magHard[3] = {0};
    double magSoft[3][3] = {0};
    bool revBB = false;
    bool revSB = false;
    bool swap_BB_SB = false;
    double compass_trim = 0.0;
    bool compass_trim_enabled = false;
    double pitch = 0.0;
    double roll = 0.0;
    // 8-point compass interpolation table, in the same units and order as the Sub's
    // measured_angles[0..7]: entry i is the compass reading observed while the buoy actually
    // pointed at i * 45 degrees true. The identity default means "no correction".
    float interpolationTable[8] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 225.0f, 270.0f, 315.0f};
    // Whether the Sub actually APPLIES interpolationTable to its heading (its interp_enabled).
    // The table and the switch are separate: a stored table that is not applied leaves the
    // compass uncorrected. Carried by SETUPDATA field 20 - see the note in RoboCode().
    bool interpEnabled = false;
    // Guided eight point calibration, carried by CAL8_SESSION. cal8Action is what a SET asks
    // for; the rest is state the Sub reports and every other node only ever reads.
    int cal8Action = 0;
    bool cal8Active = false;
    int cal8Next = 0;
    float cal8Captured[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    // Progress of a GPS Fourier calibration run, carried by GPS_FOURIER_STATUS.
    int gpsCalStep = 0;      // gpscal_step_t
    int gpsCalLeg = 0;       // 0..7
    double gpsCalDist = 0;    // metres covered on the current leg
    double gpsCalErr = 0;     // live signed error of the leg in progress, degrees
    double gpsCalLastErr = 0; // signed error of the last completed leg, degrees
    int gpsCalAbort = 0;      // gpscal_abort_t - why the run stopped, 0 while running
    // How the calibration is allowed to interpret the difference between opposite legs.
    //
    // false - pair-averaged. Anything that flips sign between a heading and its opposite is taken
    //         to be current and discarded. Immune to a steady set, but it also discards a genuine
    //         one-cycle (hard iron) deviation, which has the identical signature. Corrects the
    //         constant and two-cycle terms only.
    // true  - still water. Each leg's error is used directly as the deviation at that heading, so
    //         the one-cycle term survives and the full curve is corrected. ONLY valid when there
    //         really is no current and negligible windage - otherwise the current is permanently
    //         written into the compass table.
    bool gpsCalStillWater = false;
    float gpsCalStartHeading = 0.0f;
};

struct RoboStructGps
{
    double lat = 0;
    double lng = 0;
    double latB2 = 0;
    double lngB2 = 0;
    double latB3 = 0;
    double lngB3 = 0;
    bool fix = false;
    int dir = 0;
    float speed = 0;
    double fixage = 0;
    double latTg = 0;
    double lngTg = 0;
    int dirTg = 0;
    double distTg = 0;
    int speedBb = 0;
    int speedSb = 0;
};

typedef struct
{
    double data[SAMPELS];
    double speed[SAMPELS];
    double wDir;
    double wStd;
    double wSpeed;
    int ptr;
} RoboWindStruct;

void RoboDecode(String data, RoboStruct *dataStore);
String RoboCode(const RoboStruct *dataOut);
String rfCode(RoboStruct *loraOut);
void rfDeCode(String rfIn, RoboStruct *in);
String removeBeginAndEndToString(String input);
String addCRCToString(String input);
bool verifyCRC(String input);
void averageWindVector(RoboWindStruct *wData);
void deviationWindRose(RoboWindStruct *wData);
void PidDecode(String data, int pid, RoboStruct *buoy);
String PidEncode(int pid, const RoboStruct *buoy);
void gpsGem(double &lat, double &lon);
double distanceBetween(double lat1, double lon1, double lat2, double lon2);
double calculateBearing(double lat1, double lon1, double lat2, double lon2);
double computeWindAngle(double windDegrees, double lat, double lon, double centroidLat, double centroidLon);
double approxRollingAverage(double avg, double input);
void addNewSampleInBuffer(RoboWindStruct *wData, double nwdata);
void checkparameters(RoboStruct *buoy);
void adjustPositionDirDist(double dir, double dist, double lat, double lon, double *latOut, double *lonOut);
double smallestAngle(double heading1, double heading2);
double calculateAngleSigned(double x1, double y1, double x2, double y2);
bool determineDirection(double heading1, double heading2);
double Angle2SpeedFactor(double angle);
double CalcDocSpeed(double tgdistance);
void CalcRemoteRudderBuoy(RoboStruct *buoy);
void hooverPid(RoboStruct *buoy);
void threePointAverage(struct RoboStruct p3[3], double *latgem, double *lnggem);
void twoPointAverage(double lat1, double lon1, double lat2, double lon2, double *latgem, double *longem);
void windDirectionToVector(double windDegrees, double *windX, double *windY);
double calculateAngle(double x1, double y1, double x2, double y2);
// Both return true only when they actually computed new positions.
bool recalcStartLine(struct RoboStruct rsl[3]);
bool reCalcTrack(struct RoboStruct rsl[3]);
void trackPosPrint(int c);
RoboStruct calcTrackPos(RoboStruct rsl[3]);
void MergeBuoyData(RoboStruct *dst, const RoboStruct &src);
void AddDataToBuoyBase(const RoboStruct &dataIn, RoboStruct *buoyPara[3]);
int GetDataPosFromBuoyBase(uint64_t id, RoboStruct buoyPara[3]);

#endif
