#ifndef ROBOCOMPUTE_H
#define ROBOCOMPUTE_H

#include <Arduino.h>
#include "robo_protocol.h"

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


// What a CAL8_SESSION SET is asking the buoy to do. Carried in RoboStruct::cal8Action.
typedef enum
{
    CAL8_BEGIN = 0,
    CAL8_SET,
    CAL8_SAVE,
    CAL8_CANCEL
    // CAL8_LOCK and CAL8_UNLOCK used to sit here. They let the GPS Fourier run claim the buoy's
    // heading reference, because compassOffset was the domain the table was indexed by and moving
    // it mid-run put the legs measured before and after in different frames. The offset is now
    // applied AFTER the table, so it is no longer part of the table's domain, nothing a
    // calibration measures depends on it, and there is nothing left to lock.
} cal8_action_t;

// How many peers fit in one LORA_LINK frame. Bounded by RoboDecode()'s 25-field splitter: two
// fields for cmd and status, one for linkOmitted, leaves 22 for triples - seven of them, with a
// field to spare. Raising the splitter is deliberately not the answer: it is the decoder every
// message on this network goes through, and it would grow the stack of a path that runs on every
// frame to solve a problem this cap already solves.
#define LORA_LINK_MAX_PEERS 7

// Smallest holding radius the buoy will accept, metres. Below this the station-keeping zones in
// pidrudspeed.cpp overlap: SUB_STATUS_PIVOT_PREP owns 1 m out to holdRad, so a radius near 1 m
// leaves no pivot band at all and the buoy hunts. The Sub's web page has always enforced it; the
// CYD used to allow 0.5 and the SETUPDATA path enforced nothing, so the same setting had three
// different minima depending on where you typed it. One number, here, for all of them.
#define HOLD_RADIUS_MIN 1.5

// How far a newly set waypoint has to be for the thrusters to be cleaned on the way to it, and -
// the same number - how close the buoy has to get before the cleaning runs. Metres.
//
// One constant for both ends of the rule on purpose. A run long enough to collect weed is a run
// longer than this, and the cleaning is wanted at the END of it: the buoy arrives with clear
// thrusters and settles onto the mark, instead of trying to hold station on a fouled one. So the
// Top arms the cleaning when the target it has just been given is further away than this, and
// fires it when the distance has come back down to it. See CLEAN_THRUSTERS.
#define CLEAN_TRIGGER_DIST_M 20.0

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
    // The heading with the hard and soft iron correction applied and NOTHING else: before the
    // eight point compass table, before the mounting offset, before the adaptive trim. Reported as
    // "Imag" on the wire, alongside dirMag.
    //
    // This is the value the compass table is indexed by, which makes it the one thing a calibration
    // actually needs. Publishing it means a calibration reads a value that is always correct,
    // changes nothing on the buoy, and cannot be spoiled by the state of the correction, the
    // offset or the trim.
    //
    // It used to include compassOffset, because the offset was added before the table and so was
    // part of the table's domain. Both moved: the offset is now applied after the table, and this
    // is captured before either. The guided calibration depends on it - the operator turns the hull
    // until this reads zero, and a zero that shifted every time somebody pressed Set as North would
    // anchor the whole table somewhere different each run.
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
    // Guided eight point calibration, carried by CAL8_SESSION. cal8Action is what a SET asks for,
    // and on a CAL8_SET cal8Next/cal8Seq say which direction and which press; the rest is state the
    // Sub reports and every other node only ever reads. See CAL8_SESSION above.
    int cal8Action = 0;
    bool cal8Active = false;
    int cal8Next = 0;
    float cal8Captured[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    // Bit i set once direction i has been captured. Not derivable from cal8Next: a redo of an
    // earlier direction leaves the cursor where it was.
    uint8_t cal8Mask = 0;
    // Serial of the press. Outbound on a CAL8_SET: the one this press means to be. Inbound: the
    // last one the Sub actually applied. This is what makes a retried press safe.
    uint16_t cal8Seq = 0;

    // Serial of an operator PRESS - IDLE, LOCK, DOCK, REMOTE. Same idea as cal8Seq and for the
    // same reason, generalised to the commands a human pushes a button for.
    //
    // One press does not arrive once. It goes out on LoRa and UDP both, ack GETACK puts it in
    // RoboTop's retransmit table, every LoRa receiver repeats what it hears once, and the other
    // Top bridges the UDP copy onto the air as well. Measured: a single DOCK press arrived at its
    // Top 51 times spread over 27 seconds.
    //
    // Suppressing duplicates by content cannot fix that, because the echo tail outlives any
    // sensible window: a 5 second filter let each command through once per 5 seconds, and since
    // IDLE and DOCK are DIFFERENT commands they never suppressed each other - so an IDLE pressed
    // before a DOCK kept re-executing after it and the buoy oscillated IDLE, DOCKED, IDLE, DOCKED
    // for half a minute.
    //
    // A serial fixes it properly: the receiver executes a press only if it is NEWER than the last
    // one it acted on, so a stale echo can never overtake a fresh press however long it circulates.
    // 0 means "unnumbered" - a node that predates this - and falls back to the content filter.
    // Carried in numbers[7], which is free on every command that uses it. See RoboCode().
    uint16_t cmdSeq = 0;

    // Compass steadiness, carried in SETUPDATA so it can be reached from the handheld instead of
    // only from the Sub's own web page. Both live in the Sub's NVS and are read by CompassTask.
    //
    // prDamping is the exponential damping on pitch and roll, 0.00 (none) to 0.99 (most) - the
    // bubble level on MAN CAL. compassAvg is the heading averaging window, 1 to 200 samples. A
    // jumpy sensor makes a compass calibration very hard to take, and until now the only way to
    // steady it was a laptop on the Sub's page.
    //
    // Appended to the END of the frame, after dockingToWaypoint, so a node that predates them
    // sends a shorter frame and every count-guarded field before this is untouched.
    // Defaults are SENTINELS, not values. A sender that predates these fields leaves them at the
    // default, and the receiver must be able to tell that apart from a real setting - otherwise an
    // un-flashed handheld saving the setup page would arrive carrying "averaging = 1" and switch
    // the heading averaging off on a buoy that was running 20.
    float prDamping = -1.0f;   // < 0 means the frame did not carry it
    int compassAvg = 0;        // 0 means the frame did not carry it (the real range starts at 1)

    // Whether the automatic thruster cleaning is armed at all, carried in SETUPDATA after the two
    // compass steadiness fields. Owned by the TOP, not the Sub: the Top is the node that knows a
    // waypoint has been set and how far away it is, so it is the node that decides. It sits in
    // SETUPDATA anyway because that is the one frame every front end already reads and writes, and
    // the dock approach settings alongside it are Top-owned for the same reason.
    //
    // Defaults true, and presence is decided by the field COUNT like everything else in that
    // frame - a sender that predates this simply sends a shorter frame and the Top keeps what it
    // has in NVS. CLEAN NOW is deliberately NOT gated on this: the switch turns off the automatic
    // trigger, not the buoy's ability to clean itself when somebody asks.
    bool cleanEnabled = true;

    // What this node hears, carried by LORA_LINK. linkPeers is how many entries are filled.
    uint32_t linkPeerId[LORA_LINK_MAX_PEERS] = {0};
    int16_t linkRssi[LORA_LINK_MAX_PEERS] = {0};
    uint16_t linkCount[LORA_LINK_MAX_PEERS] = {0};
    uint8_t linkPeers = 0;
    uint8_t linkOmitted = 0;

    // Whether the buoy can USE the interpolation table it holds, carried by
    // STORE_INTERPOLATION_TABLE. An out-of-order table is stored and then ignored, so without this
    // a sender could not tell "calibration stored" from "calibration stored and being ignored".
    bool interpUsable = true;

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
// The wind direction to square a start line against: the mean of what the two END buoys report.
//
// It used to be one buoy's reading - whichever the command happened to reach. Two anemometers a
// line's length apart disagree by a few degrees in steady air and by rather more in a shifty one,
// and there is no reason to prefer either; the mean squares the line to the wind ACROSS the line,
// which is the wind the fleet actually starts in.
//
// Vector mean, not arithmetic: 350 and 10 average to 0, not to 180. A buoy reporting 0/0 has no
// reading at all rather than a northerly - the same rule the compute guards use - so it is left
// out, and `fallback` is returned when neither end has one.
double meanWindDir(double dirA, double stdA, double dirB, double stdB, double fallback);

bool recalcStartLine(struct RoboStruct rsl[3]);

// Move the two start line ends apart by `metres` in total, half each, keeping the midpoint and the
// bearing exactly as they are. Needs no wind reading, because nothing rotates. A NEGATIVE `metres`
// draws them together by the same rule and along the same two bearings, which is what SHORTENSTART
// sends. Returns false when there is no usable pair, when either end has no lock position, or when
// the result would be shorter than MIN_START_LINE_M.
//
// Not clamped to the floor, refused at it. A clamp would beep success and move the buoys somewhere
// other than where the press asked for, and the next press would report success again while
// nothing moved at all - so a line already at the floor would read exactly like a working one.
#define MIN_START_LINE_M 5.0
bool extendStartLine(struct RoboStruct rsl[3], double metres);
bool reCalcTrack(struct RoboStruct rsl[3]);
void trackPosPrint(int c);
RoboStruct calcTrackPos(RoboStruct rsl[3]);
void MergeBuoyData(RoboStruct *dst, const RoboStruct &src);
void AddDataToBuoyBase(const RoboStruct &dataIn, RoboStruct *buoyPara[3]);
int GetDataPosFromBuoyBase(uint64_t id, RoboStruct buoyPara[3]);

#endif
