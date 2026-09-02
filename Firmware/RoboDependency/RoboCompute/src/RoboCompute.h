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
    // Slots 89 and 90, once GPS_FOURIER_CALIBRATE and GPS_FOURIER_STATUS: the Top's GPS based
    // calibration run, which sailed eight legs and inferred the table from the GPS track. Removed
    // when the offset moved out of the table's input (see the compass chain in
    // RoboSub/src/compass.cpp) - its residual arithmetic assumed the old ordering and would have
    // gone on writing plausible, wrong tables without saying so.
    //
    // Held open rather than deleted. These are positional enumerators on the wire, so closing the
    // gap would renumber CAL8_SESSION from 91 to 89 and a node that had not been reflashed would
    // read every calibration press as something else entirely.
    RESERVED_WAS_GPS_FOURIER_CALIBRATE,
    RESERVED_WAS_GPS_FOURIER_STATUS,

    // Guided eight point compass calibration, driven remotely. The session itself lives on the
    // Sub - see the block comment in RoboSub/src/compass.cpp - and this is the only way for the
    // Top's page, the CYD dashboard or the CYD touchscreen to press its buttons. They do not get
    // their own copy of the arithmetic, which is exactly the point: five hand-written versions of
    // it is how a corrected heading once ended up stored in the table as if it were a raw one.
    //
    // SET carries an action in RoboStruct::cal8Action, see cal8_action_t:
    //   CAL8_BEGIN   arm a session
    //   CAL8_SET     capture the direction named in cal8Next
    //   CAL8_SAVE    write the table, once all eight are in
    //   CAL8_CANCEL  discard
    //
    // A CAL8_SET names the direction it MEANS in RoboStruct::cal8Next and carries a press serial in
    // RoboStruct::cal8Seq. Both are needed, for different reasons.
    //
    // The leg number says WHICH slot to fill. Any direction that has already been captured may be
    // captured again, so the Sub cannot infer the target from a cursor the way it used to.
    //
    // The serial says WHETHER this is a new press or an echo of one already carried out. The
    // Top-to-Sub link is a single half-duplex wire that the Sub is talking on continuously, so
    // presses have to be repeated - measured on the bench, eight presses sent one per second landed
    // twice. A repeat that arrives after the operator has rotated the hull to the next mark would
    // otherwise overwrite a good capture with a reading taken from the wrong place. So a presser
    // sends cal8Seq = the reported cal8Seq + 1, and the Sub applies a press only when it is exactly
    // one past the last it applied. A duplicate carries the value already used and is dropped.
    // cal8Seq = 0 means "no retry behind this press" - the Sub's own web page, where the button is
    // wired straight to the function - and is always applied.
    //
    // GET just asks for the state. Either way the Sub answers with the state:
    //   fields[6]  cal8Action     echo of what was asked for
    //   fields[7]  cal8Active     1 while a session is running
    //   fields[8]  cal8Next       the direction the guided cursor is asking for next, 0..7, or 8
    //                             once every direction has been captured at least once
    //   fields[9]  .. fields[16]  the eight captured entries, degrees
    //   fields[17] cal8Mask       bit i set once direction i has been captured. The cursor alone
    //                             cannot express this any more: after a redo of an earlier
    //                             direction the cursor does not move, so "how far along are we" and
    //                             "which ones are in" are two different questions.
    //   fields[18] cal8Seq        serial of the last press the Sub APPLIED. A presser reads this to
    //                             number its next one.
    // All count-guarded, so a node that predates this sends a short frame and leaves the reader's
    // own copy alone rather than reading as an empty session.
    //
    // Nothing on the buoy changes until CAL8_SAVE, so a lost frame anywhere in a run costs at most
    // one repeated press - and because the state is the Sub's, two screens can drive the same run.
    CAL8_SESSION,
    // Hull attitude - pitch and roll, degrees. Sent by the Top over UDP ONLY, never over LoRa.
    //
    // Deliberately its own frame rather than two more fields on TOPDATA. TOPDATA is the frame LoRa
    // carries, and at SF7/125 kHz it already occupies the channel for 195 ms - one a second is
    // about 20% of the air per buoy, which is why the Top throttles it while station keeping. Two
    // extra fields would be paid for on every frame for ever, to feed a bubble level that only
    // matters while somebody is stood over the hull calibrating it, with WiFi in range by
    // definition. At the LoRa rate a level would be updating once every few seconds anyway, which
    // is worse than not showing one.
    //
    //   fields[5]  pitch
    //   fields[6]  roll
    ATTITUDE,
    // Declare the hull level where it sits right now: the Sub records its current attitude as the
    // datum and reports departure from it thereafter. No payload - like SET_AS_NORTH, the buoy
    // reads its own sensor and the command only says when.
    //
    // Press it with the boat floating the way it floats, NOT tilted to centre the bubble. A fixed
    // mounting tilt is absorbed by the compass table, which maps whatever the compass reads at each
    // direction back to the truth and does not care why it reads it. What the table cannot absorb
    // is the hull sitting at a different attitude at each stop, or at a different attitude while
    // sailing than while being calibrated - so the datum exists to make CHANGE visible, and
    // chasing the bubble by tilting the boat defeats the whole point of it.
    SET_AS_LEVEL
} msg_t;

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

// Smallest holding radius the buoy will accept, metres. Below this the station-keeping zones in
// pidrudspeed.cpp overlap: SUB_STATUS_PIVOT_PREP owns 1 m out to holdRad, so a radius near 1 m
// leaves no pivot band at all and the buoy hunts. The Sub's web page has always enforced it; the
// CYD used to allow 0.5 and the SETUPDATA path enforced nothing, so the same setting had three
// different minima depending on where you typed it. One number, here, for all of them.
#define HOLD_RADIUS_MIN 1.5

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
bool recalcStartLine(struct RoboStruct rsl[3]);
bool reCalcTrack(struct RoboStruct rsl[3]);
void trackPosPrint(int c);
RoboStruct calcTrackPos(RoboStruct rsl[3]);
void MergeBuoyData(RoboStruct *dst, const RoboStruct &src);
void AddDataToBuoyBase(const RoboStruct &dataIn, RoboStruct *buoyPara[3]);
int GetDataPosFromBuoyBase(uint64_t id, RoboStruct buoyPara[3]);

#endif
