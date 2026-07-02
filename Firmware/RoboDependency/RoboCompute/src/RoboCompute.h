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
    IDELING,
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
    REMOTEING,
    CALIBRATE_MAGNETIC_COMPASS,
    START_CALIBRATE_MAGNETIC_COMPASS,
    LINEAR_CALLIBRATING,
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
    ROUTTOPOINT,
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
	SET_AS_NORTH = 125
} msg_t;

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

    int maxOfsetDist = 20;
    int minSpeed = 0;
    int maxSpeed = 75;
    double declination = 0;
    double compassOffset = 0;
    double mechanicCorrection = 0;
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
void recalcStartLine(struct RoboStruct rsl[3]);
void reCalcTrack(struct RoboStruct rsl[3]);
void trackPosPrint(int c);
RoboStruct calcTrackPos(RoboStruct rsl[3]);
void AddDataToBuoyBase(RoboStruct dataIn, RoboStruct *buoyPara[3]);
int GetDataPosFromBuoyBase(uint64_t id, RoboStruct buoyPara[3]);

#endif
