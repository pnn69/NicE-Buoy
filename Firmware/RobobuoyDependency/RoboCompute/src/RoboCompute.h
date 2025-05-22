#ifndef ROBOCOMPUTE_H
#define ROBOCOMPUTE_H
#include <Arduino.h>
// #include <mat.h>

/*some constans*/
#define EARTH_MEAN_RADIUS 6372795
// #define radian(x) (x * M_PI / 180)
// #define degre(x) (x * 180 / M_PI)
// #define ILIM 35 // Maximum interal limit (35% power)
#define BUOYIDALL 1
#define ROBOBASE 2
#define HEAD 1
#define PORT 2
#define STARBOARD 3

#define SAMPELS 30 // 60 samples
#define MAXSTRINGLENG 150

typedef enum
{
    LOTAGET = 1,                      // info request
    LORASET,                          // info to store
    LORAGETACK,                       // ack requerd
    LORAACK,                          // ack on message
    LORANAC,                          // nak
    LORAINF,                          // udate message
    IDLE,                             //
    IDELING,                          //
    PING,                             //
    PONG,                             //
    ERROR,                            // no udp communicaton
    LOCKING,                          //
    LOCKED,                           //
    LOCK_POS,                         //
    DOCKING,                          //
    DOCKED,                           //
    DOC,                              //
    STOREASDOC,                       // Store location as doc location
    BUOYPOS,                          // 19 STATUS,LAT,LON,mDir,wDir,wStd,BattPecTop,BattPercBott,gpsFix,gpsSat
    SETLOCKPOS,                       // tgLat,tgLng
    LOCKPOS,                          // 21 tgLat,tgLng,tgDir,tgDist
    SETDOCKPOS,                       // tgLat,tgLng
    DOCKPOS,                          // tgLat,tgLng,tgDir,tgDist
    UNLOCK,                           //
    REMOTE,                           // dir,speed
    REMOTEING,                        //
    CALIBRATE_MAGNETIC_COMPASS,       // calibrate magnetic compass
    START_CALIBRATE_MAGNETIC_COMPASS, // start calibrate magnetic compass
    LINEAR_CALLIBRATING,              //
    CALIBRATE_INCLINATION,            //
    STORE_INCLINATION,                //
    DOCK_STORING,                     //
    MUTE_ESC,                         // esc off
    BLINK_SLOW,                       // blink slow
    BLINK_FAST,                       // blink fast
    BLINK_OFF,                        // no Blinking/Fading
    FADE_ON,                          // fade mode
    SUBDATA,                          // all data send known by sub
    SUBACCU,                          // V,P accu voltage, accu percentage
    MDIR,                             // magnetic direction
    GDIR,                             // gps dir
    TDIR,                             // taget dir
    SPEED,                            // speed
    SUBSPEED,                         // speed BB, speed SB , speed
    DIRSPEED,                         // mDir,speedbb,speedsb,speed
    TGDIRSPEED,                       // TgDir,Speed
    DIRDIST,                          // Direction and distance
    TOPID,                            // mac[unsigned long]
    SUBID,                            // mac[unsigned long]
    REMOTEID,                         // mac[unsigned long]
    TOPDATA,                          // ?
    SPBBSPSB,                         // SpeedBb,SpeedSb
    ROUTTOPOINT,                      // route to point data
    CALCRUDDER,                       // tgDir,tgDist,Speed
    PIDRUDDER,                        // Prudder,Irudder,Drudder,kp,ki,kd PID parameters rudder + act data (p i d t) t = total
    PIDRUDDERSET,                     // Prudder,Irudder,Drudder,kp,ki,kd PID parameters rudder
    PIDSPEED,                         // Pspeed,Ispeed,Dspeed,kp,ki,kd PID parameters speed + act data (p i d t) t= total
    PIDSPEEDSET,                      // Pspeed,Ispeed,Dspeed,kp,ki,kd PID parameters speed
    SUBPWR,                           // speedSet,speed,spBb,spSb,subVbatt
    TOPPWR,                           // speed,tgDir,topVbatt
    SENDTRACK,                        // new track positions of buoys
    COMPUTESTART,                     //
    COMPUTETRACK,                     //
    NEWBUOYPOS,                       // lat,lng
    TXT,                              // Text message
    ROBODEFAULTS,                     //
    WINDDATA,                         // wDir,wStd;
} msg_t;

struct RoboStruct
{
    /* data */
    unsigned long mac = 0;
    unsigned long IDs = 0;
    unsigned long IDr = 0;
    int cmd = 0;
    int ack = -1;
    int loralstmsg = 0;
    int status = 0;
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
    int trackPos = 0;
    int speed = 0; // speed
    int speedBb = 0;
    int speedSb = 0;
    double speedSet = 0;
    double tgDist = 0;
    float subAccuV = 0;
    float topAccuV = 0;
    int subAccuP = 0;
    int topAccuP = 0;
    unsigned long lastTimes = 0;
    double errSums = 0;
    double lastErrs = 0;
    double ps, is, ds; // PID parameters speed
    double pr, ir, dr; // PID parameters rudder
    int minOfsetDist = 2;
    int maxOfsetDist = 20;
    int minSpeed = 0;
    int maxSpeed = 80;
    double compassOffset = 0;
    unsigned long buoyId = 0;
    unsigned long lastLoraIn = 0;  // last external communicatong
    unsigned long lastLoraOut = 0; // last external communicatong
    unsigned long lastUdpOut = 0;  // last external communicatong
    unsigned long lastSerOut = 0;  // last external communicatong
    unsigned long lastSerIn = 0;   // last external communicatong
    unsigned long lastUdpIn = 0;   // last external communicatong
    unsigned char retry = 0;
};

struct RoboStructGps
{
    /* data */
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

struct RoboWindStruct
{
    int ptr = 0;
    double wDir = 0;
    double wStd = 0;
    double data[SAMPELS];
};

struct RoboStruct RoboDecode(String data, RoboStruct);
String RoboCode(RoboStruct dataOut);
String rfCode(RoboStruct loraOut);
RoboStruct rfDeCode(String rfIn);
String removeBeginAndEndToString(String input);
String addCRCToString(String input); // Use reference to modify the original string
bool verifyCRC(String input);

double averigeWindRose(RoboWindStruct wData);
RoboWindStruct deviationWindRose(RoboWindStruct wData);
void initPid(int pid, RoboStruct buoy);
void PidDecode(String data, int pid, RoboStruct buoy);
String PidEncode(int pid, RoboStruct buoy);
void gpsGem(double &lat, double &lon);
double distanceBetween(double lat1, double long1, double lat2, double long2);
double computeWindAngle(double windDegrees, double lat, double lon, double centroidLat, double centroidLon);
double approxRollingAverage(double avg, double input);
RoboWindStruct addNewSampleInBuffer(RoboWindStruct wData, double nwdata);
void checkparameters(RoboStruct buoy);

void adjustPositionDirDist(double dir, double dist, double lat, double lon, double *latOut, double *lonOut);
double smallestAngle(double heading1, double heading2);
bool determineDirection(double heading1, double heading2);
double Angle2SpeedFactor(double angle);
double CalcDocSpeed(double tgdistance);
RoboStruct CalcRemoteRudderBuoy(RoboStruct buoy);
RoboStruct CalcRudderBuoy(RoboStruct buoy);
RoboStruct hooverPid(RoboStruct buoy);
void threePointAverage(struct RoboStruct p3[3], double *latgem, double *lnggem);
void twoPointAverage(double lat1, double lon1, double lat2, double lon2, double *latgem, double *longem);
void windDirectionToVector(double windDegrees, double *windX, double *windY);
double calculateAngle(double x1, double y1, double x2, double y2);
RoboStruct recalcStarLine(struct RoboStruct rsl[3]);
RoboStruct reCalcTrack(struct RoboStruct rsl[3]);
void trackPosPrint(int c);
RoboStruct calcTrackPos(RoboStruct rsl[3]);

RoboStruct AddDataToBuoyBase(RoboStruct dataIn, RoboStruct buoyPara[3]);
int GetDataPosFromBuoyBase(uint64_t id, RoboStruct buoyPara[3]);

#endif /* ROBOCOMPUTE */
