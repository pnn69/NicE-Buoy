#ifndef ROBOCOMPUTE_H
#define ROBOCOMPUTE_H
#include <Arduino.h>

#define HEAD 1
#define PORT 2
#define STARBOARD 3

typedef enum
{
    PING = 1,
    PONG,
    SUBDATA,        // all data send known by sub
    SUBACCU,        // accu voltage, accu percentage
    SUBDIR,         // magnetic direction
    SUBSPEED,       // speed(given), speed BB, speed SB
    SUBDIRSPEED,    // magnetic heading,speed(given), speed BB, speed SB
    TOPDATA,        // all dat send known by top
    TOPDIRSPEED,    // Speed and direction
    TOPDIRDIST,     // Direction and distance
    TOPSPBBSPSB,    // speed bb Speed sb
    TOPROUTTOPOINT, // route to point data
    TOPCALCRUDDER,  // rudder data
    PIDRUDDER,      // PID parameters rudder + act data (p i d t) t = total
    PIDRUDDERSET,   // PID parameters rudder
    PIDSPEED,       // PID parameters speed + act data (p i d t) t= total
    PIDSPEEDSET,    // PID parameters speed
    TOPIDLE,        //
    TOPID,          // mac top
    SUBID,          // mac sub
    LORASET,        // info to store
    LOTAGET,        // info request
    LORAGETACK,     // ack requerd
    LORAACK,        // ack on message
    LORANAC,        // nak
    LORAUPD,        // udate message
    LORABUOYPOS     // ID,MSG,ACK,STATUS,LAT,LON.mDir,wDir,wStd,BattPecTop,BattPercBott,speedbb,speedsb
} msg_t;

struct RoboStruct
{
    /* data */
    double id = 0;
    double lat = 0;
    double lng = 0;
    double tgLat = 0;
    double tgLng = 0;
    int dirSet = 0;
    int dirGps = 0;
    double wDir, wStd;
    double dirMag = 0;
    double tgDir = 0;
    int speed = 0; // speed
    int speedBb = 0;
    int speedSb = 0;
    int speedSet = 0;
    double tgDist = 0;
    float subAccuV = 0;
    float topAccuV = 0;
    int subAccuP = 0;
    int topAccuP = 0;
    int cmd = 0;
    double ps, is, ds;    // speed
    double kps, kis, kds; // rudder
    double pr, ir, dr;
    double kpr, kir, kdr;
    double lastErrs = 0;
    unsigned long lastTimes = 0;
    double lastErrr = 0;
    unsigned long lastTimer = 0;
    double iintergrates = 0;
    bool armIntergrators = false;
    double iintergrater = 0;
    bool armIntergratorr = false;
    int minOfsetDist;
    int maxOfsetDist;
    int minSpeed;
    int maxSpeed;
    int compassOffset;
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

struct RoboStruct RoboDecode(String data, RoboStruct);
// void RoboDecode(String data, RoboStruct &dataStore);
String RoboCode(RoboStruct dataOut);

/************************************************************************************************************************************************************************* */
// OLD ROBOCALC
/************************************************************************************************************************************************************************* */
String addBeginAndEndToString(String input);
String addCRCToString(String input); // Use reference to modify the original string
bool verifyCRC(String input);
double averigeWindRose(double samples[], int n);
double deviationWindRose(double samples[], int n);
void PidDecode(String data, int pid, RoboStruct buoy);
String PidEncode(int pid, RoboStruct buoy);
void gpsGem(double &lat, double &lon);
double distanceBetween(double lat1, double long1, double lat2, double long2);
double courseTo(double lat1, double long1, double lat2, double long2);
void initPid(int pid, RoboStruct buoy);
double approxRollingAverage(double avg, double input);
void addNewSampleInBuffer(double *input, int buflen, double nwdata);
void checkparameters(RoboStruct buoy);

void adjustPositionDirDist(double dir, double dist, double *lat, double *lon);
double smallestAngle(double heading1, double heading2);
bool determineDirection(double heading1, double heading2);
double Angle2SpeedFactor(double angle);
double CalcDocSpeed(double tgdistance);
RoboStruct CalcRemoteRudderBuoy(RoboStruct buoy);
RoboStruct CalcRudderBuoy(RoboStruct buoy);
RoboStruct hooverPid(RoboStruct buoy);
void threePointAverage(double lat1, double lon1, double lat2, double lon2, double lat3, double lon3, double *latgem, double *longem);
void twoPointAverage(double lat1, double lon1, double lat2, double lon2, double *latgem, double *longem);
void windDirectionToVector(double windDegrees, double *windX, double *windY);
double calculateAngle(double x1, double y1, double x2, double y2);
int checkWindDirection(double windDegrees, double lat, double lon, double centroidX, double centroidY);
void reCalcStartLine(double *lat1, double *lon1, double *lat2, double *lon2, double winddir);

#endif /* ROBOCOMPUTE */
