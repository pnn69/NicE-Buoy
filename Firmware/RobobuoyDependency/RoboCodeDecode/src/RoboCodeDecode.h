#ifndef ROBOCODEDECODE_H
#define ROBOCODEDECODE_H
#include <Arduino.h>

struct RoboStruct
{
    /* data */
    double lat = 0;
    double lng = 0;
    double tgLat = 0;
    double tgLng = 0;
    int dirSet = 0;
    int dirGps = 0;
    int dirMag = 0;
    int speedSet = 0;
    int speedBb = 0;
    int speedSb = 0;
    int tgDir = 0;
    int tgDist = 0;
    float subAccuV = 0;
    float topAccuV = 0;
    int subAccuP = 0;
    int topAccuP = 0;
    int cmd = 0;
    float p, i, d;
    float kp, ki, kd;
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
    TOPIDLE,
} msg_t;

int RoboDecode(String data, RoboStruct *dataStore);
String RoboCode(RoboStruct dataOut);

#endif
