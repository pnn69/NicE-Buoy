#ifndef ROBOCODEDECODE_H
#define ROBOCODEDECODE_H
#include <Arduino.h>

struct RoboStruct
{
    /* data */
    int dirSet = 0;
    int dirGps = 0;
    int dirMag = 0;
    int speedSet = 0;
    int speedBb = 0;
    int speedSb = 0;
    float subAccuV = 0;
    float topAccuV = 0;
    int subAccuP = 0;
    int topAccuP = 0;
    int cmd = 0;
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
};

typedef enum
{
    SUBDATA = 1, // all data send known by sub
    SUBACCU,     // accu voltage, accu percentage
    SUBDIR,      // magnetic direction
    SUBSPEED,    // seed(given), speed BB, speed SB
    SUBDIRSPEED, // magnetic heading,seed(given), speed BB, speed SB
    TOPDATA,     // all dat send known by top
    TOPDIRSPEED, // Speed and direction
    PING,
    PONG

} msg_t;

int RoboDecode(String data, RoboStruct dataStore);
String RoboCode(RoboStruct dataOut);

#endif
