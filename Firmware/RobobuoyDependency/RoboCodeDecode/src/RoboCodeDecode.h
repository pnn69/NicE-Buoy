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
    float accuV = 0;
    int accuP = 0;
    int cmd = 0;
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
