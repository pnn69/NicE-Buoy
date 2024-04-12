#ifndef GGENERAL_H_
#define GENERAL_H_
#include "../../dependency/command.h"

struct buoyDataType
{
    float mheading;
    double tglatitude, tglongitude;
    double ancorlatitude, ancorlongitude;
    double doclatitude, doclongitude;
    double tgdir, tgdistance;
    int minOfsetDist, maxOfsetDist, minSpeed, maxSpeed;
    int mdir, ddir, cdir, mcorrdir;
    int speed, speedbb, speedsb, cspeed;
    int rssi;
    int mode;
    float snr;
    int status;
    int cmnd;
    bool ackOK;
    byte gsa;
    float vbatt;
    float vperc;
    bool muteEsc;
};

struct switchStatus
{
    int switch1upcnt, switch2upcnt;
    int switch1dwncnt, switch2dwncnt;
    bool switch1upact, switch2upact;
    bool switch1dwnact, switch2dwnact;
};

extern buoyDataType buoy;
extern switchStatus frontsw;
extern char buoyID;
extern bool nwloramsg;
extern byte status;

#define BUZZERON 0
#define BUZZEROFF 1
#define BUZZTIME 100

#define DEBUG 1
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#endif /* GENERA:_H_ */
