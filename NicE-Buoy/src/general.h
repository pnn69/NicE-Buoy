#ifndef GGENERAL_H_
#define GENERAL_H_
#include "../../dependency/command.h"

struct buoyDataType
{
    float mheading = 0;
    double tglatitude, tglongitude;
    double ancorlatitude, ancorlongitude;
    double doclatitude, doclongitude;
    double tgdir, tgdistance;
    int minOfsetDist, maxOfsetDist, minSpeed, maxSpeed;
    int mdir, ddir, cdir, magneticCorrection;
    int speed = 0, speedbb = 0, speedsb = 0, cspeed = 0;
    int rssi;
    int mode;
    float snr;
    int status = IDLE;
    int cmnd;
    bool ackOK;
    byte gsa;
    float vbatt;
    float vperc;
    bool muteEsc = false;
    float speedIntergrator = 0;
};

struct switchStatus
{
    int switch1upcnt, switch2upcnt;
    int switch1dwncnt, switch2dwncnt;
    bool switch1upact, switch2upact;
    bool switch1dwnact, switch2dwnact;
};

struct pid
{
    double kp = 20;
    double ki = 0.4;
    double kd = 0;
    double p = 0;
    double i = 0;
    double d = 0;
    double lastErr = 0;
    unsigned long lastTime = 0;
    double iintergrate = 0;
};

extern buoyDataType buoy;
extern switchStatus frontsw;
extern pid speedpid;
extern pid rudderpid;
extern char buoyID;
extern bool nwloramsg;
extern byte status;

#define RADIUSEARTH 6371 // radius earth
#define BUZZERON 0
#define BUZZEROFF 1
#define BUZZTIME 100

// #define BUOYMAXSPEED 80
#define BUOYMAXSPEED 20

#define DEBUG 1
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#endif /* GENERA:_H_ */
