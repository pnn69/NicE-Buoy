#ifndef GGENERAL_H_
#define GENERAL_H_
#include "../../dependency/command.h"
#define BUFLENMHRG 60 //one sampel each sec so 60 sec for stabilisation
struct buoyDataType
{
    float mheading = 0;
    double tglatitude, tglongitude;
    double ancorlatitude, ancorlongitude;
    double doclatitude, doclongitude;
    double tgdir, tgdistance;
    int minOfsetDist, maxOfsetDist, minSpeed, maxSpeed;
    int mdir, ddir, cdir, magneticCorrection;
    double winddir[3 + BUFLENMHRG]; // winddir[0]=avarage winddir[1]=pionter winddir[2]=standarddeviation winddir[3..BUFLENWINDSPEED + 3)]=data
    int speed = 0, speedbb = 0, speedsb = 0, cspeed = 0;
    int rssi;
    int mode;
    float snr;
    int status = IDLE;
    int cmnd;
    bool ackOK;
    byte gsa;
    float vbatt;
    int vperc;
    bool muteEsc = false;
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

// #define RADIUSEARTH 6371 // radius earth
#define BUZZERON 0
#define BUZZEROFF 1
#define BUZZTIME 100

#define BUOYMAXSPEED 90
#define BUOYMINSPEED 7 // minimal speed thrusters are turning
// #define BUOYMINSPEEDBB 5 // minimal speed thrusters are turning
// #define BUOYMINSPEEDSB 20 // minimal speed thrusters are turning
// #define WEBON 1

// #define DEBUG 1
#if DEBUG
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#endif /* GENERA:_H_ */
