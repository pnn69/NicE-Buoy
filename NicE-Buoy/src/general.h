#ifndef GGENERAL_H_
#define GENERAL_H_
#include "../../dependency/command.h"

struct buoyDataType
{
    float mheading;
    double tglatitude, tglongitude;
    double ancorlatitude, ancorlongitude;
    double doclatitude, doclongitude;
    unsigned long tgdir, tgdistance;
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
extern bool ledstatus;
extern byte status;

#define BUZZERON 0
#define BUZZEROFF 1

#endif /* GENERA:_H_ */
