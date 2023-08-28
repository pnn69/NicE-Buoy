#ifndef GGENERAL_H_
#define GENERAL_H_
#include "../../dependency/command.h"

struct buoyDataType
{
    int mheading;
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
};

extern buoyDataType buoy;

extern char buoyID;
extern bool ledstatus;
extern byte status;

extern bool LEDSTRIP;
#endif /* GENERA:_H_ */
