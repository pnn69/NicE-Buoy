#ifndef GGENERAL_H_
#define GENERAL_H_
#include "../../dependency/command.h"

struct buoyDataType
{
    double gpslatitude, gpslongitude;
    int gpsheading;
    int mheading;
    double tglatitude, tglongitude;
    double ancorlatitude, ancorlongitude;
    double doclatitude, doclongitude;
    unsigned long tgdir, tgdistance;
    int mdir, ddir, cdir;
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

#endif /* GENERA:_H_ */
