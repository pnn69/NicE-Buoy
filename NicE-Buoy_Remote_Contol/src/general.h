#ifndef GGENERAL_H_
#define GENERAL_H_
#include "../../dependency/command.h"
#define NR_BUOYS 4
#define DEBUG true

struct buoyDataType
{
    double gpslatitude, gpslongitude;
    double ancorlatitude, ancorlongitude;
    double doclatitude, doclongitude;
    unsigned long tgdir, tgdistance;
    int mdir, ddir, cdir;
    int speed, speedbb, speedsb, cspeed;
    int rssi;
    int mode;
    float snr;
    int status;
};

extern bool ledstatus;
extern buoyDataType buoy[NR_BUOYS];

#endif /* GENERA:_H_ */
