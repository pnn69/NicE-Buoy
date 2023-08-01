#ifndef GGENERAL_H_
#define GENERAL_H_
#include "../../dependency/command.h"
#define NR_BUOYS 4
#define DEBUG true
#include "Arduino.h"

struct buoyDataType
{
    double gpslatitude, gpslongitude;
    double ancorlatitude, ancorlongitude;
    double doclatitude, doclongitude;
    unsigned long tgdir, tgdistance;
    int gpscource, mdir, ddir, cdir;
    int gpsspeed, speed, speedbb, speedsb, cspeed;
    int fix;
    int rssi;
    int mode;
    float snr;
    int status;
    byte remotestatus;
    int cmnd;
    bool ackOK;
    byte gsa;
};

extern bool ledstatus;
extern buoyDataType buoy[NR_BUOYS];
extern int notify;

#endif /* GENERA:_H_ */
