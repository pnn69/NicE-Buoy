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

extern bool ledstatus;
extern buoyDataType buoy[NR_BUOYS];
extern int notify;

#endif /* GENERA:_H_ */
