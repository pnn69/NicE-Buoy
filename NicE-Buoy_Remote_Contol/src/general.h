#ifndef GGENERAL_H_
#define GENERAL_H_
#include "../../dependency/command.h"
#define NR_BUOYS 4
#define DEBUG true
#include "Arduino.h"

#define PUSHED 0
#define RELEASED 1

struct buoyDataType
{
    double gpslatitude, gpslongitude;
    double ancorlatitude, ancorlongitude;
    double doclatitude, doclongitude;
    double tgdir, tgdistance;
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
    float voltage;
    int percentage;
    int dataout;
};

extern bool ledstatus;
extern buoyDataType buoy[NR_BUOYS];
extern int notify;

#endif /* GENERA:_H_ */
