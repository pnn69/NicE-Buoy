#ifndef GGENERAL_H_
#define GENERAL_H_
#include "../../dependency/command.h"

struct buoyDataType
{
    double gpslatitude, gpslongitude;
    double ancorlatitude, ancorlongitude;
    double doclatitude, doclongitude;
    unsigned long tgdir, tgdistance;
    int speedbb, speedsb;
    int rssi;
    int mode;
    float snr;
};

extern buoyDataType buoy[4];

#endif /* GENERA:_H_ */
