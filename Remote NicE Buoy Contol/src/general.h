#ifndef GGENERAL_H_
#define GENERAL_H_

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

extern buoyDataType buoy1;
extern buoyDataType buoy2;
extern buoyDataType buoy3;

#endif /* GENERA:_H_ */