#ifndef GGENERAL_H_
#define GENERAL_H_
#include "../../dependency/command.h"
#include <BluetoothSerial.h>
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
    int minOfsetDist, maxOfsetDist, minSpeed, maxSpeed;
    int nrsats;
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
    String string;
    double p = 0;
    double i = 0;
    double d = 0;
    double ki = 0;
};

extern bool ledstatus;
extern buoyDataType buoy[NR_BUOYS];
extern int notify;
extern unsigned long checkAckStamp;
extern BluetoothSerial SerialBT;

#endif /* GENERA:_H_ */
