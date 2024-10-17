#ifndef ROBOCALC_H
#define ROBOCALC_H
#include <Arduino.h>

struct pid
{
    int maxSpeed = 80;
    int minSpeed = -60;
    double rudderKp;
    double speedKp;
    double rudderKi;
    double speedKi;
    double rudderKd;
    double speedKd;
    double rudderP = 20;
    double speedP = 20;
    double rudderI = 0.4;
    double speedI = 0.4;
    double rudderD = 0;
    double speedD = 0;
    double rudderLastErr = 0;
    double speedLastErr = 0;
    unsigned long rudderLastTime = 0;
    unsigned long speedLastTime = 0;
    double rudderIintergrate = 0;
    bool rudderArmIntergrator = false;
    double speedIintergrate = 0;
    bool speedArmIntergrator = false;
    int mechanicCorrection = 0;
    int cmd;
};

extern pid buoy;
int PidDecodeSub(String data);
int PidDecodeSub(String data);
String PidCodeTop(void);
String PidCodeSup(void);

void gpsGem(double &lat, double &lon);
void addBeginAndEndToString(String &input);
void addCRCToString(String &input);
bool verifyCRC(String input);

void initRudderPid(void);
double distanceBetween(double lat1, double long1, double lat2, double long2);
double courseTo(double lat1, double long1, double lat2, double long2);
double ComputeSmallestAngleDir(double heading1, double heading2);
bool CalcRudderBuoy(double magheading, float tgheading, double tdistance, int speed, int *bb, int *sb);
void RouteToPoint(double lat1, double lon1, double lat2, double lon2, double *distance, double *direction);

#endif
