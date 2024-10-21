#ifndef ROBOCALC_H
#define ROBOCALC_H
#include <Arduino.h>

struct pid
{
    int speed = 0;
    double kp = 20;
    double ki = 0.4;
    double kd = 0;
    double p = 0.5;
    double i = 0.02;
    double d = 0;
    double lastErr = 0;
    unsigned long lastTime = 0;
    double iintergrate = 0;
    bool armIntergrator = false;
    int minOfsetDist;
    int maxOfsetDist;
    int minSpeed;
    int maxSpeed;
    int compassCorrection;
    int speedbb = 0;
    int speedsb = 0;
};

// extern pid buoy;
extern pid speed;
extern pid rudder;

void addBeginAndEndToString(String &input);
void setparameters(int &minOfsetDist, int &maxOfsetDist, int &minSpeed, int &maxSpeed);
void gpsGem(double &lat, double &lon);
void addCRCToString(String &input);
bool verifyCRC(String input);

String PidEncode(pid buoy);
int PidDecode(String data, pid buoy);
double distanceBetween(double lat1, double long1, double lat2, double long2);
double courseTo(double lat1, double long1, double lat2, double long2);
double ComputeSmallestAngleDir(double heading1, double heading2);
void adjustPositionDirDist(double dir, double dist, double *lat, double *lon);
bool CalcRudderBuoy(double magheading, double tgheading, double tdistance, int speed, int *bb, int *sb, pid buoy);
// void RouteToPoint(double lat1, double lon1, double lat2, double lon2, double &distance, double &direction);
int hooverPid(double dist, pid buoy);

#endif
