#include <Arduino.h>
#include <math.h>
#include "datastorage.h"
#include "general.h"
#include "gps.h"

#define EARTHRADIUS 6173
#define radian(x) (x * M_PI / 180)
#define degre(x) (x * 180 / M_PI)

pid speedpid;
pid rudderpid;
// static unsigned long speedpid.lastTime;
// static double SpeederrSum, speedpid.LastErr;

// static unsigned long AnglelastTime;
// static double rudderpid.iintergrate, AnglelastErr;

void initCalculate(void)
{
    computeParameters(&buoy.minOfsetDist, &buoy.maxOfsetDist, &buoy.minSpeed, &buoy.maxSpeed, true);
}

/*
change parematers for speed calculation
*/
void setparameters(int *minOfsetDist, int *maxOfsetDist, int *minSpeed, int *maxSpeed)
{
    int tbuoyMinOffsetDistance = buoy.minOfsetDist;
    int tbuoyMaxOffsetDistance = buoy.maxOfsetDist;
    int tbuoyMinSpeed = buoy.minSpeed;
    int tbuoymaxSpeed = buoy.maxSpeed;
    tbuoyMinOffsetDistance += *minOfsetDist;
    tbuoyMaxOffsetDistance += *maxOfsetDist;
    tbuoyMinSpeed += *minSpeed;
    tbuoymaxSpeed += *maxSpeed;
    /*
    sanety check
    */
    if (tbuoyMinOffsetDistance < 1)
    {
        tbuoyMinOffsetDistance = 1;
    }
    if (tbuoyMaxOffsetDistance > 20)
    {
        tbuoyMaxOffsetDistance = 20;
    }
    if (tbuoyMinSpeed < 0)
    {
        tbuoyMinSpeed = 0;
    }
    if (tbuoymaxSpeed > 80)
    {
        tbuoymaxSpeed = 80;
    }

    if (tbuoyMinOffsetDistance >= tbuoyMaxOffsetDistance)
    {
        *minOfsetDist = buoy.minOfsetDist; // Offset for taking action for position controll. Can be 0 till infinety but moste likely 1 - 5
        *maxOfsetDist = buoy.maxOfsetDist; // Max distance go full speed from here
        *minSpeed = buoy.minSpeed;         // Min speeed thrusters
        *maxSpeed = buoy.maxSpeed;         // Max speed thrusters
    }
    else
    {
        *minOfsetDist = tbuoyMinOffsetDistance; // Offset for taking action for position controll. Can be 0 till infinety but moste likely 1 - 5
        *maxOfsetDist = tbuoyMaxOffsetDistance; // Max distance go full speed from here
        *minSpeed = tbuoyMinSpeed;              // Min speeed thrusters
        *maxSpeed = tbuoymaxSpeed;              // Max speed thrusters
        buoy.minOfsetDist = tbuoyMinOffsetDistance;
        buoy.maxOfsetDist = tbuoyMaxOffsetDistance;
        buoy.minSpeed = tbuoyMinSpeed;
        buoy.maxSpeed = tbuoymaxSpeed;
        computeParameters(&buoy.minOfsetDist, &buoy.maxOfsetDist, &buoy.minSpeed, &buoy.maxSpeed, false);
        Serial.printf("Stored Parameters: Minimum offset distance: %dM Maxumum offset distance: %dM, buoy minimum speed: %d%%, buoy maximum speed: %d%%\r\n", buoy.minOfsetDist, buoy.maxOfsetDist, buoy.minSpeed, buoy.maxSpeed);
    }
}

/*
input: directon to go to and distance
result: target latitude and longitude will be set.
https://www.movable-type.co.uk/scripts/latlong.html
*/
void adjustPositionDirDist(int dir, int dist)
{
    /*compute in radians*/
    double radLat = radian(gpsdata.lat);
    double radLon = radian(gpsdata.lon);
    double brng = radian(dir);
    double d = dist / 1000;
    double radLat2 = asin(sin(radLat) * cos(d / EARTHRADIUS) + cos(radLat) * sin(d / EARTHRADIUS) * cos(brng));
    double radLon2 = radLon + atan2(sin(brng) * sin(d / EARTHRADIUS) * cos(radLat), cos(d / EARTHRADIUS) - sin(radLat) * sin(radLat2));
    buoy.tglatitude = degre(radLat2);
    buoy.tglongitude = degre(radLon2);
}

double smallestAngle(double heading1, double heading2)
{
    double angle = fmod(heading2 - heading1 + 360, 360); // Calculate the difference and keep it within 360 degrees
    if (angle > 180)
    {
        angle = 360 - angle; // Take the smaller angle between the two
    }
    return angle;
}

/*
    calculate the smallest angle between two directions
    return 1 if angle is >180
*/
bool determineDirection(double heading1, double heading2)
{
    double angle = fmod(heading2 - heading1 + 360, 360); // Calculate the difference and keep it within 360 degrees
    if (angle > 180)
    {
        return 1; // Angle is greater than 180, SB (turn right)
    }
    else
    {
        return 0; // Angle is less than or equal to 180, BB (Turn left)
    }
}

/*
    adjust speed Cosinus does not work here. We have to do it manually
*/
float Angle2SpeedFactor(float angle)
{
    // cos(correctonAngle * M_PI / 180.0); not working for mall angles
    angle = constrain(angle, 0, 180);
    if (angle <= 90)
    {
        return (map(angle, 0, 90, 100, 0) / 100.0);
    }
    else
    {
        return (map(angle, 90, 180, 0, -100) / 100.0);
    }
}

/*
calculate the speed sailing home.
The distance is in meters.
*/
int CalcDocSpeed(double tgdistance)
{
    tgdistance = constrain(tgdistance, 0, 8);
    return map(tgdistance, 1, 8, 0, 80); // map speed 1-10 meter -> buoyMinSpeed - maxCorrectionPeedPercentage %
}

void initRudderPid(void)
{
    pidRudderParameters(&rudderpid.kp, &rudderpid.ki, &rudderpid.kd, true);
    rudderpid.iintergrate = 0;
    rudderpid.lastErr = 0;
    rudderpid.lastTime = millis();
}

void CalcSpeedRudderBuoy(double magheading, float tgheading, int speed, int *bb, int *sb)
{
    double correctonAngle = 0;
    // Serial.printf("Mhead:%.0f thead:%d, Speed:%d\n\r", magheading, tgheading, speed);
    //  Angle between calculated angel to steer and the current direction of the vessel
    correctonAngle = smallestAngle(magheading, tgheading);
    // Serial.printf("correctonAngle %.4f Angle2SpeedFactor %.4f \n", correctonAngle, Angle2SpeedFactor(correctonAngle));
    if (determineDirection(magheading, tgheading))
    {
        *bb = speed;
        *sb = int(speed * Angle2SpeedFactor(correctonAngle));
        // Serial.println("Bakboord");
    }
    else
    {
        *bb = int(speed * Angle2SpeedFactor(correctonAngle));
        *sb = speed;
        // Serial.println("Stuurboord");
    }
}

/*
Compute power to trusters
*/
void CalcRemoteRudderBuoy(double magheading, float tgheading, int speed, int *bb, int *sb)
{
    bool dir = determineDirection(magheading, tgheading);
    double correctonAngle = smallestAngle(magheading, tgheading);
    float corr = Angle2SpeedFactor(abs(correctonAngle));
    float tbb, tsb;
    if (dir == 1)
    {
        tbb = speed + speed * (1 - corr);
        tsb = speed * corr;
    }
    else
    {
        tbb = speed * corr;
        tsb = speed + speed * (1 - corr);
    }
    *bb = (int)constrain(tbb, -60, 100);
    *sb = (int)constrain(tsb, -60, 100);
    return;
}
void rotateToTargedHeading(double mheading, float theading, int *bb, int *sb)
{
    double correctonAngle = smallestAngle(mheading, theading);
    int speed = map(correctonAngle, 20, 180, 5, 40);
    if (determineDirection(mheading, theading) == true)
    {
        *bb = speed;
        *sb = -speed;
    }
    else
    {
        *bb = -speed;
        *sb = speed;
    }
}

/*
Calculate power to thrusters
if heading < 180 BB motor 100% and SB motor less
if heading > 180 SB motor 100% and BB motor less
*/
bool CalcRudderBuoy(double magheading, float tgheading, int speed, int *bb, int *sb)
{
    double Output = 0;
    unsigned long now = millis();
    /*Compute all the working error variables*/
    double correctonAngle = smallestAngle(magheading, tgheading);
    if (correctonAngle > 45)
    {
        int bbb, sbb;
        rotateToTargedHeading(magheading, tgheading, &bbb, &sbb);
        *bb = bbb;
        *sb = sbb;
        return false;
    }
    double timeChange = (double)(now - rudderpid.lastTime);
    double error = correctonAngle / 1000;
    if (determineDirection(magheading, tgheading) == true)
    {
        error *= -1;
        correctonAngle *= -1;
    }
    rudderpid.iintergrate += error * timeChange;
    double dErr = (error - rudderpid.lastErr) / timeChange;
    if (rudderpid.ki * rudderpid.iintergrate > 90)
    {
        rudderpid.iintergrate = 90 / rudderpid.ki;
    }
    if (rudderpid.ki * rudderpid.iintergrate < -90)
    {
        rudderpid.iintergrate = -90 / rudderpid.ki;
    }
    rudderpid.p = rudderpid.kp * correctonAngle;
    rudderpid.i = rudderpid.ki * rudderpid.iintergrate;
    rudderpid.d = rudderpid.kd * dErr;
    Output = rudderpid.p + rudderpid.i + rudderpid.d;
    rudderpid.lastErr = correctonAngle;
    rudderpid.lastTime = now;
    Output = constrain(Output, -50, 50);

    /*calculate proportion thrusters*/
    *bb = (int)constrain((int)(speed - Output), -20, BUOYMAXSPEED);
    *sb = (int)constrain((int)(speed + Output), -20, BUOYMAXSPEED);
    // Serial.printf("Speed in:%d Correcton Rudder:%.1lf BB=%d,SB=%d p=%.2lf, i=%.2lf, d=%.2lf\r\n", speed, Output, (int)(speed + Output), (int)(speed - Output), rudderpid.p, rudderpid.i, rudderpid.d);
    Serial.printf("BB=%d,SB=%d    Speed in:%d  Corr=%2.2lf     p=%.2lf, i=%.2lf, d=%.2lf\r\n", (int)(speed + Output), (int)(speed - Output), speed, Output, rudderpid.p, rudderpid.i, rudderpid.d);
    return true;
}

void initSpeedPid(void)
{
    pidSpeedParameters(&speedpid.kp, &speedpid.ki, &speedpid.kd, true);
    speedpid.i = 0;
    speedpid.lastErr = 0;
    rudderpid.lastTime = millis();
}

int hooverPid(double dist)
{
    /*Do not use the pid loop if is out of range*/
    if (dist > buoy.maxOfsetDist)
    {
        return BUOYMAXSPEED;
    }
    /*How long since we last calculated*/
    double Output = 0;
    unsigned long now = millis();
    double timeChange = (double)(now - speedpid.lastTime);
    /*Compute all the working error variables*/
    double error = (dist - buoy.minOfsetDist) / 1000;
    speedpid.iintergrate += (error * timeChange);
    double dErr = (error - speedpid.lastErr) / timeChange;
    /* Do not sail backwards*/
    if (speedpid.iintergrate < 0)
    {
        speedpid.iintergrate = 0;
    }
    /*max 70% I correction*/
    if (speedpid.ki * speedpid.iintergrate > 70)
    {
        speedpid.iintergrate = 70 / speedpid.ki;
    }
    /*Compute PID Output*/
    speedpid.p = speedpid.kp * (dist - buoy.minOfsetDist);
    speedpid.i = speedpid.ki * speedpid.iintergrate;
    speedpid.d = speedpid.kd * dErr;
    Output = speedpid.p + speedpid.i + speedpid.d;
    /*Remember some variables for next time*/
    speedpid.lastErr = dist;
    speedpid.lastTime = now;
    Serial.printf("Speed:%.1lf p=%.2lf, i=%.2lf, d=%.2lf\r\n ", Output, speedpid.p, speedpid.i, speedpid.d);
    return constrain(Output, 0, BUOYMAXSPEED);
}

void CalculatSailSpeed(int dir, int speed, int *bb, int *sb)
{
}