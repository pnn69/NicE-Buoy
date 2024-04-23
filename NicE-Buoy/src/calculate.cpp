/*
Calulate all stuff
gecode calculations: Https://www.movable-type.co.uk/scripts/latlong.html
*/
#include <Arduino.h>
#include <math.h>
#include "datastorage.h"
#include "general.h"
#include "gps.h"

/*some constans*/
#define EARTHRADIUS 6371
#define radian(x) (x * M_PI / 180)
#define degre(x) (x * 180 / M_PI)

/*PID structs*/
pid speedpid;
pid rudderpid;

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
input: directon to go to, distance and start position
return: target latitude and longitude
*/
void adjustPositionDirDist(int dir, double dist, double *lat, double *lon)
{
    /*compute in radians*/
    double radLat = radian(*lat);
    double radLon = radian(*lon);
    double brng = radian(dir);
    double d = dist * 1.0 / 1000;
    /*compute*/
    double radLat2 = asin(sin(radLat) * cos(d / EARTHRADIUS) + cos(radLat) * sin(d / EARTHRADIUS) * cos(brng));
    double radLon2 = radLon + atan2(sin(brng) * sin(d / EARTHRADIUS) * cos(radLat), cos(d / EARTHRADIUS) - sin(radLat) * sin(radLat2));
    /*plot old pos*/
    // Serial.printf("Old lock google: https://www.google.nl/maps/@%2.12lf,%2.12lf,16z?entry=ttu\r\n", *lat, *lon);
    // Serial.printf("Old lock openst: https://www.openstreetmap.org/#map=19/%2.12lf/%2.12lf\r\n", *lat, *lon);
    /*convert back to degrees*/
    *lat = degre(radLat2);
    *lon = degre(radLon2);
    /*plot new pos*/
    // Serial.printf("New lock google: https://www.google.nl/maps/@%2.12lf,%2.12lf,16z?entry=ttu\r\n", *lat, *lon);
    // Serial.printf("New lock openst: https://www.openstreetmap.org/#map=19/%2.12lf/%2.12lf\r\n", *lat, *lon);
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

double ComputeSmallestAngleDir(double heading1, double heading2)
{
    double angle = fmod(heading2 - heading1 + 360, 360); // Calculate the difference and keep it within 360 degrees
    if (angle > 180)
    {
        return angle - 360; // Angle is greater than 180, SB (turn right)
    }
    return angle; // Angle is less than or equal to 180, BB (Turn left)
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
double CalcDocSpeed(double tgdistance)
{
    tgdistance = constrain(tgdistance, 0.5, 5);
    return map(tgdistance, 0.5, 5, BUOYMINSPEED, BUOYMAXSPEED); // map speed 0.5-5 meter -> BUOYMINSPEED <-> BUOYMAXSPEED
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

    double error = ComputeSmallestAngleDir(magheading, tgheading);
    tbb = (int)(speed * cos(radian(error)) * (1 - sin(radian(error))));
    tsb = (int)(speed * cos(radian(error)) * (1 - sin(radian(error)) * -1));
    *bb = (int)constrain(tbb, -60, 100);
    *sb = (int)constrain(tsb, -60, 100);
    Serial.printf("Error=%lf BB=%d SB=%d\r\n\r\n", error, *bb, *sb);
    return;
}

void initRudderPid(void)
{
    pidRudderParameters(&rudderpid.kp, &rudderpid.ki, &rudderpid.kd, true);
    rudderpid.iintergrate = 0;
    rudderpid.lastErr = 0;
    rudderpid.lastTime = millis();
}
/*
Calculate power to thrusters
If distance between 0.5 and 1.6 meter rotate at the slowest speed.
If the angle between the targed direction and magnetic heading is greater then +/-80 degrees easy turn back into range
else do normal rudder calculation.
*/

bool CalcRudderBuoy(double magheading, float tgheading, double tdistance, int speed, int *bb, int *sb)
{
    /*Compute all the working error variables*/
    double error = ComputeSmallestAngleDir(magheading, tgheading);
    /*If distance between 0.5 and 1.6 meter rotate at the slowest speed.*/
    // if (tdistance > 0.5 && tdistance < 1.8 && speed < BUOYMINSPEED)
    // {
    //     if (error > 10) // turn BB slow
    //     {
    //         *bb = -BUOYMINSPEED;
    //         *sb = BUOYMINSPEED;
    //     }
    //     if (error < -10) // turn SB slow
    //     {
    //         *bb = BUOYMINSPEED;
    //         *sb = -BUOYMINSPEED;
    //     }
    //     return false;
    // }

    /*quit if out of range*/
    if (error < -80)
    {
        *bb = (int)constrain(speed / 2.9, 0, 100); // 2.9=Speed*COS(80)*(1-SIN(80))
        *sb = (int)(speed * cos(radian(error)) * (1 - sin(radian(error))));
        return false;
    }
    if (error > 80)
    {
        *bb = (int)(speed * cos(radian(error)) * (1 - sin(radian(error))));
        *sb = (int)constrain(speed / 2.9, 0, 100);
        return false;
    }

    /*calculate proportion thrusters Not used now!!!!!*/
    unsigned long now = millis();
    double timeChange = (double)(now - rudderpid.lastTime);
    double dErr = (error - rudderpid.lastErr) / timeChange;
    /* This has te be sorted out*/
    double Output = 0;
    rudderpid.iintergrate += error * timeChange;
    if (rudderpid.ki * rudderpid.iintergrate > 80)
    {
        rudderpid.iintergrate = 80 / (rudderpid.ki);
    }
    if (rudderpid.ki * rudderpid.iintergrate < -80)
    {
        rudderpid.iintergrate = -80 / (rudderpid.ki);
    }
    rudderpid.p = rudderpid.kp * error;
    rudderpid.i = rudderpid.ki * rudderpid.iintergrate;
    rudderpid.d = rudderpid.kd * dErr;
    Output = rudderpid.p + rudderpid.i + rudderpid.d;
    rudderpid.lastErr = error;
    rudderpid.lastTime = now;
    int tb = (int)constrain(speed * cos(radian(Output)) * (1 - sin(radian(Output))), 0, 100);
    int ts = (int)constrain(speed * cos(radian(Output)) * (1 - sin(radian(Output)) * -1), 0, 100);
    Serial.printf("BB=%d,SB=%d    Speed in:%d  Corr=%2.2lf     p=%.2lf, i=%.2lf, d=%lf\r\n", tb, ts, speed, Output, rudderpid.p, rudderpid.i, rudderpid.d);

    /*calculate proportion thrusters*/
    *bb = (int)(speed * cos(radian(error)) * (1 - sin(radian(error))));
    *sb = (int)(speed * cos(radian(error)) * (1 - sin(radian(error)) * -1));

    //Serial.printf("Speed=%d BB=%d SB=%d\r\n", speed, *bb, *sb);
    if (*bb > BUOYMAXSPEED)
    {
        *sb += *bb - BUOYMAXSPEED;
    }
    if (*sb > BUOYMAXSPEED)
    {
        *bb += *sb - BUOYMAXSPEED;
    }

    /*Sanety check*/
    *bb = constrain(*bb, -BUOYMAXSPEED, BUOYMAXSPEED);
    *sb = constrain(*sb, -BUOYMAXSPEED, BUOYMAXSPEED);
    return true;
}

void initSpeedPid(void)
{
    pidSpeedParameters(&speedpid.kp, &speedpid.ki, &speedpid.kd, true);
    speedpid.i = 0;
    speedpid.iintergrate = 0;
    speedpid.lastErr = 0;
    speedpid.lastTime = millis();
}
/*
Only ust PID if the distancs is less than buoy.maxOfsetDist return BUOYMAXSPEED otherwise.

*/
int hooverPid(double dist)
{
    /*Do not use the pid loop if distance is to big just go full power*/
    if (dist > buoy.maxOfsetDist)
    {
        initSpeedPid();
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