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

/*Approimate roling average deafualt 100 samples*/
double approxRollingAverage(double avg, double input)
{
    avg -= avg / 100;
    avg += input / 100;
    return avg;
}

double averigeWindRose(double samples[], int n)
{
    double sumSin = 0, sumCos = 0;
    for (int i = 0; i < n; ++i)
    {
        double angle = (samples[i + 3] * M_PI) / 180.0; // Convert to radians
        sumSin += sin(angle);
        sumCos += cos(angle);
    }
    double meanAngle = atan2(sumSin / n, sumCos / n);               // Compute mean angle
    double meanDegrees = fmod(meanAngle * 180.0 / M_PI + 360, 360); // Convert mean angle to degrees
    samples[0] = meanDegrees;
    return meanDegrees;
}
/*
compute deviation of a buffer pos
Structure buf[averige][deviation][data0][datan...]
*/
double deviationWindRose(double samples[], int n)
{
    double mean = averigeWindRose(samples, n);
    double sumSquaredCircularDiff = 0;
    for (int i = 0; i < n; ++i)
    {
        double diff = samples[i + 3] - mean;
        if (diff > 180)
        {
            diff -= 360;
        }
        else if (diff < -180)
        {
            diff += 360;
        }
        sumSquaredCircularDiff += diff * diff;
    }
    samples[1] = sqrt(sumSquaredCircularDiff / n);
    return samples[1];
}

/*
Add new data in buffer
Structure buf[averige][deviation][data0][datan...]
*/
void addNewSampleInBuffer(double *input, int buflen, double nwdata)
{
    if (input[2] >= buflen)
    {
        input[2] = 0;
    }
    input[(int)input[2] + 3] = nwdata;
    input[2]++;
}

/*
change parematers for speed calculation
*/
void setparameters(int *minOfsetDist, int *maxOfsetDist, int *minSpeed, int *maxSpeed)
{
    /*
    sanety check
    */
    if (*minOfsetDist < 0)
    {
        *minOfsetDist = 2;
    }
    if (*maxOfsetDist > 100)
    {
        *maxOfsetDist = 20;
    }
    if (*minSpeed < 0)
    {
        *minSpeed = 0;
    }
    if (*maxSpeed > 100)
    {
        *maxSpeed = 80;
    }
    if (*minOfsetDist <= *maxOfsetDist)
    {
        buoy.minOfsetDist = *minOfsetDist;
        buoy.maxOfsetDist = *maxOfsetDist;
        buoy.minSpeed = *minSpeed;
        buoy.maxSpeed = *maxSpeed;
        computeParameters(&buoy.minOfsetDist, &buoy.maxOfsetDist, &buoy.minSpeed, &buoy.maxSpeed, false);
        // Serial.printf("Stored Parameters: Minimum offset distance: %dM Maxumum offset distance: %dM, buoy minimum speed: %d%%, buoy maximum speed: %d%%\r\n", buoy.minOfsetDist, buoy.maxOfsetDist, buoy.minSpeed, buoy.maxSpeed);
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
    // Serial.printf("Old lock google: https://www.google.nl/maps/@%2.12lf,%2.12lf\r\n", *lat, *lon);
    // Serial.printf("Old lock openst: https://www.openstreetmap.org/#map=19/%2.12lf/%2.12lf\r\n", *lat, *lon);
    /*convert back to degrees*/
    *lat = degre(radLat2);
    *lon = degre(radLon2);
    /*plot new pos*/
    // Serial.printf("New lock google: https://www.google.nl/maps/@%2.12lf,%2.12lf\r\n", *lat, *lon);
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
    tgdistance = constrain(tgdistance, 0.5, 8);
    return map(tgdistance, 0, 5, 0, 50); // map speed 0.5-5 meter -> BUOYMINSPEED <-> BUOYMAXSPEED
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
if distance between 0.5 and 1.5 meter rotate at the slowest speed.
else do normal rudder calculation.
*/
#define ILIM 35 // Maximum interal limit (35% power)
bool CalcRudderBuoy(double magheading, float tgheading, double tdistance, int speed, int *bb, int *sb)
{
    double error = ComputeSmallestAngleDir(magheading, tgheading);
    /*Rotate to target direction first*/
    if (tdistance > 0.5 && abs(error) > 45)
    {
        float power = map(abs(error),45,180,7,70);
        power =  sin(radians(power))*100;
        power = (int)map(power,13,100,13,buoy.maxSpeed/2);

        if (error >= 0)
        {
            *bb = -power;
            *sb = power;
        }
        else
        {
            *bb = power;
            *sb = -power;
        }
        return false;
    }
    /*calculate proportion thrusters*/
    /*Scale error in range for tan*/
    error = map(error, -180, 180, -80, 80);
    unsigned long now = millis();
    double timeChange = (double)(now - rudderpid.lastTime);
    double dErr = (error - rudderpid.lastErr) / timeChange;
    rudderpid.iintergrate += error * timeChange;
    if ((rudderpid.ki / 1000) * rudderpid.iintergrate > ILIM)
    {
        rudderpid.iintergrate = ILIM / ((rudderpid.ki / 1000));
    }
    if ((rudderpid.ki / 1000) * rudderpid.iintergrate < -ILIM)
    {
        rudderpid.iintergrate = -ILIM / ((rudderpid.ki / 1000));
    }
    rudderpid.p = rudderpid.kp * error;
    rudderpid.i = (rudderpid.ki / 1000) * rudderpid.iintergrate;
    rudderpid.d = rudderpid.kd * dErr;
    double adj = rudderpid.p + rudderpid.i + rudderpid.d;
    rudderpid.lastErr = error;
    rudderpid.lastTime = now;
    *bb = (int)(speed * (1 - tan(radians(adj))));
    *sb = (int)(speed * (1 + tan(radians(adj))));
    /*Sanety check*/
    *bb = constrain(*bb, -buoy.maxSpeed, buoy.maxSpeed);
    *sb = constrain(*sb, -buoy.maxSpeed, buoy.maxSpeed);
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
    if (buoy.maxSpeed == 51) /*old config use this by setting max speed to 51*/
    {
        if (dist > buoy.maxOfsetDist)
        {
            return buoy.maxSpeed;
        }
        if (speedpid.armIntergrator == false)
        {
            speedpid.armIntergrator = true;
            speedpid.iintergrate = 0;
        }
    }
    else
    {
        /*Do not use the pid loop if distance is to big just go full power*/
        if (speedpid.armIntergrator == false)
        {
            if (dist > 3)
            {
                speedpid.iintergrate = 0;
                return buoy.maxSpeed;
            }
            else
            {
                speedpid.armIntergrator = true;
                speedpid.iintergrate = 0;
            }
        }
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
    return (int)constrain(Output, 0, buoy.maxSpeed);
}