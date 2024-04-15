#include <Arduino.h>
#include <math.h>
#include "datastorage.h"
#include "general.h"

pid speedpid;
pid rudderpid;
// static unsigned long speedpid.lastTime;
// static double SpeederrSum, speedpid.LastErr;

// static unsigned long AnglelastTime;
// static double rudderpid.i, AnglelastErr;

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
    rudderpid.i = 0;
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

/*
Calculate power to thrusters
if heading < 180 BB motor 100% and SB motor less
if heading > 180 SB motor 100% and BB motor less
*/
void CalcRudderBuoy(double magheading, float tgheading, int speed, int *bb, int *sb)
{
    double Output = 0;
    unsigned long now = millis();
    double timeChange = (double)(now - rudderpid.lastTime);
    double correctonAngle = smallestAngle(magheading, tgheading);
     /*Compute all the working error variables*/
    double error = correctonAngle / 1000;
    if (determineDirection(magheading, tgheading) == true)
    {
        error *= -1;
        correctonAngle *= -1;
    }
    rudderpid.i += error * timeChange;
    double dErr = (error - rudderpid.lastErr) / timeChange;
    if (rudderpid.ki * rudderpid.i > 90)
    {
        rudderpid.i = 90 / rudderpid.ki;
    }
    if (rudderpid.ki * rudderpid.i < -90)
    {
        rudderpid.i = -90 / rudderpid.ki;
    }

    Output = rudderpid.kp * correctonAngle + rudderpid.ki *rudderpid.i + rudderpid.kd * dErr;
    Serial.printf("PID Dir=%1.2lf,  kp=%2.2lf,  I=%2.2lf D=%2.8lf\r\n", Output, rudderpid.kp * correctonAngle, rudderpid.ki *rudderpid.i, rudderpid.kd * dErr);

    rudderpid.lastErr = correctonAngle;
    rudderpid.lastTime = now;
    Output = constrain(Output, -179, 179);
    float corr = Angle2SpeedFactor(abs(Output));
    float tbb, tsb = 0;
    if (Output < 0)
    {
        tbb = speed; // + speed * (1 - corr);
        tsb = speed * corr;
    }
    else
    {
        tbb = speed * corr;
        tsb = speed; //+ speed * (1 - corr);
    }
    *bb = (int)constrain(tbb, -20, buoy.maxSpeed);
    *sb = (int)constrain(tsb, -20, buoy.maxSpeed);
    // Serial.printf("corr=%2.3f ,delata + %d, sb=%d, bb=%d\r\n", corr, (int)(speed * (1 - corr)), *sb, *bb);
    return;
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
    /*How long since we last calculated*/
    double Output = 0;
    unsigned long now = millis();
    double timeChange = (double)(now - speedpid.lastTime);

    /*Compute all the working error variables*/
    double error = (dist - buoy.minOfsetDist) / 1000;
    speedpid.i += (error * timeChange);
    double dErr = (error - speedpid.lastErr) / timeChange;

    /* Do not sail backwards*/
    if (speedpid.i < 0)
    {
        speedpid.i = 0;
    }

    /*max 50% I correction*/
    if (speedpid.ki * speedpid.i > 70)
    {
        speedpid.i = 70 / speedpid.ki;
    }

    /*Compute PID Output*/
    Output = speedpid.kp * (dist - buoy.minOfsetDist) + speedpid.ki * speedpid.i + speedpid.kd * dErr;
    /*Remember some variables for next time*/
    speedpid.lastErr = dist;
    speedpid.lastTime = now;
    // Serial.printf("PID speed = %1.2lf,  kp=%2.2lf,  I=%2.2lf , Error= %2lf\r\n", Outputt, speedpid.kp * (dist - buoy.minOfsetDist), speedpid.ki / 1000 * SpeederrSum, SpeederrSum);
    return constrain(Output, 0, buoy.maxSpeed);
}

void CalculatSailSpeed(int dir, int speed, int *bb, int *sb)
{
}