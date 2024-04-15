#include <Arduino.h>
#include <math.h>
#include "datastorage.h"
#include "general.h"

pid buoypid;
static unsigned long SpeedlastTime;
static double SpeederrSum, SpeedlastErr;

static unsigned long AnglelastTime;
static double AngleerrSum, AnglelastErr;

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

void resetRudder(void)
{
    AngleerrSum = 0;
    AnglelastErr = 0;
    AnglelastTime = millis();
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
Calculate power to thrusters
if heading < 180 BB motor 100% and SB motor less
if heading > 180 SB motor 100% and BB motor less
*/
double rudderp = 2;
double rudderi = 0.2;
double rudderd = 0;
void CalcEngingRudderBuoy(double magheading, float tgheading, int speed, int *bb, int *sb)
{
    double Output = 0;
    unsigned long now = millis();
    double timeChange = (double)(now - AnglelastTime);
    double correctonAngle = 0;
    correctonAngle = smallestAngle(magheading, tgheading);
    //Serial.printf("Smalest angle: %lf\r\n", correctonAngle);

    // Angle between calculated angel to steer and the current direction of the vessel
    /*Compute all the working error variables*/
    double error = correctonAngle / 1000;
    if (determineDirection(magheading, tgheading) == true)
    {
        error *= -1;
        correctonAngle *= -1;
    }
    AngleerrSum += error * timeChange;
    double dErr = (error - AnglelastErr) / timeChange;

    if (rudderi * AngleerrSum > 90)
    {
        AngleerrSum = 90 / rudderi;
    }
    if (rudderi * AngleerrSum < -90)
    {
        AngleerrSum = -90 / rudderi;
    }

    Output = rudderp * correctonAngle + rudderi * AngleerrSum + rudderi * dErr;
    //Serial.printf("PID Dir=%1.2lf,  kp=%2.2lf,  I=%2.2lf\r\n", Output, rudderp * correctonAngle, rudderi * AngleerrSum);

    AnglelastErr = correctonAngle;
    AnglelastTime = now;
    Output = constrain(Output, -179, 179);
    // Serial.println(Angle2SpeedFactor(Output));
    if (Output < 0)
    {
        *bb = speed;
        *sb = (int)(speed * Angle2SpeedFactor(abs(Output)));
    }
    else
    {
        *bb = (int)(speed * Angle2SpeedFactor(abs(Output)));
        *sb = speed;
    }
    //Serial.printf("sb=%d, bb=%d",*sb,*bb);
    return;
}

void initPid(void)
{
    pidParameters(&buoypid.kp, &buoypid.ki, &buoypid.kd, true);
    SpeederrSum = 0;
    SpeedlastErr = 0;
}

int hooverPid(double dist)
{
    /*How long since we last calculated*/
    double Output = 0;
    unsigned long now = millis();
    double timeChange = (double)(now - SpeedlastTime);

    /*Compute all the working error variables*/
    double error = (dist - buoy.minOfsetDist) / 1000;
    SpeederrSum += (error * timeChange);
    double dErr = (error - SpeedlastErr) / timeChange;

    /* Do not sail backwards*/
    if (SpeederrSum < 0)
    {
        SpeederrSum = 0;
    }

    /*max 50% I correction*/
    if (buoypid.ki * SpeederrSum > 50)
    {
        SpeederrSum = 50 / buoypid.ki;
    }

    /*Compute PID Output*/
    Output = buoypid.kp * (dist - buoy.minOfsetDist) + buoypid.ki * SpeederrSum + buoypid.kd * dErr;
    buoypid.i = buoypid.ki * SpeederrSum;
    /*Remember some variables for next time*/
    SpeedlastErr = dist;
    SpeedlastTime = now;
    // Serial.printf("PID speed = %1.2lf,  kp=%2.2lf,  I=%2.2lf , Error= %2lf\r\n", Outputt, buoypid.kp * (dist - buoy.minOfsetDist), buoypid.ki / 1000 * SpeederrSum, SpeederrSum);
    return constrain(Output, 0, buoy.maxSpeed);
}

void CalculatSailSpeed(int dir, int speed, int *bb, int *sb)
{
}