#include <Arduino.h>
#include <math.h>
#include "datastorage.h"
#include "general.h"

void initCalculate(void)
{
    computeParameters(&buoy.minOfsetDist, &buoy.maxOfsetDist, &buoy.minSpeed, &buoy.maxSpeed, true);
    Serial.printf("Stored Parameters: Minimum offset distance: %dM Maxumum offset distance: %dM, buoy minimum speed: %d%%, buoy maximum speed: %d%%\r\n", buoy.minOfsetDist, buoy.maxOfsetDist, buoy.minSpeed, buoy.maxSpeed);
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
        //Serial.printf("Stored Parameters: Minimum offset distance: %dM Maxumum offset distance: %dM, buoy minimum speed: %d%%, buoy maximum speed: %d%%\r\n", buoy.minOfsetDist, buoy.maxOfsetDist, buoy.minSpeed, buoy.maxSpeed);
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
int determineDirection(double heading1, double heading2)
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
    Calculate the new direction to stear given an relative direction change
    heading1 = delta heading2 = current direction
    retuns the new course to stear
*/
int CalcNewDirection(double heading1, double heading2)
{
    int dirout = heading2 - heading1;
    if (dirout < 0)
    {
        dirout = 360 - dirout;
    }
    return dirout;
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
calculate the speed of both motors depeniding on the angle and distance to the target.
Send the result to the esc module
if heading < 180 BB motor 100% and SB motor less
if heading > 180 SB motor 100% and BB motor less
Speed is a function of distance start slowing donw if Buoy is less than 10 meter away for targed
The distance is in meters.
*/
int CalcEngingSpeedBuoy(double magheading, float tgheading, double tgdistance, int *bb, int *sb)
{
    int speed = 0;
    double correctonAngle = 0;
    if (tgdistance < buoy.minOfsetDist) // do nothing if buoy is in 5 meter from target
    {
        speed = 0;
    }
    else
    {
        tgdistance = constrain(tgdistance, buoy.minOfsetDist, buoy.maxOfsetDist);
        speed = map(tgdistance * 100.0, buoy.minOfsetDist * 100.0, buoy.maxOfsetDist * 100.0, buoy.minSpeed * 100.0, buoy.maxSpeed * 100.0) / 100.0; // map speed 1-10 meter -> buoyMinSpeed - maxCorrectionPeedPercentage %
    }

    // Angle between calculated angel to steer and the current direction of the vessel
    correctonAngle = smallestAngle(magheading, tgheading);
    // Serial.printf("correctonAngle %.4f Angle2SpeedFactor %.4f \n",correctonAngle,Angle2SpeedFactor(correctonAngle));
    if (determineDirection(magheading, tgheading))
    {
        *bb = speed;
        *sb = int(speed * Angle2SpeedFactor(correctonAngle));
    }
    else
    {
        *bb = int(speed * Angle2SpeedFactor(correctonAngle));
        *sb = speed;
    }
    return speed;
}

void CalcEngingSpeed(float magheading, int tgheading, int speed, int *bb, int *sb)
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
void CalculatSailSpeed(int dir, int speed, int *bb, int *sb)
{
}