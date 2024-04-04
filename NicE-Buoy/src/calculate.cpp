#include <Arduino.h>
#include <math.h>

#define BOUYMINOFFSETDISTANCE 5  // Offset for taking action for position controll. Can be 0 till infinety but moste likely 1 - 5
#define BOUYMAXOFFSETDISTANCE 20 // Max distance go full speed from here
#define BOUYMINSPEED 5
#define BUOYMAXCORRECTIONSPEED 50

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
int CalcEngingSpeedBuoy(float magheading, int tgheading, unsigned long tgdistance, int *bb, int *sb)
{
    int speed = 0;
    double correctonAngle = 0;
    if (tgdistance < BOUYMINOFFSETDISTANCE) // do nothing if buoy is in 5 meter from target
    {
        speed = 0;
    }
    else
    {
        tgdistance = constrain(tgdistance, BOUYMINOFFSETDISTANCE, BOUYMAXOFFSETDISTANCE);
        speed = map(tgdistance, BOUYMINOFFSETDISTANCE, BOUYMAXOFFSETDISTANCE, BOUYMINSPEED, BUOYMAXCORRECTIONSPEED); // map speed 1-10 meter -> BOUYMINSPEED - BUOYMAXCORRECTIONSPEED %
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