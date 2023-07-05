/*
Get direction comppass
Compass LSM303DLHC Magnetic / accelorometer for comesating tilt
https://github.com/pololu/lsm303-arduino/tree/master
https://github.com/pololu/lsm303-arduino/tree/master/examples

https://www.youtube.com/watch?v=HHJEaB1iS30

*/

#include <Wire.h>
#include <stdio.h>
#include <math.h>
#include "bmm150.h"
#include "bmm150_defs.h"

#define NUM_DIRECTIONS 5
#define NUM_POSITIONS 50

BMM150 bmm = BMM150();

bool COMPASSok = false;
bool InitCompass(void)
{
    if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM)
    {
        Serial.println("Chip ID can not read!");
        return false;
    }
    Serial.println("Compass found!");
    COMPASSok = true;
    return true;
}

static int cbufpointer = 0;
static float directions[NUM_DIRECTIONS];
float CompassAverage(float in)
{
    directions[cbufpointer++] = in;
    if (cbufpointer >= NUM_DIRECTIONS)
    {
        cbufpointer = 0;
    }
    float sum_x = 0.0, sum_y = 0.0, avg_dir;
    // Convert the compass directions to Cartesian coordinates
    for (int i = 0; i < NUM_DIRECTIONS; i++)
    {
        sum_x += cos(directions[i] * M_PI / 180.0);
        sum_y += sin(directions[i] * M_PI / 180.0);
    }

    // Calculate the average direction in degrees
    avg_dir = atan2(sum_y, sum_x) * 180.0 / M_PI;
    if (avg_dir < 0)
    {
        avg_dir += 360.0;
    }
    return avg_dir;
}

float GetHeading(void)
{
    if (COMPASSok)
    {
        bmm150_mag_data value;
        bmm.read_mag_data();

        value.x = bmm.raw_mag_data.raw_datax;
        value.y = bmm.raw_mag_data.raw_datay;
        value.z = bmm.raw_mag_data.raw_dataz;

        float xyHeading = atan2(value.x, value.y);
        float zxHeading = atan2(value.z, value.x);
        float heading = xyHeading;

        if (heading < 0)
        {
            heading += 2 * PI;
        }
        if (heading > 2 * PI)
        {
            heading -= 2 * PI;
        }
        float headingDegrees = heading * 180 / M_PI;
        float xyHeadingDegrees = xyHeading * 180 / M_PI;
        float zxHeadingDegrees = zxHeading * 180 / M_PI;
        if (xyHeadingDegrees < 0)
        {
            xyHeadingDegrees += 360;
        }
        if (zxHeadingDegrees < 0)
        {
            zxHeadingDegrees += 360;
        }
        // Serial.print("Heading: ");
        // Serial.println(headingDegrees);
        return xyHeadingDegrees;
        // return headingDegrees;
    }
    return -1;
}

void CompassTask(void *arg)
{
    InitCompass();
    while (1)
    {
        float heading = GetHeading();
        heading = CompassAverage(heading);
        Serial.printf("Direction: %3.2f\n\r", heading);
        delay(100);
    }
}

static int pbufpointer = 0;
static double lats[NUM_POSITIONS];
static double lons[NUM_POSITIONS];
void GpsAverage(double *lat, double *lon)
{
    lats[pbufpointer] = *lat;
    lons[pbufpointer++] = *lon;
    if (pbufpointer >= NUM_POSITIONS)
    {
        pbufpointer = 0;
    }
    double latss = 0, lonss = 0;
    for (int i = 0; i < NUM_POSITIONS; i++)
    {
        latss += lats[i];
        lonss += lons[i];
    }
    *lat = latss / NUM_POSITIONS;
    *lon = lonss / NUM_POSITIONS;
}
