/*
Get direction comppass
Compass LSM303DLHC Magnetic / accelorometer for comesating tilt
https://github.com/pololu/lsm303-arduino/tree/master
https://github.com/pololu/lsm303-arduino/tree/master/examples

*/

#include <Wire.h>
#include <LSM303.h>
#include <stdio.h>
#include <math.h>
#include "datastorage.h"
#include "io.h"

#define NUM_DIRECTIONS 5
#define NUM_POSITIONS 50

LSM303 compass;

bool LSM303ok = false;
bool InitCompass(void)
{
    int16_t MaxX, MaxY, MaxZ, MinX, MinY, MinZ;

    LSM303ok = compass.init();
    compass.enableDefault();
    CompassCallibrationFactors(&MaxX, &MaxY, &MaxZ, &MinX, &MinY, &MinZ, true); // get stored callibration data
    // compass.m_min = (LSM303::vector<int16_t>){-535, -645, -382};
    // compass.m_max = (LSM303::vector<int16_t>){+576, +466, +754};
    Serial.printf("Calllibration factors Compass: MaxX:%d MaxY:%d MaxZ:%d MaxX:%d MaxY:%d MaxZ:%d\r\n", MaxX, MaxY, MaxZ, MinX, MinY, MinZ);
    compass.m_min = (LSM303::vector<int16_t>){MinX, MinY, MinZ};
    compass.m_max = (LSM303::vector<int16_t>){MaxX, MaxY, MaxZ};

    return LSM303ok;
}

bool CalibrateCompass(void)
{
    static unsigned long calstamp;
    LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};
    u_int lokcnt = 0;
    bool lokon = 0;
    Serial.println("Callibrating compass now!!!");
    calstamp = millis();
    while (millis() - calstamp <= 1000*60) // 1 minute callibrating
    {
        compass.read();
        running_min.x = min(running_min.x, compass.m.x);
        running_min.y = min(running_min.y, compass.m.y);
        running_min.z = min(running_min.z, compass.m.z);
        running_max.x = max(running_max.x, compass.m.x);
        running_max.y = max(running_max.y, compass.m.y);
        running_max.z = max(running_max.z, compass.m.z);
        if (lokcnt++ > 250)
        {
            lokcnt = 0;
            digitalWrite(LEDSTRIP1, lokon);
            lokon = !lokon;
            Serial.printf("Calllibration factors Compass: MaxX:%d MaxY:%d MaxZ:%d MaxX:%d MaxY:%d MaxZ:%d\r\n", running_max.x, running_max.y, running_max.z, running_min.x, running_min.y, running_min.z);
        }
    }
    CompassCallibrationFactors(&running_max.x, &running_max.y, &running_max.z, &running_min.x, &running_min.y, &running_min.z, false); //  store callibration data
    Serial.printf("New callibration stored!!!\n\r");
    return 0;
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
    // float tmp;
    if (LSM303ok)
    {
        compass.read();
        // tmp = compass.heading((LSM303::vector<int>){1, 0, 0});
        // Serial.printf("Dir x: %0.1f ", tmp);
        // tmp = compass.heading((LSM303::vector<int>){0, 1, 0});
        // Serial.printf("Dir y: %0.1f ", tmp);
        // tmp = compass.heading((LSM303::vector<int>){0, 0, 1});
        // Serial.printf("Dir z: %0.1f\r\n", tmp);
        // tmp = compass.heading();
        return compass.heading();
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
