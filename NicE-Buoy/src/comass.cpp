/*
Get direction comppass
Compass LSM303DLHC Magnetic / accelorometer for comesating tilt
https://github.com/pololu/lsm303-arduino/tree/master
https://github.com/pololu/lsm303-arduino/tree/master/examples

*/

#include <Wire.h>
#include <LSM303.h>
#include <stdio.h>
#include <stdio.h>
#include <math.h>


#define NUM_DIRECTIONS 5

LSM303 compass;

bool LSM303ok = false;
bool InitCompass(void)
{
    LSM303ok = compass.init();
    compass.enableDefault();
    compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
    compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
    return LSM303ok;
}

static int cbufpointer = 0;
static float directions[NUM_DIRECTIONS];
float CompassAverage(float in)
{
    directions[cbufpointer++] = in;
    if(cbufpointer >=  NUM_DIRECTIONS){
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
    if (LSM303ok)
    {
        compass.read();
        //return compass.heading((LSM303::vector<int>){0, 0, 1});
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
        Serial.printf("Direction: %3.2f\n\r",heading);
        delay(100);
    }
}
