#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303AGR_Mag.h>
#include <Adafruit_LSM303_Accel.h>
#include <math.h>
#include "general.h"
#include "io23017.h"
#include "io.h"
#include "datastorage.h"
#include "gps.h"
#include "calculate.h"
#include "webinterface.h"
#define NUM_DIRECTIONS 30
#define NUM_POSITIONS 50

Adafruit_LSM303AGR_Mag_Unified mag = Adafruit_LSM303AGR_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

template <typename T>
struct vector
{
    T x, y, z;
};

// Stores min and max magnetometer values from calibration
vector<float> m_max;
vector<float> m_min;
template <typename Ta, typename Tb, typename To>
void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out)
{
    out->x = (a->y * b->z) - (a->z * b->y);
    out->y = (a->z * b->x) - (a->x * b->z);
    out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb>
float vector_dot(const vector<Ta> *a, const vector<Tb> *b)
{
    return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

void vector_normalize(vector<float> *a)
{
    float mag = sqrt(vector_dot(a, a));
    a->x /= mag;
    a->y /= mag;
    a->z /= mag;
}

/*
 * Returns the angular difference in the horizontal plane between the "from" vector and north, in degrees.
 * Description of heading algorithm:
 * Shift and scale the magnetic reading based on calibration data to find
 * the North vector. Use the acceleration readings to determine the Up
 * vector (gravity is measured as an upward acceleration). The cross
 * product of North and Up vectors is East. The vectors East and North
 * form a basis for the horizontal plane. The From vector is projected
 * into the horizontal plane and the angle between the projected vector
 * and horizontal north is returned.
 */
template <typename T>
float heading(vector<T> from)
{
    sensors_event_t event;
    mag.getEvent(&event);
    vector<float> temp_m = {event.magnetic.x, event.magnetic.y, event.magnetic.z};

    accel.getEvent(&event);
    vector<float> a = {event.acceleration.x, event.acceleration.y, event.acceleration.z};

    // Important: subtract average of min and max from magnetometer calibration
    temp_m.x -= (m_min.x + m_max.x) / 2;
    temp_m.y -= (m_min.y + m_max.y) / 2;
    temp_m.z -= (m_min.z + m_max.z) / 2;

    // Compute east and north vectors
    vector<float> east;
    vector<float> north;
    vector_cross(&temp_m, &a, &east);
    vector_normalize(&east);
    vector_cross(&a, &east, &north);
    vector_normalize(&north);

    // compute heading
    // float heading = atan2(vector_dot(&east, &from), vector_dot(&north, &from)) * 180 / PI;
    float heading = atan2(vector_dot(&north, &from), vector_dot(&east, &from)) * 180 / PI;
    if (heading < 0)
    {
        heading += 360;
    }
    return heading;
}

// MaxX:63.750000 MaxY:91.199997 MaxZ:-5.700000 MinX:-12.900000 MinY:-101.550003 MinZ:-101.550003
// MaxX:68.699997 MaxY:74.250000 MaxZ:-6.750000 MinX:-29.250000 MinY:-13.200000 MinZ:-100.650002
bool InitCompass(void)
{
    float min_mag[3], max_mag[3];
    CompassCallibrationFactorsFloat(&max_mag[0], &max_mag[1], &max_mag[2], &min_mag[0], &min_mag[1], &min_mag[2], true); //  get callibration data
    // m_min = (vector<float>){min_mag[0], min_mag[1], min_mag[2]};
    // m_max = (vector<float>){max_mag[0], max_mag[1], max_mag[2]};

    if (!mag.begin())
    {
        Serial.println("Unable to initialize LSM303 magnetometer");
        while (1)
            ;
    }

    if (!accel.begin())
    {
        Serial.println("Unable to initialize LSM303 accelerometer");
    }

    accel.setRange(LSM303_RANGE_4G);
    accel.setMode(LSM303_MODE_NORMAL);
    sensors_event_t event;
    mag.getEvent(&event);
    CompassOffsetCorrection(&buoy.magneticCorrection, true);
    MechanicalCorrection(&buoy.mechanicCorrection, true);
    return 0;
}

bool CalibrateCompass(void)
{
    static unsigned long calstamp;
    sensors_event_t event;
    u_int lokcnt = 0;
    bool lokon = 0;
    Serial.println("Callibrating compass now!!!");
    calstamp = millis();
    float min_mag[3], max_mag[3];
    for (int i = 0; i < 3; i++)
    {
        min_mag[i] = +2147483647;
        max_mag[i] = -2147483648;
    }

    while (millis() - calstamp <= 1000 * 60) // 1 minute callibrating
    {
        mag.getEvent(&event);
        min_mag[0] = min(min_mag[0], event.magnetic.x);
        max_mag[0] = max(max_mag[0], event.magnetic.x);
        min_mag[1] = min(min_mag[1], event.magnetic.y);
        max_mag[1] = max(max_mag[1], event.magnetic.y);
        min_mag[2] = min(min_mag[2], event.magnetic.z);
        max_mag[2] = max(max_mag[2], event.magnetic.z);

        if (lokcnt++ > 250)
        {
            lokcnt = 0;
            mcp.digitalWrite(MAINSSWITCH_LEDGREEN_GPB, lokon);
            lokon = !lokon;
            mcp.digitalWrite(MAINSSWITCH_LEDRED_GPB, lokon);
            Serial.printf("Calllibration factors Compass: MaxXYZ: {%f, %f, %f}; MinXYZ {%f, %f, %f};\r\n", max_mag[0], max_mag[1], max_mag[2], min_mag[0], min_mag[1], min_mag[2]);
        }
    }
    CompassCallibrationFactorsFloat(&max_mag[0], &max_mag[1], &max_mag[2], &min_mag[0], &min_mag[1], &min_mag[2], false); //  store callibration data
    m_min = (vector<float>){min_mag[0], min_mag[1], min_mag[2]};
    m_max = (vector<float>){max_mag[0], max_mag[1], max_mag[2]};
    Serial.printf("New callibration stored!!!\n\r");
    Serial.printf("Calllibration factors Compass: MaxXYZ: {%f, %f, %f}; MinXYZ {%f, %f, %f};\r\n", max_mag[0], max_mag[1], max_mag[2], min_mag[0], min_mag[1], min_mag[2]);
    mcp.digitalWrite(MAINSSWITCH_LEDGREEN_GPB, 1);
    mcp.digitalWrite(MAINSSWITCH_LEDRED_GPB, 0);

    return 0;
}

float GetHeading(void)
{
    float mHeding = heading((vector<int>){0, 1, 0}); // 180 correction due placement on pcb
    mHeding = mHeding + buoy.magneticCorrection;
    if (mHeding < 0)
    {
        mHeding = mHeding + 360.0;
    }
    else if (mHeding > 360)
    {
        mHeding = mHeding - 360.0;
    }
    return mHeding;
}
float GetHeadingRaw(void)
{
    float t = heading((vector<int>){0, 1, 0});
    if (t > 360)
    {
        t -= 360;
    }
    else if (t < 0)
    {
        t += 360;
    }
    return t;
}

static int cbufpointer = 0;
static float directions[NUM_DIRECTIONS];
/*
Average of NUM_DIRECTIONS samples
*/
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
    sum_x /= NUM_DIRECTIONS;
    sum_y /= NUM_DIRECTIONS;
    // Calculate the average direction in degrees
    avg_dir = atan2(sum_y, sum_x) * 180.0 / M_PI;
    if (avg_dir < 0)
    {
        avg_dir += 360.0;
    }
    if (avg_dir > 360)
    {
        avg_dir -= 360;
    }
    return avg_dir;
}

void GpsAverage(double *lat, double *lon)
{
}

#define ABUF 20
double tbuf[2][ABUF];
int pointer = 0;
int stage = 0;
unsigned long timer;
/*
Set sailspeed at 50%
take for 10 sconds the avarage magnetic neading and gps heading
determ de differenanc.
The result is the offset off the magnetic compass
returns -1 if no solution is known yet
*/
int linMagCalib(int *corr)
{
    int ret = -1;
    float tmp_mhdg = 0;
    char bufff[100];

    switch (stage)
    {
    case 0:
        buoy.speed = 30;
        buoy.speedbb = 30;
        buoy.speedsb = 30;
        pointer = 0;
        buoy.magneticCorrection = 0;
        buoy.mechanicCorrection = 0;
        initRudderPid();
        timer = millis();
        stage++;
        break;
    case 1:
        while (timer + 5000 > millis())
            break;
        timer = millis();
        tmp_mhdg = gpsdata.cource;
        stage++;
        break;
    case 2:
        if (timer + 1000 > millis())
        {
            timer = millis();
            double error = ComputeSmallestAngleDir(tmp_mhdg, gpsdata.cource);
            if (error < 5)
            {
                stage++;
                break;
            }
            else
            {
                if (determineDirection(tmp_mhdg, gpsdata.cource))
                {
                    buoy.mechanicCorrection -= 1;
                }
                else
                {
                    buoy.mechanicCorrection += 1;
                }
                buoy.speedbb = (int)(buoy.speed * (1 - tan(radians(buoy.mechanicCorrection))));
                buoy.speedsb = (int)(buoy.speed * (1 + tan(radians(buoy.mechanicCorrection))));
                /*Sanety check*/
                buoy.speedbb = constrain(buoy.speedbb, -buoy.maxSpeed, buoy.maxSpeed);
                buoy.speedsb = constrain(buoy.speedsb, -buoy.maxSpeed, buoy.maxSpeed);
                tmp_mhdg = buoy.mheading;
                break;
            }
        }
        break;
    case 3:
        if (timer + 1500 > millis())
        {
            timer = millis();
            tbuf[0][pointer] = (double)GetHeadingRaw();
            tbuf[1][pointer++] = gpsdata.cource;
            String msg = "<Calib><mag heading>" + String(tbuf[0][pointer - 1]) + "><gps heading><" + String(tbuf[1][pointer - 1]) + ">";
            strcpy(bufff, msg.c_str());
            udpsend(bufff);

            if (pointer >= ABUF)
            {
                buoy.speedbb = 0;
                buoy.speedsb = 0;
                double msum_x = 0.0, msum_y = 0.0, mavg_dir;
                double gsum_x = 0.0, gsum_y = 0.0, gavg_dir;
                for (int tel = 0; tel < ABUF; tel++)
                {
                    msum_x += cos(tbuf[0][tel] * M_PI / 180.0);
                    msum_y += sin(tbuf[0][tel] * M_PI / 180.0);
                    gsum_x += cos(tbuf[1][tel] * M_PI / 180.0);
                    gsum_y += sin(tbuf[1][tel] * M_PI / 180.0);
                }
                mavg_dir = atan2(msum_y, msum_x) * 180.0 / M_PI;
                gavg_dir = atan2(gsum_y, gsum_x) * 180.0 / M_PI;
                if (mavg_dir < 0)
                {
                    mavg_dir += 360.0;
                }
                if (mavg_dir > 360)
                {
                    mavg_dir -= 360;
                }
                if (gavg_dir < 0)
                {
                    gavg_dir += 360.0;
                }
                if (gavg_dir > 360)
                {
                    gavg_dir -= 360;
                }
                String msg = "<Calib result><mag heading>" + String(mavg_dir) + "><gps heading><" + String(gavg_dir) + ">";
                strcpy(bufff, msg.c_str());
                udpsend(bufff);

                *corr = int(gavg_dir - mavg_dir);
                if (*corr < 0)
                {
                    *corr += 360.0;
                }
                if (*corr > 360)
                {
                    *corr -= 360;
                }
                if (*corr > 180)
                {
                    *corr = 360 - *corr;
                    *corr *= -1;
                }
                buoy.magneticCorrection = *corr;
                CompassOffsetCorrection(&buoy.magneticCorrection, false);
                pointer = buoy.mechanicCorrection;
                MechanicalCorrection(&pointer, false);
                msg = "<Calib stored><buoy.magneticCorrection>" + String(buoy.magneticCorrection) + "><buoy.mechanicCorrection><" + String(buoy.mechanicCorrection) + ">";
                strcpy(bufff, msg.c_str());
                udpsend(bufff);

                ret = 0;
                stage = 0;
            }
        }
        break;
    default:
        stage = 0;
        break;
    }
    return ret;
}
