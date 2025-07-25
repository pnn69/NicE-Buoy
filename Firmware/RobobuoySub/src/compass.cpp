#include <Arduino.h>
#include <Wire.h>
#include <RoboCompute.h>
#include "compass.h"
#include "main.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_LSM303_Accel.h>
#include <math.h>
#include "leds.h"
#include "buzzer.h"
#include "io_sub.h"
#include "datastorage.h"
#include "esc.h"
#include "float.h"
#include "../../RobobuoyDependency\RobobuoyVersion.h"

#define NUM_DIRECTIONS 10
float min_mag[3], max_mag[3];
static Message escOut;
static LedData compassLedStatus;
static PwrData compassPwrData;
static Buzz compassBuzzerData;
static double mDir = 0;

QueueHandle_t compass;
QueueHandle_t compassIn;
static RoboStruct compassInData;
static double declination = 0;

// Adafruit_LSM303AGR_Mag_Unified mag = Adafruit_LSM303AGR_Mag_Unified(12345);
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345); //  includes LSM303AGR
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

Message comp_msg; /* ESC que struckt */

static double mechanicCorrection = 0;

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

//***************************************************************************************************
//  Average of NUM_DIRECTIONS samples
//***************************************************************************************************
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
//***************************************************************************************************
//  Init compass
//***************************************************************************************************
bool InitCompass(void)
{
    // CompassOffsetCorrection(&mainData.compassOffset, GET);
    CompassCallibrationFactorsFloat(&max_mag[0], &max_mag[1], &max_mag[2], &min_mag[0], &min_mag[1], &min_mag[2], GET); //  get callibration data
    m_min = (vector<float>){min_mag[0], min_mag[1], min_mag[2]};
    m_max = (vector<float>){max_mag[0], max_mag[1], max_mag[2]};
    printf("Compass calibration: %1.3f %1.3f %1.3f %1.3f %1.3f %1.3f \r\n", max_mag[0], max_mag[1], max_mag[2], min_mag[0], min_mag[1], min_mag[2]);

    if (!mag.begin())
    {
        Serial.println("Unable to initialize LSM303 magnetometer");
        while (1)
            esp_restart(); //  reset if compass not found
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
    Declination(&declination, GET);
    // MechanicalCorrection(&mechanicCorrection, GET);
    printf("Declination set to: %3.2f\r\n", declination);
    mechanicCorrection = 0;
    return 0;
}

//***************************************************************************************************
//
//***************************************************************************************************
bool CalibrateCompass(void)
{
    sensors_event_t event;
    static unsigned long calstamp;
    u_int lokcnt = 0;
    bool lokon = 0;
    Serial.println("Calibration compass now!!!");
    calstamp = millis();
    for (int i = 0; i < 3; i++)
    {
        min_mag[i] = FLT_MAX;
        max_mag[i] = FLT_MIN;
    }

    // while (millis() - calstamp <= 1000 * 60) // 1 minute callibrating
    while (millis() - calstamp <= 1000 * 30) // 1 minute callibrating
    {
        mag.getEvent(&event);
        min_mag[0] = min(min_mag[0], event.magnetic.x);
        max_mag[0] = max(max_mag[0], event.magnetic.x);
        min_mag[1] = min(min_mag[1], event.magnetic.y);
        max_mag[1] = max(max_mag[1], event.magnetic.y);
        min_mag[2] = min(min_mag[2], event.magnetic.z);
        max_mag[2] = max(max_mag[2], event.magnetic.z);
        printf("XYZmax in: {%05.2f, %05.2f, %05.2f,%05.2f, %05.2f, %05.2f Magnetometer: %f %f %f  }\r\n", max_mag[0], min_mag[0], max_mag[1], min_mag[1], max_mag[2], min_mag[2], event.magnetic.x, event.magnetic.y, event.magnetic.z);
    }
    CompassCallibrationFactorsFloat(&max_mag[0], &max_mag[1], &max_mag[2], &min_mag[0], &min_mag[1], &min_mag[2], SET); //  store callibration data
    m_min = (vector<float>){min_mag[0], min_mag[1], min_mag[2]};
    m_max = (vector<float>){max_mag[0], max_mag[1], max_mag[2]};
    printf("New calibration factors Compass: XYZ: {%f, %f, %f,%f, %f, %f};\r\n", max_mag[0], min_mag[0], max_mag[1], min_mag[1], max_mag[2], min_mag[2]);
    printf("Heading %f\r\n", GetHeadingRaw());
    delay(5000);
    return 0;
}

//***************************************************************************************************
//
//***************************************************************************************************
// double GetHeading(void)
// {
//     double mHeading = (double)heading((vector<int>){1, 0, 0}); // Select oriontation
//     // sensors_event_t event;
//     // mag.getEvent(&event);
//     // double mHeading = (atan2(event.magnetic.x - (max_mag[0] + min_mag[0]) / 2, event.magnetic.y - (max_mag[1] + min_mag[1]) / 2) * 180) / PI;
//     mHeading += 90;
//     if (mHeading < 0)
//     {
//         mHeading = mHeading + 360.0;
//     }
//     else if (mHeading >= 360.0)
//     {
//         mHeading = mHeading - 360.0;
//     }
//     return mHeading;
// }

//***************************************************************************************************
//
//***************************************************************************************************
double GetHeadingRaw(void)
{
    return heading((vector<int>){1, 0, 0});
}
//***************************************************************************************************
//
//***************************************************************************************************
static int cbufpointer = 0;
static double directions[NUM_DIRECTIONS];
double CompassAverage(double in)
{
    directions[cbufpointer++] = in;
    if (cbufpointer >= NUM_DIRECTIONS)
    {
        cbufpointer = 0;
    }
    double sum_x = 0.0, sum_y = 0.0, avg_dir;
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
    if (avg_dir >= 360.0)
    {
        avg_dir -= 360;
    }
    return avg_dir;
}

//***************************************************************************************************
//
//***************************************************************************************************
double GetHeadingAvg(void)
{
    return CompassAverage(GetHeadingRaw());
}

//***************************************************************************************************
//
//***************************************************************************************************
void calibrateMagneticNorth(void)
{
    escOut.speedbb = 0;
    escOut.speedsb = 0;
    xQueueSend(escspeed, (void *)&escOut, 10);
    delay(1000);
    double h = GetHeadingAvg();
    if (h < 0)
    {
        h += 360;
    }
    if (h > 360)
    {
        h -= 360;
    }
    declination = smallestAngle(h, 0);
    if (h < 0)
    {
        declination += 360;
    }
    if (declination > 360)
    {
        declination -= 360;
    }

    Declination(&declination, SET);
    printf("\r\n\r\nMeaserd heading: %.2f  New magnetic declination stored: %.2f\r\n\r\n", h, declination);
}

//***************************************************************************************************
//  Compass queue
//***************************************************************************************************
void initcompassQueue(void)
{
    compass = xQueueCreate(1, sizeof(double));
    compassIn = xQueueCreate(1, sizeof(int));
}

//***************************************************************************************************
//  Compass calibratie norht
//***************************************************************************************************
void calibrateNorthCompas(void)
{
    escOut.speedbb = 0;
    escOut.speedsb = 0;
    xQueueSend(escspeed, (void *)&escOut, 10);
    compassBuzzerData.hz = 1000;
    compassBuzzerData.repeat = 5;
    compassBuzzerData.pause = 50;
    compassBuzzerData.duration = 100;
    xQueueSend(buzzer, (void *)&compassBuzzerData, 10); // update util led
    compassPwrData.bb = CRGB::Orange;
    compassPwrData.sb = CRGB::Orange;
    compassPwrData.blinkBb = BLINK_FAST;
    compassPwrData.blinkSb = BLINK_FAST;
    xQueueSend(ledPwr, (void *)&compassPwrData, 10); // update util led
    delay(1000);
    calibrateMagneticNorth();
    compassPwrData.bb = CRGB::Black;
    compassPwrData.sb = CRGB::Black;
    compassPwrData.blinkBb = BLINK_OFF;
    compassPwrData.blinkSb = BLINK_OFF;
    xQueueSend(ledPwr, (void *)&compassPwrData, 10); // update util led
    compassBuzzerData.hz = 1000;
    compassBuzzerData.repeat = 5;
    compassBuzzerData.pause = 50;
    compassBuzzerData.duration = 100;
    xQueueSend(buzzer, (void *)&compassBuzzerData, 10); // update util led
}

//***************************************************************************************************
//  Compass calibratie parameters
//***************************************************************************************************
void calibrateParametersCompas(void)
{
    compassLedStatus.color = CRGB::DarkBlue;
    compassPwrData.bb = CRGB::DarkBlue;
    compassPwrData.sb = CRGB::DarkBlue;
    compassLedStatus.blink = BLINK_FAST;
    compassPwrData.blinkBb = BLINK_FAST;
    compassPwrData.blinkSb = BLINK_FAST;
    xQueueSend(ledStatus, (void *)&compassLedStatus, 10); // update util led
    xQueueSend(ledPwr, (void *)&compassPwrData, 10);      // update util led
    compassBuzzerData.hz = 1000;
    compassBuzzerData.repeat = 10;
    compassBuzzerData.pause = 50;
    compassBuzzerData.duration = 100;
    xQueueSend(buzzer, (void *)&compassBuzzerData, 10); // update util led
    delay(1000);
    CalibrateCompass();
    compassBuzzerData.hz = 1000;
    compassBuzzerData.repeat = 10;
    compassBuzzerData.pause = 50;
    compassBuzzerData.duration = 100;
    xQueueSend(buzzer, (void *)&compassBuzzerData, 10); // update util led
    compassLedStatus.color = CRGB::Black;
    compassPwrData.bb = CRGB::Black;
    compassPwrData.sb = CRGB::Black;
    compassLedStatus.blink = BLINK_OFF;
    compassPwrData.blinkBb = BLINK_OFF;
    compassPwrData.blinkSb = BLINK_OFF;
    xQueueSend(ledStatus, (void *)&compassLedStatus, 10); // update util led
    xQueueSend(ledPwr, (void *)&compassPwrData, 10);      // update util led
}
//***************************************************************************************************
//  Compass task
//***************************************************************************************************
void CompassTask(void *arg)
{
    double mHdg = 0;
    int cmd = 0;
    unsigned long compassSampTime = millis();
    while (1)
    {
        mHdg = GetHeadingAvg() + declination;
        if (mHdg < 0)
        {
            mHdg = mHdg + 360.0;
        }
        else if (mHdg >= 360.0)
        {
            mHdg = mHdg - 360.0;
        }
        xQueueSend(compass, (void *)&mHdg, 0);                   // notify main there is new data
        if (xQueueReceive(compassIn, (void *)&cmd, 0) == pdTRUE) // send data to bottom
        {
            if (cmd == CALIBRATE_MAGNETIC_COMPASS)
            {
                printf("Calibrate compass now!!!\r\n");
                calibrateParametersCompas();
                printf("Calibrate compass done!!!\r\n");
            }
        }
        vTaskDelay(1);
    }
}
