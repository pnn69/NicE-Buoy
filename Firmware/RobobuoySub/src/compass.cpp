// https://github.com/antsundq/Magnetometer-Auto-Calibration/blob/main/Magnetometer_Autocal/Magnetometer_Autocal.ino
// https://www.pjrc.com/store/prop_shield.html
// https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?#declination
// https://www.youtube.com/watch?v=cGI8mrIanpk&t=186s

#include <Arduino.h>
#include <Wire.h>
#include <RoboCompute.h>
#include "compass.h"
#include "main.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_LSM303_Accel.h>
#include <math.h>
#include <cmath>
#include "leds.h"
#include "buzzer.h"
#include "io_sub.h"
#include "datastorage.h"
#include "esc.h"
#include "float.h"
#include "../../RobobuoyDependency\RobobuoyVersion.h"
#include "subwifi.h"
#include "sercom.h"

#define NUM_DIRECTIONS 5
static Message escOut;
static LedData compassLedStatus;
static PwrData compassPwrData;
static Buzz compassBuzzerData;
static RoboStruct udpOutCompass;
static double mDir = 0;

QueueHandle_t compass;
QueueHandle_t compassIn;
static RoboStruct compassInData, compassCalc;
static double declination = 2.56666666666;

// Adafruit_LSM303AGR_Mag_Unified mag = Adafruit_LSM303AGR_Mag_Unified(12345);
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345); //  includes LSM303AGR
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

Message comp_msg; /* ESC que struckt */

float min_mag[3], max_mag[3];
// double magHard[3] = {-49.40, -26.73, 37.11}; // offset for the magnetometer
// double magSoft[3][3] = {
//     {0.9851, 0.011, -0.001},
//     {0.011, .998, -.015},
//     {0.001, -0.016, 1.018}};

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











struct Vec3
{
    float x, y, z;
};

inline Vec3 vector_cross(const Vec3 &a, const Vec3 &b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x};
}

inline float vector_dot(const Vec3 &a, const Vec3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline void vector_normalize(Vec3 &v)
{
    float mag = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (mag > 0.00001f)
    {
        v.x /= mag;
        v.y /= mag;
        v.z /= mag;
    }
}

float heading_corrected(const Vec3 &from)
{
    sensors_event_t event;
    mag.getEvent(&event);
    Vec3 temp_m = {event.magnetic.x, event.magnetic.y, event.magnetic.z};
    Vec3 a = {event.acceleration.x, event.acceleration.y, event.acceleration.z};

    // Hard-iron correction
    temp_m.x -= compassCalc.magHard[0];
    temp_m.y -= compassCalc.magHard[1];
    temp_m.z -= compassCalc.magHard[2];

    // Soft-iron correction
    Vec3 corrected_m;
    corrected_m.x = compassCalc.magSoft[0][0] * temp_m.x + compassCalc.magSoft[0][1] * temp_m.y + compassCalc.magSoft[0][2] * temp_m.z;
    corrected_m.y = compassCalc.magSoft[1][0] * temp_m.x + compassCalc.magSoft[1][1] * temp_m.y + compassCalc.magSoft[1][2] * temp_m.z;
    corrected_m.z = compassCalc.magSoft[2][0] * temp_m.x + compassCalc.magSoft[2][1] * temp_m.y + compassCalc.magSoft[2][2] * temp_m.z;

    // East and North vectors
    Vec3 east, north;
    vector_normalize(a);
    east = vector_cross(corrected_m, a);
    vector_normalize(east);

    north = vector_cross(a, east);
    vector_normalize(north);

    // Heading
    float heading = atan2(vector_dot(north, from), vector_dot(east, from)) * 180.0f / M_PI;
    heading = heading + compassCalc.declination - compassCalc.compassOffset; // Apply declination correction and offset
    heading = fmodf(heading + 360.0f, 360.0f);
    return heading;
}
//***************************************************************************************************
//  Init compass
//***************************************************************************************************
void InitCompass(void)
{
     float min_mag[3], max_mag[3];
    CompassCallibrationFactorsFloat(&max_mag[0], &max_mag[1], &max_mag[2], &min_mag[0], &min_mag[1], &min_mag[2], true); //  get callibration data
    m_min = (vector<float>){min_mag[0], min_mag[1], min_mag[2]};
    m_max = (vector<float>){max_mag[0], max_mag[1], max_mag[2]};

 
 
    // Initialize compass
    if (!accel.begin())
    {
        Serial.println("Unable to initialize LSM303 accelerometer");
    }
    if (!mag.begin())
    {
        Serial.println("Unable to initialize LSM303 magnetometer");
        while (1)
            esp_restart(); //  reset if compass not found
        ;
    }
    accel.setRange(LSM303_RANGE_4G);
    accel.setMode(LSM303_MODE_NORMAL);
    sensors_event_t event;
    mag.getEvent(&event);

    compassCalc.mac = espMac();
    //***************************************************************************************************
    // START once to set the hard and soft iron factors
    //***************************************************************************************************
    //// Robobuoy ID: b7a5098c
    // compassCalc.magHard[0] = -57.52;
    // compassCalc.magHard[1] = -33.57;
    // compassCalc.magHard[2] = 50.46;
    // compassCalc.magSoft[0][0] = 0.994;
    // compassCalc.magSoft[0][1] = 0.015;
    // compassCalc.magSoft[0][2] = -0.005;
    // compassCalc.magSoft[1][0] = 0.015;
    // compassCalc.magSoft[1][1] = 0.990;
    // compassCalc.magSoft[1][2] = -0.014;
    // compassCalc.magSoft[2][0] = -0.005;
    // compassCalc.magSoft[2][1] = -0.014;
    // compassCalc.magSoft[2][2] = 1.017;

    //// Robobuoy ID: b7a50840
    // compassCalc.magHard[0] = -23.77;
    // compassCalc.magHard[1] = -6.62;
    // compassCalc.magHard[2] = 28.57499;
    // compassCalc.magSoft[0][0] = 0.977;
    // compassCalc.magSoft[0][1] = 0.014;
    // compassCalc.magSoft[0][2] = 0.001;
    // compassCalc.magSoft[1][0] = 0.014;
    // compassCalc.magSoft[1][1] = 1.0;
    // compassCalc.magSoft[1][2] = -0.002;
    // compassCalc.magSoft[2][0] = -0.001;
    // compassCalc.magSoft[2][1] = -0.002;
    // compassCalc.magSoft[2][2] = 1.024;

    // compassCalc.declination = 2.6666666666; // default declination Amsterdam 2025
    // CompasOffset(&compassCalc, GET);        // keep the offset from datastorage
    // CompassCallibrationFactors(&compassCalc, SET);
    //***************************************************************************************************
    // END Run once to set the hard and soft iron factors
    //***************************************************************************************************

    CompassCallibrationFactors(&compassCalc, GET);
    printf("Compass calibration:\r\nHard iron:\r\n");
    printf("   x      y      x    %2.2f %2.2f %2.2f\r\nSoft iron:\r\n", compassCalc.magHard[0], compassCalc.magHard[1], compassCalc.magHard[2]);
    printf(" [0][0] [0][1] [0][2] %4.3f %4.3f %4.3f\r\n", compassCalc.magSoft[0][0], compassCalc.magSoft[0][1], compassCalc.magSoft[0][2]);
    printf(" [1][0] [1][1] [1][2] %4.3f %4.3f %4.3f \r\n", compassCalc.magSoft[1][0], compassCalc.magSoft[1][1], compassCalc.magSoft[1][2]);
    printf(" [2][0] [2][1] [2][2] %4.3f %4.3f %4.3f \r\n", compassCalc.magSoft[2][0], compassCalc.magSoft[2][1], compassCalc.magSoft[2][2]);
    printf("Macnetic field strength: %3.2f uT\r\n", sqrt(pow(compassCalc.magHard[0], 2) + pow(compassCalc.magHard[1], 2) + pow(compassCalc.magHard[2], 2)));
    printf("Declination set to: %3.2f\r\n", compassCalc.declination);
    printf("compassOffset set to: %3.2f\r\n", compassCalc.compassOffset);
}

//***************************************************************************************************
// Compass callibration
//***************************************************************************************************
bool CalibrateCompass(void)
{
    sensors_event_t event;
    unsigned long calstamp = millis();
    unsigned long plot = millis();
    double mag0 = compassCalc.magHard[0];
    double mag1 = compassCalc.magHard[1];
    double mag2 = compassCalc.magHard[2];
    Serial.println("Calibrating compass now!");
    // Stop motors
    escOut.speedbb = 0;
    escOut.speedsb = 0;
    xQueueSend(escspeed, (void *)&escOut, 10);
    vTaskDelay(pdMS_TO_TICKS(500));
    for (int i = 0; i < 3; i++)
    {
        min_mag[i] = FLT_MAX;
        max_mag[i] = FLT_MIN;
    }
    vTaskDelay(10); // give sensor some time
    mag.getEvent(&event);
    vTaskDelay(10); // give sensor some time
    mag.getEvent(&event);
    while (millis() - calstamp <= 1000 * 60)
    // while (millis() - calstamp <= 1000 * 30)
    {
        vTaskDelay(50); // give sensor some time
        mag.getEvent(&event);
        if (abs(event.magnetic.x) > 200 || abs(event.magnetic.y) > 200 || abs(event.magnetic.z) > 200)
        {
            Serial.println("⚠️ Outlier ignored!");
            continue;
        }
        else
        {
            compassCalc.magHard[0] = event.magnetic.x * (100000.0 / 1100.0);
            compassCalc.magHard[1] = event.magnetic.y * (100000.0 / 1100.0);
            compassCalc.magHard[2] = event.magnetic.z * (100000.0 / 980.0);
            compassCalc.ack = LORAINF;
            compassCalc.status = CALIBRATE_MAGNETIC_COMPASS;
            compassCalc.cmd = RAWCOMPASSDATA;
            xQueueSend(udpOut, (void *)&compassCalc, 10);
            xQueueSend(serOut, (void *)&compassCalc, 10);

            min_mag[0] = min(min_mag[0], event.magnetic.x);
            max_mag[0] = max(max_mag[0], event.magnetic.x);
            min_mag[1] = min(min_mag[1], event.magnetic.y);
            max_mag[1] = max(max_mag[1], event.magnetic.y);
            min_mag[2] = min(min_mag[2], event.magnetic.z);
            max_mag[2] = max(max_mag[2], event.magnetic.z);

            Serial.print("Raw:");
            Serial.print(int(event.acceleration.x * 8192 / 9.8));
            Serial.print(",");
            Serial.print(int(event.acceleration.y * 8192 / 9.8));
            Serial.print(",");
            Serial.print(int(event.acceleration.z * 8192 / 9.8));
            Serial.print(",");
            Serial.print(int(0));
            Serial.print(",");
            Serial.print(int(0));
            Serial.print(",");
            Serial.print(int(0));
            Serial.print(",");
            Serial.print(int(event.magnetic.x * 10));
            Serial.print(",");
            Serial.print(int(event.magnetic.y * 10));
            Serial.print(",");
            Serial.print(int(event.magnetic.z * 10));
            Serial.println("");
        }
    }
    printf("New calibration factors Compass: X(%f), Y(%f), Z(%f)\r\n", (max_mag[0] + min_mag[0]) / 2, (max_mag[1] + min_mag[1]) / 2, (max_mag[2] + min_mag[2]) / 2);
    printf("Check with MotionCalc.exe\r\n");
    compassCalc.magHard[0] = (max_mag[0] + min_mag[0]) / 2;
    compassCalc.magHard[1] = (max_mag[1] + min_mag[1]) / 2;
    compassCalc.magHard[2] = (max_mag[2] + min_mag[2]) / 2;
    compassCalc.magSoft[0][0] = 1.0;
    compassCalc.magSoft[0][1] = 0.0;
    compassCalc.magSoft[0][2] = 0.0;
    compassCalc.magSoft[1][0] = 0.0;
    compassCalc.magSoft[1][1] = 1.0;
    compassCalc.magSoft[1][2] = 0.0;
    compassCalc.magSoft[2][0] = 0.0;
    compassCalc.magSoft[2][1] = 0.0;
    compassCalc.magSoft[2][2] = 1.0;

    compassCalc.mac = espMac();
    hardIron(&compassCalc, SET);
    softIron(&compassCalc, SET);
    CompassCallibrationFactorsFloat(&max_mag[0], &max_mag[1], &max_mag[2], &min_mag[0], &min_mag[1], &min_mag[2], SET);

    return true;
}

//***************************************************************************************************
// Calibrate magnetic north
//***************************************************************************************************
void calibrateMagneticNorth(void)
{
    // Stop motors
    escOut.speedbb = 0;
    escOut.speedsb = 0;
    xQueueSend(escspeed, (void *)&escOut, 10);
    compassBuzzerData.hz = 2000;
    compassBuzzerData.repeat = 5;
    compassBuzzerData.pause = 50;
    compassBuzzerData.duration = 100;
    xQueueSend(buzzer, (void *)&compassBuzzerData, 10); // update buzzer
    vTaskDelay(pdMS_TO_TICKS(1000));
    // Get averaged heading (device must be facing true north!)
    compassCalc.compassOffset = 0; // reset offset
    GetHeadingAvg();
    for (int i = 0; i < NUM_DIRECTIONS * 2; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(20)); // wait for next reading
        Serial.println(GetHeadingAvg(), 2);
    }
    // Store offset as difference between true north and measured heading
    compassCalc.compassOffset = GetHeadingAvg(); // Assuming current heading is magnetic north, and you're facing true north
    CompasOffset(&compassCalc, SET);
    printf("\n\nStored compassOffset: %.2f°\n\n", compassCalc.compassOffset);
    compassBuzzerData.hz = 1000;
    compassBuzzerData.repeat = 5;
    compassBuzzerData.pause = 50;
    compassBuzzerData.duration = 100;
    xQueueSend(buzzer, (void *)&compassBuzzerData, 10); // update buzzer
}

//***************************************************************************************************
// Average heading calculation
//***************************************************************************************************
static int cbufpointer = 0;
int sample_count = 0;
static double directions[NUM_DIRECTIONS];
double CompassAverage(double in)
{
    // Store new direction in circular buffer
    directions[cbufpointer++] = in;
    if (cbufpointer >= NUM_DIRECTIONS)
        cbufpointer = 0;

    // Increase sample count up to max
    if (sample_count < NUM_DIRECTIONS)
        sample_count++;

    double sum_x = 0.0, sum_y = 0.0;

    // Average using vector approach
    for (int i = 0; i < sample_count; i++)
    {
        double angle_rad = directions[i] * M_PI / 180.0;
        sum_x += cos(angle_rad);
        sum_y += sin(angle_rad);
    }

    sum_x /= sample_count;
    sum_y /= sample_count;

    double avg_dir = atan2(sum_y, sum_x) * 180.0 / M_PI;

    // Normalize to 0–360°
    if (avg_dir < 0)
        avg_dir += 360.0;

    return avg_dir;
}

//***************************************************************************************************
// get heading corrected
//***************************************************************************************************
double GetHeading(void)
{
    return heading_corrected(Vec3{1.0f, 0.0f, 0.0f});
}

//***************************************************************************************************
// Get heading average
//***************************************************************************************************
double GetHeadingAvg(void)
{
    //double mHeding = heading_corrected(Vec3{0.0f, 1.0f, 0.0f}); //  corrected heading
    double mHeding = heading((vector<int>){1, 0, 0}); // 180 correction due placement on pcb
    return CompassAverage(mHeding);
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
    delay(500);
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
        mHdg = GetHeadingAvg();
        // mHdg = GetHeading();
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
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
