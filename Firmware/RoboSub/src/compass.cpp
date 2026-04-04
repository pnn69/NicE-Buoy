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
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <math.h>
#include <cmath>
#include "leds.h"
#include "buzzer.h"
#include "io_sub.h"
#include "datastorage.h"
#include "esc.h"
#include "float.h"
#include "../../RoboDependency\RobobuoyVersion.h"
#include "subwifi.h"
#include "sercom.h"

#define NUM_DIRECTIONS 5
static int cbufpointer = 0;
static int sample_count = 0;
static double directions[NUM_DIRECTIONS];

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

double global_lsmHdg = 0.0;
double global_icmHdg = 0.0;

Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

// New ICM20948 sensor
Adafruit_ICM20948 icm;
Adafruit_Sensor *icm_mag = NULL;
Adafruit_Sensor *icm_accel = NULL;
bool icm_ready = false;

float min_mag[3], max_mag[3];
float icm_min_mag[3], icm_max_mag[3];

struct Vec3
{
    float x, y, z;
};

/**
 * @brief Computes the cross product of two 3D vectors.
 * 
 * @param a First vector.
 * @param b Second vector.
 * @return Vec3 Resulting cross product vector.
 */
inline Vec3 vector_cross(const Vec3 &a, const Vec3 &b)
{
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x};
}

/**
 * @brief Computes the dot product of two 3D vectors.
 * 
 * @param a First vector.
 * @param b Second vector.
 * @return float Resulting scalar dot product.
 */
inline float vector_dot(const Vec3 &a, const Vec3 &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/**
 * @brief Normalizes a 3D vector to unit length (magnitude of 1).
 * 
 * @param v The vector to normalize in place.
 */
inline void vector_normalize(Vec3 &v)
{
    float magnitude = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    if (magnitude > 0.00001f)
    {
        v.x /= magnitude;
        v.y /= magnitude;
        v.z /= magnitude;
    }
}

/**
 * @brief Calculates tilt-compensated heading using ICM-20948.
 * 
 * @param from The reference vector for North.
 * @return float Tilt-compensated heading in degrees.
 */
float heading_icm(const Vec3 &from)
{
    if (!icm_ready) return -1.0f;
    sensors_event_t accel_event, mag_event, gyro_event, temp_event;
    icm.getEvent(&accel_event, &gyro_event, &temp_event, &mag_event);

    // Feed raw data straight into the math. Do NOT swap axes before the Soft Iron Matrix!
    Vec3 temp_m = {
        mag_event.magnetic.x,
        mag_event.magnetic.y,
        mag_event.magnetic.z
    };

    // Invert X and Y axes of the ICM accelerometer to match the LSM's physical orientation on the PCB
    accel_event.acceleration.x = -accel_event.acceleration.x;
    accel_event.acceleration.y = -accel_event.acceleration.y;

    Vec3 a = {accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z};

    // Apply the same hard/soft iron calibrations for comparison
    temp_m.x -= compassCalc.icmMagHard[0];
    temp_m.y -= compassCalc.icmMagHard[1];
    temp_m.z -= compassCalc.icmMagHard[2];

    Vec3 corrected_m;
    corrected_m.x = compassCalc.icmMagSoft[0][0] * temp_m.x + compassCalc.icmMagSoft[0][1] * temp_m.y + compassCalc.icmMagSoft[0][2] * temp_m.z;
    corrected_m.y = compassCalc.icmMagSoft[1][0] * temp_m.x + compassCalc.icmMagSoft[1][1] * temp_m.y + compassCalc.icmMagSoft[1][2] * temp_m.z;
    corrected_m.z = compassCalc.icmMagSoft[2][0] * temp_m.x + compassCalc.icmMagSoft[2][1] * temp_m.y + compassCalc.icmMagSoft[2][2] * temp_m.z;

    vector_normalize(a);
    vector_normalize(corrected_m);

    Vec3 east = vector_cross(corrected_m, a);
    vector_normalize(east);

    Vec3 north = vector_cross(a, east);
    vector_normalize(north);

    float dot_east = vector_dot(east, from);
    float dot_north = vector_dot(north, from);
    
    if (std::isnan(dot_east) || std::isnan(dot_north)) return -1.0f;
    if (dot_east == 0.0f && dot_north == 0.0f) return -1.0f;

    // Because the ICM magnetometer is physically rotated 90 degrees relative to the LSM magnetometer,
    // we swap the East and North dot products in the atan2 function to achieve the correct orientation.
    // atan2(Y, X). LSM uses (-East, North). ICM uses (-North, -East).
    float heading = atan2(-dot_north, -dot_east) * 180.0f / M_PI;
    
    if (std::isnan(compassCalc.declination)) compassCalc.declination = 0.0f;
    if (std::isnan(compassCalc.icmCompassOffset)) compassCalc.icmCompassOffset = 0.0f;
    
    heading = heading + compassCalc.declination - compassCalc.icmCompassOffset;
    if (std::isnan(heading)) return -1.0f;
    
    while (heading < 0.0f) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;
    
    return heading;
}

/**
 * @brief Calculates tilt-compensated heading using magnetometer and accelerometer.
 * 1. Reads raw data from the magnetometer and accelerometer.
 * 2. Applies Hard Iron and Soft Iron calibration corrections to the magnetic data.
 * 3. Uses the accelerometer data as the down vector (gravity) to compute the horizontal plane.
 * 4. Projects the magnetic field onto the horizontal plane to find magnetic North.
 * 5. Adjusts the calculated angle using magnetic declination and user-defined offset.
 * 
 * @param from The reference vector for North (typically {1, 0, 0}).
 * @return float Tilt-compensated heading in degrees (0-359.9), or -1.0f on error.
 */
float heading_corrected(const Vec3 &from)
{
    sensors_event_t event;
    if (!mag.getEvent(&event)) return -1;
    Vec3 temp_m = {event.magnetic.x, event.magnetic.y, event.magnetic.z};
    
    if (!accel.getEvent(&event)) return -1;
    Vec3 a = {event.acceleration.x, event.acceleration.y, event.acceleration.z};

    // Sanity check: if magSoft is corrupted (NaN), reset it to identity matrix
    bool soft_corrupted = false;
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            if (std::isnan(compassCalc.magSoft[i][j]) || std::isinf(compassCalc.magSoft[i][j])) soft_corrupted = true;
        }
        if (std::isnan(compassCalc.magHard[i]) || std::isinf(compassCalc.magHard[i])) compassCalc.magHard[i] = 0.0f;
    }
    if (soft_corrupted) {
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                compassCalc.magSoft[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
    }

    temp_m.x -= compassCalc.magHard[0];
    temp_m.y -= compassCalc.magHard[1];
    temp_m.z -= compassCalc.magHard[2];

    Vec3 corrected_m;
    corrected_m.x = compassCalc.magSoft[0][0] * temp_m.x + compassCalc.magSoft[0][1] * temp_m.y + compassCalc.magSoft[0][2] * temp_m.z;
    corrected_m.y = compassCalc.magSoft[1][0] * temp_m.x + compassCalc.magSoft[1][1] * temp_m.y + compassCalc.magSoft[1][2] * temp_m.z;
    corrected_m.z = compassCalc.magSoft[2][0] * temp_m.x + compassCalc.magSoft[2][1] * temp_m.y + compassCalc.magSoft[2][2] * temp_m.z;

    vector_normalize(a);
    vector_normalize(corrected_m); // Also normalize m to prevent extremely large or zero values

    Vec3 east = vector_cross(corrected_m, a);
    vector_normalize(east);

    Vec3 north = vector_cross(a, east);
    vector_normalize(north);

    float dot_east = vector_dot(east, from);
    float dot_north = vector_dot(north, from);
    
    if (std::isnan(dot_east) || std::isnan(dot_north)) return -1.0f;
    if (dot_east == 0.0f && dot_north == 0.0f) return -1.0f;

    // Flip East/West by negating dot_east
    float heading = atan2(-dot_east, dot_north) * 180.0f / M_PI;
    
    if (std::isnan(compassCalc.declination)) compassCalc.declination = 0.0f;
    if (std::isnan(compassCalc.compassOffset)) compassCalc.compassOffset = 0.0f;
    
    heading = heading + compassCalc.declination - compassCalc.compassOffset;
    if (std::isnan(heading)) return -1.0f;
    
    while (heading < 0.0f) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;
    
    return heading;
}

/**
 * @brief Initializes the compass sensors (accelerometer and magnetometer).
 * 1. Sets up the I2C bus and verifies sensor communication.
 * 2. Loads the saved calibration parameters (offset, declination, etc.) from memory.
 */
void InitCompass(void)
{
    Wire.begin();
    Wire.setClock(400000); // Set I2C to 400kHz
    if (!accel.begin()) Serial.println("Unable to initialize LSM303 accelerometer");
    if (!mag.begin()) {
        Serial.println("Unable to initialize LSM303 magnetometer");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Initialize ICM20948
    if (!icm.begin_I2C()) {
        Serial.println("Failed to find ICM20948 chip");
    } else {
        Serial.println("ICM20948 Found!");
        icm_ready = true;
        icm_mag = icm.getMagnetometerSensor();
        icm_accel = icm.getAccelerometerSensor();
    }

    accel.setRange(LSM303_RANGE_4G);
    accel.setMode(LSM303_MODE_NORMAL);
    CompassCallibrationFactors(&compassCalc, GET);
    printf("Compass initialized. Offset: %.2f, Dec: %.2f\r\n", compassCalc.compassOffset, compassCalc.declination);
}

/**
 * @brief Calculates a circular moving average of heading angles.
 * Converts angles to Cartesian coordinates to avoid wrap-around issues (e.g., averaging 359 and 1),
 * sums them, and converts back to degrees.
 * 
 * @param in The latest heading angle to add to the buffer.
 * @return double The averaged heading angle.
 */
double CompassAverage(double in)
{
    directions[cbufpointer++] = in;
    if (cbufpointer >= NUM_DIRECTIONS) cbufpointer = 0;
    if (sample_count < NUM_DIRECTIONS) sample_count++;

    double sum_x = 0.0, sum_y = 0.0;
    for (int i = 0; i < sample_count; i++)
    {
        double angle_rad = directions[i] * M_PI / 180.0;
        sum_x += cos(angle_rad);
        sum_y += sin(angle_rad);
    }
    double avg_dir = atan2(sum_y, sum_x) * 180.0 / M_PI;
    if (avg_dir < 0) avg_dir += 360.0;
    return avg_dir;
}

/**
 * @brief Fetches and averages the latest tilt-corrected heading.
 * If the current reading fails, it returns the last valid reading from the buffer.
 * 
 * @return double The averaged tilt-compensated heading.
 */
double GetHeadingAvg(void)
{
    double hdg = -1.0f;
    if (icm_ready) {
        hdg = heading_icm(Vec3{1.0f, 0.0f, 0.0f});
    } else {
        hdg = heading_corrected(Vec3{1.0f, 0.0f, 0.0f});
    }

    if (hdg < 0) return directions[cbufpointer > 0 ? cbufpointer-1 : NUM_DIRECTIONS-1];
    return CompassAverage(hdg);
}

/**
 * @brief Performs a fast Hard Iron calibration.
 * Collects magnetometer data for 60 seconds while expecting the user to manually
 * rotate the buoy in all axes. Calculates the min/max values to find the Hard Iron offset.
 * 
 * @return true when calibration is complete.
 */
bool CalibrateCompass(void)
{
    sensors_event_t event;
    sensors_event_t a_evt, m_evt, g_evt, t_evt;
    unsigned long calstamp = millis();
    Serial.println("Calibrating compass now (60s)...");
    
    escOut.speedbb = 0; escOut.speedsb = 0;
    xQueueSend(escspeed, (void *)&escOut, 10);

    for (int i = 0; i < 3; i++) {
        min_mag[i] = FLT_MAX;
        max_mag[i] = -FLT_MAX;
        icm_min_mag[i] = FLT_MAX;
        icm_max_mag[i] = -FLT_MAX;
    }

    while (millis() - calstamp < 60000)
    {
        if (mag.getEvent(&event)) {
            if (abs(event.magnetic.x) < 200 && abs(event.magnetic.y) < 200 && abs(event.magnetic.z) < 200) {
                min_mag[0] = std::min(min_mag[0], event.magnetic.x);
                max_mag[0] = std::max(max_mag[0], event.magnetic.x);
                min_mag[1] = std::min(min_mag[1], event.magnetic.y);
                max_mag[1] = std::max(max_mag[1], event.magnetic.y);
                min_mag[2] = std::min(min_mag[2], event.magnetic.z);
                max_mag[2] = std::max(max_mag[2], event.magnetic.z);
                
                compassCalc.magHard[0] = event.magnetic.x;
                compassCalc.magHard[1] = event.magnetic.y;
                compassCalc.magHard[2] = event.magnetic.z;
                compassCalc.cmd = RAWCOMPASSDATA;
                xQueueOverwrite(serOut, (void *)&compassCalc);
            }
        }
        if (icm_ready) {
            icm.getEvent(&a_evt, &g_evt, &t_evt, &m_evt);
            float mx = m_evt.magnetic.x;
            float my = -m_evt.magnetic.y;
            float mz = -m_evt.magnetic.z;
            if (abs(mx) < 2000 && abs(my) < 2000 && abs(mz) < 2000) {
                icm_min_mag[0] = std::min(icm_min_mag[0], mx);
                icm_max_mag[0] = std::max(icm_max_mag[0], mx);
                icm_min_mag[1] = std::min(icm_min_mag[1], my);
                icm_max_mag[1] = std::max(icm_max_mag[1], my);
                icm_min_mag[2] = std::min(icm_min_mag[2], mz);
                icm_max_mag[2] = std::max(icm_max_mag[2], mz);
            }
        }
        
        // Suppress FLT_MAX to avoid console spam initially
        float p_lmin[3], p_lmax[3], p_imin[3], p_imax[3];
        for(int i=0; i<3; i++) {
            p_lmin[i] = (min_mag[i] == FLT_MAX) ? 0 : min_mag[i];
            p_lmax[i] = (max_mag[i] == -FLT_MAX) ? 0 : max_mag[i];
            p_imin[i] = (icm_min_mag[i] == FLT_MAX) ? 0 : icm_min_mag[i];
            p_imax[i] = (icm_max_mag[i] == -FLT_MAX) ? 0 : icm_max_mag[i];
        }

        printf("\rLSM X:%5.0f,%5.0f(%5.0f) Y:%5.0f,%5.0f(%5.0f) Z:%5.0f,%5.0f(%5.0f) | ICM X:%5.0f,%5.0f(%5.0f) Y:%5.0f,%5.0f(%5.0f) Z:%5.0f,%5.0f(%5.0f)   ", 
               p_lmin[0], p_lmax[0], event.magnetic.x,
               p_lmin[1], p_lmax[1], event.magnetic.y,
               p_lmin[2], p_lmax[2], event.magnetic.z,
               p_imin[0], p_imax[0], m_evt.magnetic.x,
               p_imin[1], p_imax[1], m_evt.magnetic.y,
               p_imin[2], p_imax[2], m_evt.magnetic.z);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
    printf("\n"); // Clear line when finished

    compassCalc.magHard[0] = (max_mag[0] + min_mag[0]) / 2.0f;
    compassCalc.magHard[1] = (max_mag[1] + min_mag[1]) / 2.0f;
    compassCalc.magHard[2] = (max_mag[2] + min_mag[2]) / 2.0f;
    
    compassCalc.icmMagHard[0] = (icm_max_mag[0] + icm_min_mag[0]) / 2.0f;
    compassCalc.icmMagHard[1] = (icm_max_mag[1] + icm_min_mag[1]) / 2.0f;
    compassCalc.icmMagHard[2] = (icm_max_mag[2] + icm_min_mag[2]) / 2.0f;
    
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            compassCalc.magSoft[i][j] = (i==j) ? 1.0f : 0.0f;
            compassCalc.icmMagSoft[i][j] = (i==j) ? 1.0f : 0.0f;
        }
    }

    hardIron(&compassCalc, SET);
    softIron(&compassCalc, SET);
    icmHardIron(&compassCalc, SET);
    icmSoftIron(&compassCalc, SET);
    printf("Calibration done. Hard iron: %.2f, %.2f, %.2f | ICM: %.2f, %.2f, %.2f\r\n", 
            compassCalc.magHard[0], compassCalc.magHard[1], compassCalc.magHard[2],
            compassCalc.icmMagHard[0], compassCalc.icmMagHard[1], compassCalc.icmMagHard[2]);
    return true;
}

/**
 * @brief Calibrates the user-defined compass offset.
 * Turns off the motors, plays a buzzer sequence, and records the current heading
 * for several samples to define it as the new 0-degree reference.
 */
void calibrateMagneticNorth(void)
{
    escOut.speedbb = 0; escOut.speedsb = 0;
    xQueueSend(escspeed, (void *)&escOut, 10);
    compassBuzzerData.hz = 2000; compassBuzzerData.repeat = 5; compassBuzzerData.pause = 50; compassBuzzerData.duration = 100;
    xQueueSend(buzzer, (void *)&compassBuzzerData, 10);
    vTaskDelay(pdMS_TO_TICKS(1000));
    compassCalc.compassOffset = 0;
    compassCalc.icmCompassOffset = 0;
    
    double sumIcmHdg = 0;
    
    for (int i = 0; i < NUM_DIRECTIONS * 2; i++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        GetHeadingAvg();
        sumIcmHdg += heading_icm(Vec3{1.0f, 0.0f, 0.0f});
    }
    
    compassCalc.compassOffset = GetHeadingAvg();
    compassCalc.icmCompassOffset = sumIcmHdg / (NUM_DIRECTIONS * 2);
    
    CompasOffset(&compassCalc, SET);
    icmCompassOffsetLoad(&compassCalc, SET);
    printf("Stored compassOffset: %.2f | icmCompassOffset: %.2f\r\n", compassCalc.compassOffset, compassCalc.icmCompassOffset);
    compassBuzzerData.hz = 1000; xQueueSend(buzzer, (void *)&compassBuzzerData, 10);
}

/**
 * @brief Initiates the standard Hard Iron calibration sequence.
 * Provides user feedback via LEDs and the buzzer, then calls CalibrateCompass().
 */
void calibrateParametersCompas(void)
{
    compassLedStatus.color = CRGB::DarkBlue; compassLedStatus.blink = BLINK_FAST;
    xQueueSend(ledStatus, (void *)&compassLedStatus, 10);
    compassBuzzerData.hz = 1000; compassBuzzerData.repeat = 10; compassBuzzerData.pause = 50; compassBuzzerData.duration = 100;
    xQueueSend(buzzer, (void *)&compassBuzzerData, 10);
    vTaskDelay(pdMS_TO_TICKS(500));
    CalibrateCompass();
    compassLedStatus.color = CRGB::Black; compassLedStatus.blink = BLINK_OFF;
    xQueueSend(ledStatus, (void *)&compassLedStatus, 10);
}

/**
 * @brief Fully autonomous in-water compass calibration routine.
 * 1. Takes control of the ESCs to perform a predefined sequence of maneuvers:
 *    - Wide right turn (60s)
 *    - Wide left turn (60s)
 *    - Pivot right (30s)
 * 2. Continuously samples min/max magnetic field values to calculate Hard Iron offsets.
 * 3. Saves the new offsets and signals completion back to the Top unit.
 */
void infieldCompassCalibration(void)
{
    sensors_event_t event;
    Serial.println("Starting In-Field Compass Calibration (3 mins)...");
    
    compassLedStatus.color = CRGB::Purple; compassLedStatus.blink = BLINK_FAST;
    xQueueSend(ledStatus, (void *)&compassLedStatus, 10);
    compassBuzzerData.hz = 1500; compassBuzzerData.repeat = 5; compassBuzzerData.pause = 50; compassBuzzerData.duration = 100;
    xQueueSend(buzzer, (void *)&compassBuzzerData, 10);
    vTaskDelay(pdMS_TO_TICKS(1000));

    for (int i = 0; i < 3; i++) {
        min_mag[i] = FLT_MAX;
        max_mag[i] = -FLT_MAX;
        icm_min_mag[i] = FLT_MAX;
        icm_max_mag[i] = -FLT_MAX;
    }

    auto sampleMag = [&]() {
        if (mag.getEvent(&event)) {
            if (abs(event.magnetic.x) < 200 && abs(event.magnetic.y) < 200 && abs(event.magnetic.z) < 200) {
                min_mag[0] = std::min(min_mag[0], event.magnetic.x);
                max_mag[0] = std::max(max_mag[0], event.magnetic.x);
                min_mag[1] = std::min(min_mag[1], event.magnetic.y);
                max_mag[1] = std::max(max_mag[1], event.magnetic.y);
                min_mag[2] = std::min(min_mag[2], event.magnetic.z);
                max_mag[2] = std::max(max_mag[2], event.magnetic.z);
                
                compassCalc.magHard[0] = event.magnetic.x;
                compassCalc.magHard[1] = event.magnetic.y;
                compassCalc.magHard[2] = event.magnetic.z;
                compassCalc.cmd = RAWCOMPASSDATA;
                xQueueOverwrite(serOut, (void *)&compassCalc);
            }
        }
        if (icm_ready) {
            sensors_event_t a_evt, m_evt, g_evt, t_evt;
            icm.getEvent(&a_evt, &g_evt, &t_evt, &m_evt);
            float mx = m_evt.magnetic.x;
            float my = -m_evt.magnetic.y;
            float mz = -m_evt.magnetic.z;
            if (abs(mx) < 2000 && abs(my) < 2000 && abs(mz) < 2000) {
                icm_min_mag[0] = std::min(icm_min_mag[0], mx);
                icm_max_mag[0] = std::max(icm_max_mag[0], mx);
                icm_min_mag[1] = std::min(icm_min_mag[1], my);
                icm_max_mag[1] = std::max(icm_max_mag[1], my);
                icm_min_mag[2] = std::min(icm_min_mag[2], mz);
                icm_max_mag[2] = std::max(icm_max_mag[2], mz);
            }
        }
    };

    // Phase 1 (~60s) - Wide Right Turn
    escOut.speedbb = compassCalc.maxSpeed * 0.20; 
    escOut.speedsb = compassCalc.maxSpeed * 0.05;
    xQueueSend(escspeed, (void *)&escOut, 10);
    unsigned long calstamp = millis();
    while (millis() - calstamp < 60000) { 
        sampleMag(); 
        
        float p_lmin[3], p_lmax[3], p_imin[3], p_imax[3];
        for(int i=0; i<3; i++) {
            p_lmin[i] = (min_mag[i] == FLT_MAX) ? 0 : min_mag[i];
            p_lmax[i] = (max_mag[i] == -FLT_MAX) ? 0 : max_mag[i];
            p_imin[i] = (icm_min_mag[i] == FLT_MAX) ? 0 : icm_min_mag[i];
            p_imax[i] = (icm_max_mag[i] == -FLT_MAX) ? 0 : icm_max_mag[i];
        }
        printf("\r[PH 1] LSM X:%5.0f,%5.0f Y:%5.0f,%5.0f Z:%5.0f,%5.0f | ICM X:%5.0f,%5.0f Y:%5.0f,%5.0f Z:%5.0f,%5.0f   ", 
               p_lmin[0], p_lmax[0], p_lmin[1], p_lmax[1], p_lmin[2], p_lmax[2],
               p_imin[0], p_imax[0], p_imin[1], p_imax[1], p_imin[2], p_imax[2]);
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }

    // Phase 2 (~60s) - Wide Left Turn
    escOut.speedbb = compassCalc.maxSpeed * 0.05; 
    escOut.speedsb = compassCalc.maxSpeed * 0.20;
    xQueueSend(escspeed, (void *)&escOut, 10);
    calstamp = millis();
    while (millis() - calstamp < 60000) { 
        sampleMag(); 
        
        float p_lmin[3], p_lmax[3], p_imin[3], p_imax[3];
        for(int i=0; i<3; i++) {
            p_lmin[i] = (min_mag[i] == FLT_MAX) ? 0 : min_mag[i];
            p_lmax[i] = (max_mag[i] == -FLT_MAX) ? 0 : max_mag[i];
            p_imin[i] = (icm_min_mag[i] == FLT_MAX) ? 0 : icm_min_mag[i];
            p_imax[i] = (icm_max_mag[i] == -FLT_MAX) ? 0 : icm_max_mag[i];
        }
        printf("\r[PH 2] LSM X:%5.0f,%5.0f Y:%5.0f,%5.0f Z:%5.0f,%5.0f | ICM X:%5.0f,%5.0f Y:%5.0f,%5.0f Z:%5.0f,%5.0f   ", 
               p_lmin[0], p_lmax[0], p_lmin[1], p_lmax[1], p_lmin[2], p_lmax[2],
               p_imin[0], p_imax[0], p_imin[1], p_imax[1], p_imin[2], p_imax[2]);
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }

    // Phase 3 (~30s) - Pivot Right
    escOut.speedbb = compassCalc.maxSpeed * 0.25; 
    escOut.speedsb = -compassCalc.maxSpeed * 0.25;
    xQueueSend(escspeed, (void *)&escOut, 10);
    calstamp = millis();
    while (millis() - calstamp < 30000) { 
        sampleMag(); 
        
        float p_lmin[3], p_lmax[3], p_imin[3], p_imax[3];
        for(int i=0; i<3; i++) {
            p_lmin[i] = (min_mag[i] == FLT_MAX) ? 0 : min_mag[i];
            p_lmax[i] = (max_mag[i] == -FLT_MAX) ? 0 : max_mag[i];
            p_imin[i] = (icm_min_mag[i] == FLT_MAX) ? 0 : icm_min_mag[i];
            p_imax[i] = (icm_max_mag[i] == -FLT_MAX) ? 0 : icm_max_mag[i];
        }
        printf("\r[PH 3] LSM X:%5.0f,%5.0f Y:%5.0f,%5.0f Z:%5.0f,%5.0f | ICM X:%5.0f,%5.0f Y:%5.0f,%5.0f Z:%5.0f,%5.0f   ", 
               p_lmin[0], p_lmax[0], p_lmin[1], p_lmax[1], p_lmin[2], p_lmax[2],
               p_imin[0], p_imax[0], p_imin[1], p_imax[1], p_imin[2], p_imax[2]);
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
    printf("\n"); // Clear line when finished

    // End maneuvers
    escOut.speedbb = 0; escOut.speedsb = 0;
    xQueueSend(escspeed, (void *)&escOut, 10);

    compassCalc.magHard[0] = (max_mag[0] + min_mag[0]) / 2.0f;
    compassCalc.magHard[1] = (max_mag[1] + min_mag[1]) / 2.0f;
    compassCalc.magHard[2] = (max_mag[2] + min_mag[2]) / 2.0f;
    
    compassCalc.icmMagHard[0] = (icm_max_mag[0] + icm_min_mag[0]) / 2.0f;
    compassCalc.icmMagHard[1] = (icm_max_mag[1] + icm_min_mag[1]) / 2.0f;
    compassCalc.icmMagHard[2] = (icm_max_mag[2] + icm_min_mag[2]) / 2.0f;
    
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
            compassCalc.magSoft[i][j] = (i==j) ? 1.0f : 0.0f;
            compassCalc.icmMagSoft[i][j] = (i==j) ? 1.0f : 0.0f;
        }
    }

    hardIron(&compassCalc, SET);
    softIron(&compassCalc, SET);
    icmHardIron(&compassCalc, SET);
    icmSoftIron(&compassCalc, SET);
    
    printf("In-Field Calibration done. Hard iron: %.2f, %.2f, %.2f | ICM: %.2f, %.2f, %.2f\r\n", 
            compassCalc.magHard[0], compassCalc.magHard[1], compassCalc.magHard[2],
            compassCalc.icmMagHard[0], compassCalc.icmMagHard[1], compassCalc.icmMagHard[2]);

    compassBuzzerData.hz = 1000; compassBuzzerData.repeat = 2; compassBuzzerData.pause = 100; compassBuzzerData.duration = 200;
    xQueueSend(buzzer, (void *)&compassBuzzerData, 10);
    
    compassLedStatus.color = CRGB::Black; compassLedStatus.blink = BLINK_OFF;
    xQueueSend(ledStatus, (void *)&compassLedStatus, 10);
    
    // Tell top we are done! We signal it by sending an IDLE command back to Top
    compassCalc.cmd = IDLE;
    compassCalc.ack = LORAINF;
    xQueueSend(serOut, (void *)&compassCalc, 10);
}

/**
 * @brief Initializes FreeRTOS queues for compass data and commands.
 */
void initcompassQueue(void)
{
    compass = xQueueCreate(1, sizeof(double));
    compassIn = xQueueCreate(1, sizeof(int));
}

/**
 * @brief FreeRTOS task handling continuous compass readings and commands.
 * 1. Periodically queries and averages the compass heading (every ~20ms).
 * 2. Overwrites the global `compass` queue with the latest heading at 10Hz.
 * 3. Listens on `compassIn` for incoming calibration commands from Top unit.
 * 
 * @param arg Unused FreeRTOS task argument.
 */
void CompassTask(void *arg)
{
    double activeHdg = 0;
    double lsmHdg = 0;
    double icmHdg = 0;
    int cmd = 0;
    unsigned long lastQueueSend = 0;
    unsigned long lastPrintSend = 0;

    while (1)
    {
        activeHdg = GetHeadingAvg(); // This automatically uses ICM if available!
        lsmHdg = heading_corrected(Vec3{1.0f, 0.0f, 0.0f});
        if (icm_ready) {
            icmHdg = heading_icm(Vec3{1.0f, 0.0f, 0.0f});
        }
        
        // Export values to global vars for sub webserver
        global_lsmHdg = lsmHdg;
        global_icmHdg = icmHdg;

        if (millis() - lastQueueSend > 100) {
            xQueueOverwrite(compass, (void *)&activeHdg);
            lastQueueSend = millis();
        }

        if (millis() - lastPrintSend > 500) {
            sensors_event_t a_icm, m_icm, g_icm, t_icm;
            if (icm_ready) icm.getEvent(&a_icm, &g_icm, &t_icm, &m_icm);
            
            sensors_event_t a_lsm, m_lsm;
            accel.getEvent(&a_lsm);
            mag.getEvent(&m_lsm);

            printf("LSM M(%5.1f, %5.1f, %5.1f) | ICM M(%5.1f, %5.1f, %5.1f) | Hdgs: L:%.1f I:%.1f\r\n", 
                   m_lsm.magnetic.x, m_lsm.magnetic.y, m_lsm.magnetic.z,
                   icm_ready ? m_icm.magnetic.x : 0, icm_ready ? m_icm.magnetic.y : 0, icm_ready ? m_icm.magnetic.z : 0,
                   lsmHdg, icmHdg);
            lastPrintSend = millis();
        }

        if (xQueueReceive(compassIn, (void *)&cmd, 0) == pdTRUE) {
            if (cmd == CALIBRATE_MAGNETIC_COMPASS) calibrateParametersCompas();
            else if (cmd == INFIELD_CALIBRATE) infieldCompassCalibration();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
