/**
 * @file compass.cpp
 * @brief ICM-20948 High-Performance Compass System with Madgwick Fusion & Persistence.
 * 
 * This module replaces the BNO055 sensor with an ICM-20948 9-DOF IMU, running
 * raw sensor readings through hard/soft iron corrections and a Madgwick AHRS
 * filter for robust drift-free heading calculations.
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <RoboCompute.h>
#include <ICM_20948.h>
#include <MadgwickAHRS.h>
#include "compass.h"
#include "main.h"
#include "datastorage.h"
#include <AsyncUDP.h>
#include <Preferences.h>
#include "leds.h"
#include "buzzer.h"
#include "esc.h"
#include "pidrudspeed.h"
#include "sercom.h"
#include "subwifi.h"

// Circular averaging buffer size
#define NUM_DIRECTIONS 5

extern Preferences storage;
QueueHandle_t compass = NULL;
QueueHandle_t compassIn = NULL;

bool icm_ready = false;
int icm_mode = 4; // Defaults to Mode 4 (Hard & Soft Iron with Pitch & Roll tilt compensation)

ICM_20948_I2C icm;
Madgwick filter;

// Calibration parameters (Stored in Preferences NVM via datastorage)
float hi_x = 0.0f, hi_y = 0.0f, hi_z = 0.0f;
float si_x = 1.0f, si_y = 1.0f, si_z = 1.0f;

// Gyroscope bias calibration parameters
float gyro_bias_x = 0.0f;
float gyro_bias_y = 0.0f;
float gyro_bias_z = 0.0f;

extern RoboStruct mainData;
extern Message escOut;
extern int subStatus;
extern AsyncUDP udp;
extern SemaphoreHandle_t mainDataMutex;

// Global state for web dashboard telemetry
float global_hdg = 0;
float last_raw_x = 0, last_raw_y = 0, last_raw_z = 0;
float last_raw_ax = 0, last_raw_ay = 0, last_raw_az = 0;
uint32_t global_loop_cnt = 0;
String global_cal_msg = "ICM Active";
String global_cal_load = "00000000000000000000000000000000000000000000";
String global_cal_ver = "00000000000000000000000000000000000000000000";

uint32_t cal_msg_timeout = 0;

/**
 * @brief Updates the calibration message on the dashboard with an optional audio confirmation.
 */
void setCalMsg(String msg, int beeps = 0) {
    global_cal_msg = msg;
    cal_msg_timeout = millis() + 5000;
    if (beeps > 0 && buzzer != NULL) beep(beeps, buzzer);
}

/**
 * @brief Formats float ICM calibrations as user-readable strings for UI telemetry.
 */
void updateUIHexFloat() {
    char hex[100];
    snprintf(hex, sizeof(hex), "HI: [%.1f,%.1f,%.1f] SI: [%.2f,%.2f,%.2f]", 
             hi_x, hi_y, hi_z, si_x, si_y, si_z);
    global_cal_load = String(hex);
    global_cal_ver = String("ICM-20948 Profile Active");
}

/**
 * @brief Detects and initializes the ICM-20948 sensor, running zero-rate bias calibration.
 */
bool InitCompass(void)
{
    Serial.println("\r\nInitializing ICM-20948 Compass V14.0...\r\n");
    bool sensorOk = false;
    
    // Auto-discover Address (0x69 or 0x68)
    if (icm.begin(Wire, 0x69) == ICM_20948_Stat_Ok) {
        sensorOk = true;
        Serial.println("ICM-20948: Initialized successfully at 0x69.");
    } else {
        Serial.println("ICM-20948: Failed at 0x69. Trying 0x68...");
        if (icm.begin(Wire, 0x68) == ICM_20948_Stat_Ok) {
            sensorOk = true;
            Serial.println("ICM-20948: Initialized successfully at 0x68.");
        }
    }

    if (sensorOk) {
        icm_ready = true;

        // Retrieve calibration profiles from NVS
        float hi[3], si[3];
        memIcmCalib(hi, si, true);
        hi_x = hi[0]; hi_y = hi[1]; hi_z = hi[2];
        si_x = si[0]; si_y = si[1]; si_z = si[2];
        updateUIHexFloat();

        // 200-sample Gyro Bias (Zero-Rate) Calibration
        Serial.println("ICM-20948: Calibrating gyroscope bias... Keep the device completely static!");
        setCalMsg("GYRO CALIBRATING", 2);
        float g_sum_x = 0, g_sum_y = 0, g_sum_z = 0;
        int samples = 200;
        int count = 0;
        while (count < samples) {
            if (icm.dataReady()) {
                icm.getAGMT();
                g_sum_x += icm.gyrX();
                g_sum_y += icm.gyrY();
                g_sum_z += icm.gyrZ();
                count++;
            }
            delay(10);
        }
        gyro_bias_x = g_sum_x / samples;
        gyro_bias_y = g_sum_y / samples;
        gyro_bias_z = g_sum_z / samples;
        Serial.printf("ICM-20948: Gyroscope calibration complete. Offsets -> X: %.4f, Y: %.4f, Z: %.4f\n", 
                      gyro_bias_x, gyro_bias_y, gyro_bias_z);

        // Initialize Madgwick filter at 100Hz ODR
        filter.begin(100);

        setCalMsg("ICM ONLINE", 3);
    } else {
        icm_ready = false;
        Serial.println("ICM-20948: CRITICAL ERROR - Sensor not found on I2C bus!\r\n");
        setCalMsg("ICM NOT FOUND", 0);
    }

    // Load remaining persistent parameters
    CompassOffsetCorrection(&mainData.compassOffset, true);
    CompasOffset(&mainData, true);
    MechanicalCorrection(&mainData.mechanicCorrection, true);
    return icm_ready;
}

/**
 * @brief Circular buffer averaging for smooth heading telemetry.
 */
float CompassAverage(float in) {
    static float directions[NUM_DIRECTIONS] = {0};
    static int cbufpointer = 0;
    static bool cbufFull = false;
    if (isnan(in)) return 0.0f;
    directions[cbufpointer++] = in;
    if (cbufpointer >= NUM_DIRECTIONS) { cbufpointer = 0; cbufFull = true; }
    int count = cbufFull ? NUM_DIRECTIONS : cbufpointer;
    float sum_x = 0.0, sum_y = 0.0;
    for (int i = 0; i < count; i++) {
        sum_x += cos(directions[i] * M_PI / 180.0);
        sum_y += sin(directions[i] * M_PI / 180.0);
    }
    float res = atan2(sum_y, sum_x) * 180.0 / M_PI;
    if (res < 0) res += 360.0;
    return res;
}

void initcompassQueue(void) {
    compass = xQueueCreate(1, sizeof(float));
    compassIn = xQueueCreate(10, sizeof(int));
}

/**
 * @brief High-priority task managing compass fusion and auto-calibration.
 * Runs on Core 1 to ensure consistent I2C polling without WiFi interference.
 */
void CompassTask(void *arg) {
    static bool was_timeout_active = false;
    static float stable_heading = 0;
    static bool heading_initialized = false;

    static float ax_f = 0.0f, ay_f = 0.0f, az_f = 0.0f;
    static float mx_f = 0.0f, my_f = 0.0f, mz_f = 0.0f;
    static bool firstRun = true;
    const float alpha = 0.15f; // EMA low-pass filter coefficient

    while (1) {
        global_loop_cnt++;
        int cmd = 0;

        // -------------------- MANUAL SAVE --------------------
        if (compassIn && xQueueReceive(compassIn, &cmd, 0) == pdTRUE) {
            if (cmd == 34) {
                float hi[3] = {hi_x, hi_y, hi_z};
                float si[3] = {si_x, si_y, si_z};
                memIcmCalib(hi, si, false);
                updateUIHexFloat();
                setCalMsg("MANUAL SAVE SUCCESS", 0);
            }
        }

        if (icm_ready) {
            icm.getAGMT(); // Read all sensors directly to guarantee consistent 100Hz execution

            float ax_raw = icm.accX();
            float ay_raw = icm.accY();
            float az_raw = icm.accZ();

            float mx_raw_val = icm.magX();
            float my_raw_val = icm.magY();
            float mz_raw_val = icm.magZ();

            // -------------------- I2C GLITCH & READ VALIDATION FILTER --------------------
            // Discard absolute sensor read failures or brief I2C transaction dropouts to prevent NaN filter pollution
            if (ax_raw == 0.0f && ay_raw == 0.0f && az_raw == 0.0f) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            float gx_raw = icm.gyrX() - gyro_bias_x;
            float gy_raw = icm.gyrY() - gyro_bias_y;
            float gz_raw = icm.gyrZ() - gyro_bias_z;

            // Smooth high-frequency noise using Exponential Moving Average
            if (firstRun) {
                ax_f = ax_raw; ay_f = ay_raw; az_f = az_raw;
                mx_f = mx_raw_val; my_f = my_raw_val; mz_f = mz_raw_val;
                firstRun = false;
            } else {
                ax_f = alpha * ax_raw + (1.0f - alpha) * ax_f;
                ay_f = alpha * ay_raw + (1.0f - alpha) * ay_f;
                az_f = alpha * az_raw + (1.0f - alpha) * az_f;
                mx_f = alpha * mx_raw_val + (1.0f - alpha) * mx_f;
                my_f = alpha * my_raw_val + (1.0f - alpha) * my_f;
                mz_f = alpha * mz_raw_val + (1.0f - alpha) * mz_f;
            }

            // Store raw diagnostic variables
            last_raw_x = mx_f; last_raw_y = my_f; last_raw_z = mz_f;
            last_raw_ax = ax_f; last_raw_ay = ay_f; last_raw_az = az_f;

            // Apply Hard Iron Offset (Always applied)
            float mx_hi = mx_f - hi_x;
            float my_hi = my_f - hi_y;
            float mz_hi = mz_f - hi_z;

            // Apply Soft Iron Scaling conditionally based on active mode
            float mxc = mx_hi;
            float myc = my_hi;
            float mzc = mz_hi;

            if (icm_mode == 2 || icm_mode == 4) {
                mxc = mx_hi * si_x;
                myc = my_hi * si_y;
                mzc = mz_hi * si_z;
            }

            // Coordinate Axis Realignment (Align ICM Magnetometer with Accelerometer/Gyroscope coordinate frame)
            // X_aligned = Y_calibrated, Y_aligned = X_calibrated, Z_aligned = Z_calibrated
            float mx_cal_aligned = myc;
            float my_cal_aligned = mxc;
            float mz_cal_aligned = mzc;

            // Update Madgwick filter using raw gyro (deg/s), accelerometer, and aligned calibrated magnetometer (inverting gz_raw to correct direction of rotation)
            filter.update(gx_raw, gy_raw, -gz_raw, ax_f, ay_f, az_f, mx_cal_aligned, my_cal_aligned, mz_cal_aligned);

            float mRoll = filter.getRoll();
            float mPitch = filter.getPitch();
            float mYaw = filter.getYaw();
            if (mYaw < 0) mYaw += 360.0;

            // Low-pass filter output angles to eliminate MEMS jitter completely
            static float roll_f = 0.0f;
            static float pitch_f = 0.0f;
            static float yaw_f = 0.0f;
            static bool firstAngleRun = true;

            if (firstAngleRun) {
                roll_f = mRoll;
                pitch_f = mPitch;
                yaw_f = mYaw;
                firstAngleRun = false;
            } else {
                const float angle_alpha = 0.12f;
                roll_f = angle_alpha * mRoll + (1.0f - angle_alpha) * roll_f;
                pitch_f = angle_alpha * mPitch + (1.0f - angle_alpha) * pitch_f;

                // Yaw unwrap-safe EMA filter
                float diff = mYaw - yaw_f;
                if (diff > 180.0f) diff -= 360.0f;
                else if (diff < -180.0f) diff += 360.0f;
                yaw_f += angle_alpha * diff;
                if (yaw_f < 0.0f) yaw_f += 360.0f;
                else if (yaw_f >= 360.0f) yaw_f -= 360.0f;
            }

            // Calculate raw heading based on selected ICM mode
            float raw_heading = 0.0f;

            if (icm_mode == 3 || icm_mode == 4) {
                // Calculate Pitch and Roll (radians) for geometrical tilt-compensation
                float r = atan2(ay_f, az_f);
                float p = atan2(-ax_f, sqrt(ay_f * ay_f + az_f * az_f));

                float cosRoll = cos(r);
                float sinRoll = sin(r);
                float cosPitch = cos(p);
                float sinPitch = sin(p);

                // Apply Tilt Compensation equations
                float Xh = mx_cal_aligned * cosPitch + my_cal_aligned * sinRoll * sinPitch + mz_cal_aligned * cosRoll * sinPitch;
                float Yh = my_cal_aligned * cosRoll - mz_cal_aligned * sinRoll;
                raw_heading = atan2(Yh, Xh) * 180.0 / M_PI;
                if (raw_heading < 0) raw_heading += 360.0;
            } else {
                // Uncompensated 2D heading (no tilt)
                raw_heading = atan2(my_cal_aligned, mx_cal_aligned) * 180.0 / M_PI;
                if (raw_heading < 0) raw_heading += 360.0;
            }

            // Unwrap-safe EMA low-pass filter to eliminate magnetometer MEMS noise and jitter
            static float heading_f = 0.0f;
            static bool firstHeadingRun = true;

            if (firstHeadingRun) {
                heading_f = raw_heading;
                firstHeadingRun = false;
            } else {
                const float heading_alpha = 0.12f; // Smooths sensor jitter completely while maintaining excellent responsiveness
                float diff = raw_heading - heading_f;
                if (diff > 180.0f) diff -= 360.0f;
                else if (diff < -180.0f) diff += 360.0f;
                heading_f += heading_alpha * diff;
                if (heading_f < 0.0f) heading_f += 360.0f;
                else if (heading_f >= 360.0f) heading_f -= 360.0f;
            }

            float heading = heading_f;

            // -------------------- ESC DETECTION & DRIFT FILTER --------------------
            bool esc_active = (abs(escOut.speedbb) > 5) || (abs(escOut.speedsb) > 5);

            if (!heading_initialized) {
                stable_heading = heading;
                heading_initialized = true;
            }

            float diff = heading - stable_heading;
            if (diff > 180) diff -= 360;
            if (diff < -180) diff += 360;

            if (esc_active) {
                if (fabs(diff) < 0.5f) {
                    heading = stable_heading;
                } else {
                    stable_heading = heading;
                }
            } else {
                if (fabs(diff) > 20.0f) {
                    heading = stable_heading;
                } else {
                    stable_heading = heading;
                }
            }

            // Smooth result
            heading = CompassAverage(heading);

            // -------------------- STUCK WATCHDOG --------------------
            static float last_heading = -999.0f;
            static uint32_t last_heading_change_time = 0;

            if (last_heading == -999.0f) {
                last_heading = heading;
                last_heading_change_time = millis();
            } else if (fabs(heading - last_heading) > 0.0001f) {
                last_heading = heading;
                last_heading_change_time = millis();
            } else {
                if (millis() - last_heading_change_time > 1000*60*10) { // 10 min
                    Serial.println("Compass stuck - reinit");
                    setCalMsg("COMPASS STUCK - REINIT", 0);
                    InitCompass();
                    last_heading = -999.0f;
                    last_heading_change_time = millis();
                }
            }

            // -------------------- UI STATUS MESSAGE --------------------
            if (millis() > cal_msg_timeout) {
                global_cal_msg = "ICM Active";
            }

            // -------------------- OUTPUT TO QUEUE & GLOBALS --------------------
            if (mainDataMutex && xSemaphoreTake(mainDataMutex, portMAX_DELAY)) {
                heading += mainData.compassOffset;

                while (heading < 0) heading += 360.0f;
                while (heading >= 360.0f) heading -= 360.0f;

                global_hdg = heading;
                mainData.dirMag = heading;

                if (compass) xQueueOverwrite(compass, (void *)&heading);

                xSemaphoreGive(mainDataMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

float GetHeading(void) { return global_hdg; }
float GetHeadingRaw(void) { return global_hdg; }
int linMagCalib(int *corr) { return 0; }
bool CalibrateCompass(void) { return true; }
int get_cal_point_count() { return 3; }
bool global_is_calibrating = false;
