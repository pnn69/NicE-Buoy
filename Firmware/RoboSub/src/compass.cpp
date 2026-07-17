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
#define NUM_DIRECTIONS 30

extern Preferences storage;
QueueHandle_t compass = NULL;
QueueHandle_t compassIn = NULL;

bool icm_ready = false;
bool magRejected = false;
float baselineMag = 50.0f;
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

        // Learn magnetometer baseline at startup
        Serial.println("ICM-20948: Measuring baseline magnetometer field strength... Keep the device completely static!");
        float mag_sum = 0.0f;
        int baseline_samples = 50;
        int baseline_count = 0;
        int retries = 0;
        while (baseline_count < baseline_samples && retries < 500) {
            if (icm.dataReady()) {
                icm.getAGMT();
                float mx = icm.magX();
                float my = icm.magY();
                float mz = icm.magZ();
                
                // Apply hard iron calibration
                float mx_hi = mx - hi_x;
                float my_hi = my - hi_y;
                float mz_hi = mz - hi_z;
                
                // Apply soft iron calibration
                float mxc = mx_hi;
                float myc = my_hi;
                float mzc = mz_hi;
                if (icm_mode == 2 || icm_mode == 4) {
                    mxc = mx_hi * si_x;
                    myc = my_hi * si_y;
                    mzc = mz_hi * si_z;
                }
                
                // Realignment to accelerometer/gyro frame
                float mx_aligned = myc;
                float my_aligned = mxc;
                float mz_aligned = -mzc;
                
                float norm = sqrt(mx_aligned * mx_aligned + my_aligned * my_aligned + mz_aligned * mz_aligned);
                if (norm > 5.0f && norm < 200.0f) {
                    mag_sum += norm;
                    baseline_count++;
                }
            }
            retries++;
            delay(10);
        }
        if (baseline_count > 0) {
            baselineMag = mag_sum / baseline_count;
            Serial.printf("ICM-20948: Magnetometer baseline learned: %.4f uT\n", baselineMag);
        } else {
            baselineMag = 50.0f; // Safe fallback
            Serial.println("ICM-20948: Magnetometer baseline learning failed, using fallback 50.0 uT");
        }

        // Initialize Madgwick filter at 100Hz ODR
        filter.begin(100);
        filter.setBeta(0.10f); // Balanced gain to anchor heading while relying on gyroscope during fast rotations, eliminating centripetal/accelerometer overshoot

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
 * @brief Circular buffer averaging for smooth heading telemetry using O(1) sliding window.
 */
float CompassAverage(float in) {
    static float directions_x[NUM_DIRECTIONS] = {0};
    static float directions_y[NUM_DIRECTIONS] = {0};
    static int cbufpointer = 0;
    static bool cbufFull = false;
    static float running_sum_x = 0.0f;
    static float running_sum_y = 0.0f;

    if (isnan(in)) return 0.0f;

    // Convert input angle to radians and get vector components
    float new_x = cos(in * M_PI / 180.0);
    float new_y = sin(in * M_PI / 180.0);

    // Retrieve old values from buffer
    float old_x = directions_x[cbufpointer];
    float old_y = directions_y[cbufpointer];

    // Overwrite oldest sample in the buffer
    directions_x[cbufpointer] = new_x;
    directions_y[cbufpointer] = new_y;

    // Adjust the running sum O(1)
    running_sum_x += (new_x - old_x);
    running_sum_y += (new_y - old_y);

    // Advance buffer pointer
    cbufpointer++;
    if (cbufpointer >= NUM_DIRECTIONS) {
        cbufpointer = 0;
        cbufFull = true;
    }

    // Drift-protection: Recalculate sums on wrap-around to eliminate rounding drift over days of operation
    if (cbufpointer == 0 && cbufFull) {
        running_sum_x = 0.0f;
        running_sum_y = 0.0f;
        for (int i = 0; i < NUM_DIRECTIONS; i++) {
            running_sum_x += directions_x[i];
            running_sum_y += directions_y[i];
        }
    }

    float res = atan2(running_sum_y, running_sum_x) * 180.0 / M_PI;
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

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // Enforce highly precise 100Hz ODR

    // Microsecond timing variables for mathematically perfect dt integration
    static uint32_t lastMicros = micros();

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
            // -------------------- DATA READY VALIDATION --------------------
            if (icm.dataReady()) {
                icm.getAGMT(); // Read all sensors directly to guarantee consistent execution
            } else {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }

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

            // -------------------- DELTA TIME MEASUREMENT --------------------
            uint32_t now = micros();
            float dt = (now - lastMicros) * 1e-6f;
            lastMicros = now;

            if (dt <= 0.0f || dt > 0.5f) {
                dt = 0.01f; // Safe fallback to 100Hz
            }

            // Dynamically synchronize filter's integration step with actual elapsed time
            filter.setDeltaTime(dt);

            // Madgwick library expects gyroscope rates in degrees per second (deg/s)
            float gx = icm.gyrX() - gyro_bias_x;
            float gy = icm.gyrY() - gyro_bias_y;
            float gz = icm.gyrZ() - gyro_bias_z;

            // Store raw diagnostic variables for telemetry/debugging
            last_raw_x = mx_raw_val; last_raw_y = my_raw_val; last_raw_z = mz_raw_val;
            last_raw_ax = ax_raw; last_raw_ay = ay_raw; last_raw_az = az_raw;

            // Apply Hard Iron Offset (Always applied)
            float mx_hi = mx_raw_val - hi_x;
            float my_hi = my_raw_val - hi_y;
            float mz_hi = mz_raw_val - hi_z;

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
            // X_aligned = Y_calibrated, Y_aligned = X_calibrated, Z_aligned = -Z_calibrated
            float mx_cal_aligned = myc;
            float my_cal_aligned = mxc;
            float mz_cal_aligned = -mzc;

            // -------------------- ADAPTIVE MAGNETOMETER VALIDITY CHECK --------------------
            float magNorm = sqrt(mx_cal_aligned * mx_cal_aligned + my_cal_aligned * my_cal_aligned + mz_cal_aligned * mz_cal_aligned);
            float minField = 0.5f * baselineMag;
            float maxField = 1.5f * baselineMag;

            // Detect magnetic anomalies (e.g. thruster interference or metal proximity)
            bool magDisturbed = (magNorm > maxField || magNorm < minField);

            if (magDisturbed) {
                // High magnetic disturbance -> ignore magnetometer, run 6-DOF IMU filter update
                filter.updateIMU(gx, gy, gz, ax_raw, ay_raw, az_raw);
                magRejected = true;
            } else {
                // Valid magnetic field -> run full 9-DOF AHRS filter update
                filter.update(gx, gy, gz, ax_raw, ay_raw, az_raw, mx_cal_aligned, my_cal_aligned, mz_cal_aligned);
                magRejected = false;
            }

            float mYaw = filter.getYaw();
            if (mYaw < 0.0f) mYaw += 360.0f;

            // Calculate raw heading based on selected ICM mode
            float raw_heading = 0.0f;

            if (icm_mode == 3 || icm_mode == 4) {
                raw_heading = mYaw; // Integrated robust Madgwick Yaw output
            } else {
                // Uncompensated 2D heading (no tilt)
                raw_heading = atan2(my_cal_aligned, mx_cal_aligned) * 180.0 / M_PI;
                if (raw_heading < 0.0f) raw_heading += 360.0f;
            }

            // Unwrap-safe EMA low-pass filter to eliminate any remaining output jitter
            static float heading_f = 0.0f;
            static bool firstHeadingRun = true;

            if (firstHeadingRun) {
                // Wait until we have a valid, non-zero magnetometer reading to avoid initializing with hard-iron dummy vectors
                if (mx_raw_val == 0.0f && my_raw_val == 0.0f) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }

                // -------------------- INSTANT 3D ATTITUDE PRE-ALIGNMENT --------------------
                // Calculate analytical 3D roll and pitch directly from gravity vector (accelerometer) at startup
                float init_roll = atan2f(ay_raw, az_raw) * 57.29578f;
                float init_pitch = atan2f(-ax_raw, sqrtf(ay_raw * ay_raw + az_raw * az_raw)) * 57.29578f;

                // Calculate analytical heading directly from calibrated magnetometer projection
                float init_heading = atan2(my_cal_aligned, mx_cal_aligned) * 180.0 / M_PI;
                if (init_heading < 0.0f) init_heading += 360.0f;
                
                // Instantly pre-align the Madgwick filter's 3D orientation state, bypassing startup convergence time completely
                filter.initializeAttitude(init_roll, init_pitch, init_heading);
                
                // Re-get the yaw to ensure mYaw starts from the initialized state
                mYaw = filter.getYaw();
                if (mYaw < 0.0f) mYaw += 360.0f;
                if (icm_mode == 3 || icm_mode == 4) {
                    raw_heading = mYaw;
                } else {
                    raw_heading = init_heading;
                }

                heading_f = raw_heading;
                firstHeadingRun = false;
            } else {
                const float heading_alpha = 0.30f; // Snappy responsiveness while eliminating MEMS jitter completely
                float diff = raw_heading - heading_f;
                if (diff > 180.0f) diff -= 360.0f;
                else if (diff < -180.0f) diff += 360.0f;
                heading_f += heading_alpha * diff;
                if (heading_f < 0.0f) heading_f += 360.0f;
                else if (heading_f >= 360.0f) heading_f -= 360.0f;
            }

            float heading = heading_f;

            // Reverse the direction of rotation mathematically to match the physical compass rose,
            // while preserving the correct chiral right-handed coordinate frame of the Madgwick filter.
            heading = 360.0f - heading;
            if (heading < 0.0f) heading += 360.0f;
            if (heading >= 360.0f) heading -= 360.0f;

            // Smooth result
            heading = CompassAverage(heading);

            // -------------------- STUCK WATCHDOG --------------------
            static float last_heading = -999.0f;
            static uint32_t last_heading_change_time = 0;

            if (last_heading == -999.0f) {
                last_heading = heading;
                last_heading_change_time = millis();
            } else if (fabs(heading - last_heading) > 0.1f) { // Prevents false triggers from floating-point noise
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
                if (magRejected) {
                    global_cal_msg = "IMU Fallback";
                } else {
                    global_cal_msg = "ICM Active";
                }
            }

            // -------------------- OUTPUT TO QUEUE & GLOBALS --------------------
            if (mainDataMutex && xSemaphoreTake(mainDataMutex, portMAX_DELAY)) {
                heading += mainData.compassOffset;

                while (heading < 0) heading += 360.0f;
                while (heading >= 360.0f) heading -= 360.0f;

                global_hdg = heading;
                mainData.dirMag = heading;
                mainData.ir = filter.getRoll();
                mainData.ip = filter.getPitch();

                if (compass) xQueueOverwrite(compass, (void *)&heading);

                xSemaphoreGive(mainDataMutex);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

float GetHeading(void) { return global_hdg; }
float GetHeadingRaw(void) { return global_hdg; }
int linMagCalib(int *corr) { return 0; }
bool CalibrateCompass(void) { return true; }
int get_cal_point_count() { return 3; }
bool global_is_calibrating = false;
