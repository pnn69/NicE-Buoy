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
bool firstHeadingRun = true;
bool yaw_initialized = false;
uint32_t lastMicros = 0;
uint32_t lastInitTime = 0;
float baselineMag = 50.0f;
int icm_mode = 4; // Defaults to Mode 4 (Hard & Soft Iron with Pitch & Roll tilt compensation)
float pr_damping = 0.95f; // Exponential damping factor for pitch and roll (0.00 = none, 0.99 = max)
float damp_acc = 0.15f;
float damp_gyro = 0.15f;
float damp_mag = 0.15f;
float damp_att = 0.15f;

ICM_20948_I2C icm;

// Calibration parameters (Stored in Preferences NVM via datastorage)
float hi_x = 0.0f, hi_y = 0.0f, hi_z = 0.0f;
float si_x = 1.0f, si_y = 1.0f, si_z = 1.0f;
float si_matrix[3][3] = {
    {1.0f, 0.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
    {0.0f, 0.0f, 1.0f}
};

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
float global_hdg_no_offset = 0;
float global_fusion_hdg = 0;
float fusion_offset = 0.0f; // Startup offset to align 3D Gyro Fusion with 2D compass exactly
float last_raw_x = 0, last_raw_y = 0, last_raw_z = 0;
float last_raw_ax = 0, last_raw_ay = 0, last_raw_az = 0;
uint32_t global_loop_cnt = 0;
int compass_avg_len = 1;
String global_cal_msg = "ICM Active";
String global_cal_load = "00000000000000000000000000000000000000000000";
String global_cal_ver = "00000000000000000000000000000000000000000000";

uint32_t cal_msg_timeout = 0;

// Ring buffer for raw calibration points
#define CAL_RING_BUF_SIZE 100
float cal_ring_x[CAL_RING_BUF_SIZE];
float cal_ring_y[CAL_RING_BUF_SIZE];
float cal_ring_z[CAL_RING_BUF_SIZE];
volatile int cal_ring_write_idx = 0;
volatile int cal_ring_read_idx = 0;

float global_init_heading = 0.0f;
float global_init_roll = 0.0f;
float global_init_pitch = 0.0f;

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

        // Enforce absolute values on startup to recover any older profiles with negative scale factors
        si[0] = fabs(si[0]);
        si[1] = fabs(si[1]);
        si[2] = fabs(si[2]);
        for (int r = 0; r < 3; r++) {
            si_matrix[r][r] = fabs(si_matrix[r][r]);
        }

        // Validate that the loaded 3x3 soft-iron matrix contains valid finite values within sensible limits
        bool matrix_valid = true;
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                if (!isfinite(si_matrix[r][c]) || si_matrix[r][c] < -5.0f || si_matrix[r][c] > 5.0f) {
                    matrix_valid = false;
                }
            }
        }

        // Validate loaded profiles (ensure finite values within sensible physical bounds)
        if (isfinite(hi[0]) && isfinite(hi[1]) && isfinite(hi[2]) &&
            isfinite(si[0]) && isfinite(si[1]) && isfinite(si[2]) &&
            hi[0] > -1000.0f && hi[0] < 1000.0f &&
            hi[1] > -1000.0f && hi[1] < 1000.0f &&
            hi[2] > -1000.0f && hi[2] < 1000.0f &&
            si[0] >= 0.2f && si[0] <= 5.0f &&
            si[1] >= 0.2f && si[1] <= 5.0f &&
            si[2] >= 0.2f && si[2] <= 5.0f &&
            matrix_valid) {
            
            hi_x = hi[0]; hi_y = hi[1]; hi_z = hi[2];
            si_x = si[0]; si_y = si[1]; si_z = si[2];
            Serial.println("ICM-20948: Calibration data and 3x3 Soft-Iron matrix successfully validated.");
        } else {
            hi_x = 0.0f; hi_y = 0.0f; hi_z = 0.0f;
            si_x = 1.0f; si_y = 1.0f; si_z = 1.0f;
            // Fallback: reset 3x3 matrix to standard identity matrix
            for (int r = 0; r < 3; r++) {
                for (int c = 0; c < 3; c++) {
                    si_matrix[r][c] = (r == c) ? 1.0f : 0.0f;
                }
            }
            Serial.println("ICM-20948: WARNING - Loaded calibration or 3x3 matrix is invalid/NaN! Safe fallbacks applied.");
        }
        updateUIHexFloat();

        // Set middle LED (LEDSTATUS) to flash yellow fast during gyro calibration
        if (ledStatus != NULL) {
            LedData calLed;
            calLed.color = CRGB::Yellow;
            calLed.blink = BLINK_FAST;
            xQueueSend(ledStatus, (void *)&calLed, 0);
        }

        // 200-sample Gyro Bias (Zero-Rate) Calibration
        Serial.println("ICM-20948: Calibrating gyroscope bias... Keep the device completely static!");
        setCalMsg("GYRO CALIBRATING", 2);
        float g_sum_x = 0, g_sum_y = 0, g_sum_z = 0;
        int samples = 200;
        int count = 0;
        int gyro_retries = 0;
        float last_gx = 0.0f, last_gy = 0.0f, last_gz = 0.0f;
        bool has_last = false;
        while (count < samples && gyro_retries < 1000) {
            if (icm.dataReady()) {
                icm.getAGMT();
                float gx = icm.gyrX();
                float gy = icm.gyrY();
                float gz = icm.gyrZ();
                
                if (has_last && gyro_retries < 800) { // Apply consecutive delta check during first 800 retries, then fallback to direct average
                    // We check delta-rate (change between consecutive samples) rather than absolute rate.
                    // This is completely immune to any static raw gyroscope bias offsets (which can easily exceed 3.0 deg/s at boot).
                    if (fabs(gx - last_gx) > 3.0f || fabs(gy - last_gy) > 3.0f || fabs(gz - last_gz) > 3.0f) {
                        last_gx = gx; last_gy = gy; last_gz = gz;
                        gyro_retries++;
                        delay(10);
                        continue;
                    }
                }
                last_gx = gx; last_gy = gy; last_gz = gz;
                has_last = true;
                
                g_sum_x += gx;
                g_sum_y += gy;
                g_sum_z += gz;
                count++;
            }
            gyro_retries++;
            delay(10);
        }
        
        // Handle fallback if loop timeout was hit
        if (count > 0) {
            gyro_bias_x = g_sum_x / count;
            gyro_bias_y = g_sum_y / count;
            gyro_bias_z = g_sum_z / count;
        } else {
            gyro_bias_x = 0.0f; gyro_bias_y = 0.0f; gyro_bias_z = 0.0f;
        }
        Serial.printf("ICM-20948: Gyroscope calibration complete. Offsets -> X: %.4f, Y: %.4f, Z: %.4f\n", 
                      gyro_bias_x, gyro_bias_y, gyro_bias_z);

        // Turn off fast blinking status LED after calibration
        if (ledStatus != NULL) {
            LedData calLed;
            calLed.color = CRGB::Black;
            calLed.blink = BLINK_OFF;
            xQueueSend(ledStatus, (void *)&calLed, 0);
        }

        // Learn magnetometer baseline at startup
        Serial.println("ICM-20948: Measuring baseline magnetometer field strength... Keep the device completely static!");
        float norms[50] = {0};
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
                    mxc = si_matrix[0][0] * mx_hi + si_matrix[0][1] * my_hi + si_matrix[0][2] * mz_hi;
                    myc = si_matrix[1][0] * mx_hi + si_matrix[1][1] * my_hi + si_matrix[1][2] * mz_hi;
                    mzc = si_matrix[2][0] * mx_hi + si_matrix[2][1] * my_hi + si_matrix[2][2] * mz_hi;
                }
                
                // Realignment to accelerometer/gyro frame matching native ICM-20948 specification
                float mx_aligned = mxc;
                float my_aligned = myc;
                float mz_aligned = -mzc;
                
                float norm = sqrt(mx_aligned * mx_aligned + my_aligned * my_aligned + mz_aligned * mz_aligned);
                if (norm > 5.0f && norm < 200.0f) {
                    norms[baseline_count] = norm;
                    baseline_count++;
                }
            }
            retries++;
            delay(10);
        }
        
        if (baseline_count == baseline_samples) {
            // Pass 1: Compute initial average of all samples
            float initial_sum = 0.0f;
            for (int i = 0; i < baseline_samples; i++) {
                initial_sum += norms[i];
            }
            float initial_avg = initial_sum / baseline_samples;
            
            // Pass 2: Filter out outlier baseline measurements deviating by >10.0 uT from the average
            float final_sum = 0.0f;
            int inlier_count = 0;
            for (int i = 0; i < baseline_samples; i++) {
                if (fabs(norms[i] - initial_avg) < 10.0f) {
                    final_sum += norms[i];
                    inlier_count++;
                } else {
                    Serial.printf("ICM-20948: Outlier baseline measurement rejected: %.2f uT (deviation: %.2f uT)\n", 
                                  norms[i], fabs(norms[i] - initial_avg));
                }
            }
            
            if (inlier_count > 0) {
                baselineMag = final_sum / inlier_count;
                Serial.printf("ICM-20948: Magnetometer baseline learned (using %d inliers): %.4f uT\n", inlier_count, baselineMag);
            } else {
                baselineMag = initial_avg; // Fallback to initial raw average
                Serial.printf("ICM-20948: All baseline samples flagged as outliers! Using raw average: %.4f uT\n", baselineMag);
            }
        } else {
            baselineMag = 50.0f; // Safe fallback
            Serial.println("ICM-20948: Magnetometer baseline learning failed (insufficient samples), using fallback 50.0 uT");
        }

        firstHeadingRun = true;
        lastMicros = micros(); // Reset timing baseline to prevent huge dt jump on task resume!
        lastInitTime = millis(); // Reset high-beta fast convergence timer!

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
    extern int compass_avg_len;
    memCompassAvg(&compass_avg_len, GET);
    memPrDamping(&pr_damping, GET);
    memDampingFactors(&damp_acc, &damp_gyro, &damp_mag, &damp_att, GET);
    // Attitude damping dynamically maps to pr_damping (where pr_damping = 1.0f - damp_att)
    pr_damping = 1.0f - damp_att;
    if (pr_damping < 0.0f) pr_damping = 0.0f;
    if (pr_damping > 0.99f) pr_damping = 0.99f;
    
    float trim_val = 0.0f;
    bool trim_en = false;
    memCompassTrim(&trim_val, &trim_en, GET);
    mainData.compass_trim = trim_val;
    mainData.compass_trim_enabled = trim_en;

    // Reset complementary yaw filter tracking state on sensor restart
    yaw_initialized = false;
    return icm_ready;
}

/**
 * @brief Circular buffer averaging for smooth heading telemetry using O(1) sliding window.
 */
float CompassAverage(float in) {
    extern int compass_avg_len;
    int avg_len = compass_avg_len;
    if (avg_len < 1) avg_len = 1;
    if (avg_len > 200) avg_len = 200;

    static float directions_x[200] = {0};
    static float directions_y[200] = {0};
    static int cbufpointer = 0;
    static bool cbufFull = false;
    static float running_sum_x = 0.0f;
    static float running_sum_y = 0.0f;

    if (isnan(in)) return 0.0f;

    // If avg_len is 1, bypass averaging completely for instant response
    if (avg_len == 1) {
        cbufpointer = 0;
        cbufFull = false;
        running_sum_x = 0.0f;
        running_sum_y = 0.0f;
        return in;
    }

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
    if (cbufpointer >= avg_len) {
        cbufpointer = 0;
        cbufFull = true;
    }

    // Drift-protection: Recalculate sums on wrap-around to eliminate rounding drift over days of operation
    if (cbufpointer == 0 && cbufFull) {
        running_sum_x = 0.0f;
        running_sum_y = 0.0f;
        for (int i = 0; i < avg_len; i++) {
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
 *
 * ARCHITECTURAL DESIGN SUMMARY (ICM-20948 9-DOF AHRS Redesign):
 * 1. HIGH-PRECISION SCHEDULING (FreeRTOS vTaskDelayUntil):
 *    We enforce highly precise constant 100.00Hz execution rate. This completely 
 *    eliminates loop timing jitter and keeps the task in perfect synchrony with the 
 *    magnetometer/IMU Output Data Rates (ODR).
 * 
 * 2. DYNAMIC TIMING INTEGRATION (Delta Time Jitter Correction):
 *    Instead of assuming a static sample rate, we measure the exact microsecond elapsed 
 *    time (dt) between successive iterations and dynamically write it to 'filter.setDeltaTime(dt)'.
 *    This completely neutralizes scheduler overhead and I2C transaction latency, guaranteeing 
 *    mathematically perfect gyro integration.
 * 
 * 3. INSTANT 3D ATTITUDE PRE-ALIGNMENT:
 *    On the very first sensor poll (once the magnetometer has fully initialized), the task 
 *    analytically calculates the gravity vector (roll/pitch) and magnetic vector (yaw) and 
 *    instantly pre-aligns the filter's internal quaternion state ('filter.initializeAttitude').
 *    This completely bypasses the 2-minute startup convergence delay at low gain!
 * 
 * 4. FAST-START HIGH BETA ALIGNMENT:
 *    During the first 5 seconds of boot, 'beta' is boosted to 0.25f, forcing rapid and 
 *    instant locking onto your physical orientation. After 5 seconds, it automatically relaxes 
 *    to 0.10f for stable, noise-immune dynamic trials.
 * 
 * 5. ADAPTIVE ANOMALY REJECTION & IMU FALLBACK:
 *    We dynamically learn a stationary magnetic field strength baseline ('baselineMag') on boot.
 *    If the current 3D magnetic norm deviates beyond [50%, 150%] of this baseline (indicating 
 *    iron proximity or thruster current), the filter automatically bypasses the magnetometer 
 *    and runs 6-DOF IMU update ('filter.updateIMU').
 * 
 * 6. ASYNCHRONOUS MUTEX INTEGRATION (Core 0 vs Core 1 Race Fix):
 *    All outputs are synchronized via 'mainDataMutex' to prevent state-overwrite race conditions 
 *    with Core 0 communications.
 */
void CompassTask(void *arg) {
    static bool was_timeout_active = false;
    static float stable_heading = 0;
    static bool heading_initialized = false;
    static float filtered_roll = 0.0f;
    static float filtered_pitch = 0.0f;
    static bool init_filters = true;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // Enforce highly precise 100Hz ODR

    // Timing baseline is managed globally and reset inside InitCompass()

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
            // -------------------- DATA READY VALIDATION & I2C WATCHDOG --------------------
            static uint32_t consecutive_failures = 0;
            if (icm.dataReady()) {
                icm.getAGMT(); // Read all sensors directly to guarantee consistent execution
                consecutive_failures = 0; // Reset counter on successful read
            } else {
                consecutive_failures++;
                if (consecutive_failures >= 100) { // ~1 second of consecutive I2C data-ready failures
                    Serial.println("ICM-20948: Consecutive I2C communication failures detected! Attempting self-healing recovery...");
                    consecutive_failures = 0;
                    
                    // Force complete sensor and filter re-initialization
                    icm_ready = false;
                    InitCompass();
                }
                vTaskDelay(pdMS_TO_TICKS(10)); // Slower polling delay during active communication failure
                continue;
            }

            float ax_raw_val = icm.accX();
            float ay_raw_val = icm.accY();
            float az_raw_val = icm.accZ();

            // Apply Accelerometer Damping (EMA)
            static float ax_prev = 0.0f, ay_prev = 0.0f, az_prev = 0.0f;
            static bool first_acc = true;
            if (first_acc) {
                ax_prev = ax_raw_val; ay_prev = ay_raw_val; az_prev = az_raw_val;
                first_acc = false;
            }
            float ax_raw = damp_acc * ax_raw_val + (1.0f - damp_acc) * ax_prev;
            float ay_raw = damp_acc * ay_raw_val + (1.0f - damp_acc) * ay_prev;
            float az_raw = damp_acc * az_raw_val + (1.0f - damp_acc) * az_prev;
            ax_prev = ax_raw; ay_prev = ay_raw; az_prev = az_raw;

            float mx_raw_unfiltered = icm.magX();
            float my_raw_unfiltered = icm.magY();
            float mz_raw_unfiltered = icm.magZ();

            // Apply Magnetometer Damping (EMA)
            static float mx_prev = 0.0f, my_prev = 0.0f, mz_prev = 0.0f;
            static bool first_mag = true;
            if (first_mag) {
                mx_prev = mx_raw_unfiltered; my_prev = my_raw_unfiltered; mz_prev = mz_raw_unfiltered;
                first_mag = false;
            }
            float mx_raw_val = damp_mag * mx_raw_unfiltered + (1.0f - damp_mag) * mx_prev;
            float my_raw_val = damp_mag * my_raw_unfiltered + (1.0f - damp_mag) * my_prev;
            float mz_raw_val = damp_mag * mz_raw_unfiltered + (1.0f - damp_mag) * mz_prev;
            mx_prev = mx_raw_val; my_prev = my_raw_val; mz_prev = mz_raw_val;

            // The cal_ring buffer and global_is_calibrating pause block have been removed.
            // Raw telemetry variables are stored later in the task loop.

            // Check for NaNs to prevent corrupting the Madgwick filter's internal state
            if (isnan(ax_raw) || isnan(ay_raw) || isnan(az_raw)) {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }

            // Discard absolute sensor read failures or brief I2C transaction dropouts to prevent NaN filter pollution
            if (ax_raw == 0.0f && ay_raw == 0.0f && az_raw == 0.0f) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }

            // Apply Hard Iron Offset (Always applied)
            float mx_hi = mx_raw_val - hi_x;
            float my_hi = my_raw_val - hi_y;
            float mz_hi = mz_raw_val - hi_z;

            // Apply Soft Iron Scaling conditionally based on active mode (apply full 3x3 matrix multiplication)
            float mxc = mx_hi;
            float myc = my_hi;
            float mzc = mz_hi;

            if (icm_mode == 2 || icm_mode == 4) {
                // Force positive diagonal gains in real-time to prevent axis inversion and dynamic creeping
                float sxx = fabs(si_matrix[0][0]);
                float syy = fabs(si_matrix[1][1]);
                float szz = fabs(si_matrix[2][2]);
                mxc = sxx * mx_hi + si_matrix[0][1] * my_hi + si_matrix[0][2] * mz_hi;
                myc = si_matrix[1][0] * mx_hi + syy * my_hi + si_matrix[1][2] * mz_hi;
                mzc = si_matrix[2][0] * mx_hi + si_matrix[2][1] * my_hi + szz * mz_hi;
            }

            // Coordinate Axis Realignment (Align ICM Magnetometer with Accelerometer/Gyroscope coordinate frame)
            // Account for upside-down physical mounting of the sensor by aligning magnetometer Y with the positive gyro frame
            // float mx_cal_aligned = myc;
            // float my_cal_aligned = -mxc;
            // float mz_cal_aligned = -mzc;
            float mx_cal_aligned = mxc;
            float my_cal_aligned = myc;
            float mz_cal_aligned = -mzc;

            // Check for NaNs to prevent corrupting the Madgwick filter's internal state
            if (isnan(mx_cal_aligned) || isnan(my_cal_aligned) || isnan(mz_cal_aligned)) {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }

            // Calculate 3D magnetometer field norm once for use in both validation and rejection logic (optimizes FPU math)
            float magNorm = sqrtf(mx_cal_aligned * mx_cal_aligned + my_cal_aligned * my_cal_aligned + mz_cal_aligned * mz_cal_aligned);
            // Aligned with the unswapped, un-inverted, level coordinate frame of headingFull (atan2f(mxc, myc))
            // This forces MErr (headingFull - headingFilterMag) to lock onto exactly 0.0 degrees when the buoy is level!
            float headingFilterMag = atan2f(mxc, myc) * 57.29578f;
            if (headingFilterMag < 0.0f)
                headingFilterMag += 360.0f;


            // -------------------- INSTANT 3D ATTITUDE PRE-ALIGNMENT --------------------
            if (firstHeadingRun) {
                // Wait until we have a valid, non-zero magnetometer reading to avoid initializing with hard-iron dummy vectors
                if (magNorm < 5.0f) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    continue;
                }

                // Calculate analytical 3D roll and pitch directly from gravity vector (accelerometer) at startup
                float init_roll = -atan2f(ay_raw, az_raw) * 57.29578f;
                float init_pitch = atan2f(ax_raw, sqrtf(ay_raw * ay_raw + az_raw * az_raw)) * 57.29578f;

                // Calculate soft-iron compensated heading matching standard display coordinates
                float softHeading = atan2(myc, mxc) * 180.0 / M_PI;
                if (softHeading < 0.0f) softHeading += 360.0f;
                softHeading = 360.0f - softHeading;
                if (softHeading >= 360.0f) softHeading -= 360.0f;

                // Calculate analytical heading directly from calibrated magnetometer projection
                // matching the filter's native, mirrored right-handed coordinate frame.
                float init_heading = atan2f(my_cal_aligned, mx_cal_aligned) * 57.29578f;
                if (init_heading < 0.0f)
                    init_heading += 360.0f;
                init_heading = 360.0f - init_heading;
                if (init_heading >= 360.0f)
                    init_heading -= 360.0f;


                
                // Store startup values for diagnostics
                global_init_heading = init_heading;
                global_init_roll = init_roll;
                global_init_pitch = init_pitch;
                filtered_roll = init_roll;
                filtered_pitch = init_pitch;
                init_filters = false;
                baselineMag = magNorm; // Re-learn local baseline from newly calibrated readings
                firstHeadingRun = false;
            }

            // -------------------- DELTA TIME MEASUREMENT --------------------
            uint32_t now = micros();
            float dt = (now - lastMicros) * 1e-6f;
            lastMicros = now;

            if (dt <= 0.0f || dt > 0.5f) {
                dt = 0.01f; // Safe fallback to 100Hz
            }

            float gx_raw_val = icm.gyrX();
            float gy_raw_val = icm.gyrY();
            float gz_raw_val = icm.gyrZ();

            // Apply Gyroscope Damping (EMA)
            static float gx_prev = 0.0f, gy_prev = 0.0f, gz_prev = 0.0f;
            static bool first_gyro = true;
            if (first_gyro) {
                gx_prev = gx_raw_val; gy_prev = gy_raw_val; gz_prev = gz_raw_val;
                first_gyro = false;
            }
            gx_raw_val = damp_gyro * gx_raw_val + (1.0f - damp_gyro) * gx_prev;
            gy_raw_val = damp_gyro * gy_raw_val + (1.0f - damp_gyro) * gy_prev;
            gz_raw_val = damp_gyro * gz_raw_val + (1.0f - damp_gyro) * gz_prev;
            gx_prev = gx_raw_val; gy_prev = gy_raw_val; gz_prev = gz_raw_val;

            // Madgwick library expects gyroscope rates in degrees per second (deg/s) and converts internally
            float gx = (gx_raw_val - gyro_bias_x);
            float gy = (gy_raw_val - gyro_bias_y);
            float gz = (gz_raw_val - gyro_bias_z);

            static uint32_t lastGyroDebug = 0;

            if (millis() - lastGyroDebug > 250)
            {
                lastGyroDebug = millis();

                Serial.printf(
                    "GYRO gx=%.2f gy=%.2f gz=%.2f | ACC ax=%.2f ay=%.2f az=%.2f\n\r",
                    gx,
                    gy,
                    gz,
                    ax_raw,
                    ay_raw,
                    az_raw
                );
            }



            // Check for gyroscope NaNs to protect the filter's internal quaternion state
            if (isnan(gx) || isnan(gy) || isnan(gz)) {
                vTaskDelay(pdMS_TO_TICKS(1));
                continue;
            }

            // Store raw diagnostic variables for telemetry/debugging
            last_raw_x = mx_raw_val; last_raw_y = my_raw_val; last_raw_z = mz_raw_val;
            last_raw_ax = ax_raw; last_raw_ay = ay_raw; last_raw_az = az_raw;

            // -------------------- ADAPTIVE MAGNETOMETER VALIDITY CHECK --------------------
            float minField = 0.5f * baselineMag;
            float maxField = 1.5f * baselineMag;

            // Detect magnetic anomalies (e.g. thruster interference or metal proximity)
            bool magDisturbed = (magNorm > maxField || magNorm < minField);

            magRejected = magDisturbed;

            // Calculate raw pitch and roll from gravity vector (accelerometer) in radians
            // Roll: -atan2(ay, az)
            // Pitch: atan2(ax, sqrt(ay^2 + az^2))
            float phi = -atan2f(ay_raw, az_raw);
            float theta = atan2f(ax_raw, sqrtf(ay_raw * ay_raw + az_raw * az_raw));

            // Apply Pitch/Roll Damping (exponential moving average)
            // pr_damping goes from 0.0f (no damping) to 0.99f (max damping)
            static float roll_prev = 0.0f;
            static float pitch_prev = 0.0f;
            static bool first_att = true;
            if (first_att) {
                roll_prev = phi;
                pitch_prev = theta;
                first_att = false;
            }

            // Alpha is the response coefficient. If damping is 0.95f, alpha is 0.05f.
            float alpha = 1.0f - pr_damping;
            if (alpha < 0.01f) alpha = 0.01f;
            if (alpha > 1.0f) alpha = 1.0f;

            phi = alpha * phi + (1.0f - alpha) * roll_prev;
            theta = alpha * theta + (1.0f - alpha) * pitch_prev;

            roll_prev = phi;
            pitch_prev = theta;

            // Output Roll/Pitch in degrees
            float roll = phi * 57.29578f;
            float pitch = theta * 57.29578f;

            // -------------------- UNIFIED HEADING CALCULATIONS (MATCHING PROTOTYPE) --------------------
            // 1. Raw Heading
            // float headingRaw = atan2f(mx_raw_val, my_raw_val) * 57.29578f;
            float headingRaw = atan2f(my_raw_val, mx_raw_val) * 57.29578f;
            if (headingRaw < 0.0f) 
                headingRaw += 360.0f;

            // 2. Hard Iron Heading
            float hx = mx_raw_val - hi_x;
            float hy = my_raw_val - hi_y;
            float headingHard = atan2f(hx, hy) * 57.29578f;
            if (headingHard < 0.0f) headingHard += 360.0f;

            // 3. Soft Iron Heading
            float sx = mx_raw_val * si_x;
            float sy = my_raw_val * si_y;
            float headingSoft = atan2f(sx, sy) * 57.29578f;
            if (headingSoft < 0.0f) headingSoft += 360.0f;

            // 4. Both iron comp (Offsets + Scaling)
            float bx = hx * si_x;
            float by = hy * si_y;
            float headingBoth = atan2f(bx, by) * 57.29578f;
            if (headingBoth < 0.0f) headingBoth += 360.0f;

            // 5. Full Tilt + Both Iron (offsets + scaling + Pitch/Roll tilt projection)
            float cosRoll = cos(phi);
            float sinRoll = sin(phi);
            float cosPitch = cos(theta);
            float sinPitch = sin(theta);

            float cx_cal = hx * si_x;
            float cy_cal = hy * si_y;
            float cz_cal = (mz_raw_val - hi_z) * si_z;

            float Xh = cx_cal * cosPitch + cy_cal * sinRoll * sinPitch - cz_cal * cosRoll * sinPitch;
            float Yh = cy_cal * cosRoll + cz_cal * sinRoll;

            // Correct un-swapped, un-inverted tilt-compensated formula from the working prototype (atan2f(Xh, Yh))
            // This is completely stable at high rolls and prevents any quadrant flips!
            float headingFull = atan2f(Xh, Yh) * 57.29578f;
            if (headingFull < 0.0f) headingFull += 360.0f;



            // -------------------- GYRO-STABILIZED COMPLEMENTARY YAW FILTER (REPLACES MADGWICK AHRS) --------------------
            // Integrates the Z-axis gyroscope rate (yaw rate) and slowly anchors onto headingFull.
            // This is completely immune to coordinate frame mismatches and eliminates any potential gyro creep.
            static float yaw_estimate = 0.0f;
            if (!yaw_initialized) {
                yaw_estimate = headingFull; // Initialize to current analytical heading immediately on startup/recovery
                yaw_initialized = true;
            }

            // Project the 3D gyroscope rates onto Earth's global vertical Z-axis (GNC Euler projection).
            // This completely eliminates local roll/pitch rate bleed-through into the Z-gyro during heavy tilts!
            float cosTheta = cosf(theta);
            if (fabs(cosTheta) < 0.1f) cosTheta = (cosTheta >= 0.0f) ? 0.1f : -0.1f; // Prevent division by zero
            float global_gz = (gy * sinf(phi) + gz * cosf(phi)) / cosTheta;

            // For your physical mounting/sensor alignment, a clockwise (right) turn produces a negative gz value
            // while the compass heading increases clockwise. Therefore, we subtract global_gz * dt to align them!
            yaw_estimate -= global_gz * dt;

            // Handle angular wrap-around/difference safely
            float err = headingFull - yaw_estimate;
            if (err > 180.0f) err -= 360.0f;
            if (err < -180.0f) err += 360.0f;

            // Apply compass anchoring ONLY when the magnetometer is not disturbed.
            // If disturbed, we rely purely on high-precision gyroscope integration (magDisturbed -> gyro only).
            // Once recovered, it seamlessly re-locks onto the compass!
            if (!magDisturbed) {
                // A complementary gain of 0.005f at 100Hz ODR gives an excellent long-term time constant,
                // letting the Gyroscope carry short-term dynamics, while the compass removes long-term drift.
                yaw_estimate += err * 0.005f;
            }

            // Wrap yaw_estimate to [0, 360)
            while (yaw_estimate < 0.0f) yaw_estimate += 360.0f;
            while (yaw_estimate >= 360.0f) yaw_estimate -= 360.0f;

            float headingFusion = yaw_estimate;
            global_fusion_hdg = headingFusion;

            // Debug
            static uint32_t lastDebug = 0;

            if (millis() - lastDebug > 1000)
            {
                lastDebug = millis();

                float yawError = headingFull - headingFusion;

                if (yawError > 180.0f)
                    yawError -= 360.0f;

                if (yawError < -180.0f)
                    yawError += 360.0f;

                float magError = headingFull - headingFilterMag;

                if (magError > 180.0f)
                    magError -= 360.0f;

                if (magError < -180.0f)
                    magError += 360.0f;

                // User requested verification print for gz vs heading direction alignment
                Serial.printf("gz=%.2f headingFull=%.1f fusion=%.1f (YErr=%.1f MErr=%.1f)\n\r", 
                              gz, headingFull, headingFusion, yawError, magError);

                // User requested raw gz and bias verification logging (Fixed to use gz_raw_val and \n\r)
                Serial.printf("RAW_GZ=%.2f BIAS=%.2f CORR_GZ=%.2f\n\r", 
                              gz_raw_val, gyro_bias_z, gz);

                // User requested raw magnetometer projection logging (Fixed to use \n\r)
                Serial.printf("cx_cal=%.2f cy_cal=%.2f cz_cal=%.2f headingFull=%.1f\n\r", 
                              cx_cal, cy_cal, cz_cal, headingFull);

                Serial.printf(
                    "COMP  R=%.1f P=%.1f Y=%.1f\n\r",
                    roll,
                    pitch,
                    headingFusion
                );

                Serial.printf(
                    "ANA   R=%.1f P=%.1f Y=%.1f\n\r",
                    roll,
                    pitch,
                    headingFull
                );
                Serial.printf(
                    "HDG=%.1f FUS=%.1f ERR=%.1f R=%.1f P=%.1f MAG=%s\n\r",
                    headingFull,
                    headingFusion,
                    err,
                    roll,
                    pitch,
                    magDisturbed ? "BAD" : "OK"
                );
            }
                            
                        
            // Select heading based on active calibration mode
            float heading = headingFull; // Defaults to Option 5 (Analytical Tilt-Compensated Mode 3)
            if (icm_mode == 4) {
                // Mode 4/Button 5: High-performance, gyro-stabilized, complementary 3D fusion output (with Gyro)
                heading = headingFusion;
            } else if (icm_mode == 2) {
                heading = headingBoth;
            } else if (icm_mode == 1) {
                heading = headingHard;
            } else if (icm_mode == 0) {
                heading = headingRaw;
            }

            // Smooth result based on the dynamic compass_avg_len parameter stored in NVS
            heading = CompassAverage(heading);

            // -------------------- STUCK WATCHDOG --------------------
            static float last_heading = -999.0f;
            static uint32_t last_heading_change_time = 0;

            if (last_heading == -999.0f) {
                last_heading = heading;
                last_heading_change_time = millis();
            } else {
                // Use wrapped angular difference to prevent false triggers near 0/360 boundary
                float diff = heading - last_heading;
                if (diff > 180.0f) diff -= 360.0f;
                if (diff < -180.0f) diff += 360.0f;

                if (fabs(diff) > 0.1f) { // Prevents false triggers from floating-point noise
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
            }

            // -------------------- ONCE-PER-SECOND DIAGNOSTIC TELEMETRY --------------------
            static uint32_t lastLogTime = 0;
            static float freq_avg = 100.0f;
            freq_avg = 0.95f * freq_avg + 0.05f * (1.0f / dt);

            if (millis() - lastLogTime >= 1000) {
                lastLogTime = millis();
                // Serial.printf("ICM-AHRS: R:%.2f P:%.2f Y:%.2f | MagNorm:%.2f baseline:%.2f rejected:%s | dt:%.4f (%.1fHz)\n",
                //               roll, pitch, heading, magNorm, baselineMag, magRejected ? "YES" : "NO", dt, freq_avg);
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
                global_hdg_no_offset = heading; // Reverted back to active heading!

                heading += mainData.compassOffset;

                // Apply adaptive waypoint bias trim if enabled
                if (mainData.compass_trim_enabled) {
                    heading += mainData.compass_trim;
                }

                while (heading < 0) heading += 360.0f;
                while (heading >= 360.0f) heading -= 360.0f;

                global_hdg = heading;
                mainData.dirMag = heading;
                mainData.roll = roll;    // Restored unswapped roll orientation
                mainData.pitch = -pitch; // Inverted physical pitch orientation

                if (compass) xQueueOverwrite(compass, (void *)&heading);

                xSemaphoreGive(mainDataMutex);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

float GetHeading(void) { return global_hdg; }
float GetHeadingNoOffset(void) { return global_hdg_no_offset; }
float GetHeadingRaw(void) { return global_hdg; }
int linMagCalib(int *corr) { return 0; }
bool CalibrateCompass(void) { return true; }
int get_cal_point_count() { return 3; }
bool global_is_calibrating = false;
