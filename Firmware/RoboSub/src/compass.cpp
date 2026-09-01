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

// Forward declaration of linear interpolation helper
float getInterpolatedHeading(float h);
float inverseInterpolatedHeading(float target);
float computeSetAsNorthOffset(void);
void computeFourierCoefficients();

extern Preferences storage;
extern float global_speed_bb;
extern float global_speed_sb;
QueueHandle_t compass = NULL;
QueueHandle_t compassIn = NULL;

bool icm_ready = false;
bool magRejected = false;
bool firstHeadingRun = true;
bool yaw_initialized = false;
volatile bool interp_enabled = false;
uint32_t lastMicros = 0;
uint32_t lastInitTime = 0;
float baselineMag = 50.0f;
volatile int icm_mode = 4; // Defaults to Mode 4 (Hard & Soft Iron with Pitch & Roll tilt compensation)
float pr_damping = 0.95f; // Exponential damping factor for pitch and roll (0.00 = none, 0.99 = max)
float damp_acc = 0.15f;
float damp_gyro = 0.15f;
float damp_mag = 0.15f;
float damp_att = 0.15f;
float measured_angles[9] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 225.0f, 270.0f, 315.0f, 360.0f};
// False when the table cannot be interpolated - see computeFourierCoefficients(). An uncorrected
// compass is wrong by the deviation; a compass corrected through a broken table is wrong by
// anything at all, so the first is the safer failure.
bool interp_table_usable = true;

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
volatile float global_hdg = 0;
volatile float global_hdg_no_offset = 0;
volatile float global_fusion_hdg = 0;
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
        Serial.printf("ICM-20948: Gyroscope calibration complete. Offsets -> X: %.4f, Y: %.4f, Z: %.4f\r\n", 
                      gyro_bias_x, gyro_bias_y, gyro_bias_z);

        // Play a super happy, ascending major arpeggio victory tune to celebrate successful gyro calibration!
        if (buzzer != NULL) {
            int notes[] = {523, 659, 784, 1047}; // C5, E5, G5, C6
            int durations[] = {120, 120, 120, 350};
            int pauses[] = {40, 40, 40, 200};
            for (int i = 0; i < 4; i++) {
                Buzz note;
                note.hz = notes[i];
                note.duration = durations[i];
                note.pause = pauses[i];
                note.repeat = 0;
                xQueueSend(buzzer, (void *)&note, 0);
            }
        }

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
                    Serial.printf("ICM-20948: Outlier baseline measurement rejected: %.2f uT (deviation: %.2f uT)\r\n", 
                                  norms[i], fabs(norms[i] - initial_avg));
                }
            }
            
            if (inlier_count > 0) {
                baselineMag = final_sum / inlier_count;
                Serial.printf("ICM-20948: Magnetometer baseline learned (using %d inliers): %.4f uT\r\n", inlier_count, baselineMag);
            } else {
                baselineMag = initial_avg; // Fallback to initial raw average
                Serial.printf("ICM-20948: All baseline samples flagged as outliers! Using raw average: %.4f uT\r\n", baselineMag);
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
    extern int compass_avg_len;
    memCompassAvg(&compass_avg_len, MEM_GET);
    memPrDamping(&pr_damping, MEM_GET);
    memDampingFactors(&damp_acc, &damp_gyro, &damp_mag, &damp_att, MEM_GET);
    // Attitude damping dynamically maps to pr_damping (where pr_damping = 1.0f - damp_att)
    pr_damping = 1.0f - damp_att;
    if (pr_damping < 0.0f) pr_damping = 0.0f;
    if (pr_damping > 0.99f) pr_damping = 0.99f;
    
    float trim_val = 0.0f;
    bool trim_en = false;
    memCompassTrim(&trim_val, &trim_en, MEM_GET);
    mainData.compass_trim = trim_val;
    mainData.compass_trim_enabled = trim_en;

    // Initialize 8-point linear interpolation table from Preferences NVM
    memInterpolationTable(measured_angles, MEM_GET);
    computeFourierCoefficients();
    bool temp_enabled = false;
    memInterpEnabled(&temp_enabled, MEM_GET);
    interp_enabled = temp_enabled;

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
            // static uint32_t lastDebug = 0;

            // if (millis() - lastDebug > 1000)
            // {
            //     lastDebug = millis();
            //     Serial.printf(
            //         "HDG=%.1f FUS=%.1f ERR=%.1f R=%.1f P=%.1f MAG=%s\r\n",
            //         headingFull,
            //         headingFusion,
            //         err,
            //         roll,
            //         pitch,
            //         magDisturbed ? "BAD" : "OK"
            //     );
            // }
                            
                        
            // Select heading based on active calibration mode
            float heading = headingFull; // Default fallback
            if (icm_mode == 4) {
                // Mode 4/Option 5: High-performance, gyro-stabilized, complementary 3D fusion output (with Gyro)
                heading = headingFusion;
            } else if (icm_mode == 3) {
                // Mode 3/Option 4: Analytical Tilt-Compensated heading
                heading = headingFull;
            } else if (icm_mode == 2) {
                // Mode 2/Option 3: Both Iron (Offsets + Scaling)
                heading = headingBoth;
            } else if (icm_mode == 1) {
                // Mode 1/Option 2: Hard Iron Comp (Offsets only)
                heading = headingHard;
            } else if (icm_mode == 0) {
                // Mode 0/Option 1: Raw uncompensated data
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
                // Serial.printf("CompassTask -> icm_mode: %d, headingRaw: %.2f, headingFusion: %.2f, heading: %.2f, offset: %.2f\r\n",
                //               icm_mode, headingRaw, headingFusion, heading, mainData.compassOffset);
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
                // Add the manual compass offset first
                heading += mainData.compassOffset;
                while (heading < 0.0f) heading += 360.0f;
                while (heading >= 360.0f) heading -= 360.0f;

                global_hdg_no_offset = heading; // Value after manual offset is added and before harmonic correction

                // Apply 8-point smooth harmonic curve correction directly as a production add-on if enabled by the user!
                if (interp_enabled) {
                    heading = getInterpolatedHeading(heading);
                }

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

float fourier_A0 = 0.0f;
float fourier_A1 = 0.0f;
float fourier_B1 = 0.0f;
float fourier_A2 = 0.0f;
float fourier_B2 = 0.0f;

/**
 * @brief Computes 1st and 2nd harmonic Fourier coefficients based on 8 measured calibration points.
 */
// Called whenever the table changes. There are no coefficients to precompute any more - the
// correction reads the table directly - but the table still has to be CHECKED before it is used.
//
// Piecewise interpolation needs the measured angles to increase all the way round. A curve fit did
// not care: it would happily smooth over a point that came out of order. This does care, because
// an out-of-order point makes a segment of zero or negative width and the interpolation across it
// is meaningless. One badly aimed direction is enough to cause it, so it is checked rather than
// assumed, and a table that fails leaves the compass UNCORRECTED instead of corrected wrongly.
void computeFourierCoefficients() {
    // Entry 8 is the 360 degree wrap of entry 0. Kept consistent here so the Sub's web page and
    // anything else reading all nine sees a sane value.
    measured_angles[8] = measured_angles[0] + 360.0f;

    float base = measured_angles[0];
    float prev = 0.0f;
    bool ok = true;

    for (int i = 1; i < 8; i++) {
        float v = fmodf(measured_angles[i] - base + 360.0f, 360.0f);
        if (!(v > prev)) { ok = false; break; }
        prev = v;
    }

    interp_table_usable = ok;

    if (ok) {
        Serial.printf("Compass table accepted: ");
        for (int i = 0; i < 8; i++) Serial.printf("%.1f ", measured_angles[i]);
        Serial.printf("\r\n");
    } else {
        Serial.printf("Compass table REJECTED - the eight measured angles are not in increasing "
                      "order, so a direction was mis-aimed. Running UNCORRECTED.\r\n");
    }
}

/**
 * @brief Performs 360-degree Fourier 2-harmonic smooth curve correction.
 */

// ---- Guided eight point calibration -------------------------------------------------------
//
// One session on the buoy, driven by whichever interface the operator happens to be using. The
// rules live here and nowhere else: the Sub's two pages, the Top's page, the CYD dashboard and the
// CYD touchscreen all just show the state and press the buttons. Five copies of this arithmetic is
// how a corrected heading ended up in the table in the first place.
//
// The flow the operator sees:
//   begin           arms the session, asks for north
//   set             point the hull at the direction being asked for, press. Captures and advances.
//   ...             eight times, 0 45 90 ... 315
//   save or cancel  nothing on the buoy changes until save
//
// North first, and it is special: it defines the reference. The offset absorbs it, so table entry
// 0 is 0 by construction and the compass reads exactly north there with the correction ON and with
// it OFF. Everything after is measured relative to that same north.
//
// The captures are of Imag - the heading with the iron correction and the mounting offset applied
// but before the table and before the trim, which is the value the table is indexed by. Nothing on
// the buoy has to be switched off to read it, so the correction and the trim can stay exactly as
// they are for the whole procedure.
bool cal8_active = false;
int cal8_next = 0;                 // which direction is being asked for, 0..7
float cal8_imag_north = 0.0f;      // Imag when north was captured; the whole session hangs off this
float cal8_captured[8] = {0};

void cal8Begin(void)
{
    cal8_active = true;
    cal8_next = 0;
    cal8_imag_north = 0.0f;
    for (int i = 0; i < 8; i++) cal8_captured[i] = 0.0f;
    Serial.println("8 point calibration: started, point the hull NORTH and press set");
}

void cal8Cancel(void)
{
    cal8_active = false;
    // Back to nothing, not "finished at step 8". A closed session that still reports a step makes
    // every screen watching it show a run that is not there.
    cal8_next = 0;
    Serial.println("8 point calibration: cancelled, nothing written");
}

// Capture the direction currently being asked for. Returns the index just filled, or -1.
//
// expect_leg is the leg the CALLER believes is next, or -1 for "whatever is next". Remote callers
// pass it and it is what makes a press safe to retry: the Top-to-Sub link is one half-duplex wire
// and drops most of what is sent while the Sub is talking, so presses have to be repeated - and a
// repeat that arrived after the original had already landed would capture the next leg against the
// hull's old heading. With the leg named, a late duplicate is simply ignored.
int cal8Set(int expect_leg)
{
    if (!cal8_active || cal8_next > 7) return -1;
    if (expect_leg >= 0 && expect_leg != cal8_next)
    {
        Serial.printf("8 point calibration: ignoring a capture meant for leg %d, we are on %d\r\n",
                      expect_leg, cal8_next);
        return -1;
    }

    float imag = GetHeadingNoOffset();
    int idx = cal8_next;

    if (idx == 0) {
        // North defines the reference. Entry 0 is 0 by definition - the offset takes the whole of
        // it at save time - so the table carries only the SHAPE of the deviation. That is what
        // keeps north correct when the correction is switched off.
        cal8_imag_north = imag;
        cal8_captured[0] = 0.0f;
    } else {
        float v = imag - cal8_imag_north;
        while (v < 0.0f) v += 360.0f;
        while (v >= 360.0f) v -= 360.0f;
        cal8_captured[idx] = v;
    }

    cal8_next++;
    Serial.printf("8 point calibration: %d deg captured (Imag %.2f -> entry %.2f), %d of 8 done\r\n",
                  idx * 45, imag, cal8_captured[idx], cal8_next);
    return idx;
}

// Commit. Both halves land together or not at all - a table written against an offset that was
// never applied would be wrong at every heading.
bool cal8Save(void)
{
    if (!cal8_active || cal8_next < 8) return false;

    // The offset absorbs north. Measured with the OLD offset in force, so it is a correction TO
    // that offset, not a replacement: sensor + newOffset then equals the entry that was stored.
    double newOffset = mainData.compassOffset - cal8_imag_north;
    while (newOffset < -180.0) newOffset += 360.0;
    while (newOffset > 180.0) newOffset -= 360.0;
    mainData.compassOffset = newOffset;
    CompasOffset(&mainData, MEM_PUT);

    for (int i = 0; i < 8; i++) measured_angles[i] = cal8_captured[i];
    measured_angles[8] = measured_angles[0] + 360.0f;
    memInterpolationTable(measured_angles, MEM_PUT);
    computeFourierCoefficients();          // validates the table, see there

    bool enable = true;
    interp_enabled = true;
    memInterpEnabled(&enable, MEM_PUT);

    cal8_active = false;
    extern int global_params_rev;
    global_params_rev++;

    Serial.printf("8 point calibration SAVED: offset %.2f, table ", newOffset);
    for (int i = 0; i < 8; i++) Serial.printf("%.1f ", measured_angles[i]);
    Serial.printf("\r\n");

    extern QueueHandle_t buzzer;
    if (buzzer != NULL) beep(5, buzzer);   // the short happy tune
    return true;
}

// Inverse of getInterpolatedHeading(): the pre-correction heading that comes out as `target`.
//
// The correction is a fitted Fourier curve, corrected = h + err(h), with err running to tens of
// degrees on a real hull - so it cannot be inverted in closed form and its slope is nowhere near 1.
// A coarse 1 degree sweep for the nearest solution, then a 0.01 degree sweep around it: 560 cheap
// evaluations, run once when a button is pressed. Plenty finer than the compass itself.
//
// Deliberately not a fixed-point iteration (h <- target - err(h)): that only converges while
// |err'| < 1, and the whole reason this function exists is tables where it is not.
float inverseInterpolatedHeading(float target) {
    while (target < 0.0f) target += 360.0f;
    while (target >= 360.0f) target -= 360.0f;

    float best_h = target, best_d = 1e9f;
    for (int i = 0; i < 360; i++) {
        float h = (float)i;
        float d = fabsf(fmodf(getInterpolatedHeading(h) - target + 540.0f, 360.0f) - 180.0f);
        if (d < best_d) { best_d = d; best_h = h; }
    }
    float coarse = best_h;
    for (int i = -100; i <= 100; i++) {
        float h = coarse + (float)i * 0.01f;
        float d = fabsf(fmodf(getInterpolatedHeading(h) - target + 540.0f, 360.0f) - 180.0f);
        if (d < best_d) { best_d = d; best_h = h; }
    }

    while (best_h < 0.0f) best_h += 360.0f;
    while (best_h >= 360.0f) best_h -= 360.0f;
    return best_h;
}

// The compassOffset that makes the CURRENT heading read exactly north, in one go.
//
// The old sum was compassOffset - dirMag, which assumes the offset moves the reported heading one
// for one. It does not: the offset is added BEFORE the correction curve (see the output block in
// the compass task), so a change of x at the input comes out as x * (1 + err') at the output. With
// a real table that factor is nowhere near 1, which is why Set as North used to need three or four
// presses to creep onto north instead of landing on it.
//
// Solved the other way round instead. The pipeline is
//     dirMag = interp(sensor + offset) + trim
// so for dirMag == 0 we need the input to the curve to be interp^-1(-trim), and the offset that
// puts it there is that value minus the bare sensor reading. Exact in a single press, and it stays
// exact whether the correction and the trim are switched on or off.
float computeSetAsNorthOffset(void)
{
    float trim = mainData.compass_trim_enabled ? (float)mainData.compass_trim : 0.0f;

    // global_hdg_no_offset is the heading with the offset already added but before the curve, so
    // backing the offset out again leaves the bare sensor heading.
    float sensor = global_hdg_no_offset - (float)mainData.compassOffset;
    while (sensor < 0.0f) sensor += 360.0f;
    while (sensor >= 360.0f) sensor -= 360.0f;

    float want = -trim;
    while (want < 0.0f) want += 360.0f;
    while (want >= 360.0f) want -= 360.0f;

    float target_pre = interp_enabled ? inverseInterpolatedHeading(want) : want;

    float offset = target_pre - sensor;
    while (offset < -180.0f) offset += 360.0f;
    while (offset > 180.0f) offset -= 360.0f;
    return offset;
}

// Correct a heading using the eight measured calibration points.
//
// Straight piecewise interpolation THROUGH the points, not a curve fitted near them. That is what
// the eight measurements were always for - the previous version condensed them into five Fourier
// coefficients (A0, A1, B1, A2, B2), which cannot pass through eight arbitrary values, so on this
// hull it left up to 3.8 degrees of error at the very headings that had just been measured. Most
// of the operator's careful aiming was being averaged away.
//
// measured_angles[i] is what the compass reads when the hull physically points at i * 45 degrees,
// so the table maps measured -> true and this walks the segment the reading falls in. Error at the
// eight nodes is now exactly zero, including north. Between nodes it is a straight line across 45
// degrees, which on real data differs from the old smooth curve by at most 2.7 degrees - magnetic
// deviation simply cannot swing far inside a 45 degree span.
//
// Everything is measured relative to measured_angles[0] so a table that wraps past 360 - which is
// normal, north often reads 355-ish - needs no special case.
float getInterpolatedHeading(float h) {
    if (!interp_table_usable) return h;   // see computeFourierCoefficients()

    while (h < 0.0f) h += 360.0f;
    while (h >= 360.0f) h -= 360.0f;

    float base = measured_angles[0];
    float hh = fmodf(h - base + 360.0f, 360.0f);

    for (int i = 0; i < 8; i++) {
        float a = (i == 0) ? 0.0f : fmodf(measured_angles[i] - base + 360.0f, 360.0f);
        float b = (i == 7) ? 360.0f : fmodf(measured_angles[i + 1] - base + 360.0f, 360.0f);

        if (hh >= a && hh <= b) {
            float span = b - a;
            if (span < 0.001f) return h;          // degenerate segment, leave it alone
            float f = (hh - a) / span;
            float out = (float)i * 45.0f + f * 45.0f;
            while (out < 0.0f) out += 360.0f;
            while (out >= 360.0f) out -= 360.0f;
            return out;
        }
    }
    return h;
}
