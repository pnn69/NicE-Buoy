/**
 * @file compass.cpp
 * @brief BNO055 Intelligent Compass System with Persistence and Atomic Injection.
 * 
 * This module handles the BNO055 sensor fusion, auto-calibration monitoring,
 * and the logic to store/restore calibration profiles to ESP32 NVS (Flash).
 * 
 * Key Features:
 * 1. Atomic Injection: Writes 22-byte calibration profiles directly to BNO registers.
 * 2. Mode-Lock Retry: Robustly ensures CONFIG/NDOF mode transitions.
 * 3. Dual-Verification: Confirms injection by reading back registers immediately.
 * 4. Auto-Persistence: Saves to NVS once Magnetometer level 3 is achieved.
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <RoboCompute.h>
#include <Adafruit_BNO055.h>
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
#define NUM_DIRECTIONS 50

// BNO055 Hardware Register Addresses
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_PAGE_ID_ADDR 0x07
#define BNO055_ACC_OFFSET_X_LSB_ADDR 0x55

extern Preferences storage;
QueueHandle_t compass = NULL;
QueueHandle_t compassIn = NULL;

bool bno_ready = false;
String bno_error = "BNO Mode";
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
uint8_t bno_i2c_addr = 0x28; // Detected dynamically (0x28 or 0x29)

extern RoboStruct mainData;
extern Message escOut;
extern int subStatus;
extern AsyncUDP udp;
extern SemaphoreHandle_t mainDataMutex;

// Global state for web dashboard telemetry
float global_hdg = 0;
float global_icmHdg = 0;
bool icm_ready = false;
uint8_t bno_cal_sys = 0, bno_cal_gyro = 0, bno_cal_accel = 0, bno_cal_mag = 0;
float last_raw_x = 0, last_raw_y = 0, last_raw_z = 0;
float last_raw_ax = 0, last_raw_ay = 0, last_raw_az = 0;
uint32_t global_loop_cnt = 0;
String global_cal_msg = "BNO Active";
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
 * @brief Converts 22-byte profile to Hex strings for UI telemetry.
 */
void updateUIHex(uint8_t *data, bool loaded) {
    char hex[100];
    char *p = hex;
    for (int i = 0; i < 22; i++) {
        p += sprintf(p, "%02X", data[i]);
    }
    if (loaded) global_cal_load = String(hex);
    else global_cal_ver = String(hex);
}

/**
 * @brief Low-level register read from BNO055.
 */
uint8_t readReg(uint8_t reg) {
    Wire.beginTransmission(bno_i2c_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    if (Wire.requestFrom(bno_i2c_addr, (uint8_t)1) != 1) return 0xFF;
    return Wire.read();
}

/**
 * @brief Low-level register write to BNO055.
 */
void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(bno_i2c_addr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

/**
 * @brief Forces the BNO055 into a target operation mode with retries.
 * Necessary because the sensor sometimes ignores mode switches if the internal
 * fusion engine is busy or the I2C bus is congested.
 */
bool forceMode(uint8_t targetMode) {
    for (int i = 0; i < 5; i++) {
        writeReg(BNO055_OPR_MODE_ADDR, targetMode);
        vTaskDelay(pdMS_TO_TICKS(200));
        uint8_t current = readReg(BNO055_OPR_MODE_ADDR);
        if (current == targetMode) return true;
        Serial.printf("BNO055: Mode switch retry %d (Target 0x%02X, Current 0x%02X)\r\n", i+1, targetMode, current);
    }
    return false;
}

/**
 * @brief Reads the full 22-byte calibration profile directly from Page 0 registers.
 * Bypasses library checks to ensure we can always capture the current state.
 */
bool getBnoOffsetsDirect(uint8_t *data) {
    if (!forceMode(0x00)) return false; // Must be in CONFIG mode to read offsets
    
    writeReg(BNO055_PAGE_ID_ADDR, 0x00); // Offsets are on Page 0
    vTaskDelay(pdMS_TO_TICKS(20));

    Wire.beginTransmission(bno_i2c_addr);
    Wire.write(BNO055_ACC_OFFSET_X_LSB_ADDR);
    if (Wire.endTransmission(false) != 0) {
        forceMode(0x0C);
        return false;
    }
    
    uint8_t count = Wire.requestFrom(bno_i2c_addr, (uint8_t)22);
    bool hasData = false;
    if (count == 22) {
        for (int i = 0; i < 22; i++) {
            data[i] = Wire.read();
            if (data[i] != 0) hasData = true;
        }
    }
    forceMode(0x0C); // Return to NDOF (Fusion) mode
    return hasData;
}

/**
 * @brief Performs an atomic write and immediate read-back of the calibration profile.
 * Ensures the data actually 'lands' in the registers while the chip is in CONFIG mode.
 */
bool setAndVerifyOffsets(const uint8_t *data, uint8_t *verify) {
    Serial.println("BNO055: Starting Injection sequence...\r\n");
    
    if (!forceMode(0x00)) {
        Serial.println("BNO055: FATAL - Could not enter CONFIG mode!\r\n");
        return false;
    }

    writeReg(BNO055_PAGE_ID_ADDR, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Burst write of all 22 registers (Acc, Mag, Gyro offsets + Radii)
    Wire.beginTransmission(bno_i2c_addr);
    Wire.write(BNO055_ACC_OFFSET_X_LSB_ADDR);
    for (int i = 0; i < 22; i++) Wire.write(data[i]);
    if (Wire.endTransmission() != 0) {
        Serial.println("BNO055: ERROR - I2C Burst Write Failed!\r\n");
        forceMode(0x0C);
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    // Immediate read-back while still in CONFIG for audit trail
    Wire.beginTransmission(bno_i2c_addr);
    Wire.write(BNO055_ACC_OFFSET_X_LSB_ADDR);
    Wire.endTransmission(false);
    uint8_t count = Wire.requestFrom(bno_i2c_addr, (uint8_t)22);
    if (count == 22) {
        for (int i = 0; i < 22; i++) verify[i] = Wire.read();
    }

    if (!forceMode(0x0C)) {
        Serial.println("BNO055: ERROR - Could not return to NDOF mode!\r\n");
    }
    
    return true;
}

/**
 * @brief Detects and initializes the BNO055 sensor, injecting stored NVS profiles.
 */
bool InitCompass(void)
{
    Serial.println("\r\nInitializing BNO055 Compass V13.6...\r\n");
    bool found = false;
    
    // Auto-discover I2C address
    Wire.beginTransmission(0x28);
    if (Wire.endTransmission() == 0) { bno_i2c_addr = 0x28; found = true; }
    else {
        Wire.beginTransmission(0x29);
        if (Wire.endTransmission() == 0) {
            bno_i2c_addr = 0x29;
            found = true;
            bno = Adafruit_BNO055(55, 0x29, &Wire);
        }
    }

    if (found && bno.begin()) {
        bno_ready = true;
        bno.setExtCrystalUse(true);
        Serial.printf("BNO055: Found and ready at 0x%02X\r\n", bno_i2c_addr);

        // Retrieve profile from ESP32 NVS
        uint8_t calData[22];
        memBnoCalib(calData, true);
        updateUIHex(calData, true);

        bool hasProfile = false;
        for(int i=0; i<22; i++) if(calData[i] != 0) hasProfile = true;

        if (hasProfile) {
            uint8_t verifyData[22];
            if (setAndVerifyOffsets(calData, verifyData)) {
                updateUIHex(verifyData, false);
                bool match = true;
                for(int i=0; i<22; i++) if(calData[i] != verifyData[i]) match = false;
                if (match) {
                    setCalMsg("Profile Loaded OK", 1);
                    Serial.println("BNO055: VERIFICATION MATCHED.\r\n");
                } else {
                    setCalMsg("Profile Mismatch", 1);
                    Serial.println("BNO055: VERIFICATION MISMATCH.\r\n");
                }
            }
        } else {
            setCalMsg("No Profile in NVS");
            Serial.println("BNO055: No profile found in NVS.\r\n");
        }
    } else {
        bno_error = "BNO NOT FOUND";
        Serial.println("BNO055: CRITICAL ERROR - Sensor not found!\r\n");
    }

    // Load remaining persistent parameters
    CompassOffsetCorrection(&mainData.compassOffset, true);
    CompasOffset(&mainData, true);
    MechanicalCorrection(&mainData.mechanicCorrection, true);
    return bno_ready;
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
    static bool autoSaved = false;
    while (1) {
        global_loop_cnt++;
        int cmd = 0;
        
        // Handle manual commands from web interface
        if (compassIn && xQueueReceive(compassIn, &cmd, 0) == pdTRUE) {
            if (cmd == 34) { // Manual Save Trigger
                uint8_t calData[22];
                if (getBnoOffsetsDirect(calData)) {
                    memBnoCalib(calData, false);
                    updateUIHex(calData, false);
                    setCalMsg("MANUAL SAVE SUCCESS", 0);
                    autoSaved = true;
                } else {
                    setCalMsg("SAVE FAILED - NO DATA", 0);
                }
            }
        }

        if (bno_ready) {
            sensors_event_t event;
            bno.getEvent(&event);
            float heading = event.orientation.x;
            
            // Monitor internal calibration status
            bno.getCalibration(&bno_cal_sys, &bno_cal_gyro, &bno_cal_accel, &bno_cal_mag);
            
            // AUTO-SAVE: Triggered when high accuracy is reached (M:3, G:3)
            if (!autoSaved && bno_cal_mag == 3 && bno_cal_gyro == 3) {
                uint8_t calData[22];
                if (getBnoOffsetsDirect(calData)) {
                    memBnoCalib(calData, false);
                    updateUIHex(calData, false);
                    autoSaved = true;
                    setCalMsg("AUTO-SAVE SUCCESS", 0);
                }
            }
            if (bno_cal_mag < 2) autoSaved = false; // Allow re-save if accuracy drops

            // Update UI status strings
            if (millis() > cal_msg_timeout) {
                char buf[100];
                if (bno_cal_mag == 3 && bno_cal_accel == 0) snprintf(buf, sizeof(buf), "TILT BUOY! (M:3 A:0)");
                else snprintf(buf, sizeof(buf), "S:%d G:%d A:%d M:%d", bno_cal_sys, bno_cal_gyro, bno_cal_accel, bno_cal_mag);
                global_cal_msg = String(buf);
            }

            // Sync with main navigation structure
            if (mainDataMutex && xSemaphoreTake(mainDataMutex, portMAX_DELAY)) {
                heading += mainData.compassOffset;
                while (heading < 0) heading += 360.0f;
                while (heading >= 360.0f) heading -= 360.0f;
                global_hdg = heading;
                global_icmHdg = heading;
                // float activeHdg = CompassAverage(heading);
                float activeHdg = heading;
                mainData.dirMag = activeHdg;
                if (compass) xQueueOverwrite(compass, (void *)&activeHdg);
                xSemaphoreGive(mainDataMutex);
            }

            // Capture raw vectors for dashboard display
            imu::Vector<3> magV = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
            imu::Vector<3> accV = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
            last_raw_x = magV.x(); last_raw_y = magV.y(); last_raw_z = magV.z();
            last_raw_ax = accV.x(); last_raw_ay = accV.y(); last_raw_az = accV.z();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

float GetHeading(void) { return global_hdg; }
float GetHeadingRaw(void) { return global_hdg; }
int linMagCalib(int *corr) { return 0; }
bool CalibrateCompass(void) { return true; }
int get_cal_point_count() { return bno_cal_mag; }
bool global_is_calibrating = false;
