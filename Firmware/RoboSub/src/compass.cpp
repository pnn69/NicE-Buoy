#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <RoboCompute.h>
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
#include "mag_calib.h"

#define NUM_DIRECTIONS 50
#define AVERIDGE_COUNT 3

#define LSM_MAG_ADDR 0x1E
#define LSM_ACC_ADDR 0x19
#define ICM_ADDR     0x69

String global_scan_results = "Hybrid Mode";
extern Preferences storage;
QueueHandle_t compass;
QueueHandle_t compassIn;

bool bno_ready = false;
String bno_error = "Hybrid Mode V11";

extern RoboStruct mainData;
extern Message escOut;
extern int subStatus;
extern AsyncUDP udp;

float global_icmHdg = 0;
uint8_t bno_cal_sys = 0, bno_cal_gyro = 0, bno_cal_accel = 0, bno_cal_mag = 0;
float last_raw_x = 0, last_raw_y = 0, last_raw_z = 0;
float last_raw_ax = 0, last_raw_ay = 0, last_raw_az = 0;

float h_xz = 0, h_xy = 0, h_yz = 0;
uint32_t global_loop_cnt = 0;
String global_cal_msg = "Ready V11";

void udpsend(const char* msg) {
    if (udp) udp.broadcast(msg);
}

bool writeReg(uint8_t dev, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(dev);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

bool readRegs(uint8_t dev, uint8_t reg, uint8_t* buf, int len) {
    Wire.beginTransmission(dev);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(dev, (uint8_t)len);
    for (int i=0; i<len; i++) {
        if (Wire.available()) buf[i] = Wire.read();
        else return false;
    }
    return true;
}

bool InitCompass(void)
{
    Serial.println("Initializing Hybrid Compass V11...");
    Wire.begin(21, 22);
    Wire.setClock(100000);
    vTaskDelay(pdMS_TO_TICKS(100));

    writeReg(ICM_ADDR, 0x7F, 0x00);
    writeReg(ICM_ADDR, 0x06, 0x01);
    vTaskDelay(pdMS_TO_TICKS(50));
    writeReg(LSM_MAG_ADDR, 0x60, 0x0C);
    writeReg(LSM_ACC_ADDR, 0x20, 0x57);

    hardIron(&mainData, true);
    softIron(&mainData, true);

    if (mainData.magSoft[0][0] == 0) {
        for(int i=0; i<3; i++) for(int j=0; j<3; j++) mainData.magSoft[i][j] = (i==j ? 1.0f : 0.0f);
    }

    bno_ready = true; 
    bno_error = "Hybrid Mode V11 OK";
    return true;
}

bool read_hybrid(float &mx, float &my, float &mz, float &ax, float &ay, float &az) {
    uint8_t buf[6];
    if (readRegs(LSM_MAG_ADDR, 0x68 | 0x80, buf, 6)) {
        int16_t x = (buf[1] << 8) | buf[0];
        int16_t y = (buf[3] << 8) | buf[2];
        int16_t z = (buf[5] << 8) | buf[4];
        mx = x * 0.15f; my = y * 0.15f; mz = z * 0.15f;
    } else return false;
    if (readRegs(ICM_ADDR, 0x2D, buf, 6)) {
        int16_t x = (buf[0] << 8) | buf[1];
        int16_t y = (buf[2] << 8) | buf[3];
        int16_t z = (buf[4] << 8) | buf[5];
        ax = x / 16384.0f; ay = y / 16384.0f; az = z / 16384.0f;
    } else return false;
    return true;
}

bool global_is_calibrating = false;
static int global_cal_points = 0;
int get_cal_point_count() { return global_cal_points; }

bool CalibrateCompass(void) {
    MagCalibrator calibrator;
    mainData.status = CALIBRATE_MAGNETIC_COMPASS;
    global_is_calibrating = true;
    global_cal_points = 0;
    global_cal_msg = "Started - Tumble Buoy!";
    beep(1, buzzer);
    
    while (global_is_calibrating) {
        int cmd = 0;
        if (xQueueReceive(compassIn, &cmd, 0) == pdTRUE && (cmd == 132 || cmd == 32)) break;
        float hmx, hmy, hmz, hax, hay, haz;
        if (read_hybrid(hmx, hmy, hmz, hax, hay, haz)) {
            int prev = calibrator.getNumPoints();
            calibrator.addPoint(hmx, hmy, hmz);
            global_cal_points = calibrator.getNumPoints();
            if (global_cal_points > prev && (global_cal_points % 10 == 0)) beep(1, buzzer);
            last_raw_x = hmx; last_raw_y = hmy; last_raw_z = hmz;
            last_raw_ax = hax; last_raw_ay = hay; last_raw_az = haz;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    float hard[3]; float soft[3][3];
    if (calibrator.calculateCalibration(hard, soft)) {
        mainData.magHard[0] = hard[0]; mainData.magHard[1] = hard[1]; mainData.magHard[2] = hard[2];
        for(int i=0; i<3; i++) for(int j=0; j<3; j++) mainData.magSoft[i][j] = soft[i][j];
        hardIron(&mainData, false); softIron(&mainData, false); 
        global_cal_msg = "SUCCESS! Ready.";
        beep(5, buzzer);
    } else {
        global_cal_msg = "FAILED - Try again.";
        beep(2, buzzer);
    }
    mainData.status = IDLE;
    global_is_calibrating = false;
    return true;
}

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

void CompassTask(void *arg) {
    while (1) {
        global_loop_cnt++;
        int cmd = 0;
        if (xQueueReceive(compassIn, &cmd, 0) == pdTRUE) {
            if (cmd == CALIBRATE_MAGNETIC_COMPASS) CalibrateCompass();
        }

        if (!global_is_calibrating) {
            float hmx, hmy, hmz, hax, hay, haz;
            if (read_hybrid(hmx, hmy, hmz, hax, hay, haz)) {
                last_raw_x = hmx; last_raw_y = hmy; last_raw_z = hmz;
                last_raw_ax = hax; last_raw_ay = hay; last_raw_az = haz;

                // 1. Calibrate & Map LSM Magnetometer into ICM Accelerometer frame
                // Accel X is Vertical (Up). Mag Y is Vertical (Up).
                // Remap Mag: [Y, X, Z] so that Mag X matches Accel X (vertical)
                // Calculated Y center: (4 + -90) / 2 = -43.0
                float mx_aligned = hmy - (-43.0f);   // Mag Y is vertical
                float my_aligned = hmx - (-30.5f);  // Mag X is horizontal 1
                float mz_aligned = (hmz - 15.1f) * 1.288f; // Mag Z is horizontal 2

                // 2. Universal Tilt Compensation in aligned frame
                float normA = sqrtf(hax*hax + hay*hay + haz*haz);
                if (normA < 0.01f) normA = 1.0f;
                float dx = -hax / normA, dy = -hay / normA, dz = -haz / normA;

                float m_dot_d = mx_aligned * dx + my_aligned * dy + mz_aligned * dz;
                float mx_h = mx_aligned - m_dot_d * dx;
                float my_h = my_aligned - m_dot_d * dy;
                float mz_h = mz_aligned - m_dot_d * dz;

                // 3. Final Heading using projected horizontal components
                float heading = atan2f(-mz_h, my_h) * 180.0f / M_PI;
                if (heading < 0) heading += 360.0f;

                // Diag
                h_xz = heading;
                h_xy = atan2f(-my_aligned, mx_aligned) * 180.0f / M_PI; if (h_xy < 0) h_xy += 360.0f;
                h_yz = atan2f(-mz_aligned, mx_aligned) * 180.0f / M_PI; if (h_yz < 0) h_yz += 360.0f;

                heading += mainData.compassOffset;
                while (heading < 0) heading += 360.0f;
                while (heading >= 360.0f) heading -= 360.0f;

                global_icmHdg = heading;
                float activeHdg = CompassAverage(heading);
                mainData.dirMag = activeHdg;
                xQueueOverwrite(compass, (void *)&activeHdg);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

float GetHeading(void) { return global_icmHdg; }
float GetHeadingRaw(void) { return global_icmHdg; }
int linMagCalib(int *corr) { return 0; }
