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

#define LSM_MAG_ADDR 0x1E
#define LSM_ACC_ADDR 0x19
#define ICM_ADDR     0x69

String global_scan_results = "Hybrid Mode";
extern Preferences storage;
QueueHandle_t compass;
QueueHandle_t compassIn;

bool bno_ready = false;
String bno_error = "Diag Mode V4";

extern RoboStruct mainData;
extern Message escOut;
extern int subStatus;
extern AsyncUDP udp;

float global_icmHdg = 0;
uint8_t bno_cal_sys = 0, bno_cal_gyro = 0, bno_cal_accel = 0, bno_cal_mag = 0;
float last_raw_x = 0, last_raw_y = 0, last_raw_z = 0;
float last_raw_ax = 0, last_raw_ay = 0, last_raw_az = 0;

// Internal diag vars
float h_xz = 0, h_xy = 0, h_yz = 0;

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
    Serial.println("Initializing Hybrid Compass (Diag Mode)...");
    Wire.begin(21, 22);
    Wire.setClock(100000);
    vTaskDelay(pdMS_TO_TICKS(100));

    writeReg(ICM_ADDR, 0x7F, 0x00);
    writeReg(ICM_ADDR, 0x06, 0x01);
    writeReg(LSM_MAG_ADDR, 0x60, 0x0C);
    writeReg(LSM_ACC_ADDR, 0x20, 0x57);

    bno_ready = true; 
    bno_error = "Diag Mode V4 OK";
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

bool global_is_calibrating = false;
int get_cal_point_count() { return 0; }

void CompassTask(void *arg) {
    while (1) {
        float hmx, hmy, hmz, hax, hay, haz;
        if (read_hybrid(hmx, hmy, hmz, hax, hay, haz)) {
            last_raw_x = hmx; last_raw_y = hmy; last_raw_z = hmz;
            last_raw_ax = hax; last_raw_ay = hay; last_raw_az = haz;

            // Apply manual hard-iron from linearity test to see if it helps
            float mx_c = hmx - (-30.5f);
            float mz_c = (hmz - 15.1f);
            float my_c = hmy;

            // Calculate 3 different raw headings with inverted rotation
            h_xz = atan2f(-mz_c, mx_c) * 180.0f / M_PI; if (h_xz < 0) h_xz += 360.0f;
            h_xy = atan2f(-my_c, mx_c) * 180.0f / M_PI; if (h_xy < 0) h_xy += 360.0f;
            h_yz = atan2f(-mz_c, my_c) * 180.0f / M_PI; if (h_yz < 0) h_yz += 360.0f;

            // Use XZ as primary for now and apply the manual offset
            float heading = h_xz;
            heading += mainData.compassOffset;
            
            while (heading < 0) heading += 360.0f;
            while (heading >= 360.0f) heading -= 360.0f;

            global_icmHdg = heading;
            float activeHdg = CompassAverage(global_icmHdg);
            mainData.dirMag = activeHdg;
            xQueueOverwrite(compass, (void *)&activeHdg);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

float GetHeading(void) { return global_icmHdg; }
float GetHeadingRaw(void) { return global_icmHdg; }
int linMagCalib(int *corr) { return 0; }
bool CalibrateCompass(void) { return true; }
