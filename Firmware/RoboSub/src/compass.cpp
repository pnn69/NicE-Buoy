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

#define NUM_DIRECTIONS 25
#define ICM20948_ADDR_1 0x68
#define ICM20948_ADDR_2 0x69
#define AK09916_ADDR 0x0C
#define AVERIDGE_COUNT 3

extern Preferences storage;
QueueHandle_t compass;
QueueHandle_t compassIn;

bool icm_ready = false;
uint8_t icm_addr = ICM20948_ADDR_1;

extern RoboStruct mainData;
extern Message escOut;
extern int subStatus;
extern AsyncUDP udp;

vector_t<float> m_max;
vector_t<float> m_min;
vector_t<float> icm_max = {50, 50, 50};
vector_t<float> icm_min = {-50, -50, -50};

void writeI2C(uint8_t dev_addr, uint8_t reg, uint8_t data) {
  Wire.beginTransmission(dev_addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

bool readI2C(uint8_t dev_addr, uint8_t reg, uint8_t *data, uint8_t len) {
  Wire.beginTransmission(dev_addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(dev_addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) {
    data[i] = Wire.read();
  }
  return true;
}

bool read_mag(float &mx, float &my, float &mz) {
    if (!icm_ready) return false;
    uint8_t data[8];
    // Read 8 bytes starting from HXL (0x11)
    if (readI2C(AK09916_ADDR, 0x11, data, 8)) {
        int16_t x = (data[1] << 8) | data[0];
        int16_t y = (data[3] << 8) | data[2];
        int16_t z = (data[5] << 8) | data[4];

        mx = x * 0.15f;
        my = y * 0.15f;
        mz = z * 0.15f;
        return true;
    }
    return false;
}

void udpsend(const char* msg) {
    if (udp) { 
        udp.broadcast(msg);
    }
}

float global_icmHdg = 0;
float last_raw_x = 0, last_raw_y = 0, last_raw_z = 0;

bool InitCompass(void)
{
    Serial.println("Initializing Compass (Manual I2C)...");
    Wire.begin(21, 22);
    Wire.setClock(400000); 
    vTaskDelay(pdMS_TO_TICKS(100));

    // Load calibration factors
    float min_mag[3], max_mag[3];
    CompassCallibrationFactorsFloat(&max_mag[0], &max_mag[1], &max_mag[2], &min_mag[0], &min_mag[1], &min_mag[2], true);
    CompassCallibrationFactors(&mainData, true);

    if (abs(max_mag[0] - min_mag[0]) < 0.01) {
        m_min = (vector_t<float>){-50, -50, -50};
        m_max = (vector_t<float>){50, 50, 50};
    } else {
        m_min = (vector_t<float>){min_mag[0], min_mag[1], min_mag[2]};
        m_max = (vector_t<float>){max_mag[0], max_mag[1], max_mag[2]};
    }

    storage.begin("NicE_Buoy_Data", false);
    icm_max.x = storage.getFloat("IcmMaxX", 50.0);
    icm_max.y = storage.getFloat("IcmMaxY", 50.0);
    icm_max.z = storage.getFloat("IcmMaxZ", 50.0);
    icm_min.x = storage.getFloat("IcmMinX", -50.0);
    icm_min.y = storage.getFloat("IcmMinY", -50.0);
    icm_min.z = storage.getFloat("IcmMinZ", -50.0);
    storage.end();

    // Probe ICM20948
    Wire.beginTransmission(ICM20948_ADDR_1);
    if (Wire.endTransmission() == 0) {
        icm_addr = ICM20948_ADDR_1;
        Serial.println("Found ICM at 0x68");
        icm_ready = true;
    } else {
        Wire.beginTransmission(ICM20948_ADDR_2);
        if (Wire.endTransmission() == 0) {
            icm_addr = ICM20948_ADDR_2;
            Serial.println("Found ICM at 0x69");
            icm_ready = true;
        } else {
            Serial.println("ICM20948 not found!");
            icm_ready = false;
        }
    }

    if (icm_ready) {
        writeI2C(icm_addr, 0x7F, 0x00); // Bank 0
        writeI2C(icm_addr, 0x06, 0x81); // Reset
        vTaskDelay(pdMS_TO_TICKS(100));
        writeI2C(icm_addr, 0x06, 0x01); // Wake
        writeI2C(icm_addr, 0x0F, 0x02); // Bypass

        uint8_t whoami = 0;
        if (readI2C(AK09916_ADDR, 0x01, &whoami, 1)) {
            Serial.printf("AK09916 WIA2: 0x%02X\n", whoami);
            if (whoami == 0x09 || whoami == 0x48) {
                icm_ready = true;
                writeI2C(AK09916_ADDR, 0x31, 0x08); // Continuous mode 4 (100Hz)
            } else {
                icm_ready = false;
            }
        } else {
            icm_ready = false;
        }
    }

    CompassOffsetCorrection(&mainData.compassOffset, true);
    icmCompassOffsetLoad(&mainData, true);
    MechanicalCorrection(&mainData.mechanicCorrection, true);

    Serial.printf("Compass Initialized. ICM Status: %s, Offset: %0.1f, Mech: %0.1f\n", 
        icm_ready ? "OK" : "Error", mainData.icmCompassOffset, mainData.mechanicCorrection);
    return true;
}

bool CalibrateCompass(void)
{
    static unsigned long calstamp;
    uint32_t lokcnt = 0;
    Serial.println("Callibrating compass now!!!");
    
    mainData.status = CALIBRATE_MAGNETIC_COMPASS;
    
    LedData calLedStatus;
    calLedStatus.color = CRGB::Purple;
    calLedStatus.blink = BLINK_FAST;
    xQueueSend(ledStatus, (void *)&calLedStatus, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    beep(1, buzzer);

    calstamp = millis();
    float min_icm[3] = {1000000.0f, 1000000.0f, 1000000.0f};
    float max_icm[3] = {-1000000.0f, -1000000.0f, -1000000.0f};

    MagCalibrator cal_icm;

    while (millis() - calstamp <= 1000 * 60)
    {
        float mx, my, mz;
        if (read_mag(mx, my, mz)) {
            cal_icm.addPoint(mx, my, mz);

            min_icm[0] = fmin(min_icm[0], mx);
            max_icm[0] = fmax(max_icm[0], mx);
            min_icm[1] = fmin(min_icm[1], my);
            max_icm[1] = fmax(max_icm[1], my);
            min_icm[2] = fmin(min_icm[2], mz);
            max_icm[2] = fmax(max_icm[2], mz);

            if (lokcnt++ > 250)
            {
                lokcnt = 0;
                Serial.printf("Calibrating: ICM points=%d\r\n", cal_icm.getNumPoints());
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    float hard[3];
    float soft[3][3];
    bool icm_success = false;

    if (cal_icm.calculateCalibration(hard, soft)) {
        for(int i=0; i<3; i++) mainData.icmMagHard[i] = hard[i];
        for(int i=0; i<3; i++) for(int j=0; j<3; j++) mainData.icmMagSoft[i][j] = soft[i][j];
        icmHardIron(&mainData, false);
        icmSoftIron(&mainData, false);
        Serial.println("ICM Ellipsoid Fit successful!");
        icm_success = true;
    } else {
        Serial.println("ICM Ellipsoid Fit failed. Falling back to Hard Iron Min/Max.");
        mainData.icmMagHard[0] = (max_icm[0] + min_icm[0]) / 2;
        mainData.icmMagHard[1] = (max_icm[1] + min_icm[1]) / 2;
        mainData.icmMagHard[2] = (max_icm[2] + min_icm[2]) / 2;
        for(int i=0; i<3; i++) for(int j=0; j<3; j++) mainData.icmMagSoft[i][j] = (i==j) ? 1.0 : 0.0;
        icmHardIron(&mainData, false);
        icmSoftIron(&mainData, false);
    }

    icm_min = (vector_t<float>){min_icm[0], min_icm[1], min_icm[2]};
    icm_max = (vector_t<float>){max_icm[0], max_icm[1], max_icm[2]};
    
    storage.begin("NicE_Buoy_Data", false);
    storage.putFloat("IcmMaxX", icm_max.x);
    storage.putFloat("IcmMaxY", icm_max.y);
    storage.putFloat("IcmMaxZ", icm_max.z);
    storage.putFloat("IcmMinX", icm_min.x);
    storage.putFloat("IcmMinY", icm_min.y);
    storage.putFloat("IcmMinZ", icm_min.z);
    storage.end();
    
    Serial.printf("New calibration stored!!!\n\r");
    
    LedData idleLedStatus;
    idleLedStatus.color = CRGB::Yellow;
    idleLedStatus.blink = BLINK_OFF;
    xQueueSend(ledStatus, (void *)&idleLedStatus, 0);

    if (icm_success) {
        beep(5, buzzer);
        mainData.status = IDLE;
    } else {
        beep(3, buzzer);
        mainData.status = ERROR;
        vTaskDelay(pdMS_TO_TICKS(1000));
        mainData.status = IDLE;
    }

    return true;
}

float GetHeading(void)
{
    return global_icmHdg;
}

float GetHeadingRaw(void)
{
    return global_icmHdg;
}

float CompassAverage(float in)
{
    static float directions[NUM_DIRECTIONS] = {0};
    static int cbufpointer = 0;
    static bool cbufFull = false;

    if (isnan(in)) return 0.0f;

    directions[cbufpointer++] = in;
    if (cbufpointer >= NUM_DIRECTIONS)
    {
        cbufpointer = 0;
        cbufFull = true;
    }

    int count = cbufFull ? NUM_DIRECTIONS : cbufpointer;
    if (count == 0) return in;

    float sum_x = 0.0, sum_y = 0.0, avg_dir;
    for (int i = 0; i < count; i++)
    {
        sum_x += cos(directions[i] * M_PI / 180.0);
        sum_y += sin(directions[i] * M_PI / 180.0);
    }

    if (abs(sum_x) < 0.0001 && abs(sum_y) < 0.0001) return in;

    avg_dir = atan2(sum_y, sum_x) * 180.0 / M_PI;
    if (avg_dir < 0) avg_dir += 360.0;
    if (avg_dir >= 360) avg_dir -= 360;
    return avg_dir;
}

#define ABUF 20
static double tbuf[2][ABUF];
static int pointer = 0;
static unsigned long timer1;
static char bufff[150];
static int stage = 0;
static double tmp_mhdg = 0;
static int tel = 0;
static int ret = 0;

int linMagCalib(int *corr)
{
    ret = -1;
    switch (stage)
    {
    case 0:
        udpsend("Start calibrating compass");
        mainData.speed = 30;
        mainData.speedBb = 30;
        mainData.speedSb = 30;
        pointer = 0;
        mainData.icmCompassOffset = 0;
        mainData.mechanicCorrection = 0;
        initRudPid(&mainData);
        timer1 = millis();
        tel = 0;
        stage = 1;
        ret = -1;
        break;
    case 1:
        if (timer1 + 1000 < millis())
        {
            timer1 = millis();
            double error = smallestAngle(tmp_mhdg, mainData.gpsDir);
            if (abs(error) < 5.0)
            {
                stage = 2;
                ret = -1;
                break;
            }
            tel++;
            if (tel % 2)
            {
                tmp_mhdg = mainData.gpsDir;
                ret = -1;
                break;
            }
            if (error < 0)
            {
                mainData.mechanicCorrection += 1;
            }
            else
            {
                mainData.mechanicCorrection -= 1;
            }
            mainData.speedBb = (int)(mainData.speed * (1 - tan(mainData.mechanicCorrection * M_PI / 180.0)));
            mainData.speedSb = (int)(mainData.speed * (1 + tan(mainData.mechanicCorrection * M_PI / 180.0)));
            mainData.speedBb = constrain(mainData.speedBb, -mainData.maxSpeed, mainData.maxSpeed);
            mainData.speedSb = constrain(mainData.speedSb, -mainData.maxSpeed, mainData.maxSpeed);
            escOut.speedbb = mainData.speedBb;
            escOut.speedsb = mainData.speedSb;
            xQueueSend(escspeed, (void *)&escOut, 10); 

            if (tel > 60) 
            {
                icmCompassOffsetLoad(&mainData, true);
                MechanicalCorrection(&mainData.mechanicCorrection, true);
                mainData.speedBb = 0;
                mainData.speedSb = 0;
                escOut.speedbb = mainData.speedBb;
                escOut.speedsb = mainData.speedSb;
                xQueueSend(escspeed, (void *)&escOut, 10);
                stage = 0;
                udpsend("Mechanical correction canceled!!!");
                ret = 0;
                break;
            }
        }
        break;
    case 2:
        if (timer1 + 1000 < millis())
        {
            timer1 = millis();
            tbuf[0][pointer] = (double)GetHeadingRaw();
            tbuf[1][pointer++] = (double)mainData.gpsDir;
            snprintf(bufff, sizeof(bufff), "<Calib><mag heading: %0.1f>< gps heading: %0.1f>", tbuf[0][pointer-1], tbuf[1][pointer-1]);
            udpsend(bufff);

            if (pointer >= ABUF)
            {
                mainData.speedBb = 0;
                mainData.speedSb = 0;
                subStatus = IDLE;
                double msum_x = 0.0, msum_y = 0.0, mavg_dir;
                double gsum_x = 0.0, gsum_y = 0.0, gavg_dir;
                for (int i = 0; i < ABUF; i++)
                {
                    msum_x += cos(tbuf[0][i] * M_PI / 180.0);
                    msum_y += sin(tbuf[0][i] * M_PI / 180.0);
                    gsum_x += cos(tbuf[1][i] * M_PI / 180.0);
                    gsum_y += sin(tbuf[1][i] * M_PI / 180.0);
                }
                mavg_dir = atan2(msum_y, msum_x) * 180.0 / M_PI;
                gavg_dir = atan2(gsum_y, gsum_x) * 180.0 / M_PI;
                if (mavg_dir < 0) mavg_dir += 360.0;
                if (gavg_dir < 0) gavg_dir += 360.0;

                double correction = gavg_dir - mavg_dir;
                if (correction < 0) correction += 360.0;
                if (correction > 360) correction -= 360.0;
                if (correction > 180) correction -= 360.0;

                mainData.icmCompassOffset = correction;
                icmCompassOffsetLoad(&mainData, false);
                MechanicalCorrection(&mainData.mechanicCorrection, false);

                snprintf(bufff, sizeof(bufff), "<Calib stored><magCorr>%0.1f><mechCorr><%0.1f>", mainData.icmCompassOffset, mainData.mechanicCorrection);
                udpsend(bufff);
                ret = 1;
                stage = 0;

                escOut.speedbb = 0;
                escOut.speedsb = 0;
                xQueueSend(escspeed, (void *)&escOut, 10);
            }
        }
        break;
    default:
        stage = 0;
        break;
    }
    return ret;
}

void initcompassQueue(void)
{
    compass = xQueueCreate(1, sizeof(double));
    compassIn = xQueueCreate(10, sizeof(int));
}

void CompassTask(void *arg)
{
    int cmd = 0;
    int calib_mode = 0;
    int corr = 0;

    static float avgX_samples[AVERIDGE_COUNT] = {0}, avgY_samples[AVERIDGE_COUNT] = {0}, avgZ_samples[AVERIDGE_COUNT] = {0};
    static int avg_idx = 0;
    static int avg_count = 0;

    static float sin_samples[AVERIDGE_COUNT] = {0};
    static float cos_samples[AVERIDGE_COUNT] = {0};
    static int h_avg_idx = 0;
    static int h_avg_count = 0;

    while (1)
    {
        if (xQueueReceive(compassIn, &cmd, 0) == pdTRUE) {
            if (cmd == CALIBRATE_MAGNETIC_COMPASS) {
                CalibrateCompass();
            } else if (cmd == INFIELD_CALIBRATE) {
                calib_mode = 1;
            }
        }

        if (calib_mode == 1) {
            if (linMagCalib(&corr) != -1) {
                calib_mode = 0; 
            }
        }

        float rx, ry, rz;
        if (read_mag(rx, ry, rz)) {
            last_raw_x = rx; last_raw_y = ry; last_raw_z = rz;

            avgX_samples[avg_idx] = rx;
            avgY_samples[avg_idx] = ry;
            avgZ_samples[avg_idx] = rz;
            avg_idx = (avg_idx + 1) % AVERIDGE_COUNT;
            if (avg_count < AVERIDGE_COUNT) avg_count++;

            float sumX = 0, sumY = 0, sumZ = 0;
            for (int i = 0; i < avg_count; i++) {
                sumX += avgX_samples[i];
                sumY += avgY_samples[i];
                sumZ += avgZ_samples[i];
            }
            float mx = sumX / avg_count;
            float my = sumY / avg_count;
            float mz = sumZ / avg_count;

            float cal_x = mx - mainData.icmMagHard[0];
            float cal_y = my - mainData.icmMagHard[1];
            float cal_z = mz - mainData.icmMagHard[2];

            float final_x = mainData.icmMagSoft[0][0] * cal_x + mainData.icmMagSoft[0][1] * cal_y + mainData.icmMagSoft[0][2] * cal_z;
            float final_y = mainData.icmMagSoft[1][0] * cal_x + mainData.icmMagSoft[1][1] * cal_y + mainData.icmMagSoft[1][2] * cal_z;
            float final_z = mainData.icmMagSoft[2][0] * cal_x + mainData.icmMagSoft[2][1] * cal_y + mainData.icmMagSoft[2][2] * cal_z;

            // Proven axis mapping: atan2f(final_z, final_y)
            float current_h_rad = atan2f(final_z, final_y);
            
            sin_samples[h_avg_idx] = sinf(current_h_rad);
            cos_samples[h_avg_idx] = cosf(current_h_rad);
            h_avg_idx = (h_avg_idx + 1) % AVERIDGE_COUNT;
            if (h_avg_count < AVERIDGE_COUNT) h_avg_count++;

            float sum_sin = 0, sum_cos = 0;
            for (int i = 0; i < h_avg_count; i++) {
                sum_sin += sin_samples[i];
                sum_cos += cos_samples[i];
            }

            float heading_val = atan2f(sum_sin, sum_cos) * 180.0f / M_PI;
            if (heading_val < 0) heading_val += 360.0f;
            
            heading_val += mainData.icmCompassOffset;
            heading_val = fmod(heading_val, 360.0f);
            if (heading_val < 0) heading_val += 360.0f;

            global_icmHdg = heading_val;
        }

        float rawHdg = global_icmHdg;
        double activeHdg = (double)CompassAverage(rawHdg);
        mainData.dirMag = (float)activeHdg;
        xQueueOverwrite(compass, (void *)&activeHdg);

        static uint32_t lastPrint = 0;
        if (millis() - lastPrint > 2000) {
            lastPrint = millis();
            Serial.printf("Compass (Manual): Raw=%0.1f, Avg=%0.1f, OK=%d\n", rawHdg, activeHdg, icm_ready);
        }

        static uint32_t lastUdp = 0;
        if (millis() - lastUdp > 200) { 
            lastUdp = millis();
            char dbg[250];
            snprintf(dbg, sizeof(dbg),
                "{\"lsm_hdg\":%.1f,\"icm_hdg\":%.1f,\"off\":%.1f,\"i_m\":[%.1f,%.1f,%.1f]}",
                0.0f, global_icmHdg, mainData.icmCompassOffset,
                last_raw_x, last_raw_y, last_raw_z
            );
            udpsend(dbg);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
