#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_LSM303_Accel.h>
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
#include "compass.h"
#include "main.h"
#include "datastorage.h"
#include "pidrudspeed.h"
#include "sercom.h"
#include "subwifi.h"
#include "mag_calib.h"

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>

#define NUM_DIRECTIONS 30
#define NUM_POSITIONS 50

extern Preferences storage;
QueueHandle_t compass;
QueueHandle_t compassIn;

Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

// New ICM20948 sensor
Adafruit_ICM20948 icm;
Adafruit_Sensor *icm_mag = NULL;
Adafruit_Sensor *icm_accel = NULL;
bool icm_ready = false;

extern RoboStruct mainData;
extern Message escOut;
extern int subStatus;
extern AsyncUDP udp;

template <typename T>
struct vector_t
{
    T x, y, z;
};

// Stores min and max magnetometer values from calibration
vector_t<float> m_max;
vector_t<float> m_min;
vector_t<float> icm_max = {50, 50, 50};
vector_t<float> icm_min = {-50, -50, -50};

template <typename Ta, typename Tb, typename To>
void vector_cross(const vector_t<Ta> *a, const vector_t<Tb> *b, vector_t<To> *out)
{
    out->x = (a->y * b->z) - (a->z * b->y);
    out->y = (a->z * b->x) - (a->x * b->z);
    out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb>
float vector_dot(const vector_t<Ta> *a, const vector_t<Tb> *b)
{
    return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

void vector_normalize(vector_t<float> *a)
{
    float mag_val = sqrt(vector_dot(a, a));
    if (mag_val > 0) {
        a->x /= mag_val;
        a->y /= mag_val;
        a->z /= mag_val;
    }
}

void udpsend(const char* msg) {
    if (udp) { // Only broadcast if the UDP service is initialized
        udp.broadcast(msg);
    }
}

bool lsm_ready = false;
float global_lsmHdg = 0;
float global_icmHdg = 0;
sensors_event_t m_lsm_last, m_icm_last;
sensors_event_t m_lsm_a_last, m_icm_a_last;

template <typename T>
float heading(vector_t<T> from)
{
    if (!lsm_ready) return 0.0f;
    sensors_event_t event;
    if (!mag.getEvent(&event)) return 0.0f;
    m_lsm_last = event;
    vector_t<float> temp_m = {event.magnetic.x, event.magnetic.y, event.magnetic.z};

    if (!accel.getEvent(&event)) return 0.0f;
    m_lsm_a_last = event;
    vector_t<float> a = {event.acceleration.x, event.acceleration.y, event.acceleration.z};

    // Apply Soft and Hard Iron calibration
    float raw_x = temp_m.x - mainData.magHard[0];
    float raw_y = temp_m.y - mainData.magHard[1];
    float raw_z = temp_m.z - mainData.magHard[2];

    temp_m.x = mainData.magSoft[0][0] * raw_x + mainData.magSoft[0][1] * raw_y + mainData.magSoft[0][2] * raw_z;
    temp_m.y = mainData.magSoft[1][0] * raw_x + mainData.magSoft[1][1] * raw_y + mainData.magSoft[1][2] * raw_z;
    temp_m.z = mainData.magSoft[2][0] * raw_x + mainData.magSoft[2][1] * raw_y + mainData.magSoft[2][2] * raw_z;

    // Compute east and north vectors
    vector_t<float> east;
    vector_t<float> north;
    vector_cross(&temp_m, &a, &east);
    vector_normalize(&east);
    vector_cross(&a, &east, &north);
    vector_normalize(&north);

    // compute heading
    float n_dot_f = vector_dot(&north, &from);
    float e_dot_f = vector_dot(&east, &from);

    if (n_dot_f == 0 && e_dot_f == 0) return 0.0f;

    float heading_val = atan2(n_dot_f, e_dot_f) * 180 / M_PI;
    if (isnan(heading_val)) return 0.0f;

    if (heading_val < 0)
    {
        heading_val += 360;
    }
    return heading_val;
}

bool InitCompass(void)
{
    Serial.println("Initializing I2C bus (21, 22)...");
    Wire.begin(21, 22);
    Wire.setClock(400000); // Set I2C to 400kHz
    vTaskDelay(pdMS_TO_TICKS(100)); // Give sensors time to power up

    float min_mag[3], max_mag[3];
    CompassCallibrationFactorsFloat(&max_mag[0], &max_mag[1], &max_mag[2], &min_mag[0], &min_mag[1], &min_mag[2], true); // Legacy float
    
    // Load full compass calibration (Soft + Hard iron + Offset) into mainData
    CompassCallibrationFactors(&mainData, true);

    // Sanity check for calibration data
    if (abs(max_mag[0] - min_mag[0]) < 0.01) {
        Serial.println("Compass: No valid calibration found, using defaults");
        m_min = (vector_t<float>){-50, -50, -50};
        m_max = (vector_t<float>){50, 50, 50};
    } else {
        m_min = (vector_t<float>){min_mag[0], min_mag[1], min_mag[2]};
        m_max = (vector_t<float>){max_mag[0], max_mag[1], max_mag[2]};
    }

    // Load ICM calibration from NVS
    storage.begin("NicE_Buoy_Data", false);
    icm_max.x = storage.getFloat("IcmMaxX", 50.0);
    icm_max.y = storage.getFloat("IcmMaxY", 50.0);
    icm_max.z = storage.getFloat("IcmMaxZ", 50.0);
    icm_min.x = storage.getFloat("IcmMinX", -50.0);
    icm_min.y = storage.getFloat("IcmMinY", -50.0);
    icm_min.z = storage.getFloat("IcmMinZ", -50.0);
    storage.end();

    bool mag_ok = mag.begin();
    if (!mag_ok)
    {
        Serial.println("Unable to initialize LSM303/LIS2MDL magnetometer");
    }

    bool acc_ok = accel.begin();
    if (!acc_ok)
    {
        Serial.println("Unable to initialize LSM303 accelerometer");
    }

    lsm_ready = (mag_ok && acc_ok);

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

    CompassOffsetCorrection(&mainData.compassOffset, true);
    icmCompassOffsetLoad(&mainData, true);
    MechanicalCorrection(&mainData.mechanicCorrection, true);

    Serial.printf("Compass Initialized. LSM Offset: %0.1f, ICM Offset: %0.1f, Mech: %0.1f\n", mainData.compassOffset, mainData.icmCompassOffset, mainData.mechanicCorrection);
    return true;
}

bool CalibrateCompass(void)
{
    static unsigned long calstamp;
    uint32_t lokcnt = 0;
    Serial.println("Callibrating compass now!!!");
    
    // Feedback: Fast blinking purple LED and initial beep
    LedData calLedStatus;
    calLedStatus.color = CRGB::Purple;
    calLedStatus.blink = BLINK_FAST;
    xQueueSend(ledStatus, (void *)&calLedStatus, 0);
    vTaskDelay(pdMS_TO_TICKS(500)); // Delay to ensure previous beep finished
    beep(1, buzzer);

    calstamp = millis();
    float min_mag[3], max_mag[3];
    float min_icm[3], max_icm[3];
    min_mag[0] = min_mag[1] = min_mag[2] = 1000000.0f;
    max_mag[0] = max_mag[1] = max_mag[2] = -1000000.0f;
    min_icm[0] = min_icm[1] = min_icm[2] = 1000000.0f;
    max_icm[0] = max_icm[1] = max_icm[2] = -1000000.0f;

    MagCalibrator cal_lsm;
    MagCalibrator cal_icm;

    while (millis() - calstamp <= 1000 * 60) // 1 minute callibrating
    {
        sensors_event_t event_lsm, event_icm;
        mag.getEvent(&event_lsm);
        icm.getMagnetometerSensor()->getEvent(&event_icm);

        // Map ICM magnetometer to LSM frame: X = Y, Y = -X, Z = Z
        float temp_mag_x = event_icm.magnetic.x;
        event_icm.magnetic.x = event_icm.magnetic.y;
        event_icm.magnetic.y = -temp_mag_x;
        event_icm.magnetic.z = event_icm.magnetic.z;

        // Add points to the calibrators
        cal_lsm.addPoint(event_lsm.magnetic.x, event_lsm.magnetic.y, event_lsm.magnetic.z);
        cal_icm.addPoint(event_icm.magnetic.x, event_icm.magnetic.y, event_icm.magnetic.z);

        min_mag[0] = fmin(min_mag[0], event_lsm.magnetic.x);
        max_mag[0] = fmax(max_mag[0], event_lsm.magnetic.x);
        min_mag[1] = fmin(min_mag[1], event_lsm.magnetic.y);
        max_mag[1] = fmax(max_mag[1], event_lsm.magnetic.y);
        min_mag[2] = fmin(min_mag[2], event_lsm.magnetic.z);
        max_mag[2] = fmax(max_mag[2], event_lsm.magnetic.z);
        
        min_icm[0] = fmin(min_icm[0], event_icm.magnetic.x);
        max_icm[0] = fmax(max_icm[0], event_icm.magnetic.x);
        min_icm[1] = fmin(min_icm[1], event_icm.magnetic.y);
        max_icm[1] = fmax(max_icm[1], event_icm.magnetic.y);
        min_icm[2] = fmin(min_icm[2], event_icm.magnetic.z);
        max_icm[2] = fmax(max_icm[2], event_icm.magnetic.z);

        if (lokcnt++ > 250)
        {
            lokcnt = 0;
            Serial.printf("Calibrating: LSM points=%d | ICM points=%d\r\n", cal_lsm.getNumPoints(), cal_icm.getNumPoints());
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    float hard[3];
    float soft[3][3];

    if (cal_lsm.calculateCalibration(hard, soft)) {
        for(int i=0; i<3; i++) mainData.magHard[i] = hard[i];
        for(int i=0; i<3; i++) for(int j=0; j<3; j++) mainData.magSoft[i][j] = soft[i][j];
        hardIron(&mainData, false);
        softIron(&mainData, false);
        Serial.println("LSM Ellipsoid Fit successful!");
    } else {
        Serial.println("LSM Ellipsoid Fit failed. Falling back to Hard Iron Min/Max.");
        mainData.magHard[0] = (max_mag[0] + min_mag[0]) / 2;
        mainData.magHard[1] = (max_mag[1] + min_mag[1]) / 2;
        mainData.magHard[2] = (max_mag[2] + min_mag[2]) / 2;
        for(int i=0; i<3; i++) for(int j=0; j<3; j++) mainData.magSoft[i][j] = (i==j) ? 1.0 : 0.0;
        hardIron(&mainData, false);
        softIron(&mainData, false);
    }

    if (cal_icm.calculateCalibration(hard, soft)) {
        for(int i=0; i<3; i++) mainData.icmMagHard[i] = hard[i];
        for(int i=0; i<3; i++) for(int j=0; j<3; j++) mainData.icmMagSoft[i][j] = soft[i][j];
        icmHardIron(&mainData, false);
        icmSoftIron(&mainData, false);
        Serial.println("ICM Ellipsoid Fit successful!");
    } else {
        Serial.println("ICM Ellipsoid Fit failed. Falling back to Hard Iron Min/Max.");
        mainData.icmMagHard[0] = (max_icm[0] + min_icm[0]) / 2;
        mainData.icmMagHard[1] = (max_icm[1] + min_icm[1]) / 2;
        mainData.icmMagHard[2] = (max_icm[2] + min_icm[2]) / 2;
        for(int i=0; i<3; i++) for(int j=0; j<3; j++) mainData.icmMagSoft[i][j] = (i==j) ? 1.0 : 0.0;
        icmHardIron(&mainData, false);
        icmSoftIron(&mainData, false);
    }

    CompassCallibrationFactorsFloat(&max_mag[0], &max_mag[1], &max_mag[2], &min_mag[0], &min_mag[1], &min_mag[2], false); //  store legacy data
    m_min = (vector_t<float>){min_mag[0], min_mag[1], min_mag[2]};
    m_max = (vector_t<float>){max_mag[0], max_mag[1], max_mag[2]};
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
    
    Serial.printf("New callibration stored!!!\n\r");
    
    // Restore LED to original state (Yellow Idle)
    LedData idleLedStatus;
    idleLedStatus.color = CRGB::Yellow;
    idleLedStatus.blink = BLINK_OFF;
    xQueueSend(ledStatus, (void *)&idleLedStatus, 0);
    beep(5, buzzer); // Final confirmation melody

    return true;
}

float heading_icm(vector_t<int> from)
{
    if (!icm_ready) return -1.0f;
    sensors_event_t accel_event, mag_event;
    icm_accel->getEvent(&accel_event);
    icm_mag->getEvent(&mag_event);
    
    // Map ICM magnetometer to LSM frame: X = Y, Y = -X, Z = Z (internal AK09916 is NOT inverted on this board)
    float temp_mag_x = mag_event.magnetic.x;
    mag_event.magnetic.x = mag_event.magnetic.y;
    mag_event.magnetic.y = -temp_mag_x;
    mag_event.magnetic.z = mag_event.magnetic.z;
    m_icm_last = mag_event;

    // Map ICM accelerometer to LSM frame: X = Y, Y = -X, Z = Z (physically rotated 90 deg)
    float temp_accel_x = accel_event.acceleration.x;
    accel_event.acceleration.x = accel_event.acceleration.y;
    accel_event.acceleration.y = -temp_accel_x;
    // Z stays the same
    m_icm_a_last = accel_event;

    vector_t<float> temp_m = {mag_event.magnetic.x, mag_event.magnetic.y, mag_event.magnetic.z};
    vector_t<float> a = {accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z};

    // Apply specific ICM calibration bounds
    float raw_x = temp_m.x - mainData.icmMagHard[0];
    float raw_y = temp_m.y - mainData.icmMagHard[1];
    float raw_z = temp_m.z - mainData.icmMagHard[2];

    temp_m.x = mainData.icmMagSoft[0][0] * raw_x + mainData.icmMagSoft[0][1] * raw_y + mainData.icmMagSoft[0][2] * raw_z;
    temp_m.y = mainData.icmMagSoft[1][0] * raw_x + mainData.icmMagSoft[1][1] * raw_y + mainData.icmMagSoft[1][2] * raw_z;
    temp_m.z = mainData.icmMagSoft[2][0] * raw_x + mainData.icmMagSoft[2][1] * raw_y + mainData.icmMagSoft[2][2] * raw_z;

    vector_t<float> east;
    vector_t<float> north;
    vector_cross(&temp_m, &a, &east);
    vector_normalize(&east);
    vector_cross(&a, &east, &north);
    vector_normalize(&north);

    vector_t<float> f = {(float)from.x, (float)from.y, (float)from.z};
    float n_dot_f = vector_dot(&north, &f);
    float e_dot_f = vector_dot(&east, &f);

    if (n_dot_f == 0 && e_dot_f == 0) return 0.0f;

    float heading_val = atan2(n_dot_f, e_dot_f) * 180 / M_PI;
    if (isnan(heading_val)) return 0.0f;

    if (heading_val < 0) heading_val += 360;
    return heading_val;
}

float GetHeading(void)
{
    float mHeding = 0;
    if (lsm_ready) {
        mHeding = heading((vector_t<int>){0, 1, 0});
        mHeding = mHeding + mainData.compassOffset;
    } else {
        mHeding = heading_icm((vector_t<int>){0, 1, 0});
        mHeding = mHeding + mainData.icmCompassOffset;
    }
    
    if (!isnan(mHeding)) {
        mHeding = fmod(mHeding, 360.0);
        if (mHeding < 0) mHeding += 360.0;
    } else {
        mHeding = 0.0;
    }
    return mHeding;
}

float GetHeadingRaw(void)
{
    float t = 0;
    if (lsm_ready) {
        t = heading((vector_t<int>){0, 1, 0});
    } else {
        t = heading_icm((vector_t<int>){0, 1, 0});
    }
    
    if (!isnan(t)) {
        t = fmod(t, 360.0);
        if (t < 0) t += 360.0;
    } else {
        t = 0.0;
    }
    return t;
}

static int cbufpointer = 0;
static float directions[NUM_DIRECTIONS] = {0};
static bool cbufFull = false;

float CompassAverage(float in)
{
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
    // Convert the compass directions to Cartesian coordinates
    for (int i = 0; i < count; i++)
    {
        sum_x += cos(directions[i] * M_PI / 180.0);
        sum_y += sin(directions[i] * M_PI / 180.0);
    }

    if (abs(sum_x) < 0.0001 && abs(sum_y) < 0.0001) return in;

    // Calculate the average direction in degrees
    avg_dir = atan2(sum_y, sum_x) * 180.0 / M_PI;
    if (avg_dir < 0)
    {
        avg_dir += 360.0;
    }
    if (avg_dir >= 360)
    {
        avg_dir -= 360;
    }
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
        mainData.compassOffset = 0;
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
                /*take a sample every 2 sec*/
                tmp_mhdg = mainData.gpsDir;
                ret = -1;
                break;
            }
            /*adjust power distributon on thrusters*/
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
            /*Sanity check*/
            mainData.speedBb = constrain(mainData.speedBb, -mainData.maxSpeed, mainData.maxSpeed);
            mainData.speedSb = constrain(mainData.speedSb, -mainData.maxSpeed, mainData.maxSpeed);
            escOut.speedbb = mainData.speedBb;
            escOut.speedsb = mainData.speedSb;
            xQueueSend(escspeed, (void *)&escOut, 10); // update esc

            if (tel > 60) // time out no stable course
            {
                /*restore previous calibration*/
                CompassOffsetCorrection(&mainData.compassOffset, true);
                MechanicalCorrection(&mainData.mechanicCorrection, true);
                mainData.speedBb = 0;
                mainData.speedSb = 0;
                escOut.speedbb = mainData.speedBb;
                escOut.speedsb = mainData.speedSb;
                xQueueSend(escspeed, (void *)&escOut, 10); // update esc
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

                snprintf(bufff, sizeof(bufff), "<Calib result><mag heading>%0.1f><gps heading><%0.1f>", mavg_dir, gavg_dir);
                udpsend(bufff);

                double correction = gavg_dir - mavg_dir;
                if (correction < 0) correction += 360.0;
                if (correction > 360) correction -= 360.0;
                if (correction > 180) correction -= 360.0;

                mainData.compassOffset = correction;
                CompassOffsetCorrection(&mainData.compassOffset, false);
                MechanicalCorrection(&mainData.mechanicCorrection, false);

                snprintf(bufff, sizeof(bufff), "<Calib stored><magCorr>%0.1f><mechCorr><%0.1f>", mainData.compassOffset, mainData.mechanicCorrection);
                udpsend(bufff);
                ret = 1;
                stage = 0;

                // Stop motors
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
                calib_mode = 0; // finished
            }
        }

        global_lsmHdg = heading((vector_t<int>){0, 1, 0});
        if (lsm_ready) {
            global_lsmHdg += mainData.compassOffset;
            if (!isnan(global_lsmHdg)) {
                global_lsmHdg = fmod(global_lsmHdg, 360.0);
                if (global_lsmHdg < 0) global_lsmHdg += 360.0;
            } else {
                global_lsmHdg = 0.0;
            }
        }

        if (icm_ready) {
            global_icmHdg = heading_icm((vector_t<int>){0, 1, 0});
            global_icmHdg += mainData.icmCompassOffset;
            if (!isnan(global_icmHdg)) {
                global_icmHdg = fmod(global_icmHdg, 360.0);
                if (global_icmHdg < 0) global_icmHdg += 360.0;
            } else {
                global_icmHdg = 0.0;
            }
        }

        float rawHdg = GetHeading();
        double activeHdg = (double)CompassAverage(rawHdg);

        if (isnan(activeHdg)) {
            activeHdg = 0.0;
        }

        xQueueOverwrite(compass, (void *)&activeHdg);

        static uint32_t lastPrint = 0;
        if (millis() - lastPrint > 2000) {
            lastPrint = millis();
            Serial.printf("Compass: Raw=%0.1f, Avg=%0.1f, LSM=%.1f, ICM=%.1f\n", rawHdg, activeHdg, global_lsmHdg, global_icmHdg);
        }

        static uint32_t lastUdp = 0;
        if (millis() - lastUdp > 200) { // Broadcast at 5Hz
            lastUdp = millis();
            char dbg[250];
            snprintf(dbg, sizeof(dbg),
                "{\"lsm_hdg\":%.1f,\"icm_hdg\":%.1f,\"off\":%.1f,\"l_m\":[%.1f,%.1f,%.1f],\"i_m\":[%.1f,%.1f,%.1f],\"l_a\":[%.1f,%.1f,%.1f],\"i_a\":[%.1f,%.1f,%.1f]}",
                global_lsmHdg, global_icmHdg, mainData.compassOffset,
                m_lsm_last.magnetic.x, m_lsm_last.magnetic.y, m_lsm_last.magnetic.z,
                m_icm_last.magnetic.x, m_icm_last.magnetic.y, m_icm_last.magnetic.z,
                m_lsm_a_last.acceleration.x, m_lsm_a_last.acceleration.y, m_lsm_a_last.acceleration.z,
                m_icm_a_last.acceleration.x, m_icm_a_last.acceleration.y, m_icm_a_last.acceleration.z
            );
            udpsend(dbg);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
