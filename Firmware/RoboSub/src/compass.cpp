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
#include "esc.h"
#include "subwifi.h"
#include "pidrudspeed.h"
#include "sercom.h"
#include <AsyncUDP.h>

#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>

#define NUM_DIRECTIONS 30
#define NUM_POSITIONS 50

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

template <typename T>
float heading(vector_t<T> from)
{
    if (!lsm_ready) return 0.0f;
    sensors_event_t event;
    if (!mag.getEvent(&event)) return 0.0f;
    m_lsm_last = event;
    vector_t<float> temp_m = {event.magnetic.x, event.magnetic.y, event.magnetic.z};

    if (!accel.getEvent(&event)) return 0.0f;
    vector_t<float> a = {event.acceleration.x, event.acceleration.y, event.acceleration.z};

    // Important: subtract average of min and max from magnetometer calibration
    temp_m.x -= (m_min.x + m_max.x) / 2;
    temp_m.y -= (m_min.y + m_max.y) / 2;
    temp_m.z -= (m_min.z + m_max.z) / 2;

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
    CompassCallibrationFactorsFloat(&max_mag[0], &max_mag[1], &max_mag[2], &min_mag[0], &min_mag[1], &min_mag[2], true); //  get callibration data
    
    // Sanity check for calibration data
    if (abs(max_mag[0] - min_mag[0]) < 0.01) {
        Serial.println("Compass: No valid calibration found, using defaults");
        m_min = (vector_t<float>){-50, -50, -50};
        m_max = (vector_t<float>){50, 50, 50};
    } else {
        m_min = (vector_t<float>){min_mag[0], min_mag[1], min_mag[2]};
        m_max = (vector_t<float>){max_mag[0], max_mag[1], max_mag[2]};
    }

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
    MechanicalCorrection(&mainData.mechanicCorrection, true);
    
    Serial.printf("Compass Initialized. Offset: %0.1f, Mech: %0.1f\n", mainData.compassOffset, mainData.mechanicCorrection);
    return true;
}

bool CalibrateCompass(void)
{
    static unsigned long calstamp;
    sensors_event_t event;
    uint32_t lokcnt = 0;
    Serial.println("Callibrating compass now!!!");
    calstamp = millis();
    float min_mag[3], max_mag[3];
    min_mag[0] = min_mag[1] = min_mag[2] = 1000000.0f;
    max_mag[0] = max_mag[1] = max_mag[2] = -1000000.0f;

    while (millis() - calstamp <= 1000 * 60) // 1 minute callibrating
    {
        mag.getEvent(&event);
        min_mag[0] = fmin(min_mag[0], event.magnetic.x);
        max_mag[0] = fmax(max_mag[0], event.magnetic.x);
        min_mag[1] = fmin(min_mag[1], event.magnetic.y);
        max_mag[1] = fmax(max_mag[1], event.magnetic.y);
        min_mag[2] = fmin(min_mag[2], event.magnetic.z);
        max_mag[2] = fmax(max_mag[2], event.magnetic.z);

        if (lokcnt++ > 250)
        {
            lokcnt = 0;
            Serial.printf("Calllibration factors Compass: MaxXYZ: {%f, %f, %f}; MinXYZ {%f, %f, %f};\r\n", max_mag[0], max_mag[1], max_mag[2], min_mag[0], min_mag[1], min_mag[2]);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    CompassCallibrationFactorsFloat(&max_mag[0], &max_mag[1], &max_mag[2], &min_mag[0], &min_mag[1], &min_mag[2], false); //  store callibration data
    m_min = (vector_t<float>){min_mag[0], min_mag[1], min_mag[2]};
    m_max = (vector_t<float>){max_mag[0], max_mag[1], max_mag[2]};
    Serial.printf("New callibration stored!!!\n\r");
    Serial.printf("Calllibration factors Compass: MaxXYZ: {%f, %f, %f}; MinXYZ {%f, %f, %f};\r\n", max_mag[0], max_mag[1], max_mag[2], min_mag[0], min_mag[1], min_mag[2]);

    return true;
}

float heading_icm(vector_t<int> from)
{
    if (!icm_ready) return -1.0f;
    sensors_event_t accel_event, mag_event, gyro_event, temp_event;
    icm.getEvent(&accel_event, &gyro_event, &temp_event, &mag_event);
    m_icm_last = mag_event;

    vector_t<float> temp_m = {mag_event.magnetic.x, mag_event.magnetic.y, mag_event.magnetic.z};
    vector_t<float> a = {accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z};

    // Note: yesterday's code had specific ICM calibration handling. 
    // For now, we will use the same hard/soft iron logic if available, 
    // but since we are refactoring, we'll keep it simple: 
    // subtract defaults or shared m_min/m_max if they were meant for both.
    temp_m.x -= (m_min.x + m_max.x) / 2;
    temp_m.y -= (m_min.y + m_max.y) / 2;
    temp_m.z -= (m_min.z + m_max.z) / 2;

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
    if (icm_ready) {
        mHeding = heading_icm((vector_t<int>){0, 1, 0});
    } else {
        mHeding = heading((vector_t<int>){0, 1, 0});
    }
    mHeding = mHeding + mainData.compassOffset;
    if (mHeding < 0)
    {
        mHeding = mHeding + 360.0;
    }
    else if (mHeding > 360)
    {
        mHeding = mHeding - 360.0;
    }
    return mHeding;
}

float GetHeadingRaw(void)
{
    float t = 0;
    if (icm_ready) {
        t = heading_icm((vector_t<int>){0, 1, 0});
    } else {
        t = heading((vector_t<int>){0, 1, 0});
    }
    if (t > 360)
    {
        t -= 360;
    }
    else if (t < 0)
    {
        t += 360;
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
        if (icm_ready) {
            global_icmHdg = heading_icm((vector_t<int>){0, 1, 0});
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
                "{\"debug\":\"compass\",\"lsm_hdg\":%.1f,\"icm_hdg\":%.1f,\"lsm\":[%.1f,%.1f,%.1f],\"icm\":[%.1f,%.1f,%.1f]}",
                global_lsmHdg, global_icmHdg,
                m_lsm_last.magnetic.x, m_lsm_last.magnetic.y, m_lsm_last.magnetic.z,
                m_icm_last.magnetic.x, m_icm_last.magnetic.y, m_icm_last.magnetic.z
            );
            udpsend(dbg);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
