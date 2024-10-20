// https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/src/Preferences.cpp
#include <Preferences.h>
#include <RoboCalc.h>
Preferences storage;

void initMemory(void)
{
    // Open Preferences with my-app namespace. Each application module, library, etc
    // has to use a namespace name to prevent key name collisions. We will open storage in
    // RW-mode (second parameter has to be false).
    // Note: Namespace name is limited to 15 chars.
    storage.begin("NicE_Buoy_Data", false);
    char id = storage.getChar("NicE_BuoyID", 0);
    if (id == 0)
    {
        Serial.printf("Configuring Non-volatile memory now!\n\r");
        storage.putChar("NicE_BuoyID", 100);
        Serial.printf("Storing data Bouy ID to 100\r\n");
        // tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop
        storage.putDouble("Doclat", 52.29308075283747);
        storage.putDouble("Doclon", 4.932570409845357);
        Serial.printf("Storing data Target position \r\nWSVOP landing stage tglatitude = 52.29308075283747, tglongitude = 4.932570409845357 ");
        Serial.printf("Buoy Memory configured\r\n");
        delay(1000);
    }
    id = storage.getChar("NicE_BuoyID", 0);
    if (id != 0)
    {
        Serial.printf("Buoy Memory OK\r\n");
    }
}
/*
    ID of the buoy
*/
void startMem(void)
{
    storage.begin("NicE_Buoy_Data", false);
}
void stopMem(void)
{
    storage.end();
}

void memBuoyId(int8_t *id, bool get)
{
    startMem();
    if (get)
    {
        *id = (storage.getChar("NicE_BuoyID", 0));
    }
    else
    {
        storage.putChar("NicE_BuoyID", *id);
    }
    stopMem();
}

/*
    Dock position
*/
void memDockPos(double *lat, double *lon, bool get)
{
    startMem();
    if (get)
    {
        *lat = storage.getDouble("Docklat", 0);
        *lon = storage.getDouble("Docklon", 0);
        // Serial.printf("Get Doc pos form memory  %.8lf %.8lf\r\n", *lat, *lon);
    }
    else
    {
        // Serial.printf("Store Doc pos in memory  %.8lf %.8lf\r\n", *lat, *lon);
        storage.putDouble("Docklat", *lat);
        storage.putDouble("Docklon", *lon);
    }
    stopMem();
}

void memComputeParameters(int *minOfsetDist, int *maxOfsetDist, int *minSpeed, int *maxSpeed, bool get)
{
    startMem();
    if (get)
    {
        *minOfsetDist = (int)storage.getInt("minOfsetDist", 1);
        *maxOfsetDist = (int)storage.getInt("maxOfsetDist", 8);
        *minSpeed = (int)storage.getInt("minSpeed", 0);
        *maxSpeed = (int)storage.getInt("maxSpeed", 80);
    }
    else
    {
        storage.putInt("minOfsetDist", *minOfsetDist);
        storage.putInt("maxOfsetDist", *maxOfsetDist);
        storage.putInt("minSpeed", *minSpeed);
        storage.putInt("maxSpeed", *maxSpeed);
    }
    stopMem();
}

/*
    Defaut callibration factors determed earyer
    compass.m_min = (LSM303::vector<int16_t>){-535, -645, -382};
    compass.m_max = (LSM303::vector<int16_t>){+576, +466, +754};
*/
void CompassCallibrationFactorsFloat(float *MaxX, float *MaxY, float *MaxZ, float *MinX, float *MinY, float *MinZ, bool get)
{
    startMem();
    if (get)
    {
        *MaxX = storage.getFloat("MaxX", 576);
        *MaxY = storage.getFloat("MaxY", 466);
        *MaxZ = storage.getFloat("MaxZ", 754);
        *MinX = storage.getFloat("MinX", -535);
        *MinY = storage.getFloat("MinY", -645);
        *MinZ = storage.getFloat("MinZ", -382);
    }
    else
    {
        storage.putFloat("MaxX", *MaxX);
        storage.putFloat("MaxY", *MaxY);
        storage.putFloat("MaxZ", *MaxZ);
        storage.putFloat("MinX", *MinX);
        storage.putFloat("MinY", *MinY);
        storage.putFloat("MinZ", *MinZ);
    }
    stopMem();
}

void CompassOffsetCorrection(int *delta, bool get)
{
    startMem();
    if (get)
    {
        *delta = storage.getInt("Delta", 0);
    }
    else
    {
        storage.putInt("Delta", *delta);
    }
    stopMem();
}
void MechanicalCorrection(int *delta, bool get)
{
    startMem();
    if (get)
    {
        *delta = storage.getInt("Mecanic", 0);
    }
    else
    {
        storage.putInt("Mecanic", *delta);
    }
    stopMem();
}

void computeParameters(int *minOfsetDist, int *maxOfsetDist, int *minSpeed, int *maxSpeed, bool get)
{
    startMem();
    if (get)
    {
        *minOfsetDist = storage.getInt("minOfsetDist", 1);
        *maxOfsetDist = storage.getInt("maxOfsetDist", 8);
        *minSpeed = storage.getInt("minSpeed", 0);
        *maxSpeed = storage.getInt("maxSpeed", 80);
    }
    else
    {
        storage.putInt("minOfsetDist", *minOfsetDist);
        storage.putInt("maxOfsetDist", *maxOfsetDist);
        storage.putInt("minSpeed", *minSpeed);
        storage.putInt("maxSpeed", *maxSpeed);
    }
    stopMem();
}
void pidSpeedParameters(double *p, double *i, double *d, bool get)
{
    startMem();
    if (get)
    {
        *p = storage.getDouble("Psp", 20);
        *i = storage.getDouble("Isp", 0.4);
        *d = storage.getDouble("Dsp", 0);
    }
    else
    {
        storage.putDouble("Psp", *p);
        storage.putDouble("Isp", *i);
        storage.putDouble("Dsp", *d);
    }
    stopMem();
}
void pidRudderParameters(double *p, double *i, double *d, bool get)
{
    startMem();
    if (get)
    {
        *p = storage.getDouble("Prd", 0.5);
        *i = storage.getDouble("Ird", 0.02);
        *d = storage.getDouble("Drd", 0);
    }
    else
    {
        storage.putDouble("Prd", *p);
        storage.putDouble("Ird", *i);
        storage.putDouble("Drd", *d);
    }
    stopMem();
}

void apParameters(String *ap, String *ww, bool get)
{
    startMem();
    if (get)
    {
        *ap = storage.getString("ap", "PAIR_ME_");
        *ww = storage.getString("ww", "");
    }
    else
    {
        storage.putString("ap", *ap);
        storage.putString("ww", *ww);
    }
    stopMem();
}

void memStuct(pid &data, bool get)
{
    startMem();
    if (get)
    {
        storage.getBytes("structdata", &data, sizeof(data));
    }
    else
    {
        storage.putBytes("structdata", &data, sizeof(data));
    }
    stopMem();
}