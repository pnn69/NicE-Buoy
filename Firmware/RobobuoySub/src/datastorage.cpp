// https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/src/Preferences.cpp
#include <Preferences.h>
#include <main.h>
Preferences storage;

void initMemory(void)
{
    // Open Preferences with my-app namespace. Each application module, library, etc
    // has to use a namespace name to prevent key name collisions. We will open storage in
    // RW-mode (second parameter has to be false).
    // Note: Namespace name is limited to 15 chars.
    storage.begin("NicE_Buoy_Data", false);
    char id = storage.getChar("NicE_BuoyID", 0);
    // id = 0;
    if (id == 0)
    {
        Serial.printf("Configuring Non-volatile memory now!\n\r");
        storage.putChar("NicE_BuoyID", 100);
        Serial.printf("Storing data Bouy ID to 100\r\n");
        storage.putDouble("Doclat", 52.29308075283747);
        storage.putDouble("Doclon", 4.932570409845357);
        // tglatitude = 52.29308075283747, tglongitude = 4.932570409845357; // steiger wsvop
        Serial.printf("Storing data Target position \r\nWSVOP landing stage tglatitude = 52.29308075283747, tglongitude = 4.932570409845357 ");
        Serial.printf("Buoy Memory configured\r\n");
        delay(1000);
    }
    else
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
void memDockPos(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->tgLat = storage.getDouble("Docklat", 0);
        buoy->tgLng = storage.getDouble("Docklon", 0);
        // Serial.printf("Get Doc pos form memory  %.8lf %.8lf\r\n", *lat, *lon);
    }
    else
    {
        Serial.printf("Store Doc pos in memory  %.8lf %.8lf\r\n", buoy->tgLat, buoy->tgLng);
        storage.putDouble("Docklat", buoy->tgLat);
        storage.putDouble("Docklon", buoy->tgLng);
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

void Declination(double *declination, bool get)
{
    startMem();
    if (get)
    {
        *declination = storage.getDouble("declination", 1.5);
    }
    else
    {
        storage.putDouble("declination", *declination);
    }
    stopMem();
}
void MechanicalCorrection(double *delta, bool get)
{
    startMem();
    if (get)
    {
        *delta = storage.getDouble("Mecanic", 0);
    }
    else
    {
        storage.putDouble("Mecanic", *delta);
    }
    stopMem();
}

void computeParameters(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->minOfsetDist = storage.getInt("minOfsetDist", 1);
        buoy->maxOfsetDist = storage.getInt("maxOfsetDist", 8);
    }
    else
    {
        storage.putInt("minOfsetDist", buoy->minOfsetDist);
        storage.putInt("maxOfsetDist", buoy->maxOfsetDist);
    }
    stopMem();
}

void speedMaxMin(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->minSpeed = storage.getInt("minSpeed", -73);
        buoy->maxSpeed = storage.getInt("maxSpeed", 73);
    }
    else
    {
        storage.putInt("minSpeed", buoy->minSpeed);
        storage.putInt("maxSpeed", buoy->maxSpeed);
    }
    stopMem();
}

void pidSpeedParameters(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->Kps = storage.getDouble("Kps", 20);
        buoy->Kis = storage.getDouble("Kis", 0.4);
        buoy->Kds = storage.getDouble("Kds", 0);
    }
    else
    {
        storage.putDouble("Kps", buoy->Kps);
        storage.putDouble("Kis", buoy->Kis);
        storage.putDouble("Kds", buoy->Kds);
    }
    stopMem();
}
void pidRudderParameters(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->Kpr = storage.getDouble("Kpr", 0.5);
        buoy->Kir = storage.getDouble("Kir", 0.02);
        buoy->Kdr = storage.getDouble("Kdr", 0);
    }
    else
    {
        storage.putDouble("Kpr", buoy->Kpr);
        storage.putDouble("Kir", buoy->Kir);
        storage.putDouble("Kdr", buoy->Kdr);
    }
    stopMem();
}

void apParameters(String *ap, String *ww, bool get)
{
    startMem();
    if (get)
    {
        *ap = storage.getString("ap", "");
        *ww = storage.getString("ww", "");
    }
    else
    {
        storage.putString("ap", *ap);
        storage.putString("ww", *ww);
    }
    stopMem();
}
