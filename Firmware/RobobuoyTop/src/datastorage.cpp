// https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/src/Preferences.cpp
#include <Preferences.h>

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
        storage.putDouble("Tglat", 52.29308075283747);
        storage.putDouble("Tglon", 4.932570409845357);
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
        *lat = storage.getDouble("latDock", 0);
        *lon = storage.getDouble("lonDock", 0);
        // Serial.printf("Get Doc pos form memory  %.8lf %.8lf\r\n", *lat, *lon);
    }
    else
    {
        // Serial.printf("Store Doc pos in memory  %.8lf %.8lf\r\n", *lat, *lon);
        storage.putDouble("latDock", *lat);
        storage.putDouble("lonDock", *lon);
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
void memPidSpeedParameters(double *p, double *i, double *d, bool get)
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
