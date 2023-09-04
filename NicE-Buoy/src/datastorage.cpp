// https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/src/Preferences.cpp
#include <Preferences.h>

Preferences storage;

void InitMemory(void)
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
        storage.putChar("NicE_BuoyID", 1);
        Serial.printf("Storing data Bouy ID to 1\r\n");
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
void StartMem(void)
{
    storage.begin("NicE_Buoy_Data", false);
}
void StopMem(void)
{
    storage.end();
}

void MemoryBuoyID(char *id, bool get)
{
    StartMem();
    if (get)
    {
        *id = (storage.getChar("NicE_BuoyID", 0));
        Serial.printf("Get BuoyID from memory  %d\r\n", *id);
    }
    else
    {

        storage.putChar("NicE_BuoyID", *id);
        Serial.printf("Stored BuoyID in memory  %d\r", *id);
    }
    StopMem();
}

/*
    Anchor position
*/
void MemoryAnchorPos(double *lat, double *lon, bool get)
{
    StartMem();
    if (get)
    {
        *lat = storage.getDouble("latAnchor", 0);
        *lon = storage.getDouble("lonAnchor", 0);
        Serial.printf("Anchor pos form memory  %.8lf %.8lf\r\n", *lat, *lon);
    }
    else
    {
        Serial.printf("Store anchor pos to memory  %.8lf %.8lf\r\n", *lat, *lon);
        storage.putDouble("latAnchor", *lat);
        storage.putDouble("lonAnchor", *lon);
    }
    StopMem();
}

/*
    Dock position
*/
void MemoryDockPos(double *lat, double *lon, bool get)
{
    StartMem();
    if (get)
    {
        *lat = storage.getDouble("latDock", 0);
        *lon = storage.getDouble("lonDock", 0);
        Serial.printf("Get Doc pos form memory  %.8lf %.8lf\r\n", *lat, *lon);
    }
    else
    {
        Serial.printf("Store Doc pos in memory  %.8lf %.8lf\r\n", *lat, *lon);
        storage.putDouble("latDock", *lat);
        storage.putDouble("lonDock", *lon);
    }
    StopMem();
}

void Bootcnt(int *bootcnt, bool add)
{
    StartMem();
    *bootcnt = storage.getInt("BootCnt", 0);
    Serial.printf("Bootcounter:%d\r\n", *bootcnt);
    if (add)
    {
        ++*bootcnt;
        storage.putInt("BootCnt", *bootcnt);
        Serial.printf("Storing bootcounter:%d\r\n", *bootcnt);
    }
    Serial.print("BootCnt:");
    Serial.println(*bootcnt);
    StopMem();
}

void LastStatus(byte *data, double *tglat, double *tglon, bool get)
{
    StartMem();
    if (get)
    {
        *data = storage.getChar("Status", 0);
        *tglat = storage.getDouble("Tglat", 0);
        *tglon = storage.getDouble("Tglon", 0);
        Serial.printf("Reading data\r\n");
    }
    else
    {
        storage.putChar("Status", *data);
        storage.putDouble("Tglat", *tglat);
        storage.putDouble("Tglon", *tglon);
        Serial.printf("Storing data\r\n");
    }
    Serial.printf("Status:%d TGLat:%.8lf TGLon:%.8lf\r\n", *data, *tglat, *tglon);
    StopMem();
}

/*
    Defaut callibration factors determed earyer
    compass.m_min = (LSM303::vector<int16_t>){-535, -645, -382};
    compass.m_max = (LSM303::vector<int16_t>){+576, +466, +754};
*/
void CompassCallibrationFactorsFloat(float *MaxX, float *MaxY, float *MaxZ, float *MinX, float *MinY, float *MinZ, bool get)
{
    StartMem();
    if (get)
    {
        *MaxX = (int16_t)storage.getFloat("MaxX", 576);
        *MaxY = (int16_t)storage.getFloat("MaxY", 466);
        *MaxZ = (int16_t)storage.getFloat("MaxZ", 754);
        *MinX = (int16_t)storage.getFloat("MinX", -535);
        *MinY = (int16_t)storage.getFloat("MinY", -645);
        *MinZ = (int16_t)storage.getFloat("MinZ", -382);
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
    StopMem();
}

void CompassOffsetCorrection(int *delta, bool get)
{
    StartMem();
    if (get)
    {
        *delta = storage.getInt("Delta", 0);
    }
    else
    {
        storage.putInt("Delta", *delta);
    }
    StopMem();
}

void CompassCallibrationFactors(int16_t *MaxX, int16_t *MaxY, int16_t *MaxZ, int16_t *MinX, int16_t *MinY, int16_t *MinZ, bool get)
{
    StartMem();
    if (get)
    {
        *MaxX = (int16_t)storage.getInt("MaxX", 576);
        *MaxY = (int16_t)storage.getInt("MaxY", 466);
        *MaxZ = (int16_t)storage.getInt("MaxZ", 754);
        *MinX = (int16_t)storage.getInt("MinX", -535);
        *MinY = (int16_t)storage.getInt("MinY", -645);
        *MinZ = (int16_t)storage.getInt("MinZ", -382);
    }
    else
    {
        storage.putInt("MaxX", *MaxX);
        storage.putInt("MaxY", *MaxY);
        storage.putInt("MaxZ", *MaxZ);
        storage.putInt("MinX", *MinX);
        storage.putInt("MinY", *MinY);
        storage.putInt("MinZ", *MinZ);
    }
    StopMem();
}

