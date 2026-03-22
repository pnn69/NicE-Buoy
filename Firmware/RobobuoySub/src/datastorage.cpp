// https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/src/Preferences.cpp
#include <Preferences.h>
#include <main.h>
#include <RoboCompute.h>
#include "subwifi.h"
Preferences storage;

void initMemory(void)
{
    storage.begin("NicE_Buoy_Data", false);

    // 1. Check for legacy typo migration
    if (storage.isKey("Doclat") && !storage.isKey("Docklat")) {
        double lat = storage.getDouble("Doclat");
        double lon = storage.getDouble("Doclon");
        storage.putDouble("Docklat", lat);
        storage.putDouble("Docklon", lon);
        storage.remove("Doclat");
        storage.remove("Doclon");
        Serial.println("# Migrated legacy Dock keys.");
    }

    // 2. Check if this is a fresh processor
    unsigned long id = espMac();
    uint64_t stored_id = storage.getULong64("NicE_BuoyID", 0);

    if (id != stored_id)
    {
        Serial.printf("# Configuring Fresh Processor Memory...\n\r");
        storage.putULong64("NicE_BuoyID", id);
        
        // Factory Default Dock: WSVOP landing stage
        storage.putDouble("Docklat", 52.29308075283747);
        storage.putDouble("Docklon", 4.932570409845357);
        storage.putDouble("declination", 2.6666666666); // Amsterdam default

        Serial.printf("# Buoy Memory initialized for MAC: %08X\r\n", id);
        delay(500);
    }
    else
    {
        Serial.printf("# Buoy Memory OK (MAC: %08X)\r\n", id);
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

void Declination(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->declination = storage.getDouble("declination", 2.66666666); // amsterdam 2025
    }
    else
    {
        storage.putDouble("declination", buoy->declination);
    }
    stopMem();
}
void CompasOffset(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->compassOffset = storage.getDouble("CompasOffset", 0);
    }
    else
    {
        storage.putDouble("CompasOffset", buoy->compassOffset);
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

void hardIron(RoboStruct *buoy, bool get)
{
    startMem();
    String key = "";
    for (int i = 0; i < 3; i++)
    {
        String key = "mH" + String(buoy->mac, HEX) + String(i);
        if (get)
        {
            buoy->magHard[i] = storage.getDouble(key.c_str(), 0.0);
        }
        else
        {
            storage.putDouble(key.c_str(), buoy->magHard[i]);
        }
    }
    stopMem();
}

void softIron(RoboStruct *buoy, bool get)
{
    startMem();
    String key = "";
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            // Key includes MAC address to separate buoy data
            key = "mS" + String(buoy->mac, HEX) + String(i) + String(j);
            if (get)
            {
                double defaultValue = (i == j) ? 1.0 : 0.0; // Identity matrix default
                buoy->magSoft[i][j] = storage.getDouble(key.c_str(), defaultValue);
            }
            else
            {
                storage.putDouble(key.c_str(), buoy->magSoft[i][j]);
            }
        }
    }

    stopMem();
}
void CompassCallibrationFactors(RoboStruct *buoy, bool get)
{
    softIron(buoy, get);
    hardIron(buoy, get);
    Declination(buoy, get);
    CompasOffset(buoy, get);
}

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
