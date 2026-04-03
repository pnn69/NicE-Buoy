// https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/src/Preferences.cpp
#include <Preferences.h>
#include <main.h>
#include <RoboCompute.h>
#include "subwifi.h"
Preferences storage;

/**
 * @brief Initializes the non-volatile storage (NVS) on startup.
 * 1. Opens the "NicE_Buoy_Data" namespace.
 * 2. Migrates legacy spelling mistakes in keys (e.g., Doclat -> Docklat).
 * 3. Checks if this is a fresh ESP32 by comparing the stored MAC ID.
 *    If new, seeds the NVS with default safe values (e.g., 0,0 dock position, Amsterdam declination).
 */
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
        
        // Default Dock: User should set their own home/dock position via the UI or other means.
        storage.putDouble("Docklat", 0.0);
        storage.putDouble("Docklon", 0.0);
        storage.putDouble("declination", 2.6666666666); // Amsterdam default

        Serial.printf("# Buoy Memory initialized for MAC: %08X\r\n", id);
        delay(500);
    }
    else
    {
        Serial.printf("# Buoy Memory OK (MAC: %08X)\r\n", id);
    }
    storage.end();
}

/**
 * @brief Opens the NVS storage namespace. Must be called before reading/writing.
 */
void startMem(void)
{
    storage.begin("NicE_Buoy_Data", false);
}

/**
 * @brief Closes the NVS storage namespace to commit writes and free resources.
 */
void stopMem(void)
{
    storage.end();
}

/**
 * @brief Reads or writes the Buoy ID (MAC address) from/to NVS.
 * 
 * @param id Pointer to the ID variable.
 * @param get True to read from memory, false to write to memory.
 */
void memBuoyId(uint64_t *id, bool get)
{
    startMem();
    if (get)
    {
        *id = storage.getULong64("NicE_BuoyID", 0);
    }
    else
    {
        storage.putULong64("NicE_BuoyID", *id);
    }
    stopMem();
}

/**
 * @brief Reads or writes the Dock position (Home Target) from/to NVS.
 * 
 * @param buoy Pointer to the main state structure containing tgLat and tgLng.
 * @param get True to read from memory, false to write to memory.
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

/**
 * @brief Reads or writes the magnetic declination offset from/to NVS.
 * Defaults to 2.666 degrees (Amsterdam 2025).
 * 
 * @param buoy Pointer to the state structure containing declination.
 * @param get True to read from memory, false to write to memory.
 */
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

/**
 * @brief Reads or writes the user-defined compass heading offset from/to NVS.
 * 
 * @param buoy Pointer to the state structure containing compassOffset.
 * @param get True to read from memory, false to write to memory.
 */
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

/**
 * @brief Reads or writes target approach distance constraints from/to NVS.
 * Controls the active holding radius around a target point.
 * 
 * @param buoy Pointer to the state structure containing min/max offset distances.
 * @param get True to read from memory, false to write to memory.
 */
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

/**
 * @brief Reads or writes maximum/minimum ESC speed limits and pivot speeds.
 * 
 * @param buoy Pointer to the state structure containing motor speed parameters.
 * @param get True to read from memory, false to write to memory.
 */
void speedMaxMin(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->minSpeed = storage.getInt("minSpeed", -73);
        buoy->maxSpeed = storage.getInt("maxSpeed", 73);
        buoy->pivotSpeed = storage.getDouble("pivotSpeed", 0.2);
    }
    else
    {
        storage.putInt("minSpeed", buoy->minSpeed);
        storage.putInt("maxSpeed", buoy->maxSpeed);
        storage.putDouble("pivotSpeed", buoy->pivotSpeed);
    }
    stopMem();
}

/**
 * @brief Reads or writes the Speed/Distance PID tuning parameters.
 * 
 * @param buoy Pointer to the state structure containing Kps, Kis, Kds.
 * @param get True to read from memory, false to write to memory.
 */
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

/**
 * @brief Reads or writes the Rudder/Heading PID tuning parameters.
 * 
 * @param buoy Pointer to the state structure containing Kpr, Kir, Kdr.
 * @param get True to read from memory, false to write to memory.
 */
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

/**
 * @brief Reads or writes WiFi Access Point credentials.
 * 
 * @param ap Pointer to the Access Point SSID string.
 * @param ww Pointer to the Access Point password string.
 * @param get True to read from memory, false to write to memory.
 */
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

// NOTE: ESP32 Preferences (NVS) has a strict limit of 15 characters for keys.
// Do not include long strings like MAC addresses in keys to avoid silent failures.

/**
 * @brief Reads or writes the Hard Iron compass calibration offsets (x, y, z) from/to NVS.
 * 
 * @param buoy Pointer to the state structure containing magHard array.
 * @param get True to read from memory, false to write to memory.
 */
void hardIron(RoboStruct *buoy, bool get)
{
    startMem();
    String key = "";
    for (int i = 0; i < 3; i++)
    {
        key = "mH0" + String(i);
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

/**
 * @brief Reads or writes the Soft Iron compass calibration 3x3 matrix from/to NVS.
 * Defaults to an identity matrix if no calibration data is present.
 * 
 * @param buoy Pointer to the state structure containing magSoft array.
 * @param get True to read from memory, false to write to memory.
 */
void softIron(RoboStruct *buoy, bool get)
{
    startMem();
    String key = "";
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            key = "mS0" + String(i) + String(j);
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

void icmHardIron(RoboStruct *buoy, bool get)
{
    startMem();
    String key = "";
    for (int i = 0; i < 3; i++)
    {
        key = "iH0" + String(i);
        if (get)
        {
            buoy->icmMagHard[i] = storage.getDouble(key.c_str(), 0.0);
        }
        else
        {
            storage.putDouble(key.c_str(), buoy->icmMagHard[i]);
        }
    }
    stopMem();
}

void icmSoftIron(RoboStruct *buoy, bool get)
{
    startMem();
    String key = "";
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            key = "iS0" + String(i) + String(j);
            if (get)
            {
                double defaultValue = (i == j) ? 1.0 : 0.0; // Identity matrix default
                buoy->icmMagSoft[i][j] = storage.getDouble(key.c_str(), defaultValue);
            }
            else
            {
                storage.putDouble(key.c_str(), buoy->icmMagSoft[i][j]);
            }
        }
    }
    stopMem();
}

void icmCompassOffsetLoad(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->icmCompassOffset = storage.getDouble("icm_c_off", 0.0);
    }
    else
    {
        storage.putDouble("icm_c_off", buoy->icmCompassOffset);
    }
    stopMem();
}

/**
 * @brief Aggregates reads or writes for all compass-related calibration factors.
 * Includes Soft Iron matrix, Hard Iron offsets, Declination, and user Offset.
 *
 * @param buoy Pointer to the main state structure.
 * @param get True to read from memory, false to write to memory.
 */
void CompassCallibrationFactors(RoboStruct *buoy, bool get)
{
    softIron(buoy, get);
    hardIron(buoy, get);
    Declination(buoy, get);
    CompasOffset(buoy, get);

    // Also load the ICM-20948 parameters
    icmSoftIron(buoy, get);
    icmHardIron(buoy, get);
    icmCompassOffsetLoad(buoy, get);
}
/**
 * @brief Reads or writes raw Min/Max magnetic float values (Legacy/Fallback).
 * 
 * @param MaxX Pointer to max X value.
 * @param MaxY Pointer to max Y value.
 * @param MaxZ Pointer to max Z value.
 * @param MinX Pointer to min X value.
 * @param MinY Pointer to min Y value.
 * @param MinZ Pointer to min Z value.
 * @param get True to read from memory, false to write to memory.
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
