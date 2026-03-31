// https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/src/Preferences.cpp
#include <Preferences.h>
#include <main.h>
Preferences storage;

/**
 * @brief Initializes the persistent memory.
 * 
 * Migrates legacy keys, checks if this is a fresh processor by comparing MAC IDs,
 * and sets default values if it is the first boot.
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
        
        // Initial PID Defaults (if not already set by factory)
        if (!storage.isKey("Kps")) {
            storage.putDouble("Kps", 20.0);
            storage.putDouble("Kis", 0.4);
            storage.putDouble("Kds", 0.0);
            storage.putDouble("Kpr", 0.5);
            storage.putDouble("Kir", 0.02);
            storage.putDouble("Kdr", 0.0);
        }

        Serial.printf("# Buoy Memory initialized for MAC: %08X\r\n", id);
        delay(500);
    }
    else
    {
        Serial.printf("# Buoy Memory OK (MAC: %08X)\r\n", id);
    }
    storage.end();
}
/*
    ID of the buoy
*/
/**
 * @brief Opens the persistent memory store in read/write mode.
 */
void startMem(void)
{
    storage.begin("NicE_Buoy_Data", false);
}

/**
 * @brief Closes the persistent memory store.
 */
void stopMem(void)
{
    storage.end();
}

/**
 * @brief Gets or sets the buoy MAC ID in persistent memory.
 * 
 * @param id Pointer to a uint64_t to store/retrieve the ID.
 * @param get If true, retrieves the ID. If false, stores the ID.
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

/*
    Dock position
*/
/**
 * @brief Gets or sets the dock coordinates in persistent memory.
 * 
 * @param buoy Pointer to the RoboStruct to store/retrieve dock position.
 * @param get If true, retrieves the position. If false, stores it.
 */
void memDockPos(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->tgLat = storage.getDouble("Docklat", 0);
        buoy->tgLng = storage.getDouble("Docklon", 0);
        Serial.printf("Get Doc pos form memory  %.8lf %.8lf\r\n", buoy->tgLat, buoy->tgLng);
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
/**
 * @brief Gets or sets the compass calibration factors in persistent memory.
 * 
 * @param MaxX Pointer to max X.
 * @param MaxY Pointer to max Y.
 * @param MaxZ Pointer to max Z.
 * @param MinX Pointer to min X.
 * @param MinY Pointer to min Y.
 * @param MinZ Pointer to min Z.
 * @param get If true, retrieves values. If false, stores them.
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

/**
 * @brief Gets or sets the compass offset correction in memory.
 * 
 * @param delta Pointer to store/retrieve the delta offset.
 * @param get If true, retrieves the value. If false, stores it.
 */
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

/**
 * @brief Gets or sets the mechanical correction delta in memory.
 * 
 * @param delta Pointer to store/retrieve the delta correction.
 * @param get If true, retrieves the value. If false, stores it.
 */
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

/**
 * @brief Gets or sets compute parameters for movement and distances.
 * 
 * @param buoy Pointer to RoboStruct to load/save compute parameters.
 * @param get If true, retrieves parameters. If false, stores them.
 */
void computeParameters(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->minOfsetDist = storage.getInt("minOfsetDist", 1);
        buoy->maxOfsetDist = storage.getInt("maxOfsetDist", 8);
        buoy->minSpeed = storage.getInt("minSpeed", 0);
        buoy->maxSpeed = storage.getInt("maxSpeed", 80);
        buoy->pivotSpeed = storage.getDouble("pivotSpeed", 0.2);
    }
    else
    {
        storage.putInt("minOfsetDist", buoy->minOfsetDist);
        storage.putInt("maxOfsetDist", buoy->maxOfsetDist);
        storage.putInt("minSpeed", buoy->minSpeed);
        storage.putInt("maxSpeed", buoy->maxSpeed);
        storage.putDouble("pivotSpeed", buoy->pivotSpeed);
    }
    stopMem();
}

/**
 * @brief Gets or sets PID parameters for speed control.
 * 
 * @param buoy Pointer to RoboStruct to load/save speed PID values.
 * @param get If true, retrieves parameters. If false, stores them.
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
 * @brief Gets or sets PID parameters for rudder/steering control.
 * 
 * @param buoy Pointer to RoboStruct to load/save rudder PID values.
 * @param get If true, retrieves parameters. If false, stores them.
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
 * @brief Gets or sets the access point SSID and password in memory.
 * 
 * @param ap Pointer to the String for the SSID.
 * @param ww Pointer to the String for the password.
 * @param get If true, retrieves credentials. If false, stores them.
 */
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

/**
 * @brief Restores default settings for dock position and PID controls.
 * 
 * @param buoy Pointer to the RoboStruct to reset.
 */
void defautls(RoboStruct *buoy)
{
    //***************************************************************************************************
    //  Default to 0,0 - User must set a valid dock position
    //***************************************************************************************************
    buoy->tgLat = 0.0;
    buoy->tgLng = 0.0;
    memDockPos(buoy, false); // store default
                              //***************************************************************************************************
                              //  PID rudder
                              //***************************************************************************************************
    buoy->Kpr = 0.5;
    buoy->Kir = 0.02;
    buoy->Kdr = 0;
    pidRudderParameters(buoy, false);
    //***************************************************************************************************
    //  PID speed
    //***************************************************************************************************
    buoy->Kps = 20;
    buoy->Kis = 0.4;
    buoy->Kds = 0;
    pidSpeedParameters(buoy, false);
}