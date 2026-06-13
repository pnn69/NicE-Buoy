// https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/src/Preferences.cpp
#include <Preferences.h>
#include <main.h>
Preferences storage;

/**
 * @brief Initializes the persistent memory.
 * 
 * Migrates data from legacy keys if necessary.
 */
void initMemory(void)
{
    storage.begin("RobobuoyTop", false);
    // Legacy migration logic here if needed
    storage.end();
}

void startMem(void)
{
    storage.begin("RobobuoyTop", false);
}

void stopMem(void)
{
    storage.end();
}

/**
 * @brief Gets or sets the buoy ID in memory.
 */
void memBuoyId(int8_t *id, bool get)
{
    startMem();
    if (get)
    {
        *id = storage.getChar("buoyId", 1);
    }
    else
    {
        storage.putChar("buoyId", *id);
    }
    stopMem();
}

/**
 * @brief Legacy wrapper for compass offset.
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
 * @brief Gets or sets the compass offset correction in memory.
 */
void CompasOffset(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->compassOffset = storage.getDouble("magCorr", 0);
    }
    else
    {
        storage.putDouble("magCorr", buoy->compassOffset);
    }
    stopMem();
}

/**
 * @brief Gets or sets the mechanical correction delta in memory.
 */
void MechanicalCorrection(double *correction, bool get)
{
    startMem();
    if (get)
    {
        *correction = storage.getDouble("mechCorr", 0);
        if (isnan(*correction)) *correction = 0;
    }
    else
    {
        storage.putDouble("mechCorr", *correction);
    }
    stopMem();
}

/**
 * @brief Gets or sets the docking position in memory.
 */
void memDockPos(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->tgLat = storage.getDouble("Docklat", 0);
        buoy->tgLng = storage.getDouble("Docklon", 0);
    }
    else
    {
        storage.putDouble("Docklat", buoy->tgLat);
        storage.putDouble("Docklon", buoy->tgLng);
    }
    stopMem();
}

/**
 * @brief Gets or sets the thruster inversion settings in memory.
 */
void thrusterInversion(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->revBB = storage.getBool("revBB", false);
        buoy->revSB = storage.getBool("revSB", false);
    }
    else
    {
        storage.putBool("revBB", buoy->revBB);
        storage.putBool("revSB", buoy->revSB);
    }
    stopMem();
}

/**
 * @brief Gets or sets general computation parameters in memory.
 */
void computeParameters(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->maxOfsetDist = storage.getInt("maxOfsetDist", 8);
        buoy->maxSpeed = storage.getInt("maxSpeed", 80);
        buoy->minSpeed = storage.getInt("minSpeed", 0);
        buoy->pivotSpeed = storage.getDouble("pivotSpeed", 0.2);
        buoy->holdRad = storage.getDouble("holdRad", 2.0);
    }
    else
    {
        storage.putInt("maxOfsetDist", buoy->maxOfsetDist);
        storage.putInt("maxSpeed", buoy->maxSpeed);
        storage.putInt("minSpeed", buoy->minSpeed);
        storage.putDouble("pivotSpeed", buoy->pivotSpeed);
        storage.putDouble("holdRad", buoy->holdRad);
    }
    stopMem();
}

/**
 * @brief Gets or sets PID speed parameters in memory.
 */
void pidSpeedParameters(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->Kps = storage.getDouble("Kps", 20);
        buoy->Kis = storage.getDouble("Kis", 0.05);
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
 * @brief Gets or sets PID rudder parameters in memory.
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

void thrusterSwap(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->swap_BB_SB = storage.getBool("tSwap", false);
    }
    else
    {
        storage.putBool("tSwap", buoy->swap_BB_SB);
    }
    stopMem();
}

void defautls(RoboStruct *buoy)
{
    buoy->Kpr = 0.5;
    buoy->Kir = 0.02;
    buoy->Kdr = 0;
    pidRudderParameters(buoy, false);

    buoy->Kps = 20;
    buoy->Kis = 0.05;
    buoy->Kds = 0;
    pidSpeedParameters(buoy, false);
}
