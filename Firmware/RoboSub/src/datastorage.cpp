// https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/src/Preferences.cpp
#include <Preferences.h>
#include <main.h>
#include <RoboCompute.h>
#include "subwifi.h"
Preferences storage;
static SemaphoreHandle_t nvsMutex = NULL;

/**
 * @brief Opens the NVS storage namespace. Must be called before reading/writing.
 */
void startMem(void)
{
    if (nvsMutex == NULL) {
        nvsMutex = xSemaphoreCreateMutex();
    }
    xSemaphoreTake(nvsMutex, portMAX_DELAY);
    storage.begin("NicE_Buoy_Data", false);
}

void stopMem(void)
{
    storage.end();
    xSemaphoreGive(nvsMutex);
}

/**
 * @brief Initializes the non-volatile storage (NVS) on startup.
 */
void initMemory(void)
{
    startMem();

    // 1. Check for legacy typo migration
    if (storage.isKey("Doclat") && !storage.isKey("Docklat")) {
        double lat = storage.getDouble("Doclat");
        double lon = storage.getDouble("Doclon");
        storage.putDouble("Docklat", lat);
        storage.putDouble("Docklon", lon);
        storage.remove("Doclat");
        storage.remove("Doclon");
    }

    // 2. Check if this is a fresh processor or if we need a data fix
    unsigned long id = espMac();
    uint64_t stored_id = storage.getULong64("NicE_BuoyID", 0);
    double kpr_check = storage.getDouble("Kpr", 1.0);
    int maxSpeed_check = storage.getInt("maxSpeed", 0);

    // If Kpr is detected to be a Speed P value (e.g. 20.0), or if parameters are zeroed/corrupted, reset to defaults
    if (id != stored_id || kpr_check > 10.0 || kpr_check == 0.0 || maxSpeed_check == 0)
    {
        storage.putULong64("NicE_BuoyID", id);
        storage.putDouble("Docklat", 0.0);
        storage.putDouble("Docklon", 0.0);
        storage.putDouble("declination", 2.6666666666); // Amsterdam default
        storage.putBool("revBB", false);
        storage.putBool("revSB", false);
        storage.putBool("tSwap", false);
        
        // Reset PIDs to known good defaults
        storage.putDouble("Kpr", 1.0);
        storage.putDouble("Kir", 0.0);
        storage.putDouble("Kdr", 0.0);
        storage.putDouble("Kps", 20.0);
        storage.putDouble("Kis", 0.4);
        storage.putDouble("Kds", 0.2);
        storage.putInt("maxSpeed", 75);
        storage.putInt("minSpeed", -75);
        storage.putDouble("pivotSpeed", 0.5);
        storage.putDouble("magCorr", 90.0); // User's stated good value
        storage.putDouble("holdRad", 2.0); // User's stated good value
    }
    stopMem();
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
        if (isnan(buoy->declination)) buoy->declination = 2.66666666;
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
        buoy->compassOffset = storage.getDouble("magCorr", 0);
        if (isnan(buoy->compassOffset)) buoy->compassOffset = 0;
    }
    else
    {
        storage.putDouble("magCorr", buoy->compassOffset);
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
        buoy->holdRad = storage.getDouble("holdRad", 2.0);
        if (isnan(buoy->holdRad)) buoy->holdRad = 2.0;
        buoy->maxOfsetDist = storage.getInt("maxOfsetDist", 8);
    }
    else
    {
        storage.putDouble("holdRad", buoy->holdRad);
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
        buoy->maxSpeed = storage.getInt("maxSpeed", 0);
        buoy->minSpeed = storage.getInt("minSpeed", 0);
        buoy->pivotSpeed = storage.getDouble("pivotSpeed", 0.5);
        if (isnan(buoy->pivotSpeed)) buoy->pivotSpeed = 0.5;
        if (buoy->pivotSpeed < 0.4) buoy->pivotSpeed = 0.5; // Enforce minimum floor to overcome water resistance/stiction
    }
    else
    {
        storage.putInt("maxSpeed", buoy->maxSpeed);
        storage.putInt("minSpeed", buoy->minSpeed);
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
        buoy->Kps = storage.getDouble("Kps", 20.0);
        if (isnan(buoy->Kps)) buoy->Kps = 20.0;
        buoy->Kis = storage.getDouble("Kis", 0.4);
        if (isnan(buoy->Kis)) buoy->Kis = 0.4;
        buoy->Kds = storage.getDouble("Kds", 0.2);
        if (isnan(buoy->Kds)) buoy->Kds = 0.2;
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
        buoy->Kpr = storage.getDouble("Kpr", 1.0);
        if (isnan(buoy->Kpr)) buoy->Kpr = 1.0;
        buoy->Kir = storage.getDouble("Kir", 0.0);
        if (isnan(buoy->Kir)) buoy->Kir = 0.0;
        buoy->Kdr = storage.getDouble("Kdr", 0.0);
        if (isnan(buoy->Kdr)) buoy->Kdr = 0.0;
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
}

/**
 * @brief Reads or writes raw Min/Max magnetic float values (Legacy/Fallback).
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

void CompassOffsetCorrection(double *offset, bool get)
{
    startMem();
    if (get)
    {
        *offset = storage.getDouble("magCorr", 0);
        if (isnan(*offset)) *offset = 0;
    }
    else
    {
        storage.putDouble("magCorr", *offset);
    }
    stopMem();
}

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

void memBnoCalib(uint8_t *data, bool get)
{
    startMem();
    if (get)
    {
        if (storage.isKey("bnoCal")) {
            storage.getBytes("bnoCal", data, 22);
            Serial.printf("memBnoCalib: Profile LOADED from NVS (First byte: 0x%02X)\n", data[0]);
        } else {
            memset(data, 0, 22);
            Serial.println("memBnoCalib: No profile found.");
        }
    }
    else
    {
        // Don't save if it's all zeros
        bool allZero = true;
        for(int i=0; i<22; i++) if(data[i] != 0) allZero = false;

        if(!allZero) {
            storage.putBytes("bnoCal", data, 22);
            Serial.println("memBnoCalib: Profile SAVED to NVS.");
        } else {
            Serial.println("memBnoCalib: Refused to save all-zero profile.");
        }
    }
    stopMem();
}

void memIcmCalib(float *hi, float *si, bool get)
{
    extern int icm_mode;
    extern float si_matrix[3][3];
    startMem();
    if (get)
    {
        hi[0] = storage.getFloat("icm_hi_x", 0.0f);
        hi[1] = storage.getFloat("icm_hi_y", 0.0f);
        hi[2] = storage.getFloat("icm_hi_z", 0.0f);
        
        si[0] = storage.getFloat("icm_si_x", 1.0f);
        si[1] = storage.getFloat("icm_si_y", 1.0f);
        si[2] = storage.getFloat("icm_si_z", 1.0f);
        
        icm_mode = storage.getInt("icm_mode", 4);
        
        // Load the 3x3 matrix, fallback to diagonal scale factors if not found
        si_matrix[0][0] = storage.getFloat("icm_si_xx", si[0]);
        si_matrix[0][1] = storage.getFloat("icm_si_xy", 0.0f);
        si_matrix[0][2] = storage.getFloat("icm_si_xz", 0.0f);
        si_matrix[1][0] = storage.getFloat("icm_si_yx", 0.0f);
        si_matrix[1][1] = storage.getFloat("icm_si_yy", si[1]);
        si_matrix[1][2] = storage.getFloat("icm_si_yz", 0.0f);
        si_matrix[2][0] = storage.getFloat("icm_si_zx", 0.0f);
        si_matrix[2][1] = storage.getFloat("icm_si_zy", 0.0f);
        si_matrix[2][2] = storage.getFloat("icm_si_zz", si[2]);
        
        Serial.printf("memIcmCalib: LOADED -> HI: [%.4f, %.4f, %.4f], SI diagonal: [%.4f, %.4f, %.4f]\n", hi[0], hi[1], hi[2], si[0], si[1], si[2]);
        Serial.printf("memIcmCalib: LOADED 3x3 matrix ->\n");
        Serial.printf("  [%.4f, %.4f, %.4f]\n", si_matrix[0][0], si_matrix[0][1], si_matrix[0][2]);
        Serial.printf("  [%.4f, %.4f, %.4f]\n", si_matrix[1][0], si_matrix[1][1], si_matrix[1][2]);
        Serial.printf("  [%.4f, %.4f, %.4f]\n", si_matrix[2][0], si_matrix[2][1], si_matrix[2][2]);
    }
    else
    {
        storage.putFloat("icm_hi_x", hi[0]);
        storage.putFloat("icm_hi_y", hi[1]);
        storage.putFloat("icm_hi_z", hi[2]);
        
        storage.putFloat("icm_si_x", si[0]);
        storage.putFloat("icm_si_y", si[1]);
        storage.putFloat("icm_si_z", si[2]);
        
        storage.putInt("icm_mode", icm_mode);
        
        // Save the 3x3 matrix
        storage.putFloat("icm_si_xx", si_matrix[0][0]);
        storage.putFloat("icm_si_xy", si_matrix[0][1]);
        storage.putFloat("icm_si_xz", si_matrix[0][2]);
        storage.putFloat("icm_si_yx", si_matrix[1][0]);
        storage.putFloat("icm_si_yy", si_matrix[1][1]);
        storage.putFloat("icm_si_yz", si_matrix[1][2]);
        storage.putFloat("icm_si_zx", si_matrix[2][0]);
        storage.putFloat("icm_si_zy", si_matrix[2][1]);
        storage.putFloat("icm_si_zz", si_matrix[2][2]);
        
        Serial.printf("memIcmCalib: SAVED -> HI: [%.4f, %.4f, %.4f], SI diagonal: [%.4f, %.4f, %.4f]\n", hi[0], hi[1], hi[2], si[0], si[1], si[2]);
    }
    stopMem();
}

/**
 * @brief Reads or writes the Compass Heading Averaging length (cavg) to Preferences NVM.
 */
void memCompassAvg(int *avg, bool get)
{
    startMem();
    if (get)
    {
        *avg = storage.getInt("cavg", 1);
        if (*avg < 1) *avg = 1;
        if (*avg > 200) *avg = 200;
    }
    else
    {
        storage.putInt("cavg", *avg);
    }
    stopMem();
}

/**
 * @brief Reads or writes the Adaptive Waypoint Bias Trim (compass_trim) to Preferences NVM.
 */
void memCompassTrim(float *trim, bool *enabled, bool get)
{
    startMem();
    if (get)
    {
        *trim = storage.getFloat("c_trim", 0.0f);
        *enabled = storage.getBool("c_trim_en", false); // Disabled by default!
        if (!isfinite(*trim) || *trim < -15.0f || *trim > 15.0f) {
            *trim = 0.0f;
        }
    }
    else
    {
        storage.putFloat("c_trim", *trim);
        storage.putBool("c_trim_en", *enabled);
    }
    stopMem();
}
