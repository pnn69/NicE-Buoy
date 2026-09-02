// https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/src/Preferences.cpp
#include <Preferences.h>
#include <main.h>
#include <RoboCompute.h>
#include "subwifi.h"
#include "esc.h"   // ESC_NEUTRAL_* limits used by memEscNeutral()
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

    // 2. Check if this is a fresh processor or if NVS has been cleared
    unsigned long id = espMac();
    uint64_t stored_id = storage.getULong64("NicE_BuoyID", 0);

    // Only reset to defaults on a completely fresh/cleared NVS partition (when stored_id is 0)
    // This respects the operator's custom PIDs, speed limits, and prevents unwanted resets on reboot!
    if (stored_id == 0 || id != stored_id)
    {
        storage.putULong64("NicE_BuoyID", id);
        storage.putDouble("Docklat", 0.0);
        storage.putDouble("Docklon", 0.0);
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
        // One-time cleanup of the retired declination key, see RoboCompute.h.
        if (storage.isKey("declination")) storage.remove("declination");
    }
    stopMem();
}

/**
 * @brief Reads or writes the station-keeping holding radius from/to NVS.
 *
 * maxOfsetDist used to live here too. It was never on the SETUPDATA wire, had no UI, and nothing
 * in any firmware ever read it - but the SETUPDATA handler wrote it on every save from a struct
 * field the decoder never filled in, so each save quietly stored the compiled-in default over
 * whatever was there. That is the same pattern that used to reset the dock approach settings
 * (see the note in RoboCompute.cpp). Retired rather than fixed: nothing consumed it.
 *
 * @param buoy Pointer to the state structure containing holdRad.
 * @param get True to read from memory, false to write to memory.
 */
void computeParameters(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->holdRad = storage.getDouble("holdRad", 2.0);
        if (isnan(buoy->holdRad)) buoy->holdRad = 2.0;
        // Raised to the floor, not reset to a default - unlike the pivot clamp this replaced,
        // which snapped a legitimate 0.2 all the way to 0.5 and made low values unreachable.
        // Here the floor IS the setting's minimum, so lifting a legacy value to exactly 1.5 is
        // the honest answer and the UI sees the same number the buoy is using.
        if (buoy->holdRad < HOLD_RADIUS_MIN) buoy->holdRad = HOLD_RADIUS_MIN;
    }
    else
    {
        double hr = buoy->holdRad;
        if (isnan(hr) || hr < HOLD_RADIUS_MIN) hr = HOLD_RADIUS_MIN;
        storage.putDouble("holdRad", hr);
        // One-time cleanup of the retired key, so it stops occupying an NVS entry.
        if (storage.isKey("maxOfsetDist")) storage.remove("maxOfsetDist");
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
        // Sanity guard only - NOT a tuning floor. This used to snap anything below 0.4 up to 0.5
        // "to overcome water resistance/stiction", and because the clamp sat on the READ side it
        // made low pivot speeds impossible to set at all: the CYD's SETUPDATA save wrote 0.2 to
        // NVS quite happily, then the reload two lines later in main.cpp handed 0.5 back to the
        // running config and echoed 0.5 to the Top and the CYD. The Setup page just snapped back
        // and the operator could see no reason why. How hard the buoy has to push to pivot is the
        // operator's call, so the only thing rejected here is a value that cannot be a setting:
        // NaN, zero/negative, or above full scale - i.e. a corrupt or never-written key.
        if (isnan(buoy->pivotSpeed) || buoy->pivotSpeed <= 0.0 || buoy->pivotSpeed > 1.0)
        {
            buoy->pivotSpeed = 0.5;
        }
    }
    else
    {
        storage.putInt("maxSpeed", buoy->maxSpeed);
        storage.putInt("minSpeed", buoy->minSpeed);
        // Same range as the read side, so a malformed frame cannot park an unusable value in NVS.
        double pv = buoy->pivotSpeed;
        if (isnan(pv) || pv <= 0.0 || pv > 1.0) pv = 0.5;
        storage.putDouble("pivotSpeed", pv);
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
 * Includes Soft Iron matrix, Hard Iron offsets and the user compass offset.
 *
 * @param buoy Pointer to the main state structure.
 * @param get True to read from memory, false to write to memory.
 */
void CompassCalibrationFactors(RoboStruct *buoy, bool get)
{
    softIron(buoy, get);
    hardIron(buoy, get);
    CompasOffset(buoy, get);
}

/**
 * @brief Reads or writes raw Min/Max magnetic float values (Legacy/Fallback).
 */
void CompassCalibrationFactorsFloat(float *MaxX, float *MaxY, float *MaxZ, float *MinX, float *MinY, float *MinZ, bool get)
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

void memBnoCalib(uint8_t *data, bool get)
{
    startMem();
    if (get)
    {
        if (storage.isKey("bnoCal")) {
            storage.getBytes("bnoCal", data, 22);
            Serial.printf("memBnoCalib: Profile LOADED from NVS (First byte: 0x%02X)\n\r", data[0]);
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
    extern volatile int icm_mode;
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
        
        Serial.printf("memIcmCalib: LOADED -> HI: [%.4f, %.4f, %.4f], SI diagonal: [%.4f, %.4f, %.4f]\n\r", hi[0], hi[1], hi[2], si[0], si[1], si[2]);
        Serial.printf("memIcmCalib: LOADED 3x3 matrix ->\n\r");
        Serial.printf("  [%.4f, %.4f, %.4f]\n\r", si_matrix[0][0], si_matrix[0][1], si_matrix[0][2]);
        Serial.printf("  [%.4f, %.4f, %.4f]\n\r", si_matrix[1][0], si_matrix[1][1], si_matrix[1][2]);
        Serial.printf("  [%.4f, %.4f, %.4f]\n\r", si_matrix[2][0], si_matrix[2][1], si_matrix[2][2]);
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
        
        Serial.printf("memIcmCalib: SAVED -> HI: [%.4f, %.4f, %.4f], SI diagonal: [%.4f, %.4f, %.4f]\n\r", hi[0], hi[1], hi[2], si[0], si[1], si[2]);
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

/**
 * @brief Reads or writes the Pitch/Roll Damping (pr_damping) to Preferences NVM.
 */
void memPrDamping(float *damping, bool get)
{
    startMem();
    if (get)
    {
        *damping = storage.getFloat("pr_damping", 0.95f);
        if (!isfinite(*damping) || *damping < 0.0f || *damping > 0.99f) {
            *damping = 0.95f; // Best guess default!
        }
    }
    else
    {
        storage.putFloat("pr_damping", *damping);
    }
    stopMem();
}

/**
 * @brief Reads or writes all 4 damping factors to Preferences NVM.
 */
void memDampingFactors(float *acc, float *gyro, float *mag, float *att, bool get)
{
    startMem();
    if (get)
    {
        *acc = storage.getFloat("damp_acc", 0.15f);
        if (!isfinite(*acc) || *acc < 0.01f || *acc > 1.0f) *acc = 0.15f;

        *gyro = storage.getFloat("damp_gyro", 0.15f);
        if (!isfinite(*gyro) || *gyro < 0.01f || *gyro > 1.0f) *gyro = 0.15f;

        *mag = storage.getFloat("damp_mag", 0.15f);
        if (!isfinite(*mag) || *mag < 0.01f || *mag > 1.0f) *mag = 0.15f;

        *att = storage.getFloat("damp_att", 0.15f);
        if (!isfinite(*att) || *att < 0.01f || *att > 1.0f) *att = 0.15f;
    }
    else
    {
        storage.putFloat("damp_acc", *acc);
        storage.putFloat("damp_gyro", *gyro);
        storage.putFloat("damp_mag", *mag);
        storage.putFloat("damp_att", *att);
    }
    stopMem();
}

/**
 * @brief Reads or writes all 9 interpolation angles to Preferences NVM.
 */
/**
 * @brief Reads or writes which convention the stored interpolation table was measured under.
 *
 * A number, not a flag, because this has already changed once and will change again if the chain
 * is ever reordered. See INTERP_TABLE_REV in compass.h.
 */
void memInterpTableRev(int *rev, bool get)
{
    startMem();
    if (get)
    {
        // 1 = "written before this was recorded", i.e. the pre-reorder convention. Deliberately not
        // defaulted to the current revision: a table of unknown provenance is exactly the one that
        // must not be trusted, and every table already in the field predates this key.
        *rev = storage.getInt("meas_ang_rev", 1);
    }
    else
    {
        storage.putInt("meas_ang_rev", *rev);
    }
    stopMem();
}

/**
 * @brief Reads or writes the ICM filtering mode the interpolation table was measured in.
 *
 * The selected mode IS the table's input. measured_angles[i] is what the selected mode reads when
 * the hull points at direction i, so switching the mode afterwards feeds the table a different
 * heading source and every entry lands in the wrong place. On a real hull the modes sit 7 to 26
 * degrees apart, so this is not a subtlety.
 *
 * Nothing used to record it, which made the failure invisible: the table still looked valid, the
 * usability check still passed, and the compass was simply wrong.
 */
void memInterpTableMode(int *mode, bool get)
{
    startMem();
    if (get)
    {
        // -1 means "measured before this was recorded", which must not be read as a mismatch.
        *mode = storage.getInt("meas_ang_mode", -1);
    }
    else
    {
        storage.putInt("meas_ang_mode", *mode);
    }
    stopMem();
}

void memInterpolationTable(float *angles, bool get)
{
    startMem();
    if (get)
    {
        size_t len = storage.getBytes("meas_ang", angles, sizeof(float) * 9);
        if (len != sizeof(float) * 9) {
            // Default to 0, 45, 90, ... 360
            for (int i = 0; i < 9; i++) {
                angles[i] = i * 45.0f;
            }
            storage.putBytes("meas_ang", angles, sizeof(float) * 9);
        }
    }
    else
    {
        storage.putBytes("meas_ang", angles, sizeof(float) * 9);
    }
    stopMem();
}

/**
 * @brief Reads or writes the per-thruster ESC neutral pulse width, in microseconds.
 *
 * 1500 us is only nominally "stop". A bidirectional ESC stores its own neutral point, and the two
 * units on a buoy are rarely calibrated identically - one of ours sits far enough off that a
 * nominal 1500 is read as a small forward command and the thruster creeps for as long as the ESC
 * has power. Rather than recalibrate the ESC in the field, trim the pulse the firmware sends.
 *
 * Clamped to +/-100 us of nominal. That is wide enough for any real ESC offset and narrow enough
 * that a fat-fingered value cannot turn "stop" into meaningful thrust.
 */
void memEscNeutral(int *bb, int *sb, bool get)
{
    startMem();
    if (get)
    {
        *bb = storage.getInt("escNeutBb", ESC_NEUTRAL_NOMINAL_US);
        *sb = storage.getInt("escNeutSb", ESC_NEUTRAL_NOMINAL_US);
        if (*bb < ESC_NEUTRAL_MIN_US || *bb > ESC_NEUTRAL_MAX_US) *bb = ESC_NEUTRAL_NOMINAL_US;
        if (*sb < ESC_NEUTRAL_MIN_US || *sb > ESC_NEUTRAL_MAX_US) *sb = ESC_NEUTRAL_NOMINAL_US;
    }
    else
    {
        int b = *bb, s = *sb;
        if (b < ESC_NEUTRAL_MIN_US || b > ESC_NEUTRAL_MAX_US) b = ESC_NEUTRAL_NOMINAL_US;
        if (s < ESC_NEUTRAL_MIN_US || s > ESC_NEUTRAL_MAX_US) s = ESC_NEUTRAL_NOMINAL_US;
        storage.putInt("escNeutBb", b);
        storage.putInt("escNeutSb", s);
    }
    stopMem();
}

/**
 * @brief Reads or writes the linear interpolation enable state to Preferences NVM.
 */
void memInterpEnabled(bool *enabled, bool get)
{
    startMem();
    if (get)
    {
        *enabled = storage.getBool("interp_en", false);
    }
    else
    {
        storage.putBool("interp_en", *enabled);
    }
    stopMem();
}
