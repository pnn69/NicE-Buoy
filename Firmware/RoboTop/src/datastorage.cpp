// https://github.com/espressif/arduino-esp32/blob/master/libraries/Preferences/src/Preferences.cpp
// The Top stores only what the Top owns: its buoy id, WiFi credentials, the dock position and
// the dock approach. Everything else the Setup dialog shows - both PID sets, the speed limits,
// the compass offset and the thruster inversion/swap flags - is the Sub's, because the Sub is
// what runs the PID loops, carries the compass and drives the ESCs. It keeps them in its own
// NVS and reports them in SETUPDATA; the Top holds the reply in RAM to display and relay.
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
 * @brief Gets or sets the docking position in memory.
 */
void memDockPos(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->tgLat = storage.getDouble("Docklat", 52.29302221327865);
        buoy->tgLng = storage.getDouble("Docklon", 4.932541137977593);
        
        // If empty, NaN, or invalid, assign default and write back to flash immediately to ensure it's stored
        if (isnan(buoy->tgLat) || isnan(buoy->tgLng) || buoy->tgLat == 0.0 || buoy->tgLng == 0.0)
        {
            buoy->tgLat = 52.29302221327865;
            buoy->tgLng = 4.932541137977593;
            
            storage.putDouble("Docklat", buoy->tgLat);
            storage.putDouble("Docklon", buoy->tgLng);
            printf("Dock position was empty/invalid in NVM. Saved default: 52.29302221327865, 4.932541137977593\r\n");
        }
    }
    else
    {
        // Prevent storing 0.0, NaN, or invalid values
        if (buoy->tgLat != 0.0 && buoy->tgLng != 0.0 && !isnan(buoy->tgLat) && !isnan(buoy->tgLng)) {
            storage.putDouble("Docklat", buoy->tgLat);
            storage.putDouble("Docklon", buoy->tgLng);
        } else {
            printf("WARNING: Blocked attempt to overwrite Dock position with 0.0/0.0 or NaN!\r\n");
        }
    }
    stopMem();
}

void memDockApproach(RoboStruct *buoy, bool get)
{
    startMem();
    if (get)
    {
        buoy->dockApproachDist = storage.getInt("dockAppDist", 0); //meters
        buoy->dockApproachDir = storage.getInt("dockAppDir", 0); // degrees
        buoy->dockingToWaypoint = storage.getBool("dockToWP", false);
        printf("NVM LOAD memDockApproach: Dist=%d, Dir=%d, WP=%s\r\n", buoy->dockApproachDist, buoy->dockApproachDir, buoy->dockingToWaypoint ? "YES" : "NO");
    }
    else
    {
        storage.putInt("dockAppDist", buoy->dockApproachDist);
        storage.putInt("dockAppDir", buoy->dockApproachDir);
        storage.putBool("dockToWP",buoy->dockingToWaypoint);
        printf("NVM SAVE memDockApproach: Dist=%d, Dir=%d, WP=%s\r\n", buoy->dockApproachDist, buoy->dockApproachDir, buoy->dockingToWaypoint ? "YES" : "NO");
    }
    stopMem();
}

