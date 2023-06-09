#include <Preferences.h>

Preferences storage;

void InitMemory(void)
{
    // Open Preferences with my-app namespace. Each application module, library, etc
    // has to use a namespace name to prevent key name collisions. We will open storage in
    // RW-mode (second parameter has to be false).
    // Note: Namespace name is limited to 15 chars.
    storage.begin("NicE_Buoy_Data");
}
/*
    ID of the buoy
*/
void SetMemoryBuoyID(char id)
{
    storage.putChar("NicE_BuoyID", id);
    Serial.printf("Stored BuoyID in memory  %d\r", id);
}
void GetMemoryBuoyID(char *id)
{
    *id = (storage.getChar("NicE_BuoyID", 0));
    Serial.printf("Get BuoyID from memory  %d\r\n", *id);
}

/*
    Anchor position
*/
void GetMemoryAnchorPos(double *lat, double *lon)
{
    *lat = storage.getDouble("latAnchor", 0);
    *lon = storage.getDouble("lonAnchor", 0);
    Serial.printf("Anchor pos form memory  %.8lf %.8lf\r\n", *lat, *lon);
}
void SetMemoryAnchorPos(double lat, double lon)
{
    Serial.printf("Store anchor pos to memory  %.8lf %.8lf\r\n", lat, lon);
    storage.putDouble("latAnchor", lat);
    storage.putDouble("lonAnchor", lon);
}

/*
    Dock position
*/
void SetMemoryDockPos(double lat, double lon)
{
    Serial.printf("Store Doc pos in memory  %.8lf %.8lf\r\n", lat, lon);
    storage.putDouble("latDock", lat);
    storage.putDouble("lonDock", lon);
}
void GetMemoryDockPos(double *lat, double *lon)
{
    *lat = storage.getDouble("latDock", 0);
    *lon = storage.getDouble("lonDock", 0);
    Serial.printf("Get Doc pos form memory  %.8lf %.8lf\r\n", *lat, *lon);
}
