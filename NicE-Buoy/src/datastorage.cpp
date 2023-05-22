#include <Preferences.h>

Preferences storage;

void InitMemory(void)
{
    // Open Preferences with my-app namespace. Each application module, library, etc
    // has to use a namespace name to prevent key name collisions. We will open storage in
    // RW-mode (second parameter has to be false).
    // Note: Namespace name is limited to 15 chars.
    storage.begin("NicE_Buoy_Data", false);
}
/*
    ID of the buoy
*/
void BuoyID(bool store, char *id)
{
    if (store == true)
    {
        storage.putChar("NicE_BuoyID", *id);
    }
    else
    {
        *id = (storage.getChar("NicE_BuoyID", 0));
    }
}

/*
    Anchor position
*/
void AnchorPosMemory(bool store, double *lat, double *lon)
{
    if (store == true)
    {
        storage.putDouble("latAnchor", *lat);
        storage.putDouble("lonAnchor", *lon);
    }
    else
    {
        *lat = storage.getDouble("latAnchor", 0);
        *lon = storage.getDouble("lonAnchor", 0);
    }
}

/*
    Dock position
*/
void DockMemory(bool store, double *lat, double *lon)
{
    if (store == true)
    {
        storage.putDouble("latDock", *lat);
        storage.putDouble("lonDock", *lon);
    }
    else
    {
        *lat = storage.getDouble("latDock", 0);
        *lon = storage.getDouble("lonDock", 0);
    }
}
