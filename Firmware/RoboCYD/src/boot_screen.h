#ifndef BOOT_SCREEN_H
#define BOOT_SCREEN_H

#include <Arduino.h>

// The ROBOBUOY splash: the artwork from Doc/robobuoy_startup_lvgl_reference/, shown while setup()
// brings WiFi and the radio up, and replaced by draw_resting_ui() the moment boot finishes.
//
// Every call here is inert once boot_screen_end() has run, so the WiFi code may keep calling
// boot_screen_wifi()/boot_screen_tick() from its background home-check without ever touching the
// running dashboard. That is the whole reason these are functions and not direct tft writes.

// Indicator colours follow the startup package's README: gray = disconnected,
// green = joined an access point, blue = we ARE the access point.
enum BootWifiState {
    BOOT_WIFI_DOWN = 0,
    BOOT_WIFI_CLIENT,
    BOOT_WIFI_AP
};

void boot_screen_begin();                   // push the logo, take over the screen
void boot_screen_status(const char *text);  // one short line in the black band under the artwork
void boot_screen_tick();                    // animate the ellipsis from inside a blocking wait
void boot_screen_wifi(BootWifiState state);
void boot_screen_lora(bool up);
void boot_screen_end();                     // hold the minimum dwell, then go inert

#endif // BOOT_SCREEN_H
