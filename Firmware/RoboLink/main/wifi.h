#ifndef ROBOLINK_WIFI_H_
#define ROBOLINK_WIFI_H_

#include "esp_err.h"

enum class RoboLinkWifiMode
{
    DISCONNECTED = 0,
    STA_HOME,
    STA_FIELD,
    AP_FIELD
};

esp_err_t robolink_wifi_init();

bool robolink_wifi_connected();

RoboLinkWifiMode robolink_wifi_mode();

#endif /* ROBOLINK_WIFI_H_ */
