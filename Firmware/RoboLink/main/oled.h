#ifndef OLED_H_
#define OLED_H_

#include "driver/i2c_master.h"
#include "esp_err.h"

esp_err_t oled_init(i2c_master_bus_handle_t bus);
esp_err_t oled_clear();
esp_err_t oled_write_robolink();

#endif /* OLED_H_ */