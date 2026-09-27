#include <cstring>

#include "oled.h"
#include "io.h"
#include "esp_log.h"

static const char *TAG = "OLED";

static i2c_master_dev_handle_t oled_dev = nullptr;

static esp_err_t oled_command(uint8_t command)
{
    uint8_t data[2] = {
        0x00,
        command};

    return i2c_master_transmit(
        oled_dev,
        data,
        sizeof(data),
        100);
}

static esp_err_t oled_data(const uint8_t *data, size_t length)
{
    uint8_t buffer[129];

    if (length > 128)
    {
        return ESP_ERR_INVALID_SIZE;
    }

    buffer[0] = 0x40;
    memcpy(&buffer[1], data, length);

    return i2c_master_transmit(
        oled_dev,
        buffer,
        length + 1,
        100);
}

esp_err_t oled_init(i2c_master_bus_handle_t bus)
{
    i2c_device_config_t dev_config = {};

    dev_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_config.device_address = OLED_ADDRESS;
    dev_config.scl_speed_hz = 400000;

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(
            bus,
            &dev_config,
            &oled_dev));

    const uint8_t init_sequence[] = {
        0xAE,
        0xD5, 0x80,
        0xA8, 0x3F,
        0xD3, 0x00,
        0x40,
        0x8D, 0x14,
        0x20, 0x00,
        0xA1,
        0xC8,
        0xDA, 0x12,
        0x81, 0xCF,
        0xD9, 0xF1,
        0xDB, 0x40,
        0xA4,
        0xA6,
        0xAF};

    for (uint8_t command : init_sequence)
    {
        ESP_ERROR_CHECK(oled_command(command));
    }

    ESP_LOGI(TAG, "SSD1306 initialized at 0x%02X", OLED_ADDRESS);

    return ESP_OK;
}

esp_err_t oled_clear()
{
    ESP_ERROR_CHECK(oled_command(0x21));
    ESP_ERROR_CHECK(oled_command(0));
    ESP_ERROR_CHECK(oled_command(127));

    ESP_ERROR_CHECK(oled_command(0x22));
    ESP_ERROR_CHECK(oled_command(0));
    ESP_ERROR_CHECK(oled_command(7));

    uint8_t blank[128] = {};

    for (int page = 0; page < 8; page++)
    {
        ESP_ERROR_CHECK(oled_data(blank, sizeof(blank)));
    }

    return ESP_OK;
}

static const uint8_t FONT_R[8] = {
    0x00, 0x7F, 0x09, 0x19, 0x29, 0x46, 0x00, 0x00};

static const uint8_t FONT_o[8] = {
    0x00, 0x38, 0x44, 0x44, 0x44, 0x38, 0x00, 0x00};

static const uint8_t FONT_b[8] = {
    0x00, 0x7F, 0x48, 0x44, 0x44, 0x38, 0x00, 0x00};

static const uint8_t FONT_L[8] = {
    0x00, 0x7F, 0x40, 0x40, 0x40, 0x40, 0x00, 0x00};

static const uint8_t FONT_i[8] = {
    0x00, 0x00, 0x44, 0x7D, 0x40, 0x00, 0x00, 0x00};

static const uint8_t FONT_n[8] = {
    0x00, 0x7C, 0x08, 0x04, 0x04, 0x78, 0x00, 0x00};

static const uint8_t FONT_k[8] = {
    0x00, 0x7F, 0x10, 0x28, 0x44, 0x00, 0x00, 0x00};

esp_err_t oled_write_robolink()
{
    const uint8_t *text[] = {
        FONT_R,
        FONT_o,
        FONT_b,
        FONT_o,
        FONT_L,
        FONT_i,
        FONT_n,
        FONT_k};

    ESP_ERROR_CHECK(oled_command(0x21));
    ESP_ERROR_CHECK(oled_command(32));
    ESP_ERROR_CHECK(oled_command(95));

    ESP_ERROR_CHECK(oled_command(0x22));
    ESP_ERROR_CHECK(oled_command(3));
    ESP_ERROR_CHECK(oled_command(3));

    for (const uint8_t *character : text)
    {
        ESP_ERROR_CHECK(oled_data(character, 8));
    }

    return ESP_OK;
}