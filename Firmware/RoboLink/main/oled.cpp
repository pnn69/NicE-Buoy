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
static const uint8_t FONT_0[8] = {
    0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E, 0x00, 0x00};

static const uint8_t FONT_1[8] = {
    0x00, 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00, 0x00};

static const uint8_t FONT_2[8] = {
    0x00, 0x62, 0x51, 0x49, 0x49, 0x46, 0x00, 0x00};

static const uint8_t FONT_3[8] = {
    0x00, 0x22, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00};

static const uint8_t FONT_4[8] = {
    0x00, 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00, 0x00};

static const uint8_t FONT_5[8] = {
    0x00, 0x2F, 0x49, 0x49, 0x49, 0x31, 0x00, 0x00};

static const uint8_t FONT_6[8] = {
    0x00, 0x3E, 0x49, 0x49, 0x49, 0x32, 0x00, 0x00};

static const uint8_t FONT_7[8] = {
    0x00, 0x01, 0x71, 0x09, 0x05, 0x03, 0x00, 0x00};

static const uint8_t FONT_8[8] = {
    0x00, 0x36, 0x49, 0x49, 0x49, 0x36, 0x00, 0x00};

static const uint8_t FONT_9[8] = {
    0x00, 0x26, 0x49, 0x49, 0x49, 0x3E, 0x00, 0x00};

static const uint8_t FONT_DOT[8] = {
    0x00, 0x00, 0x60, 0x60, 0x00, 0x00, 0x00, 0x00};

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

static const uint8_t *oled_ip_character(char c)
{
    switch (c)
    {
    case '0':
        return FONT_0;
    case '1':
        return FONT_1;
    case '2':
        return FONT_2;
    case '3':
        return FONT_3;
    case '4':
        return FONT_4;
    case '5':
        return FONT_5;
    case '6':
        return FONT_6;
    case '7':
        return FONT_7;
    case '8':
        return FONT_8;
    case '9':
        return FONT_9;
    case '.':
        return FONT_DOT;
    default:
        return nullptr;
    }
}

esp_err_t oled_write_ip(const char *ip)
{
    if (ip == nullptr)
    {
        return ESP_ERR_INVALID_ARG;
    }

    size_t length = strlen(ip);

    if (length == 0 || length > 15)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t width =
        static_cast<uint8_t>(length * 8);

    uint8_t start_column =
        static_cast<uint8_t>((128 - width) / 2);

    ESP_ERROR_CHECK(oled_command(0x21));
    ESP_ERROR_CHECK(oled_command(start_column));
    ESP_ERROR_CHECK(oled_command(start_column + width - 1));

    ESP_ERROR_CHECK(oled_command(0x22));
    ESP_ERROR_CHECK(oled_command(5));
    ESP_ERROR_CHECK(oled_command(5));

    for (size_t i = 0; i < length; i++)
    {
        const uint8_t *character =
            oled_ip_character(ip[i]);

        if (character == nullptr)
        {
            return ESP_ERR_INVALID_ARG;
        }

        ESP_ERROR_CHECK(
            oled_data(character, 8));
    }

    return ESP_OK;
}