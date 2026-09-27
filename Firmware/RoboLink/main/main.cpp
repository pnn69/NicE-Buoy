#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "io.h"
#include "oled.h"
#include "lora.h"
#include "packet_queue.h"
#include "wifi.h"
#include "udp.h"
#include "router.h"
#include "ota.h"

static const char *TAG = "RoboLink";

static i2c_master_bus_handle_t i2c_bus = nullptr;

static void init_led()
{
    gpio_config_t led_config = {};

    led_config.pin_bit_mask = (1ULL << LED_PIN);
    led_config.mode = GPIO_MODE_OUTPUT;
    led_config.pull_up_en = GPIO_PULLUP_DISABLE;
    led_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    led_config.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&led_config));
}

static void init_i2c()
{
    i2c_master_bus_config_t bus_config = {};

    bus_config.i2c_port = I2C_NUM_0;
    bus_config.sda_io_num = (gpio_num_t)SDA;
    bus_config.scl_io_num = (gpio_num_t)SCL;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(
        i2c_new_master_bus(
            &bus_config,
            &i2c_bus));

    ESP_LOGI(
        TAG,
        "I2C initialized: SDA=%d SCL=%d",
        SDA,
        SCL);
}

static void scan_i2c()
{
    ESP_LOGI(TAG, "Scanning I2C bus...");

    int devices_found = 0;

    for (uint8_t address = 1; address < 127; address++)
    {
        esp_err_t result = i2c_master_probe(
            i2c_bus,
            address,
            50);

        if (result == ESP_OK)
        {
            ESP_LOGI(
                TAG,
                "I2C device found at 0x%02X",
                address);

            devices_found++;
        }
    }

    if (devices_found == 0)
    {
        ESP_LOGW(TAG, "No I2C devices found");
    }
    else
    {
        ESP_LOGI(
            TAG,
            "I2C scan complete: %d device(s) found",
            devices_found);
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "RoboLink starting");
    ESP_LOGI(TAG, "Native ESP-IDF firmware");

    init_led();

    init_i2c();
    scan_i2c();

    ESP_ERROR_CHECK(oled_init(i2c_bus));
    ESP_ERROR_CHECK(oled_clear());
    ESP_ERROR_CHECK(oled_write_robolink());

    ESP_ERROR_CHECK(packet_queue_init());

    ESP_ERROR_CHECK(robolink_wifi_init());
    ESP_ERROR_CHECK(robolink_ota_start());
    ESP_ERROR_CHECK(udp_start());

    ESP_ERROR_CHECK(lora_spi_init());
    ESP_ERROR_CHECK(lora_check_radio());
    ESP_ERROR_CHECK(lora_configure());
    ESP_ERROR_CHECK(lora_start_receive());

    RoboPacket packet = {};

    bool led_active = false;
    TickType_t led_off_time = 0;

    gpio_set_level((gpio_num_t)LED_PIN, 0);

    while (true)
    {
        lora_receive_service();

        while (packet_queue_receive(packet))
        {
            RouterResult route = router_classify(packet);

            if (route.valid)
            {
            if (route.decision == RouterDecision::ALLOW_LORA)
            {
                ESP_LOGI(
                    TAG,
                    "Router: UDP cmd=%d -> LoRa TX",
                    route.command
                );

                esp_err_t tx_result = lora_send(
                    packet.data,
                    packet.length
                );

                if (tx_result == ESP_OK)
                {
                    gpio_set_level(
                        (gpio_num_t)LED_PIN,
                        1
                    );

                    led_active = true;
                    led_off_time =
                        xTaskGetTickCount() + pdMS_TO_TICKS(50);
                }
                else
                {
                    ESP_LOGE(
                        TAG,
                        "Router: LoRa TX failed for cmd=%d: %s",
                        route.command,
                        esp_err_to_name(tx_result)
                    );
                }
            }                else if (route.decision == RouterDecision::ALLOW_UDP)
                {
                    ESP_LOGI(
                        TAG,
                        "Router: LoRa cmd=%d -> UDP ALLOW",
                        route.command);
                }
                else if (route.decision == RouterDecision::DUPLICATE)
                {
                    ESP_LOGI(
                        TAG,
                        "Router: UDP cmd=%d -> DUPLICATE DROP",
                        route.command);
                }
                else if (route.decision == RouterDecision::BLOCK)
                {
                    ESP_LOGW(
                        TAG,
                        "Router: cmd=%d BLOCK",
                        route.command);
                }
            }

            if (packet.source == PacketSource::LORA)
            {
                ESP_LOGI(
                    TAG,
                    "LoRa RX RSSI=%d len=%u: %s",
                    packet.rssi,
                    packet.length,
                    packet.data);

                gpio_set_level(
                    (gpio_num_t)LED_PIN,
                    1);

                led_active = true;
                led_off_time =
                    xTaskGetTickCount() + pdMS_TO_TICKS(50);
            }
        }

        if (led_active)
        {
            TickType_t now = xTaskGetTickCount();

            if ((int32_t)(now - led_off_time) >= 0)
            {
                gpio_set_level(
                    (gpio_num_t)LED_PIN,
                    0);

                led_active = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}