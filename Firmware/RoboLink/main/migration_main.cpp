#include <stdint.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// -----------------------------------------------------------------------------
// RoboLink
//
// Communications-only repeater.
//
// RoboLink must NOT:
//   - control a buoy locally
//   - generate REMOTE / IDLE / LOCK commands
//   - calculate start lines
//   - depend on switches or potentiometers
//
// Those functions belong to RoboTop/RoboSub or RoboCYD.
//
// Future RoboLink transports:
//   PC Serial <-> Robo protocol <-> Wi-Fi / UDP / Mesh / LoRa
// -----------------------------------------------------------------------------

static const char *TAG = "RoboLink";

static constexpr gpio_num_t LED_PIN = GPIO_NUM_25;

// -----------------------------------------------------------------------------
// Status counters
//
// These will later be incremented by the actual transport tasks.
// Keeping them here now gives us a stable diagnostic interface.
// -----------------------------------------------------------------------------

struct RoboLinkStats
{
    uint32_t serial_rx = 0;
    uint32_t serial_tx = 0;

    uint32_t wifi_rx = 0;
    uint32_t wifi_tx = 0;

    uint32_t mesh_rx = 0;
    uint32_t mesh_tx = 0;

    uint32_t lora_rx = 0;
    uint32_t lora_tx = 0;

    uint32_t dropped = 0;
};

static RoboLinkStats stats;

// -----------------------------------------------------------------------------
// Status LED
// -----------------------------------------------------------------------------

static void init_status_led()
{
    gpio_config_t cfg = {};

    cfg.pin_bit_mask = 1ULL << LED_PIN;
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&cfg));

    gpio_set_level(LED_PIN, 0);
}

// -----------------------------------------------------------------------------
// Periodic diagnostic report
// -----------------------------------------------------------------------------

static void report_status()
{
    ESP_LOGI(
        TAG,
        "RX serial=%lu wifi=%lu mesh=%lu lora=%lu | "
        "TX serial=%lu wifi=%lu mesh=%lu lora=%lu | "
        "drop=%lu",
        static_cast<unsigned long>(stats.serial_rx),
        static_cast<unsigned long>(stats.wifi_rx),
        static_cast<unsigned long>(stats.mesh_rx),
        static_cast<unsigned long>(stats.lora_rx),

        static_cast<unsigned long>(stats.serial_tx),
        static_cast<unsigned long>(stats.wifi_tx),
        static_cast<unsigned long>(stats.mesh_tx),
        static_cast<unsigned long>(stats.lora_tx),

        static_cast<unsigned long>(stats.dropped));
}

// -----------------------------------------------------------------------------
// ESP-IDF entry point
// -----------------------------------------------------------------------------

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, " RoboLink native ESP-IDF");
    ESP_LOGI(TAG, " Communications repeater");
    ESP_LOGI(TAG, " ESP-IDF 5.5.5");
    ESP_LOGI(TAG, "====================================");

    init_status_led();

    ESP_LOGI(TAG, "Physical remote controls: DISABLED");
    ESP_LOGI(TAG, "RoboLink application commands: DISABLED");

    // Later these will become:
    //
    // init_pc_serial();
    // init_wifi();
    // init_udp();
    // init_lora();
    // init_mesh();
    //
    // Each transport will place complete Robo packets into one common
    // repeater/router queue.

    bool heartbeat = false;

    int64_t next_heartbeat_us = esp_timer_get_time();
    int64_t next_report_us = esp_timer_get_time();

    for (;;)
    {
        const int64_t now = esp_timer_get_time();

        // -------------------------------------------------------------
        // Heartbeat
        // -------------------------------------------------------------

        if (now >= next_heartbeat_us)
        {
            heartbeat = !heartbeat;

            gpio_set_level(
                LED_PIN,
                heartbeat ? 1 : 0);

            next_heartbeat_us = now + 1000000LL;
        }

        // -------------------------------------------------------------
        // Diagnostic report every 5 seconds
        // -------------------------------------------------------------

        if (now >= next_report_us)
        {
            report_status();

            next_report_us = now + 5000000LL;
        }

        // -------------------------------------------------------------
        // Future repeater service
        //
        // The main application must remain transport-independent.
        //
        // Conceptually:
        //
        //     Serial RX ----+
        //     WiFi RX -------+
        //     Mesh RX --------> repeater/router --> destination(s)
        //     LoRa RX -------+
        //
        // RoboLink should forward packets, not interpret buoy-control
        // semantics except where necessary for routing/deduplication.
        // -------------------------------------------------------------

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}