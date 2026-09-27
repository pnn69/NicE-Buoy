#include <cstring>

#include "wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"

#include "robo_secrets.h"

static const char *TAG = "WiFi";

static constexpr EventBits_t WIFI_CONNECTED_BIT = BIT0;
static constexpr EventBits_t WIFI_FAILED_BIT = BIT1;
static constexpr int CONNECT_TIMEOUT_MS = 5000;

static EventGroupHandle_t wifi_event_group = nullptr;

static RoboLinkWifiMode current_mode =
    RoboLinkWifiMode::DISCONNECTED;

static bool ap_latched = false;

static void wifi_event_handler(
    void *arg,
    esp_event_base_t event_base,
    int32_t event_id,
    void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        if (event_id == WIFI_EVENT_STA_DISCONNECTED)
        {
            if (!ap_latched)
            {
                xEventGroupSetBits(
                    wifi_event_group,
                    WIFI_FAILED_BIT);
            }
        }
    }

    if (event_base == IP_EVENT &&
        event_id == IP_EVENT_STA_GOT_IP)
    {
        const ip_event_got_ip_t *event =
            static_cast<const ip_event_got_ip_t *>(event_data);

        ESP_LOGI(
            TAG,
            "Station IP: " IPSTR,
            IP2STR(&event->ip_info.ip));

        xEventGroupSetBits(
            wifi_event_group,
            WIFI_CONNECTED_BIT);
    }
}

static bool try_station(
    const char *ssid,
    const char *password,
    RoboLinkWifiMode success_mode)
{
    ESP_LOGI(TAG, "Trying Wi-Fi STA '%s'", ssid);

    xEventGroupClearBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAILED_BIT);

    wifi_config_t config = {};

    std::strncpy(
        reinterpret_cast<char *>(config.sta.ssid),
        ssid,
        sizeof(config.sta.ssid) - 1);

    std::strncpy(
        reinterpret_cast<char *>(config.sta.password),
        password,
        sizeof(config.sta.password) - 1);

    config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    config.sta.pmf_cfg.capable = true;
    config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(
        esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(
        esp_wifi_set_config(
            WIFI_IF_STA,
            &config));

    ESP_ERROR_CHECK(
        esp_wifi_connect());

    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAILED_BIT,
        pdTRUE,
        pdFALSE,
        pdMS_TO_TICKS(CONNECT_TIMEOUT_MS));

    if ((bits & WIFI_CONNECTED_BIT) != 0)
    {
        current_mode = success_mode;

        ESP_LOGI(
            TAG,
            "Connected to '%s'",
            ssid);

        return true;
    }

    ESP_LOGW(
        TAG,
        "Could not connect to '%s'",
        ssid);

    esp_wifi_disconnect();

    return false;
}

static esp_err_t start_field_ap()
{
    ESP_LOGI(TAG, "No existing Wi-Fi network found");

    ESP_LOGI(
        TAG,
        "Starting fallback AP '%s'",
        ROBOLINK_WIFI_SSID);

    wifi_config_t config = {};

    std::strncpy(
        reinterpret_cast<char *>(config.ap.ssid),
        ROBOLINK_WIFI_SSID,
        sizeof(config.ap.ssid) - 1);

    config.ap.ssid_len =
        std::strlen(ROBOLINK_WIFI_SSID);

    std::strncpy(
        reinterpret_cast<char *>(config.ap.password),
        ROBOLINK_WIFI_PASS,
        sizeof(config.ap.password) - 1);

    config.ap.channel = 1;
    config.ap.max_connection = 8;
    config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    config.ap.pmf_cfg.required = false;

    ESP_ERROR_CHECK(
        esp_wifi_set_mode(WIFI_MODE_AP));

    ESP_ERROR_CHECK(
        esp_wifi_set_config(
            WIFI_IF_AP,
            &config));

    ap_latched = true;
    current_mode = RoboLinkWifiMode::AP_FIELD;

    ESP_LOGI(
        TAG,
        "RoboLink now owns '%s'",
        ROBOLINK_WIFI_SSID);

    ESP_LOGI(
        TAG,
        "AP remains active until reboot");

    return ESP_OK;
}

esp_err_t robolink_wifi_init()
{
    ESP_LOGI(TAG, "Initializing Wi-Fi");

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(
            nvs_flash_erase());

        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(
        esp_netif_init());

    esp_err_t event_result =
        esp_event_loop_create_default();

    if (event_result != ESP_OK &&
        event_result != ESP_ERR_INVALID_STATE)
    {
        return event_result;
    }

    wifi_event_group = xEventGroupCreate();

    if (wifi_event_group == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t init_config =
        WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(
        esp_wifi_init(&init_config));

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            wifi_event_handler,
            nullptr));

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            wifi_event_handler,
            nullptr));

    ESP_ERROR_CHECK(
        esp_wifi_set_storage(WIFI_STORAGE_RAM));

    ESP_ERROR_CHECK(
        esp_wifi_start());

    if (try_station(
            HOMELINK_WIFI_SSID,
            HOMELINK_WIFI_PASS,
            RoboLinkWifiMode::STA_HOME))
    {
        return ESP_OK;
    }

    vTaskDelay(
        pdMS_TO_TICKS(250));

    if (try_station(
            ROBOLINK_WIFI_SSID,
            ROBOLINK_WIFI_PASS,
            RoboLinkWifiMode::STA_FIELD))
    {
        return ESP_OK;
    }

    vTaskDelay(
        pdMS_TO_TICKS(250));

    return start_field_ap();
}

bool robolink_wifi_connected()
{
    return current_mode == RoboLinkWifiMode::STA_HOME ||
           current_mode == RoboLinkWifiMode::STA_FIELD ||
           current_mode == RoboLinkWifiMode::AP_FIELD;
}

RoboLinkWifiMode robolink_wifi_mode()
{
    return current_mode;
}