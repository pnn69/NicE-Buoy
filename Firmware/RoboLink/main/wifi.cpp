#include <cstring>
#include <cstdlib>

#include "wifi.h"
#include "esp_bridge.h"
#include "esp_mesh_lite.h"
#include "esp_mesh_lite_core.h"
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
static constexpr EventBits_t WIFI_SCAN_DONE_BIT = BIT2;

static EventGroupHandle_t wifi_event_group = nullptr;

static RoboLinkWifiMode scan_selected_mode =
    RoboLinkWifiMode::DISCONNECTED;

static volatile bool app_scan_active = false;
static volatile bool app_scan_ready = false;

static RoboLinkWifiMode current_mode =
    RoboLinkWifiMode::DISCONNECTED;

static bool ap_latched = false;

static void mesh_scan_start_cb()
{
    if (app_scan_active)
    {
        app_scan_ready = true;
    }
}

static void mesh_scan_end_cb()
{
    if (app_scan_active)
    {
        app_scan_active = false;
        app_scan_ready = false;
    }
}

static esp_mesh_lite_scan_cb_t mesh_scan_callbacks = {
    .scan_start_cb = mesh_scan_start_cb,
    .scan_end_cb = mesh_scan_end_cb,
};

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

        


        if (event_id == WIFI_EVENT_SCAN_DONE)
        {
            if (!app_scan_ready)
            {
                return;
            }

            uint16_t ap_count = 0;

            if (esp_wifi_scan_get_ap_num(&ap_count) != ESP_OK ||
                ap_count == 0)
            {
                scan_selected_mode =
                    RoboLinkWifiMode::DISCONNECTED;

                xEventGroupSetBits(
                    wifi_event_group,
                    WIFI_SCAN_DONE_BIT);

                return;
            }

            wifi_ap_record_t *records =
                static_cast<wifi_ap_record_t *>(
                    calloc(
                        ap_count,
                        sizeof(wifi_ap_record_t)));

            if (records == nullptr)
            {
                scan_selected_mode =
                    RoboLinkWifiMode::DISCONNECTED;

                xEventGroupSetBits(
                    wifi_event_group,
                    WIFI_SCAN_DONE_BIT);

                return;
            }

            uint16_t record_count = ap_count;

            esp_err_t result =
                esp_wifi_scan_get_ap_records(
                    &record_count,
                    records);

            bool found_home = false;
            bool found_field = false;

            if (result == ESP_OK)
            {
                for (uint16_t i = 0;
                    i < record_count;
                    i++)
                {
                    const char *ssid =
                        reinterpret_cast<const char *>(
                            records[i].ssid);

                    if (std::strcmp(
                            ssid,
                            HOMELINK_WIFI_SSID) == 0)
                    {
                        found_home = true;
                    }

                    if (std::strcmp(
                            ssid,
                            ROBOLINK_WIFI_SSID) == 0)
                    {
                        found_field = true;
                    }
                }
            }

            free(records);

            if (found_home)
            {
                scan_selected_mode =
                    RoboLinkWifiMode::STA_HOME;
            }
            else if (found_field)
            {
                scan_selected_mode =
                    RoboLinkWifiMode::STA_FIELD;
            }
            else
            {
                scan_selected_mode =
                    RoboLinkWifiMode::DISCONNECTED;
            }

            xEventGroupSetBits(
                wifi_event_group,
                WIFI_SCAN_DONE_BIT);
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

static esp_err_t configure_upstream(
    const char *ssid,
    const char *password,
    RoboLinkWifiMode mode)
{
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
        esp_wifi_set_mode(WIFI_MODE_APSTA));

    esp_err_t result =
        esp_bridge_wifi_set_config(
            WIFI_IF_STA,
            &config);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to configure upstream '%s'",
            ssid);

        return result;
    }

    current_mode = mode;

    ESP_LOGI(
        TAG,
        "Mesh-Lite upstream configured: %s",
        ssid);

    return ESP_OK;
}

static esp_err_t select_upstream()
{
    scan_selected_mode =
        RoboLinkWifiMode::DISCONNECTED;

    xEventGroupClearBits(
        wifi_event_group,
        WIFI_SCAN_DONE_BIT);

    app_scan_active = true;
    app_scan_ready = false;

    ESP_ERROR_CHECK(
        esp_mesh_lite_scan_cb_register(
            &mesh_scan_callbacks));

    ESP_LOGI(
        TAG,
        "Starting Mesh-Lite coordinated upstream scan");

    esp_err_t result =
        esp_mesh_lite_wifi_scan_start(
            nullptr,
            pdMS_TO_TICKS(3000));

    if (result != ESP_OK)
    {
        app_scan_active = false;
        app_scan_ready = false;

        ESP_LOGW(
            TAG,
            "Mesh-Lite scan start failed: %s",
            esp_err_to_name(result));

        return result;
    }

    EventBits_t bits =
        xEventGroupWaitBits(
            wifi_event_group,
            WIFI_SCAN_DONE_BIT,
            pdTRUE,
            pdFALSE,
            pdMS_TO_TICKS(5000));

    if ((bits & WIFI_SCAN_DONE_BIT) == 0)
    {
        app_scan_active = false;
        app_scan_ready = false;

        ESP_LOGW(
            TAG,
            "Mesh-Lite upstream scan timed out");

        return ESP_ERR_TIMEOUT;
    }

    if (scan_selected_mode ==
        RoboLinkWifiMode::STA_HOME)
    {
        ESP_LOGI(
            TAG,
            "Preferred upstream found: %s",
            HOMELINK_WIFI_SSID);

        return configure_upstream(
            HOMELINK_WIFI_SSID,
            HOMELINK_WIFI_PASS,
            RoboLinkWifiMode::STA_HOME);
    }

    if (scan_selected_mode ==
        RoboLinkWifiMode::STA_FIELD)
    {
        ESP_LOGI(
            TAG,
            "Field upstream found: %s",
            ROBOLINK_WIFI_SSID);

        return configure_upstream(
            ROBOLINK_WIFI_SSID,
            ROBOLINK_WIFI_PASS,
            RoboLinkWifiMode::STA_FIELD);
    }

    ESP_LOGW(
        TAG,
        "No configured upstream Wi-Fi found");

    current_mode =
        RoboLinkWifiMode::AP_FIELD;

    return ESP_OK;
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

static esp_err_t configure_mesh_softap()
{
    wifi_config_t config = {};

    std::strncpy(
        reinterpret_cast<char *>(config.ap.ssid),
        ROBOMESH_WIFI_SSID,
        sizeof(config.ap.ssid) - 1);

    config.ap.ssid_len =
        std::strlen(ROBOMESH_WIFI_SSID);

    std::strncpy(
        reinterpret_cast<char *>(config.ap.password),
        ROBOMESH_WIFI_PASS,
        sizeof(config.ap.password) - 1);

    config.ap.channel = 1;
    config.ap.max_connection = 8;
    config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    config.ap.pmf_cfg.required = false;

    esp_err_t result =
        esp_bridge_wifi_set_config(
            WIFI_IF_AP,
            &config);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Failed to configure Mesh-Lite SoftAP");
        return result;
    }

    esp_mesh_lite_set_softap_info(
        ROBOMESH_WIFI_SSID,
        ROBOMESH_WIFI_PASS);

    ESP_LOGI(
        TAG,
        "Mesh-Lite SoftAP configured: %s",
        ROBOMESH_WIFI_SSID);

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

    esp_bridge_create_all_netif();

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

    esp_mesh_lite_config_t mesh_config =
        ESP_MESH_LITE_DEFAULT_INIT();

    esp_mesh_lite_init(&mesh_config);

    ESP_LOGI(
        TAG,
        "Mesh-Lite initialized");

    ESP_ERROR_CHECK(
        configure_mesh_softap());

    esp_err_t upstream_result =
        select_upstream();

    if (upstream_result != ESP_OK)
    {
        ESP_LOGW(
            TAG,
            "Upstream selection failed: %s",
            esp_err_to_name(upstream_result)
        );

        current_mode =
            RoboLinkWifiMode::AP_FIELD;
    }

    if (current_mode == RoboLinkWifiMode::STA_HOME ||
        current_mode == RoboLinkWifiMode::STA_FIELD)
    {
        ESP_LOGI(
            TAG,
            "Starting Mesh-Lite with upstream"
        );
    }
    else
    {
        ESP_LOGI(
            TAG,
            "Starting Mesh-Lite without upstream"
        );
    }
    esp_mesh_lite_start();

    ESP_LOGI(
        TAG,
        "Mesh-Lite started"
    );

    return ESP_OK;
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