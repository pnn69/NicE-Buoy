#include "ota.h"

#include <cstdio>
#include <cstdint>

#include "esp_app_desc.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "OTA";

static httpd_handle_t ota_server = nullptr;

// -----------------------------------------------------------------------------
// GET /ota
// -----------------------------------------------------------------------------

static esp_err_t ota_status_handler(httpd_req_t *req)
{
    const esp_partition_t *running =
        esp_ota_get_running_partition();

    const esp_app_desc_t *app =
        esp_app_get_description();

    char response[256];

    snprintf(
        response,
        sizeof(response),
        "RoboLink OTA ready\n"
        "Running partition: %s\n"
        "Version: %s\n",
        running ? running->label : "unknown",
        app ? app->version : "unknown");

    httpd_resp_set_type(
        req,
        "text/plain");

    return httpd_resp_sendstr(
        req,
        response);
}

// -----------------------------------------------------------------------------
// Delayed reboot
// -----------------------------------------------------------------------------

static void ota_reboot_task(void *arg)
{
    ESP_LOGI(
        TAG,
        "Rebooting into new firmware");

    vTaskDelay(
        pdMS_TO_TICKS(1000));

    esp_restart();
}

// -----------------------------------------------------------------------------
// POST /ota
// -----------------------------------------------------------------------------

static esp_err_t ota_upload_handler(httpd_req_t *req)
{
    if (req->content_len <= 0)
    {
        httpd_resp_send_err(
            req,
            HTTPD_400_BAD_REQUEST,
            "Empty firmware image");

        return ESP_FAIL;
    }

    const esp_partition_t *running =
        esp_ota_get_running_partition();

    const esp_partition_t *update =
        esp_ota_get_next_update_partition(nullptr);

    if (update == nullptr)
    {
        ESP_LOGE(
            TAG,
            "No inactive OTA partition available");

        httpd_resp_send_err(
            req,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "No OTA partition available");

        return ESP_FAIL;
    }

    ESP_LOGI(
        TAG,
        "Running partition: %s",
        running ? running->label : "unknown");

    ESP_LOGI(
        TAG,
        "Update partition: %s",
        update->label);

    ESP_LOGI(
        TAG,
        "Firmware upload size: %d bytes",
        req->content_len);

    if (static_cast<size_t>(req->content_len) >
        update->size)
    {
        ESP_LOGE(
            TAG,
            "Firmware image too large: %d bytes, partition size: %lu",
            req->content_len,
            static_cast<unsigned long>(update->size));

        httpd_resp_send_err(
            req,
            HTTPD_400_BAD_REQUEST,
            "Firmware image too large");

        return ESP_FAIL;
    }

    esp_ota_handle_t ota_handle = 0;

    esp_err_t result =
        esp_ota_begin(
            update,
            OTA_WITH_SEQUENTIAL_WRITES,
            &ota_handle);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "esp_ota_begin failed: %s",
            esp_err_to_name(result));

        httpd_resp_send_err(
            req,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "OTA begin failed");

        return result;
    }

    uint8_t buffer[4096];

    int remaining = req->content_len;
    int written = 0;
    int next_progress = 128 * 1024;

    while (remaining > 0)
    {
        int wanted = remaining;

        if (wanted >
            static_cast<int>(sizeof(buffer)))
        {
            wanted =
                static_cast<int>(sizeof(buffer));
        }

        int received =
            httpd_req_recv(
                req,
                reinterpret_cast<char *>(buffer),
                wanted);

        if (received == HTTPD_SOCK_ERR_TIMEOUT)
        {
            continue;
        }

        if (received <= 0)
        {
            ESP_LOGE(
                TAG,
                "Firmware receive failed");

            esp_ota_abort(
                ota_handle);

            httpd_resp_send_err(
                req,
                HTTPD_500_INTERNAL_SERVER_ERROR,
                "Firmware receive failed");

            return ESP_FAIL;
        }

        result =
            esp_ota_write(
                ota_handle,
                buffer,
                received);

        if (result != ESP_OK)
        {
            ESP_LOGE(
                TAG,
                "esp_ota_write failed: %s",
                esp_err_to_name(result));

            esp_ota_abort(
                ota_handle);

            httpd_resp_send_err(
                req,
                HTTPD_500_INTERNAL_SERVER_ERROR,
                "Firmware write failed");

            return result;
        }

        written += received;
        remaining -= received;

        if (written >= next_progress ||
            remaining == 0)
        {
            int percent =
                static_cast<int>(
                    (static_cast<int64_t>(written) * 100) /
                    req->content_len);

            ESP_LOGI(
                TAG,
                "OTA progress: %d%% (%d / %d bytes)",
                percent,
                written,
                req->content_len);

            while (next_progress <= written)
            {
                next_progress +=
                    128 * 1024;
            }
        }
    }

    result =
        esp_ota_end(
            ota_handle);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "Firmware validation failed: %s",
            esp_err_to_name(result));

        httpd_resp_send_err(
            req,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "Firmware validation failed");

        return result;
    }

    result =
        esp_ota_set_boot_partition(
            update);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "esp_ota_set_boot_partition failed: %s",
            esp_err_to_name(result));

        httpd_resp_send_err(
            req,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "Could not select new firmware");

        return result;
    }

    ESP_LOGI(
        TAG,
        "OTA successful");

    ESP_LOGI(
        TAG,
        "Next boot partition: %s",
        update->label);

    httpd_resp_set_type(
        req,
        "text/plain");

    esp_err_t response_result =
        httpd_resp_sendstr(
            req,
            "OTA OK - RoboLink rebooting\n");

    if (response_result != ESP_OK)
    {
        ESP_LOGW(
            TAG,
            "Could not send OTA completion response: %s",
            esp_err_to_name(response_result));
    }

    BaseType_t task_result =
        xTaskCreate(
            ota_reboot_task,
            "ota_reboot",
            2048,
            nullptr,
            5,
            nullptr);

    if (task_result != pdPASS)
    {
        ESP_LOGW(
            TAG,
            "Could not create reboot task");

        vTaskDelay(
            pdMS_TO_TICKS(1000));

        esp_restart();
    }

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// Start HTTP OTA service
// -----------------------------------------------------------------------------

esp_err_t robolink_ota_start()
{
    if (ota_server != nullptr)
    {
        return ESP_OK;
    }

    httpd_config_t config =
        HTTPD_DEFAULT_CONFIG();

    config.server_port = 80;
    config.stack_size = 8192;

    esp_err_t result =
        httpd_start(
            &ota_server,
            &config);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "HTTP server start failed: %s",
            esp_err_to_name(result));

        ota_server = nullptr;

        return result;
    }

    httpd_uri_t status_uri = {};

    status_uri.uri = "/ota";
    status_uri.method = HTTP_GET;
    status_uri.handler = ota_status_handler;
    status_uri.user_ctx = nullptr;

    result =
        httpd_register_uri_handler(
            ota_server,
            &status_uri);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "GET /ota registration failed: %s",
            esp_err_to_name(result));

        httpd_stop(
            ota_server);

        ota_server = nullptr;

        return result;
    }

    httpd_uri_t upload_uri = {};

    upload_uri.uri = "/ota";
    upload_uri.method = HTTP_POST;
    upload_uri.handler = ota_upload_handler;
    upload_uri.user_ctx = nullptr;

    result =
        httpd_register_uri_handler(
            ota_server,
            &upload_uri);

    if (result != ESP_OK)
    {
        ESP_LOGE(
            TAG,
            "POST /ota registration failed: %s",
            esp_err_to_name(result));

        httpd_stop(
            ota_server);

        ota_server = nullptr;

        return result;
    }

    ESP_LOGI(
        TAG,
        "Wi-Fi OTA server ready on port 80");

    ESP_LOGI(
        TAG,
        "GET /ota = status");

    ESP_LOGI(
        TAG,
        "POST /ota = firmware upload");

    return ESP_OK;
}