#include <cstring>

#include "udp.h"
#include "packet_queue.h"

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lwip/inet.h"
#include "lwip/sockets.h"

static const char *TAG = "UDP";

static constexpr uint16_t UDP_PORT = 1001;
static constexpr size_t UDP_BUFFER_SIZE = 160;

static void udp_receive_task(void *parameter)
{
    while (true)
    {
        int sock = socket(
            AF_INET,
            SOCK_DGRAM,
            IPPROTO_IP);

        if (sock < 0)
        {
            ESP_LOGE(TAG, "Failed to create UDP socket");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        struct sockaddr_in listen_addr = {};

        listen_addr.sin_family = AF_INET;
        listen_addr.sin_port = htons(UDP_PORT);
        listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);

        int bind_result = bind(
            sock,
            reinterpret_cast<struct sockaddr *>(&listen_addr),
            sizeof(listen_addr));

        if (bind_result < 0)
        {
            ESP_LOGE(
                TAG,
                "Failed to bind UDP port %u",
                UDP_PORT);

            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(
            TAG,
            "Listening on UDP port %u",
            UDP_PORT);

        while (true)
        {
            char buffer[UDP_BUFFER_SIZE] = {};

            struct sockaddr_in source_addr = {};
            socklen_t source_addr_len = sizeof(source_addr);

            int received = recvfrom(
                sock,
                buffer,
                sizeof(buffer) - 1,
                0,
                reinterpret_cast<struct sockaddr *>(&source_addr),
                &source_addr_len);

            if (received < 0)
            {
                ESP_LOGW(TAG, "UDP receive failed");
                break;
            }

            if (received == 0)
            {
                continue;
            }

            buffer[received] = '\0';

            RoboPacket packet = {};

            packet.source = PacketSource::UDP;
            packet.length = static_cast<uint16_t>(received);
            packet.rssi = 0;

            std::memcpy(
                packet.data,
                buffer,
                received + 1);

            if (!packet_queue_send(packet))
            {
                ESP_LOGW(
                    TAG,
                    "UDP packet dropped: packet queue full");

                continue;
            }
        }

        shutdown(sock, 0);
        close(sock);

        ESP_LOGW(
            TAG,
            "UDP socket restarting");

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

esp_err_t udp_start()
{
    BaseType_t result = xTaskCreate(
        udp_receive_task,
        "udp_rx",
        4096,
        nullptr,
        5,
        nullptr);

    if (result != pdPASS)
    {
        ESP_LOGE(
            TAG,
            "Failed to create UDP receive task");

        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(
        TAG,
        "UDP receive task started");

    return ESP_OK;
}