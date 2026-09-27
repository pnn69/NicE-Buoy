#include "packet_queue.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"

static const char *TAG = "PacketQueue";

static constexpr UBaseType_t PACKET_QUEUE_LENGTH = 10;

static QueueHandle_t packet_queue = nullptr;

esp_err_t packet_queue_init()
{
    packet_queue = xQueueCreate(
        PACKET_QUEUE_LENGTH,
        sizeof(RoboPacket));

    if (packet_queue == nullptr)
    {
        ESP_LOGE(TAG, "Failed to create packet queue");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(
        TAG,
        "Packet queue created: %u entries",
        (unsigned int)PACKET_QUEUE_LENGTH);

    return ESP_OK;
}

bool packet_queue_send(const RoboPacket &packet)
{
    if (packet_queue == nullptr)
    {
        return false;
    }

    if (xQueueSend(packet_queue, &packet, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "Packet queue full");
        return false;
    }

    return true;
}

bool packet_queue_receive(RoboPacket &packet)
{
    if (packet_queue == nullptr)
    {
        return false;
    }

    return xQueueReceive(
               packet_queue,
               &packet,
               0) == pdTRUE;
}