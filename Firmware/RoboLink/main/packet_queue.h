#ifndef PACKET_QUEUE_H_
#define PACKET_QUEUE_H_

#include <stdint.h>

#include "esp_err.h"

enum class PacketSource : uint8_t
{
    LORA = 0,
    UDP,
    MESH,
    SERIAL
};

struct RoboPacket
{
    PacketSource source;
    uint16_t length;
    int16_t rssi;
    char data[160];
};

esp_err_t packet_queue_init();
bool packet_queue_send(const RoboPacket &packet);
bool packet_queue_receive(RoboPacket &packet);

#endif /* PACKET_QUEUE_H_ */