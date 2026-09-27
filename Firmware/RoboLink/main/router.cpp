#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "router.h"
#include "robo_protocol.h"

#include <cstdlib>
#include <cstring>

static int extract_command(const char *data)
{
    if (data == nullptr)
    {
        return NOCMD;
    }

    if (data[0] != '$')
    {
        return NOCMD;
    }

    char buffer[160];

    std::strncpy(buffer, data, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    char *save_ptr = nullptr;
    char *field = strtok_r(buffer, ",", &save_ptr);

    int field_index = 0;

    while (field != nullptr)
    {
        if (field_index == 3)
        {
            return std::atoi(field);
        }

        field = strtok_r(nullptr, ",", &save_ptr);
        field_index++;
    }

    return NOCMD;
}

static constexpr int DEDUP_SLOTS = 16;
static constexpr TickType_t DEDUP_TIME = pdMS_TO_TICKS(30000);

struct DedupEntry
{
    uint32_t hash;
    TickType_t timestamp;
    bool used;
};

static DedupEntry dedup_table[DEDUP_SLOTS] = {};

static uint32_t packet_hash(const char *data)
{
    uint32_t hash = 2166136261u;

    while (*data != '\0')
    {
        hash ^= static_cast<uint8_t>(*data);
        hash *= 16777619u;
        data++;
    }

    return hash;
}

static bool packet_is_duplicate(const char *data)
{
    uint32_t hash = packet_hash(data);
    TickType_t now = xTaskGetTickCount();

    int oldest_slot = 0;
    TickType_t oldest_age = 0;

    for (int i = 0; i < DEDUP_SLOTS; i++)
    {
        if (dedup_table[i].used)
        {
            TickType_t age =
                now - dedup_table[i].timestamp;

            if (dedup_table[i].hash == hash &&
                age < DEDUP_TIME)
            {
                return true;
            }

            if (age > oldest_age)
            {
                oldest_age = age;
                oldest_slot = i;
            }
        }
        else
        {
            oldest_slot = i;
            oldest_age = DEDUP_TIME;
            break;
        }
    }

    dedup_table[oldest_slot].hash = hash;
    dedup_table[oldest_slot].timestamp = now;
    dedup_table[oldest_slot].used = true;

    return false;
}

static bool udp_to_lora_allowed(int command)
{
    switch (command)
    {
    case IDLE:
    case LOCKING:
    case DOCKING:
    case UNLOCK:
    case REMOTE:
    case ROUTETOPOINT:
    case SETLOCKPOS:
    case SETDOCKPOS:
    case WAKEUP:
    case REBOOT:
    case STOREASDOC:
    case COMPUTESTART:
    case EXTENDSTART:
    case SHORTENSTART:
    case CLEAN_THRUSTERS:
    case SET_AS_NORTH:
    case SET_AS_LEVEL:
        return true;

    default:
        return false;
    }
}
static bool lora_to_udp_allowed(int command)
{
    if (command == LORA_LINK)
    {
        return false;
    }

    return false;
}

RouterResult router_classify(const RoboPacket &packet)
{
    RouterResult result = {};

    result.decision = RouterDecision::BLOCK;
    result.command = NOCMD;
    result.valid = false;

    int command = extract_command(packet.data);

    result.command = command;

    if (command == NOCMD)
    {
        return result;
    }

    result.valid = true;

    if (packet.source == PacketSource::UDP)
    {
        if (!udp_to_lora_allowed(command))
        {
            result.decision = RouterDecision::LOCAL_ONLY;
            return result;
        }

        if (packet_is_duplicate(packet.data))
        {
            result.decision = RouterDecision::DUPLICATE;
            return result;
        }

        result.decision = RouterDecision::ALLOW_LORA;
        return result;
    }

    if (packet.source == PacketSource::LORA)
    {
        if (lora_to_udp_allowed(command))
        {
            result.decision = RouterDecision::ALLOW_UDP;
        }
        else
        {
            result.decision = RouterDecision::LOCAL_ONLY;
        }

        return result;
    }

    result.decision = RouterDecision::LOCAL_ONLY;

    return result;
}