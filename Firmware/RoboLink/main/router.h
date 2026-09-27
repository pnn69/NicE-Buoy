#ifndef ROUTER_H_
#define ROUTER_H_

#include <stdint.h>

#include "packet_queue.h"

enum class RouterDecision : uint8_t
{
    LOCAL_ONLY = 0,
    ALLOW_LORA,
    ALLOW_UDP,
    DUPLICATE,
    BLOCK
};

struct RouterResult
{
    RouterDecision decision;
    int command;
    bool valid;
};

RouterResult router_classify(const RoboPacket &packet);

#endif /* ROUTER_H_ */