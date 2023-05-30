#ifndef GGENERAL_H_
#define GENERAL_H_
#include "../../dependency/command.h"

typedef enum
{
    IDLE = 1,
    LOCK,
    LOCK_POS,
    LOCK_ANCHOR_POS,
    UNLOCK,
    SAIL
} Status_t;

extern char buoyID;

#endif /* GENERA:_H_ */
