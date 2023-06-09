#ifndef COMMANDS_H_
#define COMMANDS_H_

/*
    [Designator][Reciever][MSG_ID][lengt_msg][msg]
*/

typedef enum
{
    POSITION = 1,
    BATTERY_LEVEL,
    BUOY_MODE,
    TARGET_POSITION,
    GOTO_TARGET_POSITION,
    ANCHOR_POSITION,
    GOTO_ANGHOR_POSITION,
    DOC_POSITION,
    GOTO_DOC_POSITION,
    BUOY_ID,
    DIR_DISTANSE_ANCHOR_POSITION,
    ANCHOR_POSITION_AS_TARGET_POSITION,
    CURREND_POSITION_AS_ANCHOR_POSITION,
    DIR_DISTANSE_TO_TARGET_POSITION,
    DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION,
    SAIL_DIR_SPEED,
    SBPWR_BBPWR,
    BUOY_MODE_IDLE,
    SYSTEM_STASTUS,
    DGPS, //deltaLattitude, deltaLongitude
    DGPSPOSITION,
    TXT,
    RESET

} Command_t;

typedef enum
{
    IDLE = 1,
    LOCKING,
    LOCKED,
    LOCK_POS,
    LOCK_ANCHOR_POS,
    UNLOCK,
    REMOTE,
    DOC,
    REMOTEING,
    GET,
    SET,
    ACK
} Status_t;

#endif /* COMMANDS_H_ */