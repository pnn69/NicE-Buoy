#ifndef COMMANDS_H_
#define COMMANDS_H_

#define COLOR_PRINT_BLACK "30"
#define COLOR_PRINT_RED "31"
#define COLOR_PRINT_GREEN "32"
#define COLOR_PRINT_BROWN "33"
#define COLOR_PRINT_BLUE "34"
#define COLOR_PRINT_PURPLE "35"
#define COLOR_PRINT_CYAN "36"
#define color_printf(COLOR, format, ...)                               \
    {                                                                  \
        printf("\033[0;" COLOR "m" format "\033[0m\n", ##__VA_ARGS__); \
    }

/*
    [Designator][Reciever][MSG_ID][lengt_msg][msg]
*/

typedef enum
{
    POSITION = 1,
    BATTERY_VOLTAGE,
    BUOY_MODE,
    TARGET_POSITION,
    GOTO_TARGET_POSITION,
    ANCHOR_POSITION,
    GOTO_ANGHOR_POSITION,
    ANCHOR_POSITION_AS_TARGET_POSITION,
    DOC_POSITION,
    NO_POSITION,
    GOTO_DOC_POSITION,
    CURREND_POSITION_AS_DOC_POSITION,
    BUOY_ID,
    DIR_DISTANSE_ANCHOR_POSITION,
    CURREND_POSITION_AS_ANCHOR_POSITION,
    DIR_DISTANSE_TO_TARGET_POSITION,
    DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_TARGET_POSITION,
    SAIL_DIR_SPEED,
    SBPWR_BBPWR,
    BUOY_MODE_IDLE,
    SYSTEM_STASTUS,
    DGPS, // deltaLattitude, deltaLongitude
    DGPSPOSITION,
    GPS_LAT_LON_FIX_HEADING_SPEED_MHEADING, // lat,lon,fix,heading,speed,m_heading
    TXT,
    RESET,
    BATTERY_VOLTAGE_PERCENTAGE
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
    ACK,
    NACK,
    INF, // info no action
    CALIBRATE_OFFSET_MAGNETIC_COMPASS

} Status_t;

#endif /* COMMANDS_H_ */