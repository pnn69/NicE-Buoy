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
    [Designator][Reciever][MSG_ID][STATUS][ACK][msg]
*/

typedef enum
{
    POSITION = 1,                                  //
    BATTERY_VOLTAGE,                               //
    BUOY_MODE,                                     //
    TARGET_POSITION,                               //
    GOTO_TARGET_POSITION,                          //
    ANCHOR_POSITION,                               //
    GOTO_ANGHOR_POSITION,                          //
    ANCHOR_POSITION_AS_TARGET_POSITION,            //
    NO_POSITION,                                   //
    DOC_POSITION,                                  //
    STORE_DOC_POSITION,                            //
    STORE_POS_AS_DOC_POSITION,                     //
    CURREND_POSITION_AS_DOC_POSITION,              //
    BUOY_ID,                                       //
    DIR_DISTANSE_ANCHOR_POSITION,                  //
    CURREND_POSITION_AS_ANCHOR_POSITION,           //
    DIR_DISTANSE_TO_TARGET_POSITION,               //
    DIR_DISTANSE_SPEED_BBSPPEED_SBSPEED_M_HEADING, // When locked use this
    SAIL_DIR_SPEED,                                // heading,0-100%
    SBPWR_BBPWR,                                   // -100/100sb,-100/100bb
    BUOY_MODE_IDLE,                                // ?
    SYSTEM_STASTUS,                                // ?
    GPS_LAT_LON_NRSAT_FIX_HEADING_SPEED_MHEADING,  // lat,lon,fix,heading,speed,m_heading
    BATTERY_VOLTAGE_PERCENTAGE,                    // 0.0V, %
    TXT,                                           // for text transistons
    GPS_DUMMY,                                     // 1 enable, 0 disable
    GPS_DUMMY_DELTA_LAT_LON,                       // lat(double)/lon(double)
    COMPUTE_PARAMETERS,                            // (int)buoyMinOffsetDistance, (int)buoyMaxOffsetDistance, (int)buoyMinSpeed, (int)maxCorrectionPeedPercentage
    PID_SPEED_PARAMETERS,                          // kp=20, ki=0.4, kd=0;
    PID_RUDDER_PARAMETERS,                         // kp=.5, ki=0.02, kd=0;
    CHANGE_LOCK_POS_DIR_DIST,                      // adjust position retative to currend magnetic heading dir in degrees distance in meters
    WIND_DIR_DEV,                                  // wind direction deviation
    ESC_ON_OFF,                                    // enable disable esc (defaut enabled)
    MAGNETIC_HEADING,                              // magnetic heading
    LINEAR_CALIBRATE_MAGNETIC_COMPASS,             // calibrate compas with gps as base
    COMPASS_OFSET,                                 // offset correcton
    MECANICAL_OFSET,                               // mecanical offset
    CHANGE_LOCK_POS_DIR_DIST_ABS,                  // Absolute new angel to sail to
    RESET                                          // reset esp
} Command_t;

typedef enum
{
    GET = 0,
    SET,
    ACK,
    NAK,
    INF, // info no action
    IDLE,
    LOCKING,
    LOCKED,
    LOCK_POS,
    LOCK_ANCHOR_POS,
    UNLOCK,
    REMOTE,
    DOCKING,
    DOCKED,
    DOC,
    REMOTEING,
    CALIBRATE_MAGNETIC_COMPASS,
    LINEAR_CAL,
    CALIBRATE_OFFSET_MAGNETIC_COMPASS,
    STORE_CALIBRATE_OFFSET_MAGNETIC_COMPASS,
    DOCK_STORING,
    MUTE_ESC,
    CLEANING_THRUSTERS,
    LOW_BAT

} Status_t;

#endif /* COMMANDS_H_ */