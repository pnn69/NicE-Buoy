#ifndef ROBOBOBUOYMSG_H_
#define ROBOBOBUOYMSG_H_

/*
    [Designator][Reciever][MSG_ID][STATUS][ACK][msg]
    if Designator == 0xFF relay command with buoy ID
*/


typedef enum
{
    BUOY_ID = 1,                                       //
    POSITION,                                  //
    NO_POSITION,                                   //
    BATTERY_VOLTAGE,                               //
    BUOY_MODE,                                     //
    TARGET_POSITION,                               //
    GOTO_TARGET_POSITION,                          //
    DOC_POSITION,                                  //
    STORE_DOC_POSITION,                            //
    STORE_POS_AS_DOC_POSITION,                     //
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
    DOCKING,
    DOCKED,
    DOC,
    UNLOCK,
    REMOTE,
    REMOTEING,
    CALIBRATE_MAGNETIC_COMPASS,
    LINEAR_CALLIBRATING,
    CALIBRATE_OFFSET_MAGNETIC_COMPASS,
    STORE_CALIBRATE_OFFSET_MAGNETIC_COMPASS,
    DOCK_STORING,
    MUTE_ESC

} Status_t;


#endif /* ROBOBOBUOYMSG_H_ */
