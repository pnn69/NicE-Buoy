#include "general.h"
#include "LiLlora.h"
/*
    Menu structure
*/
void menu(int inp, int buoy_nr)
{
    switch (inp)
    {
    case GET_DIR_DISTANSE_SPEED_SBSPPEED_BBSPEED_TARGET_POSITION_STATUS:
        sendLoraSetGetPosition(buoy_nr);
        break;
    case SET_TARGET_POSITION:
        sendLoraSetTargetPosition(buoy_nr);
        break;
    case SET_DOC_POSITION:
        sendLoraSetDocPosition(buoy_nr);
        break;
    case GOTO_DOC_POSITION:
        sendLoraGoToDocPosition(buoy_nr);
        break;
    case REMOTE:
        sendLoraSetSailDirSpeed(buoy_nr, buoy[buoy_nr].cdir, buoy[buoy_nr].cspeed);
        break;
    case RESET:
        sendLoraReset(buoy_nr);
        break;
    case SET_SAIL_DIR_SPEED:
        sendLoraSetSailDirSpeed(buoy_nr, buoy[buoy_nr].cdir, buoy[buoy_nr].cspeed);
        break;

    case SET_BUOY_MODE_IDLE:
        sendLoraSetIdle(buoy_nr);
        break;
    default:
        Serial.println("Unkown menu command!");
        break;
    }
}
