#include <arduino.h>
#include "RoboCodeDecode.h"

int RoboDecode(String data, RoboStruct dataStore)
{
    char buf[100];
    int msg = -1;
    strcpy(buf, data.c_str());
    sscanf(buf, "%d", msg);
    switch (msg)
    {
    case TOPDATA:
        printf("TOPDATA Not implementend yet/r/n");
        break;
    case TOPDIRSPEED:
        sscanf(buf, "%d,%d,%d", &msg, &dataStore.dirSet, &dataStore.speedSet);
        break;
    case SUBDIRSPEED:
        sscanf(buf, "%d,%d,%d,%d", &msg, &dataStore.dirMag, &dataStore.speedBb, dataStore.speedSb);
        break;
    case SUBACCU:
        sscanf(buf, "%d,%d,%d", &msg, &dataStore.subAccuV, &dataStore.subAccuP);
        break;
    default:
        printf("Unkown decode formatter/r/n");
        break;
    }
    return msg;
}

String RoboCode(RoboStruct dataOut)
{
    String out = "";
    switch (dataOut.cmd)
    {
    case SUBDATA:
        out = SUBDATA;
        out += "," + dataOut.dirMag;
        out += "," + dataOut.speedSb;
        out += "," + dataOut.speedBb;
        out += "," + String(dataOut.subAccuV, 2);
        out += "," + dataOut.subAccuP;
        break;
    case SUBDIR:
        out = SUBDIR;
        out += "," + dataOut.dirMag;
        break;
    case SUBDIRSPEED:
        out = SUBDIRSPEED;
        out += "," + dataOut.dirMag;
        out += "," + dataOut.speedSb;
        out += "," + dataOut.speedBb;
        break;
    case SUBACCU:
        out = SUBACCU;
        out += "," + String(dataOut.subAccuV);
        out += "," + dataOut.subAccuP;
        break;
    case PING:
        out = PING;
        break;
    case PONG:
        out = PONG;
        break;
    default:
        printf("Unkown code formatter/r/n");
        break;
    }
    return out;
}
