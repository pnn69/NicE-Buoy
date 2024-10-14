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
        break;
    case TOPDIRSPEED:
        sscanf(buf, "%d,%d,%d", &msg, &dataStore.dirSet, &dataStore.speedSet);
        printf("Got TOPDIRSPEED dir:%d Speed:%d\r\n", dataStore.dirSet, dataStore.speedSet);
        break;
    default:
        break;
        return msg;
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
        out += "," + String(dataOut.accuV, 2);
        out += "," + dataOut.accuP;
        break;
    case SUBDIR:
        out = SUBDIR;
        out += "," + dataOut.dirMag;
        break;
    case SUBDIRSPEED:
        out = SUBDIR;
        out += "," + dataOut.dirMag;
        out += "," + dataOut.speedSb;
        out += "," + dataOut.speedBb;
        break;
    case PING:
        out = PING;
        break;
    case PONG:
        out = PONG;
        break;
    }
    return out;
}
