#include <arduino.h>
#include "RoboCodeDecode.h"

int RoboDecode(String data, RoboStruct *dataStore)
{
    int numbers[10];                        // Array to hold the decoded numbers (adjust size as needed)
    int count = 0;                          // Keep track of the number of extracted numbers
    int startIndex = data.indexOf('$') + 1; // Start after the '$'
    int endIndex = data.indexOf('*');       // End at the '*'
                                            // Split the substring by commas
    String substring = data.substring(startIndex, endIndex);
    while (substring.length() > 0)
    {
        int commaIndex = substring.indexOf(',');

        // If there's no more comma, this is the last number
        if (commaIndex == -1)
        {
            numbers[count++] = substring.toInt(); // Convert the last part to an integer
            break;
        }
        // Extract the number before the comma
        String numStr = substring.substring(0, commaIndex);
        numbers[count++] = numStr.toInt(); // Convert to integer

        // Remove the extracted number and the comma from the substring
        substring = substring.substring(commaIndex + 1);
    }
    dataStore->cmd = numbers[0];
    switch (dataStore->cmd)
    {
    case TOPDATA:
        printf("TOPDATA Not implementend yet/r/n");
        break;
    case TOPDIRSPEED:
        dataStore->dirSet = numbers[1];
        dataStore->dirSet = numbers[2];
        dataStore->speedSet = numbers[3];
        break;
    case TOPSPBBSPSB:
        dataStore->speedBb = numbers[1];
        dataStore->speedSb = numbers[2];
        break;
    case SUBDIRSPEED:
        dataStore->dirMag = numbers[1];
        dataStore->speedBb = numbers[2];
        dataStore->speedSb = numbers[3];
        break;
    case SUBACCU:
        dataStore->subAccuV = numbers[1];
        dataStore->subAccuP = numbers[2];
        break;
    case PIDRUDDERSET:
    case PIDSPEEDSET:
    case PIDRUDDER:
    case PIDSPEED:
        dataStore->p = numbers[1];
        dataStore->i = numbers[2];
        dataStore->d = numbers[3];
        dataStore->kp = numbers[4];
        dataStore->ki = numbers[5];
        dataStore->kd = numbers[6];
        break;
    case PING:
        break;
    case PONG:
        break;
    default:
        printf("RoboDecode: Unkown decode formatter %d\r\n", numbers[0]);
        dataStore->cmd = -1;
        break;
    }
    return dataStore->cmd;
}

String RoboCode(RoboStruct dataOut)
{
    String out = "$";
    out += dataOut.cmd;
    switch (dataOut.cmd)
    {
    case SUBDATA:
        out += "," + dataOut.dirMag;
        out += "," + dataOut.speedSb;
        out += "," + dataOut.speedBb;
        out += "," + String(dataOut.subAccuV, 2);
        out += "," + dataOut.subAccuP;
        break;
    case SUBDIR:
        out += "," + dataOut.dirMag;
        break;
    case SUBDIRSPEED:
        out += "," + dataOut.dirMag;
        out += "," + dataOut.speedSb;
        out += "," + dataOut.speedBb;
        break;
    case SUBACCU:
        out += "," + String(dataOut.subAccuV);
        out += "," + dataOut.subAccuP;
        break;
    case PIDRUDDER:
    case PIDSPEED:
    case PIDRUDDERSET:
        out += "," + String(dataOut.p);
        out += "," + String(dataOut.i);
        out += "," + String(dataOut.d);
        out += "," + String(dataOut.kp);
        out += "," + String(dataOut.ki);
        out += "," + String(dataOut.kd);
        break;
    case PING:
        out = "$" + PING;
        break;
    case PONG:
        out = "$" + PONG;
        break;
    default:
        printf("Unkown code formatter/r/n");
        break;
    }
    out += "*";
    return out;
}
