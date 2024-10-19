#include <arduino.h>
#include "RoboCodeDecode.h"

int RoboDecode(String data, RoboStruct *dataStore)
{
    String numbers[10];                     // Array to hold the decoded numbers (adjust size as needed)
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
            // numbers[count++] = substring.toInt(); // Convert the last part to an integer
            numbers[count++] = substring;
            break;
        }
        // Extract the number before the comma
        // String numStr = substring.substring(0, commaIndex);
        // numbers[count++] = numStr.toInt(); // Convert to integer
        numbers[count++] = substring.substring(0, commaIndex);

        // Remove the extracted number and the comma from the substring
        substring = substring.substring(commaIndex + 1);
    }
    dataStore->cmd = numbers[0].toInt();
    switch (dataStore->cmd)
    {
    case TOPDATA:
        printf("TOPDATA Not implementend yet/r/n");
        break;
    case TOPDIRSPEED:
        dataStore->dirSet = numbers[1].toInt();
        dataStore->speedSet = numbers[3].toInt();
        break;
    case TOPROUTTOPOINT:

        break;
    case TOPSPBBSPSB:
        dataStore->speedBb = numbers[1].toInt();
        dataStore->speedSb = numbers[2].toInt();
        break;
    case TOPCALCRUDDER:
        dataStore->tgDir = numbers[1].toInt();
        dataStore->tgDist = numbers[2].toInt();
        dataStore->speedSet = numbers[3].toInt();
    case TOPIDLE:
        break;
    case SUBDIRSPEED:
        dataStore->dirMag = numbers[1].toInt();
        dataStore->speedBb = numbers[2].toInt();
        dataStore->speedSb = numbers[3].toInt();
        break;
    case SUBACCU:
        dataStore->subAccuV = numbers[1].toFloat();
        dataStore->subAccuP = numbers[2].toInt();
        break;
    case PIDRUDDERSET:
    case PIDSPEEDSET:
    case PIDRUDDER:
    case PIDSPEED:
        dataStore->p = numbers[1].toFloat();
        dataStore->i = numbers[2].toFloat();
        dataStore->d = numbers[3].toFloat();
        dataStore->kp = numbers[4].toFloat();
        dataStore->ki = numbers[5].toFloat();
        dataStore->kd = numbers[6].toFloat();
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
    out += String(dataOut.cmd);
    switch (dataOut.cmd)
    {
    case SUBDATA:
        out += "," + String(dataOut.dirMag);
        out += "," + String(dataOut.speedSb);
        out += "," + String(dataOut.speedBb);
        out += "," + String(dataOut.subAccuV, 2);
        out += "," + String(dataOut.subAccuP);
        break;
    case SUBDIR:
        out += "," + String(dataOut.dirMag);
        break;
    case SUBDIRSPEED:
        out += "," + String(dataOut.dirMag);
        out += "," + String(dataOut.speedSb);
        out += "," + String(dataOut.speedBb);
        break;
    case SUBACCU:
        out += "," + String((float)dataOut.subAccuV);
        out += "," + String(dataOut.subAccuP);
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
    case TOPIDLE:
        out = "$" + String(TOPIDLE);
        break;
    case TOPCALCRUDDER:
        out += "," + String(dataOut.tgDir);
        out += "," + String(dataOut.tgDist);
        out += "," + String(dataOut.speedSet);
        break;
    case PING:
        out = "$" + String(PING);
        break;
    case PONG:
        out = "$" + String(PONG);
        break;
    default:
        printf("Unkown code formatter %d\r\n", dataOut.cmd);
        break;
    }
    out += "*";
    return out;
}
