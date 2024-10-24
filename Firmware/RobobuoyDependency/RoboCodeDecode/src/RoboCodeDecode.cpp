#include <arduino.h>
#include "RoboCodeDecode.h"

/*
    decode incomming data.
    input string format $ID,x,y,z,x,x,x*
    output parameters in to stuct type RoboStruct
    crc does not have any value
*/
void RoboDecode(String data, RoboStruct &dataStore)
{
    dataStore.cmd = -1;
    String numbers[10];                     // Array to hold the decoded numbers (adjust size as needed)
    int count = 0;                          // Keep track of the number of extracted numbers
    int startIndex = data.indexOf('$') + 1; // Start after the '$'
    int endIndex = data.indexOf('*');       // End at the '*'
                                            // Split the substring by commas
    // Serial.println("String to decode: " + data);
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
    dataStore.cmd = numbers[0].toInt();
    // printf("Command to decode:%d\r\n", dataStore.cmd);
    switch (dataStore.cmd)
    {
    case TOPDATA:
        printf("TOPDATA Not implementend yet/r/n");
        break;
    case TOPDIRSPEED:
        dataStore.dirSet = numbers[1].toInt();
        dataStore.speedSet = numbers[3].toInt();
        break;
    case TOPROUTTOPOINT:

        break;
    case TOPSPBBSPSB:
        dataStore.speedBb = numbers[1].toInt();
        dataStore.speedSb = numbers[2].toInt();
        break;
    case TOPCALCRUDDER:
        dataStore.tgDir = numbers[1].toDouble();
        dataStore.tgDist = numbers[2].toDouble();
        dataStore.speedSet = numbers[3].toInt();
    case TOPIDLE:
        break;
    case SUBDIRSPEED:
        dataStore.dirMag = numbers[1].toDouble();
        dataStore.speedBb = numbers[2].toInt();
        dataStore.speedSb = numbers[3].toInt();
        dataStore.speedcalcr = numbers[4].toInt();
        break;
    case SUBACCU:
        dataStore.subAccuV = numbers[1].toFloat();
        dataStore.subAccuP = numbers[2].toInt();
        break;
    case PIDRUDDERSET:
    case PIDRUDDER:
        dataStore.pr = numbers[1].toDouble();
        dataStore.ir = numbers[2].toDouble();
        dataStore.dr = numbers[3].toDouble();
        dataStore.kpr = numbers[4].toDouble();
        dataStore.kir = numbers[5].toDouble();
        dataStore.kdr = numbers[6].toDouble();
        break;
    case PIDSPEEDSET:
    case PIDSPEED:
        dataStore.ps = numbers[1].toDouble();
        dataStore.is = numbers[2].toDouble();
        dataStore.ds = numbers[3].toDouble();
        dataStore.kps = numbers[4].toDouble();
        dataStore.kis = numbers[5].toDouble();
        dataStore.kds = numbers[6].toDouble();
        break;
    case PING:
        break;
    case PONG:
        break;
    default:
        printf("RoboDecode: Unkown decode formatter %d\r\n", numbers[0]);
        dataStore.cmd = -1;
        break;
    }
}

/*
    Encode outgoing data.
    input DATA RoboSruct and depeding on field msg
    output string format $ID,x,y,z,x,x,x*
    crc has to be added!
*/
String RoboCode(RoboStruct dataOut)
{
    String out = "$";
    out += String(dataOut.cmd);
    switch (dataOut.cmd)
    {
    case SUBDATA:
        out += "," + String(dataOut.dirMag, 2);
        out += "," + String(dataOut.speedSb);
        out += "," + String(dataOut.speedBb);
        out += "," + String(dataOut.subAccuV, 2);
        out += "," + String(dataOut.subAccuP);
        break;
    case SUBDIR:
        out += "," + String(dataOut.dirMag, 2);
        break;
    case SUBDIRSPEED:
        out += "," + String(dataOut.dirMag, 2);
        out += "," + String(dataOut.speedSb);
        out += "," + String(dataOut.speedBb);
        out += "," + String(dataOut.speedcalcs);
        break;
    case SUBACCU:
        out += "," + String(dataOut.subAccuV, 2);
        out += "," + String(dataOut.subAccuP);
        break;
    case PIDSPEED:
        out += "," + String(dataOut.ps);
        out += "," + String(dataOut.is);
        out += "," + String(dataOut.ds);
        out += "," + String(dataOut.kps);
        out += "," + String(dataOut.kis);
        out += "," + String(dataOut.kds);
        break;
    case PIDRUDDER:
    case PIDRUDDERSET:
        out += "," + String(dataOut.pr);
        out += "," + String(dataOut.ir);
        out += "," + String(dataOut.dr);
        out += "," + String(dataOut.kpr);
        out += "," + String(dataOut.kir);
        out += "," + String(dataOut.kdr);
        break;
    case TOPIDLE:
        out = "$" + String(TOPIDLE);
        break;
    case TOPCALCRUDDER:
        out += "," + String(dataOut.tgDir, 2);
        out += "," + String(dataOut.tgDist, 2);
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
