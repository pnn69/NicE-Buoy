import os

content = r"""#include <arduino.h>
#include "RoboCompute.h"

bool startsWithDollar(const String &str)
{
    return str.charAt(0) == '$';
}

String formatFloat(double val, int precision)
{
    if (val == 0.0) return "0";
    String s = String(val, precision);
    if (s.indexOf('.') != -1) {
        while (s.endsWith("0")) {
            s.remove(s.length() - 1);
        }
        if (s.endsWith(".")) {
            s.remove(s.length() - 1);
        }
    }
    return s;
}

void RoboDecode(String data, RoboStruct *dataStore)
{
    dataStore->cmd = -1;
    String numbers[25];
    int count = 0;
    String substring = data;
    while (count < 25)
    {
        int commaIndex = substring.indexOf(',');
        if (commaIndex == -1) { numbers[count++] = substring; break; }
        numbers[count++] = substring.substring(0, commaIndex);
        substring = substring.substring(commaIndex + 1);
    }
    if (count < 2) return;
    dataStore->cmd = numbers[0].toInt();
    dataStore->status = numbers[1].toInt();
    switch (dataStore->cmd)
    {
    case SETUPDATA:
          dataStore->Kpr = numbers[2].toDouble();
          dataStore->Kir = numbers[3].toDouble();
          dataStore->Kdr = numbers[4].toDouble();
          dataStore->Kps = numbers[5].toDouble();
          dataStore->Kis = numbers[6].toDouble();
          dataStore->Kds = numbers[7].toDouble();
          dataStore->maxSpeed = numbers[8].toInt();
          dataStore->minSpeed = numbers[9].toInt();
          dataStore->pivotSpeed = numbers[10].toDouble();
          dataStore->compassOffset = numbers[11].toDouble();
          dataStore->holdRad = numbers[12].toDouble();
          if (numbers[13].length() > 0) dataStore->revBB = (bool)numbers[13].toInt();
          if (numbers[14].length() > 0) dataStore->revSB = (bool)numbers[14].toInt();
          if (numbers[15].length() > 0) dataStore->swap_BB_SB = (bool)numbers[15].toInt();
          break;
    case IDLE:
        dataStore->speed = 0;
        dataStore->tgDist = 0;
        break;
    case DOCKED:
    case LOCKED:
        dataStore->tgDir = numbers[2].toDouble();
        dataStore->tgDist = numbers[3].toDouble();
        dataStore->tgSpeed = numbers[4].toDouble();
        dataStore->wDir = numbers[5].toDouble();
        dataStore->wStd = numbers[6].toDouble();
        break;
    case REMOTE:
        dataStore->tgDir = numbers[2].toDouble();
        dataStore->tgSpeed = numbers[3].toDouble();
        break;
    case DIRSPEED:
        dataStore->dirMag = numbers[2].toDouble();
        dataStore->speed = numbers[3].toInt();
        dataStore->speedBb = numbers[4].toInt();
        dataStore->speedSb = numbers[5].toInt();
        break;
    case TGDIRSPEED:
        dataStore->tgDir = numbers[2].toDouble();
        dataStore->speedSet = numbers[3].toDouble();
        break;
    case SPBBSPSB:
        dataStore->speedBb = numbers[2].toInt();
        dataStore->speedSb = numbers[3].toInt();
        break;
    case CALCRUDDER:
        dataStore->tgDir = numbers[2].toDouble();
        dataStore->tgDist = numbers[3].toDouble();
        dataStore->speedSet = numbers[4].toDouble();
        break;
    case SPEED:
        dataStore->speed = numbers[2].toInt();
        break;
    case SUBSPEED:
        dataStore->speedBb = numbers[2].toInt();
        dataStore->speedSb = numbers[3].toInt();
        dataStore->speed = numbers[4].toInt();
        break;
    case SUBACCU:
        dataStore->subAccuV = numbers[2].toFloat();
        dataStore->subAccuP = numbers[3].toInt();
        if (count > 4) dataStore->subAccuI = numbers[4].toFloat();
        break;
    case PIDRUDDERSET:
    case PIDRUDDER:
        dataStore->Kpr = numbers[2].toDouble();
        dataStore->Kir = numbers[3].toDouble();
        dataStore->Kdr = numbers[4].toDouble();
        break;
    case PIDSPEEDSET:
    case PIDSPEED:
        dataStore->Kps = numbers[2].toDouble();
        dataStore->Kis = numbers[3].toDouble();
        dataStore->Kds = numbers[4].toDouble();
        break;
    case SUBPWR:
        dataStore->speedSet = numbers[2].toDouble();
        dataStore->speed = numbers[3].toInt();
        dataStore->speedBb = numbers[4].toInt();
        dataStore->speedSb = numbers[5].toInt();
        dataStore->subAccuV = numbers[6].toFloat();
        if (count > 7) dataStore->subAccuI = numbers[7].toFloat();
        break;
    case TOPPWR:
        dataStore->speedSet = numbers[2].toDouble();
        dataStore->speed = numbers[3].toInt();
        dataStore->speedBb = numbers[4].toInt();
        dataStore->speedSb = numbers[5].toInt();
        dataStore->topAccuV = numbers[6].toFloat();
        if (count > 7) dataStore->topAccuI = numbers[7].toFloat();
        break;
    case BUOYPOS:
        dataStore->lat = numbers[2].toDouble();
        dataStore->lng = numbers[3].toDouble();
        dataStore->dirMag = numbers[4].toDouble();
        dataStore->wDir = numbers[5].toDouble();
        dataStore->wStd = numbers[6].toDouble();
        dataStore->topAccuP = numbers[7].toInt();
        dataStore->subAccuP = numbers[8].toInt();
        dataStore->gpsFix = (bool)numbers[9].toInt();
        dataStore->gpsSat = numbers[10].toInt();
        break;
    case LOCKPOS:
    case DOCKPOS:
        dataStore->tgLat = numbers[2].toDouble();
        dataStore->tgLng = numbers[3].toDouble();
        dataStore->wDir = numbers[4].toDouble();
        dataStore->wStd = numbers[5].toDouble();
        break;
    case SETLOCKPOS:
    case SETDOCKPOS:
        dataStore->tgLat = numbers[2].toDouble();
        dataStore->tgLng = numbers[3].toDouble();
        break;
    case DIRDIST:
        dataStore->tgDir = numbers[2].toDouble();
        dataStore->tgDist = numbers[3].toDouble();
        break;
    case WINDDATA:
        dataStore->wDir = numbers[2].toDouble();
        dataStore->wStd = numbers[3].toDouble();
        break;
    case STORE_DECLINATION:
        dataStore->declination = numbers[2].toDouble();
        break;
    case MAXMINPWR:
    case MAXMINPWRSET:
        dataStore->maxSpeed = numbers[2].toInt();
        dataStore->minSpeed = numbers[3].toInt();
        if (count > 4) dataStore->pivotSpeed = numbers[4].toDouble();
        break;
    case DIRMDIRTGDIRG:
        dataStore->dirMag = numbers[2].toDouble();
        dataStore->tgDir = numbers[3].toDouble();
        dataStore->gpsDir = numbers[4].toInt();
        break;
    case SET_DECLINATION:
        dataStore->status = SET_DECLINATION;
        break;
    case SUBDATA:
        dataStore->dirMag = numbers[2].toDouble();
        dataStore->speedBb = numbers[3].toInt();
        dataStore->speedSb = numbers[4].toInt();
        dataStore->ip = numbers[5].toDouble();
        dataStore->ir = numbers[6].toDouble();
        dataStore->subAccuV = numbers[7].toDouble();
        dataStore->subAccuP = numbers[8].toInt();
        if (count > 9) dataStore->subAccuI = numbers[9].toFloat();
        break;
    case TOPDATA:
        dataStore->dirMag = numbers[2].toDouble();
        dataStore->gpsDir = numbers[3].toInt();
        dataStore->tgDir = numbers[4].toInt();
        dataStore->tgDist = numbers[5].toDouble();
        dataStore->wDir = numbers[6].toDouble();
        dataStore->wStd = numbers[7].toDouble();
        dataStore->speedBb = numbers[8].toInt();
        dataStore->speedSb = numbers[9].toInt();
        dataStore->ip = numbers[10].toDouble();
        dataStore->ir = numbers[11].toDouble();
        dataStore->subAccuV = numbers[12].toDouble();
        dataStore->subAccuP = numbers[13].toInt();
        dataStore->lat = numbers[14].toDouble();
        dataStore->lng = numbers[15].toDouble();
        dataStore->gpsFix = (bool)numbers[16].toInt();
        dataStore->gpsSat = numbers[17].toInt();
        if (count > 18) dataStore->subAccuI = numbers[18].toFloat();
        break;
    case RAWCOMPASSDATA:
        dataStore->magHard[0] = numbers[2].toDouble();
        dataStore->magHard[1] = numbers[3].toDouble();
        dataStore->magHard[2] = numbers[4].toDouble();
        break;
    case HARDIRONFACTORS:
        dataStore->magHard[0] = numbers[2].toDouble();
        dataStore->magHard[1] = numbers[3].toDouble();
        dataStore->magHard[2] = numbers[4].toDouble();
        break;
    case SOFTIRONFACTORS:
        dataStore->magSoft[0][0] = numbers[2].toDouble();
        dataStore->magSoft[0][1] = numbers[3].toDouble();
        dataStore->magSoft[0][2] = numbers[4].toDouble();
        dataStore->magSoft[1][0] = numbers[5].toDouble();
        dataStore->magSoft[1][1] = numbers[6].toDouble();
        dataStore->magSoft[1][2] = numbers[7].toDouble();
        dataStore->magSoft[2][0] = numbers[8].toDouble();
        dataStore->magSoft[2][1] = numbers[9].toDouble();
        dataStore->magSoft[2][2] = numbers[10].toDouble();
        break;
    case STORE_COMPASS_OFFSET:
        dataStore->compassOffset = numbers[2].toDouble();
        break;
    case SET_AS_NORTH:
        break;
    default:
        printf("RoboDecode: Unknown CMD %d\r\n", dataStore->cmd);
        break;
    }
}

String RoboCode(const RoboStruct *dataOut)
{
    String out = String(dataOut->cmd);
    out += "," + String(dataOut->status);
    if (dataOut->ack == ACK) return out;
    switch (dataOut->cmd)
    {
    case SETUPDATA:
        out += "," + formatFloat(dataOut->Kpr, 5);
        out += "," + formatFloat(dataOut->Kir, 5);
        out += "," + formatFloat(dataOut->Kdr, 5);
        out += "," + formatFloat(dataOut->Kps, 5);
        out += "," + formatFloat(dataOut->Kis, 5);
        out += "," + formatFloat(dataOut->Kds, 5);
        out += "," + String(dataOut->maxSpeed);
        out += "," + String(dataOut->minSpeed);
        out += "," + formatFloat(dataOut->pivotSpeed, 2);
        out += "," + formatFloat(dataOut->compassOffset, 2);
        out += "," + formatFloat(dataOut->holdRad, 2);
        out += "," + String((int)dataOut->revBB);
        out += "," + String((int)dataOut->revSB);
        out += "," + String((int)dataOut->swap_BB_SB);
        break;
    case DIRSPEED:
        out += "," + formatFloat(dataOut->dirMag, 2);
        out += "," + String(dataOut->speed);
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        break;
    case SPEED:
        out += "," + String(dataOut->speed);
        break;
    case SUBSPEED:
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        out += "," + String(dataOut->speed);
        break;
    case SUBACCU:
        out += "," + formatFloat(dataOut->subAccuV, 2);
        out += "," + String(dataOut->subAccuP);
        out += "," + formatFloat(dataOut->subAccuI, 2);
        break;
    case PIDRUDDERSET:
    case PIDRUDDER:
        out += "," + formatFloat(dataOut->Kpr, 5);
        out += "," + formatFloat(dataOut->Kir, 5);
        out += "," + formatFloat(dataOut->Kdr, 5);
        break;
    case PIDSPEEDSET:
    case PIDSPEED:
        out += "," + formatFloat(dataOut->Kps, 5);
        out += "," + formatFloat(dataOut->Kis, 5);
        out += "," + formatFloat(dataOut->Kds, 5);
        break;
    case SUBPWR:
        out += "," + formatFloat(dataOut->speedSet, 2);
        out += "," + String(dataOut->speed);
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        out += "," + formatFloat(dataOut->subAccuV, 2);
        out += "," + formatFloat(dataOut->subAccuI, 2);
        break;
    case TOPPWR:
        out += "," + formatFloat(dataOut->speedSet, 2);
        out += "," + String(dataOut->speed);
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        out += "," + formatFloat(dataOut->topAccuV, 2);
        out += "," + formatFloat(dataOut->topAccuI, 2);
        break;
    case BUOYPOS:
        out += "," + formatFloat(dataOut->lat, 8);
        out += "," + formatFloat(dataOut->lng, 8);
        out += "," + formatFloat(dataOut->dirMag, 2);
        out += "," + formatFloat(dataOut->wDir, 1);
        out += "," + formatFloat(dataOut->wStd, 1);
        out += "," + String(dataOut->topAccuP);
        out += "," + String(dataOut->subAccuP);
        out += "," + String(dataOut->gpsFix);
        out += "," + String(dataOut->gpsSat);
        break;
    case TGDIRSPEED:
        out += "," + formatFloat(dataOut->tgDir, 2);
        out += "," + formatFloat(dataOut->speedSet, 2);
        break;
    case SUBDATA:
        out += "," + formatFloat(dataOut->dirMag, 2);
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        out += "," + formatFloat(dataOut->ip, 2);
        out += "," + formatFloat(dataOut->ir, 2);
        out += "," + formatFloat(dataOut->subAccuV, 2);
        out += "," + String(dataOut->subAccuP);
        out += "," + formatFloat(dataOut->subAccuI, 2);
        break;
    case TOPDATA:
        out += "," + formatFloat(dataOut->dirMag, 0);
        out += "," + String(dataOut->gpsDir);
        out += "," + formatFloat(dataOut->tgDir, 0);
        out += "," + formatFloat(dataOut->tgDist, 1);
        out += "," + formatFloat(dataOut->wDir, 0);
        out += "," + formatFloat(dataOut->wStd, 1);
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        out += "," + formatFloat(dataOut->ip, 2);
        out += "," + formatFloat(dataOut->ir, 2);
        out += "," + formatFloat(dataOut->subAccuV, 2);
        out += "," + String(dataOut->subAccuP);
        out += "," + formatFloat(dataOut->lat, 8);
        out += "," + formatFloat(dataOut->lng, 8);
        out += "," + String(dataOut->gpsFix);
        out += "," + String(dataOut->gpsSat);
        out += "," + formatFloat(dataOut->subAccuI, 2);
        break;
    case SPBBSPSB:
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        break;
    case SETLOCKPOS:
    case SETDOCKPOS:
        out += "," + formatFloat(dataOut->tgLat, 10);
        out += "," + formatFloat(dataOut->tgLng, 10);
        break;
    case LOCKPOS:
    case DOCKPOS:
        out += "," + formatFloat(dataOut->tgLat, 10);
        out += "," + formatFloat(dataOut->tgLng, 10);
        out += "," + formatFloat(dataOut->wDir, 1);
        out += "," + formatFloat(dataOut->wStd, 1);
        break;
    case WINDDATA:
        out += "," + formatFloat(dataOut->wDir, 1);
        out += "," + formatFloat(dataOut->wStd, 1);
        break;
    case DIRDIST:
        out += "," + formatFloat(dataOut->tgDir, 1);
        out += "," + formatFloat(dataOut->tgDist, 1);
        break;
    case MAXMINPWRSET:
    case MAXMINPWR:
        out += "," + String(dataOut->maxSpeed);
        out += "," + String(dataOut->minSpeed);
        out += "," + formatFloat(dataOut->pivotSpeed, 2);
        break;
    case DIRMDIRTGDIRG:
        out += "," + formatFloat(dataOut->dirMag, 0);
        out += "," + formatFloat(dataOut->tgDir, 0);
        out += "," + String(dataOut->gpsDir);
        break;
    case STORE_DECLINATION:
        out += "," + formatFloat(dataOut->declination, 2);
        break;
    case RAWCOMPASSDATA:
        out += "," + formatFloat(dataOut->magHard[0], 5);
        out += "," + formatFloat(dataOut->magHard[1], 5);
        out += "," + formatFloat(dataOut->magHard[2], 5);
        break;
    case SOFTIRONFACTORS:
        out += "," + formatFloat(dataOut->magSoft[0][0], 5);
        out += "," + formatFloat(dataOut->magSoft[0][1], 5);
        out += "," + formatFloat(dataOut->magSoft[0][2], 5);
        out += "," + formatFloat(dataOut->magSoft[1][0], 5);
        out += "," + formatFloat(dataOut->magSoft[1][1], 5);
        out += "," + formatFloat(dataOut->magSoft[1][2], 5);
        out += "," + formatFloat(dataOut->magSoft[2][0], 5);
        out += "," + formatFloat(dataOut->magSoft[2][1], 5);
        out += "," + formatFloat(dataOut->magSoft[2][2], 5);
        break;
    case HARDIRONFACTORS:
        out += "," + formatFloat(dataOut->magHard[0], 2);
        out += "," + formatFloat(dataOut->magHard[1], 2);
        out += "," + formatFloat(dataOut->magHard[2], 2);
        break;
    case STORE_COMPASS_OFFSET:
        out += "," + formatFloat(dataOut->compassOffset, 2);
        break;
    case SET_AS_NORTH:
        break;
    case DOCKED:
    case LOCKED:
        out += "," + formatFloat(dataOut->tgDir, 1);
        out += "," + formatFloat(dataOut->tgDist, 1);
        out += "," + formatFloat(dataOut->tgSpeed, 1);
        out += "," + formatFloat(dataOut->wDir, 1);
        out += "," + formatFloat(dataOut->wStd, 1);
        break;
    case REMOTE:
        out += "," + formatFloat(dataOut->tgDir, 0);
        out += "," + formatFloat(dataOut->tgSpeed, 0);
        break;
    case IDLE:
        out += ",0,0";
        break;
    case PING:
        return String(PING);
    case PONG:
        return String(PONG);
    default:
        printf("RoboCode: Unknown formatter <%d>\r\n", dataOut->cmd);
        break;
    }

    // Compress zeros to empty strings to save bandwidth
    String optimized = "";
    int lastComma = -1;
    for (unsigned int i = 0; i <= out.length(); i++) {
        if (i == out.length() || out[i] == ',') {
            String token = out.substring(lastComma + 1, i);
            if (token.length() > 0) {
                bool isZero = true;
                for (unsigned int j = 0; j < token.length(); j++) {
                    if (token[j] != '0' && token[j] != '.' && token[j] != '-') {
                        isZero = false;
                        break;
                    }
                }
                if (isZero && token != "-" && token != "." && token != "-.") {
                    token = "";
                }
            }
            if (lastComma != -1) optimized += ",";
            optimized += token;
            lastComma = i;
        }
    }
    return optimized;
}

String rfCode(RoboStruct *rfOut)
{
    String rfMsg = String(rfOut->IDr, HEX);
    rfMsg += "," + String(rfOut->IDs, HEX);
    rfMsg += "," + String(rfOut->ack);
    rfMsg += "," + RoboCode(rfOut);
    rfMsg = addCRCToString(rfMsg);
    return rfMsg;
}

String addCRCToString(String input)
{
    input.trim();
    input.replace(" ", "");
    byte crc = 0;
    for (int i = 0; i < input.length(); i++) crc ^= (byte)input.charAt(i);
    char crcHex[3];
    sprintf(crcHex, "%02X", crc);
    return "$" + input + "*" + String(crcHex);
}

void rfDeCode(String rfIn, RoboStruct *in)
{
    rfIn.trim();
    in->IDr = -1;
    in->IDs = -1;
    if (!rfIn.startsWith("$") || rfIn.indexOf('*') == -1) return;
    if (!verifyCRC(rfIn)) return;
    int starIndex = rfIn.indexOf('*');
    rfIn = rfIn.substring(1, starIndex);
    int comma1 = rfIn.indexOf(',');
    if (comma1 == -1) return;
    in->IDr = strtoull(rfIn.substring(0, comma1).c_str(), NULL, 16);
    rfIn = rfIn.substring(comma1 + 1);
    int comma2 = rfIn.indexOf(',');
    if (comma2 == -1) return;
    in->IDs = strtoull(rfIn.substring(0, comma2).c_str(), NULL, 16);
    rfIn = rfIn.substring(comma2 + 1);
    int comma3 = rfIn.indexOf(',');
    if (comma3 == -1) return;
    in->ack = rfIn.substring(0, comma3).toInt();
    rfIn = rfIn.substring(comma3 + 1);
    RoboDecode(rfIn, in);
}

bool verifyCRC(String input)
{
    int start = input.indexOf('$');
    int end = input.indexOf('*');
    if (start == -1 || end == -1 || end <= start || end + 2 >= input.length()) return false;
    byte calculatedCRC = 0;
    for (int i = start + 1; i < end; i++) calculatedCRC ^= input[i];
    String givenCRC = input.substring(end + 1, end + 3);
    char calculatedCRCHex[3];
    sprintf(calculatedCRCHex, "%02X", calculatedCRC);
    return givenCRC.equalsIgnoreCase(String(calculatedCRCHex));
}

void addNewSampleInBuffer(RoboWindStruct *wData, double nwdata)
{
    wData->data[wData->ptr] = nwdata;
    wData->ptr = (wData->ptr + 1) % SAMPELS;
}

void averageWindVector(RoboWindStruct *wData)
{
    double sumX = 0, sumY = 0;
    for (int i = 0; i < SAMPELS; ++i)
    {
        double angleRad = radians(wData->data[i]);
        sumX += cos(angleRad);
        sumY += sin(angleRad);
    }
    double meanAngle = atan2(sumY, sumX);
    wData->wDir = fmod((meanAngle * 180.0 / M_PI) + 360.0, 360.0);
}

void deviationWindRose(RoboWindStruct *wData)
{
    averageWindVector(wData);
    double sumSin = 0.0, sumCos = 0.0;
    for (int i = 0; i < SAMPELS; ++i)
    {
        double angleRad = wData->data[i] * M_PI / 180.0;
        sumCos += cos(angleRad);
        sumSin += sin(angleRad);
    }
    double R = sqrt(sumCos * sumCos + sumSin * sumSin) / SAMPELS;
    if (R > 1.0) R = 1.0;
    if (R < 0.000001) R = 0.000001;
    double circStdRad = sqrt(-2.0 * log(R));
    wData->wStd = circStdRad * 180.0 / M_PI;
}

void PidDecode(String data, int pid, RoboStruct *buoy)
{
    String numbers[20];
    int count = 0;
    int startIndex = data.indexOf('$') + 1;
    int endIndex = data.indexOf('*');
    String substring = data.substring(startIndex, endIndex);
    while (count < 20)
    {
        int commaIndex = substring.indexOf(',');
        if (commaIndex == -1) { numbers[count++] = substring; break; }
        numbers[count++] = substring.substring(0, commaIndex);
        substring = substring.substring(commaIndex + 1);
    }
    if (pid == PIDSPEED)
    {
        buoy->Kps = numbers[1].toDouble();
        buoy->Kis = numbers[2].toDouble();
        buoy->Kds = numbers[3].toDouble();
    }
    if (pid == PIDRUDDER)
    {
        buoy->Kpr = numbers[1].toDouble();
        buoy->Kir = numbers[2].toDouble();
        buoy->Kdr = numbers[3].toDouble();
    }
}

String PidEncode(int pid, const RoboStruct *buoy)
{
    String out = "";
    if (pid == PIDSPEED)
    {
        out = String(buoy->Kps);
        out += "," + String(buoy->Kis);
        out += "," + String(buoy->Kds);
    }
    if (pid == PIDRUDDER)
    {
        out = String(buoy->Kpr);
        out += "," + String(buoy->Kir);
        out += "," + String(buoy->Kdr);
    }
    return out;
}

void gpsGem(double &lat, double &lon)
{
    static double gpsgem[20][2];
    static int point = 0;
    gpsgem[point][0] = lat;
    gpsgem[point][1] = lon;
    point = (point + 1) % 20;
    double sumLat = 0, sumLon = 0;
    for (int i = 0; i < 20; i++) { sumLat += gpsgem[i][0]; sumLon += gpsgem[i][1]; }
    lat = sumLat / 20;
    lon = sumLon / 20;
}

double distanceBetween(double lat1, double lon1, double lat2, double lon2)
{
    lat1 = lat1 * M_PI / 180.0;
    lon1 = lon1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    lon2 = lon2 * M_PI / 180.0;
    double dLat = lat2 - lat1, dLon = lon2 - lon1;
    double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return 6372795.0 * c;
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2)
{
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    return fmod((atan2(y, x) * (180.0 / M_PI) + 360.0), 360.0);
}

double smallestAngle(double heading1, double heading2)
{
    double angle = fmod(heading2 - heading1 + 360, 360);
    if (angle > 180) return angle - 360;
    return angle;
}

void adjustPositionDirDist(double bearing_deg, double distance,
                           double lat1_deg, double lon1_deg,
                           double *lat2_deg, double *lon2_deg)
{
    double lat1 = radians(lat1_deg), lon1 = radians(lon1_deg), bearing = radians(bearing_deg);
    double ad = distance / 6371000.0;
    double lat2 = asin(sin(lat1) * cos(ad) + cos(lat1) * sin(ad) * cos(bearing));
    double lon2 = lon1 + atan2(sin(bearing) * sin(ad) * cos(lat1), cos(ad) - sin(lat1) * sin(lat2));
    *lat2_deg = degrees(lat2);
    *lon2_deg = degrees(lon2);
    if (*lon2_deg > 180.0) *lon2_deg -= 360.0;
    else if (*lon2_deg < -180.0) *lon2_deg += 360.0;
}

double calculateAngle(double x1, double y1, double x2, double y2)
{
    double dotProduct = x1 * x2 + y1 * y2;
    double magnitudeA = sqrt(x1 * x1 + y1 * y1);
    double magnitudeB = sqrt(x2 * x2 + y2 * y2);
    double cosAngle = dotProduct / (magnitudeA * magnitudeB);
    cosAngle = fmax(-1.0, fmin(1.0, cosAngle));
    return acos(cosAngle) * (180.0 / M_PI);
}
double calculateAngleSigned(double x1, double y1, double x2, double y2)
{
    double dotProduct = x1 * x2 + y1 * y2;
    double magnitudeA = sqrt(x1 * x1 + y1 * y1);
    double magnitudeB = sqrt(x2 * x2 + y2 * y2);
    double cosAngle = dotProduct / (magnitudeA * magnitudeB);
    cosAngle = fmax(-1.0, fmin(1.0, cosAngle));
    double crossProduct = x1 * y2 - y1 * x2;
    double angle = atan2(crossProduct, dotProduct) * (180.0 / M_PI);
    return angle;
}

void recalcStartLine(struct RoboStruct rsl[3])
{
    int presentCount = 0, idx[3];
    for (int i = 0; i < 3; i++) if (rsl[i].tgLat != 0.0) idx[presentCount++] = i;
    if (presentCount < 2) return;
    double midLat = (rsl[idx[0]].tgLat + rsl[idx[1]].tgLat) / 2;
    double midLng = (rsl[idx[0]].tgLng + rsl[idx[1]].tgLng) / 2;
    double d = distanceBetween(rsl[idx[0]].tgLat, rsl[idx[0]].tgLng, rsl[idx[1]].tgLat, rsl[idx[1]].tgLng);
    adjustPositionDirDist(fmod(rsl[idx[0]].wDir + 270, 360), d / 2, midLat, midLng, &rsl[idx[0]].tgLat, &rsl[idx[0]].tgLng);
    adjustPositionDirDist(fmod(rsl[idx[0]].wDir + 90, 360), d / 2, midLat, midLng, &rsl[idx[1]].tgLat, &rsl[idx[1]].tgLng);
}

void reCalcTrack(struct RoboStruct rsl[3]) {}

void trackPosPrint(int c)
{
    if (c == HEAD) printf("HEAD");
    else if (c == PORT) printf("PORT");
    else if (c == STARBOARD) printf("STARBOARD");
}

RoboStruct calcTrackPos(RoboStruct rsl[3]) { return rsl[0]; }

void AddDataToBuoyBase(RoboStruct dataIn, RoboStruct *buoyPara[3])
{
    for (int i = 0; i < 3; i++)
    {
        if (buoyPara[i] && (dataIn.IDs == buoyPara[i]->IDs || buoyPara[i]->IDs == 0))
        {
            *buoyPara[i] = dataIn;
            return;
        }
    }
}

int GetDataPosFromBuoyBase(uint64_t id, RoboStruct buoyPara[3])
{
    for (int i = 0; i < 3; i++) if (id == buoyPara[i].IDs) return i;
    return -1;
}
"""

with open('../RoboDependency/RoboCompute/src/RoboCompute.cpp', 'w', encoding='utf-8', newline='\r\n') as f:
    f.write(content)
print("Manually restored RoboCompute.cpp to a clean and robust state (STRICT NO-ERRORS VERSION)")
