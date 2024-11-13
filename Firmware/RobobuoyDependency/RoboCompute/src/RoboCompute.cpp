#include <arduino.h>
#include "RoboCompute.h"

bool startsWithDollar(const String &str)
{
    // Check if the string first character is '$'
    return str.charAt(0) == '$';
}

/*
    decode incomming data.
    input string format $ID,<data>*
    output parameters in to stuct type RoboStruct
    crc does not have any value
*/
struct RoboStruct RoboDecode(String data, RoboStruct dataStore)
{
    dataStore.cmd = -1;
    String numbers[15];           // Array to hold the decoded numbers (adjust size as needed)
    int count = 0;                // Keep track of the number of extracted numbers
    int startIndex = 0;           // Start after the '$'
    int endIndex = data.length(); // End at the '*'
                                  // Split the substring by commas
    if (startsWithDollar(data))   // strip $ and *
    {
        data = removeBeginAndEndToString(data);
    }
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
    case IDLE:
        dataStore.speed = 0;
        dataStore.tgDist = 0;
        break;
    case LOCKING:
        break;
    case DOCKED: // tgDir,tgDist
    case LOCKED: // tgDir,tgDist
        dataStore.tgDir = numbers[1].toDouble();
        dataStore.tgDist = numbers[2].toDouble();
        break;
    case REMOTE: // speed,tgdir
        dataStore.speed = numbers[1].toInt();
        dataStore.tgDir = numbers[2].toDouble();
        break;
    case TOPDATA: // ?
        printf("TOPDATA Not implementend yet/r/n");
        break;
    case UDPTGDIRSPEED:
    case LORADIRSPEED: // Dir,speed
    case TOPDIRSPEED:  // Dir,Speed
        dataStore.dirSet = numbers[1].toInt();
        dataStore.speedSet = numbers[2].toDouble();
        break;
    case TOPROUTTOPOINT: //?
        break;
    case TOPSPBBSPSB: // SpeedBb,SpeedSb
        dataStore.speedBb = numbers[1].toInt();
        dataStore.speedSb = numbers[2].toInt();
        break;
    case TOPCALCRUDDER: // tgDir,tgDist,Speed
        dataStore.tgDir = numbers[1].toDouble();
        dataStore.tgDist = numbers[2].toDouble();
        dataStore.speedSet = numbers[3].toDouble();
    case TOPIDLE:
        break;
    case SUBDIRSPEED: // mDir,speedbb,speedsb,speed
        dataStore.dirMag = numbers[1].toDouble();
        dataStore.speedBb = numbers[2].toInt();
        dataStore.speedSb = numbers[3].toInt();
        dataStore.speed = numbers[4].toInt();
        break;
    case SUBSPEED:
        dataStore.speedBb = numbers[1].toInt();
        dataStore.speedSb = numbers[2].toInt();
        dataStore.speed = numbers[3].toInt();
        break;
    case SUBACCU: // V,P
        dataStore.subAccuV = numbers[1].toFloat();
        dataStore.subAccuP = numbers[2].toInt();
        break;
    case PIDRUDDERSET: // Prudder,Irudder,Drudder,kp,ki,kd
    case PIDRUDDER:    // Prudder,Irudder,Drudder,kp,ki,kd
        dataStore.pr = numbers[1].toDouble();
        dataStore.ir = numbers[2].toDouble();
        dataStore.dr = numbers[3].toDouble();
        dataStore.kpr = numbers[4].toDouble();
        dataStore.kir = numbers[5].toDouble();
        dataStore.kdr = numbers[6].toDouble();
        break;
    case PIDSPEEDSET: // Pspeed,Ispeed,Dspeed,kp,ki,kd
    case PIDSPEED:    // Pspeed,Ispeed,Dspeed,kp,ki,kd
        dataStore.ps = numbers[1].toDouble();
        dataStore.is = numbers[2].toDouble();
        dataStore.ds = numbers[3].toDouble();
        dataStore.kps = numbers[4].toDouble();
        dataStore.kis = numbers[5].toDouble();
        dataStore.kds = numbers[6].toDouble();
        break;
    case LORABUOYPOS: // STATUS,LAT,LON,mDir,wDir,wStd,BattPecTop,BattPercBott,speedbb,speedsb
        dataStore.status = numbers[1].toInt();
        dataStore.lat = numbers[2].toDouble();
        dataStore.lng = numbers[3].toDouble();
        dataStore.dirMag = numbers[4].toDouble();
        dataStore.wDir = numbers[5].toDouble();
        dataStore.wStd = numbers[6].toDouble();
        dataStore.topAccuP = numbers[7].toInt();
        dataStore.subAccuP = numbers[8].toInt();
        break;
    case DOCKING:
        break;
    case LORALOCKPOS: // LAT,LON,wDir,wStd
    case LORADOCKPOS: // LAT,LON,wDir,wStd
        dataStore.tgLat = numbers[1].toDouble();
        dataStore.tgLng = numbers[2].toDouble();
        dataStore.wDir = numbers[3].toDouble();
        dataStore.wStd = numbers[4].toDouble();
        break;
    case LORADIRDIST:
        dataStore.tgDir = numbers[1].toDouble();
        dataStore.tgDist = numbers[2].toDouble();
        break;
    case IDELING:
        break;
    case PING:
        break;
    case PONG:
        break;
    default:
        Serial.println("RoboDecode: Unkown decode formatter data in " + String(data));
        dataStore.cmd = -1;
        break;
    }
    return dataStore;
}

/*
    Encode outgoing data.
    input DATA RoboSruct and depeding on field msg
    output string format $ID,<data>*xx
    crc has to be added!
*/
String RoboCode(RoboStruct dataOut, int cmd)
{
    String out = String(cmd);
    switch (cmd)
    {
    case IDLE:
        break;
    case DOCKED:
    case LOCKED:
        out += "," + String(dataOut.tgDir, 2);
        out += "," + String(dataOut.tgDist, 2);
        break;
    case REMOTE:
        out += "," + String(dataOut.speed);
        out += "," + String(dataOut.tgDir, 2);
        break;
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
        out += "," + String(dataOut.speed);
        break;
    case SUBSPEED:
        out += "," + String(dataOut.speedSb);
        out += "," + String(dataOut.speedBb);
        out += "," + String(dataOut.speed);
        break;
    case SUBACCU:
        out += "," + String(dataOut.subAccuV, 2);
        out += "," + String(dataOut.subAccuP);
        break;
    case PIDSPEED:
        out += "," + String(dataOut.ps, 3);
        out += "," + String(dataOut.is, 3);
        out += "," + String(dataOut.ds, 3);
        out += "," + String(dataOut.kps, 3);
        out += "," + String(dataOut.kis, 3);
        out += "," + String(dataOut.kds, 3);
        break;
    case PIDRUDDER:
    case PIDRUDDERSET:
        out += "," + String(dataOut.pr, 3);
        out += "," + String(dataOut.ir, 3);
        out += "," + String(dataOut.dr, 3);
        out += "," + String(dataOut.kpr, 3);
        out += "," + String(dataOut.kir, 3);
        out += "," + String(dataOut.kdr, 3);
        break;
    case TOPIDLE:
        break;
    case UDPTGDIRSPEED:
    case LORADIRSPEED: // Dir,speed
    case TOPDIRSPEED:  // Dir,Speed
        out += "," + String(dataOut.tgDir, 2);
        out += "," + String(dataOut.speedSet, 2);
        break;
    case TOPDIRDIST:
        out += "," + String(dataOut.tgDir, 2);
        out += "," + String(dataOut.tgDist, 2);
        break;
    case TOPCALCRUDDER:
        out += "," + String(dataOut.tgDir, 2);
        out += "," + String(dataOut.tgDist, 2);
        out += "," + String(dataOut.speedSet, 2);
        break;
    case LORABUOYPOS:
        out += "," + String(dataOut.status);
        out += "," + String(dataOut.lat, 8);
        out += "," + String(dataOut.lng, 8);
        out += "," + String(dataOut.dirMag, 2);
        out += "," + String(dataOut.wDir, 2);
        out += "," + String(dataOut.wStd, 2);
        out += "," + String(dataOut.topAccuP);
        out += "," + String(dataOut.subAccuP);
        break;
    case DOCKING:
        break;
    case LORALOCKPOS:
    case LORADOCKPOS:
        out += "," + String(dataOut.tgLat, 8);
        out += "," + String(dataOut.tgLng, 8);
        out += "," + String(dataOut.wDir, 1);
        out += "," + String(dataOut.wStd, 1);
        break;
    case LORADIRDIST:
        out += "," + String(dataOut.tgDir, 2);
        out += "," + String(dataOut.tgDist, 2);
        break;
    case LOCKING:
        break;
    case IDELING:
        break;
    case PING:
        out = String(PING);
        break;
    case PONG:
        out = String(PONG);
        break;
    default:
        printf("Robocode: Unkown formatter <%d>\r\n", cmd);
        break;
    }
    return out;
}

String removeBeginAndEndToString(String input)
{
    int startIdx = input.indexOf('$'); // Find the index of $
    int endIdx = input.indexOf('*');   // Find the index of *

    // Check if both $ and * are present and * comes after $
    if (startIdx != -1 && endIdx != -1 && endIdx > startIdx)
    {
        // Extract the substring between $ and *
        return input.substring(startIdx + 1, endIdx);
    }
    else
    {
        // If $ or * is not found, or * comes before $, return an empty string
        return "";
    }
}

// Subroutine to add CRC to a string (similar to NMEA format)
String addCRCToString(String input) // Use reference to modify the original string
{
    // Find where the checksum starts (between '$' and '*')
    int end = input.length() + 1;

    // Calculate the checksum (XOR of all characters between '$' and '*')
    byte crc = 0;
    for (int i = 0; i < end; i++)
    {
        crc ^= input[i]; // XOR operation for each character
    }

    // Convert checksum to hexadecimal format
    char crcHex[3];               // Buffer to hold two hex digits + null terminator
    sprintf(crcHex, "%02X", crc); // Convert byte to uppercase hex string

    // Append the checksum after the asterisk in the string
    input = "$" + input + "*" + crcHex; // Modify input directly
    return input;
}

// Subroutine to check if the checksum in the string is valid
bool verifyCRC(String input)
{
    // Find where the checksum starts (between '├' and '┤')
    int start = input.indexOf('$');
    int end = input.indexOf('*');

    // If the string doesn't contain '$' or '*, it's invalid
    if (start == -1 || end == -1 || end <= start || end + 2 >= input.length())
    {
        printf("crc error\r\n");
        return false; // Invalid format
    }

    // Calculate the checksum (XOR of all characters between '$' and '*')
    byte calculatedCRC = 0;
    for (int i = start + 1; i < end; i++)
    {
        calculatedCRC ^= input[i]; // XOR operation for each character
    }

    // Extract the given checksum from the string (the part after the '*')
    String givenCRC = input.substring(end + 1, end + 3);

    // Convert calculated CRC to a hexadecimal string
    char calculatedCRCHex[3];
    sprintf(calculatedCRCHex, "%02X", calculatedCRC); // Convert byte to hex

    // Compare the calculated checksum with the provided checksum
    return givenCRC.equalsIgnoreCase(calculatedCRCHex);
}

/*
Add new data in buffer
Structure buf[averige][deviation][data0][datan...]
*/
RoboWindStruct addNewSampleInBuffer(RoboWindStruct wData, double nwdata)
{
    if (wData.ptr >= SAMPELS)
    {
        wData.ptr = 0;
    }
    wData.data[wData.ptr] = nwdata;
    wData.ptr++;
    return wData;
}

double averigeWindRose(RoboWindStruct wData)
{
    double sumSin = 0, sumCos = 0;
    for (int i = 0; i < SAMPELS; ++i)
    {
        double angle = (wData.data[i] * M_PI) / 180.0; // Convert to radians
        sumSin += sin(angle);
        sumCos += cos(angle);
    }
    double meanAngle = atan2(sumSin / SAMPELS, sumCos / SAMPELS);   // Compute mean angle
    double meanDegrees = fmod(meanAngle * 180.0 / M_PI + 360, 360); // Convert mean angle to degrees
    wData.wDir = meanDegrees;
    return meanDegrees;
}
/*
compute deviation of a buffer pos
Structure buf[averige][deviation][data0][datan...]
*/
RoboWindStruct deviationWindRose(RoboWindStruct wData)
{
    wData.wDir = averigeWindRose(wData);
    double sumSquaredCircularDiff = 0;
    for (int i = 0; i < SAMPELS; ++i)
    {
        double diff = wData.data[i] - wData.wDir;
        if (diff > 180)
        {
            diff -= 360;
        }
        else if (diff < -180)
        {
            diff += 360;
        }
        sumSquaredCircularDiff += diff * diff;
    }
    wData.wStd = sqrt(sumSquaredCircularDiff / SAMPELS);
    return wData;
}

void PidDecode(String data, int pid, RoboStruct buoy)
{
    int numbers[10]; // Array to hold the decoded numbers (adjust size as needed)
    int count = 0;   // Keep track of the number of extracted numbers
    int cmd = -1;
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
    if (pid == PIDSPEED)
    {
        buoy.ps = numbers[1];
        buoy.is = numbers[2];
        buoy.is = numbers[3];
        buoy.kps = 0;
        buoy.kis = 0;
        buoy.kds = 0;
    }
    if (pid == PIDRUDDER)
    {
        buoy.pr = numbers[1];
        buoy.ir = numbers[2];
        buoy.ir = numbers[3];
        buoy.kpr = 0;
        buoy.kir = 0;
        buoy.kdr = 0;
    }
}

String PidEncode(int pid, RoboStruct buoy)
{
    String out = "";
    if (pid == PIDSPEED)
    {
        out = String(buoy.ps);
        out += "," + String(buoy.is);
        out += "," + String(buoy.ds);
    }
    if (pid == PIDRUDDER)
    {
        out = String(buoy.pr);
        out += "," + String(buoy.ir);
        out += "," + String(buoy.dr);
    }
    return out;
}

double gpsgem[21][2];
int point = 0;
void gpsGem(double &lat, double &lon)
{
    gpsgem[point][0] = lat;
    gpsgem[point++][1] = lon;
    if (point >= 20)
    {
        point = 0;
    }
    gpsgem[20][0] = 0;
    gpsgem[20][1] = 0;
    for (int i = 0; i < 20; i++)
    {
        gpsgem[20][0] += gpsgem[i][0];
        gpsgem[20][1] += gpsgem[i][1];
    }
    lat = gpsgem[20][0] / 20;
    lon = gpsgem[20][1] / 20;
}
/*
    returns distance in meters between two positions, both specified
    as signed decimal-degrees latitude and longitude. Uses great-circle
    distance computation for hypothetical sphere of radius 6372795 meters.
    Because Earth is no exact sphere, rounding errors may be up to 0.5%.
    Courtesy of Maarten Lamers
    Convert degrees to radians
*/
double distanceBetween(double lat1, double lon1, double lat2, double lon2)
{
    lat1 = lat1 * M_PI / 180.0;
    lon1 = lon1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    lon2 = lon2 * M_PI / 180.0;
    // Calculate differences
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    // Haversine formula
    double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    // Distance in kilometers
    return EARTH_MEAN_RADIUS * c;
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2)
{
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

    double bearing = atan2(y, x) * (180.0 / M_PI); // Convert to degrees
    return fmod((bearing + 360), 360);             // Normalize to 0-360
}

/*Approimate roling average deafualt 100 samples*/
double approxRollingAverage(double avg, double input)
{
    avg -= avg / 100;
    avg += input / 100;
    return avg;
}

/*
change parematers for speed calculation
*/
void checkparameters(RoboStruct buoy)
{
    /*
    sanety check
    */
    if (buoy.minOfsetDist < 0)
    {
        buoy.minOfsetDist = 2;
    }
    if (buoy.maxOfsetDist > 100)
    {
        buoy.maxOfsetDist = 20;
    }
    if (buoy.minSpeed < 0)
    {
        buoy.minSpeed = 0;
    }
    if (buoy.maxSpeed > 80)
    {
        buoy.maxSpeed = 80;
    }
    if (buoy.minOfsetDist >= buoy.maxOfsetDist)
    {
        buoy.maxOfsetDist = buoy.minOfsetDist + 2;
    }
}

/*
input: directon to go to, distance and start position
return: target latitude and longitude
*/
void adjustPositionDirDist(double bearing, double distance, double lat1, double lon1, double *lat2, double *lon2)
{
    double d, phi1;
    double dlat, dlon;
    double theta = radians(bearing);
    d = degrees(distance) / EARTH_MEAN_RADIUS; // arc length in degrees
    phi1 = radians(lat1);                      // convert lat to radians
    dlat = d * cos(theta);                     // latitude change in degrees*10^6
    dlon = d * sin(theta) / cos(phi1);         // longitude change, corrected for latitude
    *lat2 = lat1 + dlat;
    *lon2 = lon1 + dlon;
}

double smallestAngle(double heading1, double heading2)
{
    double angle = fmod(heading2 - heading1 + 360, 360); // Calculate the difference and keep it within 360 degrees
    if (angle > 180)
    {
        return angle - 360; // Angle is greater than 180, SB (turn right)
    }
    return angle; // Angle is less than or equal to 180, BB (Turn left)
}

/*
    calculate the smallest angle between two directions
    return 1 if angle is >180
*/
bool determineDirection(double heading1, double heading2)
{
    double angle = fmod(heading2 - heading1 + 360, 360); // Calculate the difference and keep it within 360 degrees
    if (angle > 180)
    {
        return 1; // Angle is greater than 180, SB (turn right)
    }
    else
    {
        return 0; // Angle is less than or equal to 180, BB (Turn left)
    }
}

/*
    adjust speed Cosinus does not work here. We have to do it manually
*/
double Angle2SpeedFactor(double angle)
{
    // cos(correctonAngle * M_PI / 180.0); not working for mall angles
    angle = constrain(angle, 0, 180);
    if (angle <= 90)
    {
        return (map(angle, 0, 90, 100, 0) / 100.0);
    }
    else
    {
        return (map(angle, 90, 180, 0, -100) / 100.0);
    }
}

/*
calculate the speed sailing home.
The distance is in meters.
*/
double CalcDocSpeed(double tgdistance)
{
    tgdistance = constrain(tgdistance, 0.5, 8);
    return map(tgdistance, 0, 5, 0, 50); // map speed 0.5-5 meter -> BUOYMINSPEED <-> BUOYMAXSPEED
}

/*
Compute power to trusters
*/
RoboStruct CalcRemoteRudderBuoy(RoboStruct buoy)
{
    double error = smallestAngle(buoy.dirMag, buoy.tgDir);
    double corr = Angle2SpeedFactor(abs(error));
    double tbb, tsb;
    if (error > 0)
    {
        tbb = buoy.speedSet + buoy.speedSet * (1 - corr);
        tsb = buoy.speedSet * corr;
    }
    else
    {
        tbb = buoy.speedSet * corr;
        tsb = buoy.speedSet + buoy.speedSet * (1 - corr);
    }

    tbb = (int)(buoy.speedSet * cos(radian(error)) * (1 - sin(radian(error))));
    tsb = (int)(buoy.speedSet * cos(radian(error)) * (1 - sin(radian(error)) * -1));
    buoy.speedBb = (int)constrain(tbb, -60, 100);
    buoy.speedSb = (int)constrain(tsb, -60, 100);
    Serial.printf("Error=%lf BB=%d SB=%d\r\n\r\n", error, buoy.speedBb, buoy.speedSb);
    return buoy;
}

/*
Only used PID if the distancs is less than buoy.maxOfsetDist return BUOYMAXSPEED otherwise.
*/
RoboStruct hooverPid(RoboStruct buoy)
{

    /*How long since we last calculated*/
    double Output = 0;
    unsigned long now = millis();
    if (now > buoy.lastTimes + 1000) // reset afterlong time
    {
        buoy.lastErrs = 0;
        buoy.lastTimes = now;
    }
    double timeChange = (double)(now - buoy.lastTimes);
    /*Compute all the working error variables*/
    // double error = (buoy.tgDist - buoy.minOfsetDist);
    double error = (buoy.tgDist - 2);
    buoy.errSums += (error * timeChange);
    double dErr = (error - buoy.lastErrs) / timeChange;
    /* Do not sail backwards*/
    // if (buoy.iintergrates < 0)
    if (buoy.kis * buoy.errSums < 0)
    {
        buoy.errSums = 0;
    }
    // /*max 70% I correction*/
    if (buoy.kis * buoy.errSums > 70)
    {
        buoy.errSums = 70 / buoy.kis;
    }
    /*Compute PID Output*/
    buoy.ps = buoy.kps * error;
    buoy.is = buoy.kis * buoy.errSums;
    buoy.ds = buoy.kds * dErr;
    Output = buoy.ps + buoy.is + buoy.ds;
    /* Do not sail backwards*/
    if (Output < 0)
    {
        Output = 0;
    }
    /*Remember some variables for next time*/
    buoy.lastErrs = error;
    buoy.lastTimes = now;
    if (Output > buoy.maxSpeed)
    {
        Output = buoy.maxSpeed;
    }
    buoy.speedSet = Output;
    // printf("Output:%f error%f buoy.iintergrates:%f p:%.0f i:%.4f ", Output, error, buoy.errSums, buoy.ps, buoy.is);
    return buoy;
}

/*
    Position calculations
*/
// Function to calculate the centroid (average point)
void threePointAverage(struct RoboStruct p3[3], double *latgem, double *lnggem)
{
    *latgem = (p3[0].tgLat + p3[1].tgLat + p3[2].tgLat) / 3;
    *lnggem = (p3[0].tgLng + p3[1].tgLng + p3[2].tgLng) / 3;
}
// Function to calculate the centroid (average point)
void twoPointAverage(double lat1, double lon1, double lat2, double lon2, double *latgem, double *longem)
{
    *latgem = (lat1 + lat2) / 2;
    *longem = (lon1 + lon2) / 2;
}

// Function to convert wind direction in degrees to a vector
void windDirectionToVector(double windDegrees, double *windX, double *windY)
{
    // Normalize wind direction to [0, 360)
    while (windDegrees < 0)
    {
        windDegrees += 360;
    }
    while (windDegrees >= 360)
    {
        windDegrees -= 360;
    }
    // Convert degrees to radians
    double windRadians = windDegrees * M_PI / 180.0;
    // Calculate the wind vector components
    *windX = cos(windRadians);
    *windY = sin(windRadians);
}

// Function to calculate the angle between two vectors in degrees
double calculateAngle(double x1, double y1, double x2, double y2)
{
    double dotProduct = x1 * x2 + y1 * y2;
    double magnitudeA = sqrt(x1 * x1 + y1 * y1);
    double magnitudeB = sqrt(x2 * x2 + y2 * y2);
    // Calculate the cosine of the angle
    double cosAngle = dotProduct / (magnitudeA * magnitudeB);
    // Clamp the value to the range [-1, 1] to avoid NaN from acos
    if (cosAngle > 1.0)
        cosAngle = 1.0;
    if (cosAngle < -1.0)
        cosAngle = -1.0;
    // Return the angle in degrees
    return acos(cosAngle) * (180.0 / M_PI);
}

// Function to calculate bearing from one point to another using latitude and longitude
double computeWindAngle(double windDegrees, double lat, double lon, double centroidLat, double centroidLon)
{
    // Step 1: Calculate the bearing from the centroid to the target point
    double bearing = calculateBearing(centroidLat, centroidLon, lat, lon);
    // Step 2: Calculate the angle difference between the bearing and wind direction
    double angleDifference = fabs(windDegrees - bearing);
    if (angleDifference > 180)
        angleDifference = 360 - angleDifference; // Normalize to 0-180 range
    return angleDifference;
}

/************************************************************************************************************************************************************************* */
// Conversion ready until here
/************************************************************************************************************************************************************************* */
RoboStruct recalcStarLine(struct RoboStruct rsl[3])
{
    double d0, d1, d2, dir;
    double mid[2];
    double angleSb = 0;
    rsl[0].trackPos = -1;
    rsl[1].trackPos = -1;
    rsl[2].trackPos = -1;
    angleSb = rsl[0].wDir + 90;
    if (angleSb > 360)
    {
        angleSb -= 360;
    }
    double angleBb = angleSb + 180;
    if (angleBb > 360)
    {
        angleBb -= 360;
    }

    d0 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng); // compute length p0 p1
    d1 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng); // compute length p0 p2
    d2 = distanceBetween(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng); // compute length p1 p2
    if (d0 < d1 && d0 < d2)
    {
        if (rsl[0].tgLat == 0 || rsl[0].tgLng == 0 || rsl[1].tgLat == 0 || rsl[1].tgLng == 0) // check if data is ok
        {
            printf("No data to compute with\r\n");
            return rsl[3];
        }
        angleSb = rsl[0].wDir + 90;
        if (angleSb > 360)
        {
            angleSb -= 360;
        }
        double angleBb = angleSb + 180;
        if (angleBb > 360)
        {
            angleBb -= 360;
        }
        twoPointAverage(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng, &mid[0], &mid[1]); // calculate startline centerpoint
        printf("midpoint =(%.12f,%.12f)\r\n", mid[0], mid[1]);                                     // mid point of thre (for plotting the windrose and calculations)
        adjustPositionDirDist(angleBb, d0 / 2, mid[0], mid[1], &rsl[0].tgLat, &rsl[0].tgLng);      // compute port
        adjustPositionDirDist(angleSb, d0 / 2, mid[0], mid[1], &rsl[1].tgLat, &rsl[1].tgLng);      // compute starboard
        printf("winddir =(%.1f)\r\n", rsl[0].wDir);                                                // wind direction
        rsl[0].trackPos = PORT;
        rsl[1].trackPos = STARBOARD;
        return rsl[3];
    }

    if (d1 < d0 && d0 < d2)
    {
        if (rsl[0].tgLat == 0 || rsl[0].tgLng == 0 || rsl[2].tgLat == 0 || rsl[2].tgLng == 0) // check if data is ok
        {
            printf("No data to compute with\r\n");
            return rsl[3];
        }
        angleSb = rsl[0].wDir + 90;
        if (angleSb > 360)
        {
            angleSb -= 360;
        }
        double angleBb = angleSb + 180;
        if (angleBb > 360)
        {
            angleBb -= 360;
        }
        twoPointAverage(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng, &mid[0], &mid[1]); // calculate startline centerpoint
        printf("midpoint =(%.12f,%.12f)\r\n", mid[0], mid[1]);                                     // mid point of thre (for plotting the windrose and calculations)
        adjustPositionDirDist(angleBb, d1 / 2, mid[0], mid[1], &rsl[0].tgLat, &rsl[0].tgLng);      // compute port
        adjustPositionDirDist(angleSb, d1 / 2, mid[0], mid[1], &rsl[2].tgLat, &rsl[2].tgLng);      // compute starboard
        printf("winddir =(%.1f)\r\n", rsl[0].wDir);                                                // wind direction
        rsl[0].trackPos = PORT;
        rsl[2].trackPos = STARBOARD;
        return rsl[3];
    }

    if (d2 < d0 && d2 < d1)
    {
        if (rsl[1].tgLat == 0 || rsl[1].tgLng == 0 || rsl[2].tgLat == 0 || rsl[2].tgLng == 0) // check if data is ok
        {
            printf("No data to compute with\r\n");
            return rsl[3];
        }
        angleSb = rsl[1].wDir + 90;
        if (angleSb > 360)
        {
            angleSb -= 360;
        }
        double angleBb = angleSb + 180;
        if (angleBb > 360)
        {
            angleBb -= 360;
        }
        twoPointAverage(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng, &mid[0], &mid[1]); // calculate startline centerpoint
        printf("midpoint =(%.12f,%.12f)\r\n", mid[0], mid[1]);                                     // mid point of thre (for plotting the windrose and calculations)
        adjustPositionDirDist(angleBb, d2 / 2, mid[0], mid[1], &rsl[1].tgLat, &rsl[1].tgLng);      // compute port
        adjustPositionDirDist(angleSb, d2 / 2, mid[0], mid[1], &rsl[2].tgLat, &rsl[2].tgLng);      // compute starboard
        printf("winddir =(%.1f)\r\n", rsl[1].wDir);                                                // wind direction
        rsl[1].trackPos = PORT;
        rsl[2].trackPos = STARBOARD;
        return rsl[3];
    }
    return rsl[3];
}

/*
    compute new positons for 3 buoys
    Automaticly determs the startline and wich buoy is Head
    Restriction wind does not change more than 90 degrees
    INPUT:
    - Wind direction
    - Position buoy1
    - Position buoy2
    - Position buoy3
    Return:
    - New position buoy1, HEAD,PORT,STARBORD
    - New position buoy2, HEAD,PORT,STARBORD
    - New position buoy3, HEAD,PORT,STARBORD
*/
RoboStruct reCalcTrack(struct RoboStruct rsl[3])
{
    double d0, d1, d2;
    double startLineL, centerPont2Startline, centerPoint2Head;
    double angleSb, angleBb, angle180, dir;
    double lat2gem, lng2gem;
    double lat3gem, lng3gem;
    rsl[0].trackPos = -1;
    rsl[1].trackPos = -1;
    rsl[2].trackPos = -1;
    for (int i = 0; i < 2; i++)
    {
        if (rsl[i].tgLng == 0 || rsl[i].tgLat == 0)
        {
            printf("No data to compute with\r\n");
            return rsl[3];
        }
    }
    // mid point of three buoys (for plotting the windrose and calculations)
    threePointAverage(rsl, &lat3gem, &lng3gem);
    printf("midpoint =(%.12f,%.12f)\r\n", lat3gem, lng3gem);

    // determ the length of the startline
    d0 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng); // compute length p0 p1
    d1 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng); // compute length p0 p2
    d2 = distanceBetween(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng); // compute length p1 p2
    if (d0 < d1 && d0 < d2)
    {
        startLineL = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng);        // compute length startline
        twoPointAverage(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng, &lat2gem, &lng2gem); // calculate startline centerpoint
    }
    if (d1 < d2 && d1 < d0)
    {
        startLineL = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng);        // compute length startline
        twoPointAverage(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng, &lat2gem, &lng2gem); // calculate startline centerpoint
    }
    if (d2 < d0 && d2 < d1)
    {
        startLineL = distanceBetween(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng);        // compute length startline
        twoPointAverage(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng, &lat2gem, &lng2gem); // calculate startline centerpoint
    }
    // determ  he wind angles for the start line (+-90 degrees from the wind direction)
    angle180 = rsl[0].wDir + 180;
    if (angle180 > 360)
    {
        angle180 -= 360;
    }
    angleSb = rsl[0].wDir + 90;
    if (angleSb > 360)
    {
        angleSb -= 360;
    }
    angleBb = angleSb + 180;
    if (angleBb > 360)
    {
        angleBb -= 360;
    }
    // search for the buoy clocesed to the wind direction
    double b0 = computeWindAngle(rsl[0].wDir, rsl[0].tgLat, rsl[0].tgLng, lat3gem, lng3gem);
    double b1 = computeWindAngle(rsl[0].wDir, rsl[1].tgLat, rsl[1].tgLng, lat3gem, lng3gem);
    double b2 = computeWindAngle(rsl[0].wDir, rsl[2].tgLat, rsl[2].tgLng, lat3gem, lng3gem);
    if (b0 < b1 && b0 < b2)
    {
        centerPoint2Head = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, lat3gem, lng3gem);                     // calcultate the distance between the centerpoint and the startline
        centerPont2Startline = distanceBetween(lat2gem, lng2gem, lat3gem, lng3gem);                           // calcultate the distance between the centerpoint and the startline
        adjustPositionDirDist(rsl[0].wDir, centerPoint2Head, lat3gem, lng3gem, &rsl[0].tgLat, &rsl[0].tgLng); // calculate new position HEAD
        adjustPositionDirDist(angle180, centerPont2Startline, lat3gem, lng3gem, &lat2gem, &lng2gem);          // point to new centerpoint of the startline
        adjustPositionDirDist(angleBb, startLineL / 2, lat2gem, lng2gem, &rsl[1].tgLat, &rsl[1].tgLng);       // compute port
        adjustPositionDirDist(angleSb, startLineL / 2, lat2gem, lng2gem, &rsl[2].tgLat, &rsl[2].tgLng);       // compute starboard
        rsl[0].trackPos = HEAD;
        rsl[1].trackPos = PORT;
        rsl[2].trackPos = STARBOARD;
        return rsl[3];
    }
    if (b1 < b0 && b1 < b2)
    {
        centerPont2Startline = distanceBetween(lat2gem, lng2gem, lat3gem, lng3gem);                           // calcultate the distance between the centerpoint and the startline
        centerPoint2Head = distanceBetween(rsl[1].tgLat, rsl[1].tgLng, lat3gem, lng3gem);                     // calcultate the distance between the centerpoint and the startline
        adjustPositionDirDist(rsl[0].wDir, centerPoint2Head, lat3gem, lng3gem, &rsl[1].tgLat, &rsl[1].tgLng); // calculate new position HEAD
        adjustPositionDirDist(angle180, centerPont2Startline, lat3gem, lng3gem, &lat2gem, &lng2gem);          // point to new centerpoint of the startline
        adjustPositionDirDist(angleBb, startLineL / 2, lat2gem, lng2gem, &rsl[0].tgLat, &rsl[0].tgLng);       // compute port
        adjustPositionDirDist(angleSb, startLineL / 2, lat2gem, lng2gem, &rsl[2].tgLat, &rsl[2].tgLng);       // compute starboard
        rsl[0].trackPos = PORT;
        rsl[1].trackPos = HEAD;
        rsl[2].trackPos = STARBOARD;
        return rsl[3];
    }
    if (b2 < b0 && b2 < b1)
    {
        centerPont2Startline = distanceBetween(lat2gem, lng2gem, lat3gem, lng3gem);                           // calcultate the distance between the centerpoint and the startline
        centerPoint2Head = distanceBetween(rsl[2].tgLat, rsl[2].tgLng, lat3gem, lng3gem);                     // calcultate the distance between the centerpoint and the startline
        adjustPositionDirDist(rsl[0].wDir, centerPoint2Head, lat3gem, lng3gem, &rsl[2].tgLat, &rsl[2].tgLng); // calculate new position HEAD
        adjustPositionDirDist(angle180, centerPont2Startline, lat3gem, lng3gem, &lat2gem, &lng2gem);          // point to new centerpoint of the startline
        adjustPositionDirDist(angleBb, startLineL / 2, lat2gem, lng2gem, &rsl[0].tgLat, &rsl[0].tgLng);       // compute port
        adjustPositionDirDist(angleSb, startLineL / 2, lat2gem, lng2gem, &rsl[1].tgLat, &rsl[1].tgLng);       // compute starboard
        rsl[0].trackPos = PORT;
        rsl[1].trackPos = STARBOARD;
        rsl[2].trackPos = HEAD;
        return rsl[3];
    }
    return rsl[3];
}

/*
    print positions
*/
void trackPosPrint(int c)
{
    if (c == HEAD)
    {
        printf("HEAD");
    }
    else if (c == PORT)
    {
        printf("PORT");
    }
    else if (c == STARBOARD)
    {
        printf("STARBOARD");
    }
    else
    {
        printf("NON");
    }
}

RoboStruct calcTrackPos(RoboStruct rsl[3])
{
    double d0 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng); // compute length p0 p1
    double d1 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng); // compute length p0 p2
    double d2 = distanceBetween(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng); // compute length p1 p2
    if (d0 < d1 && d0 < d2)
    {
        rsl[0].trackPos = PORT;
        rsl[1].trackPos = STARBOARD;
        rsl[2].trackPos = HEAD;
    }
    if (d1 < d0 && d1 < d2)
    {
        rsl[0].trackPos = PORT;
        rsl[1].trackPos = HEAD;
        rsl[2].trackPos = STARBOARD;
    }
    if (d2 < d0 && d2 < d1)
    {
        rsl[0].trackPos = HEAD;
        rsl[1].trackPos = PORT;
        rsl[2].trackPos = STARBOARD;
    }
    return rsl[3];
}