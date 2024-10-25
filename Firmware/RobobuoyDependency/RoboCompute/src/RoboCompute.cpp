#include <arduino.h>
#include "RoboCompute.h"

/*
    decode incomming data.
    input string format $ID,x,y,z,x,x,x*
    output parameters in to stuct type RoboStruct
    crc does not have any value
*/
struct RoboStruct RoboDecode(String data, RoboStruct dataStore)
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
        dataStore.speed = numbers[4].toInt();
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
    return dataStore;
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
        out += "," + String(dataOut.speed);
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

/************************************************************************************************************************************************************************* */
// OLD ROBOCALC
/************************************************************************************************************************************************************************* */
#include <math.h>
/*some constans*/
#define _GPS_EARTH_MEAN_RADIUS 6371009 // old: 6372795
#define EARTHRADIUS 6371
#define radian(x) (x * M_PI / 180)
#define degre(x) (x * 180 / M_PI)
#define ILIM 35 // Maximum interal limit (35% power)

String addBeginAndEndToString(String input)
{
    input = "$" + input + "*";
    return input;
}

// Subroutine to add CRC to a string (similar to NMEA format)
String addCRCToString(String input) // Use reference to modify the original string
{
    // Find where the checksum starts (between '$' and '*')
    int start = input.indexOf('$');
    int end = input.indexOf('*');

    // If there's no '$' or '*' in the string, return without changes
    if (start == -1 || end == -1 || end <= start)
    {
        return input; // Invalid format, do nothing and return
    }

    // Calculate the checksum (XOR of all characters between '$' and '*')
    byte crc = 0;
    for (int i = start + 1; i < end; i++)
    {
        crc ^= input[i]; // XOR operation for each character
    }

    // Convert checksum to hexadecimal format
    char crcHex[3];               // Buffer to hold two hex digits + null terminator
    sprintf(crcHex, "%02X", crc); // Convert byte to uppercase hex string

    // Append the checksum after the asterisk in the string
    input += crcHex; // Modify input directly
    return input;
}

// Subroutine to check if the checksum in the string is valid
bool verifyCRC(String input)
{
    // Find where the checksum starts (between '├' and '┤')
    int start = input.indexOf('$');
    int end = input.indexOf('*');

    // If the string doesn't contain '├' or '┤', it's invalid
    if (start == -1 || end == -1 || end <= start || end + 2 >= input.length())
    {
        printf("crc error\r\n");
        return false; // Invalid format
    }

    // Calculate the checksum (XOR of all characters between '├' and '┤')
    byte calculatedCRC = 0;
    for (int i = start + 1; i < end; i++)
    {
        calculatedCRC ^= input[i]; // XOR operation for each character
    }

    // Extract the given checksum from the string (the part after the '┤')
    String givenCRC = input.substring(end + 1, end + 3);

    // Convert calculated CRC to a hexadecimal string
    char calculatedCRCHex[3];
    sprintf(calculatedCRCHex, "%02X", calculatedCRC); // Convert byte to hex

    // Compare the calculated checksum with the provided checksum
    return givenCRC.equalsIgnoreCase(calculatedCRCHex);
}

double averigeWindRose(double samples[], int n)
{
    double sumSin = 0, sumCos = 0;
    for (int i = 0; i < n; ++i)
    {
        double angle = (samples[i + 3] * M_PI) / 180.0; // Convert to radians
        sumSin += sin(angle);
        sumCos += cos(angle);
    }
    double meanAngle = atan2(sumSin / n, sumCos / n);               // Compute mean angle
    double meanDegrees = fmod(meanAngle * 180.0 / M_PI + 360, 360); // Convert mean angle to degrees
    samples[0] = meanDegrees;
    return meanDegrees;
}
/*
compute deviation of a buffer pos
Structure buf[averige][deviation][data0][datan...]
*/
double deviationWindRose(double samples[], int n)
{
    double mean = averigeWindRose(samples, n);
    double sumSquaredCircularDiff = 0;
    for (int i = 0; i < n; ++i)
    {
        double diff = samples[i + 3] - mean;
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
    samples[1] = sqrt(sumSquaredCircularDiff / n);
    return samples[1];
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

double distanceBetween(double lat1, double long1, double lat2, double long2)
{
    // returns distance in meters between two positions, both specified
    // as signed decimal-degrees latitude and longitude. Uses great-circle
    // distance computation for hypothetical sphere of radius 6371009 meters.
    // Because Earth is no exact sphere, rounding errors may be up to 0.5%.
    // Courtesy of Maarten Lamers
    double delta = radians(long1 - long2);
    double sdlong = sin(delta);
    double cdlong = cos(delta);
    lat1 = radians(lat1);
    lat2 = radians(lat2);
    double slat1 = sin(lat1);
    double clat1 = cos(lat1);
    double slat2 = sin(lat2);
    double clat2 = cos(lat2);
    delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
    delta = sq(delta);
    delta += sq(clat2 * sdlong);
    delta = sqrt(delta);
    double denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
    delta = atan2(delta, denom);
    return delta * _GPS_EARTH_MEAN_RADIUS;
}

double courseTo(double lat1, double long1, double lat2, double long2)
{
    // returns course in degrees (North=0, West=270) from position 1 to position 2,
    // both specified as signed decimal-degrees latitude and longitude.
    // Because Earth is no exact sphere, calculated course may be off by a tiny fraction.
    // Courtesy of Maarten Lamers
    double dlon = radians(long2 - long1);
    lat1 = radians(lat1);
    lat2 = radians(lat2);
    double a1 = sin(dlon) * cos(lat2);
    double a2 = sin(lat1) * cos(lat2) * cos(dlon);
    a2 = cos(lat1) * sin(lat2) - a2;
    a2 = atan2(a1, a2);
    if (a2 < 0.0)
    {
        a2 += TWO_PI;
    }
    return degrees(a2);
}

void initPid(int pid, RoboStruct buoy)
{
    if (pid == PIDSPEED)
    {
        buoy.lastErrs = 0;
        buoy.lastTimes = millis();
        buoy.iintergrates = 0;
    }
    if (pid == PIDRUDDER)
    {
        buoy.iintergrater = 0;
        buoy.lastErrr = 0;
        buoy.lastTimer = millis();
    }
}

/*Approimate roling average deafualt 100 samples*/
double approxRollingAverage(double avg, double input)
{
    avg -= avg / 100;
    avg += input / 100;
    return avg;
}

/*
Add new data in buffer
Structure buf[averige][deviation][data0][datan...]
*/
void addNewSampleInBuffer(double *input, int buflen, double nwdata)
{
    if (input[2] >= buflen)
    {
        input[2] = 0;
    }
    input[(int)input[2] + 3] = nwdata;
    input[2]++;
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
void adjustPositionDirDist(double dir, double dist, double *lat, double *lon)
{
    /*compute in radians*/
    double radLat = radian(*lat);
    double radLon = radian(*lon);
    double brng = radian(dir);
    double d = dist * 1.0 / 1000;
    /*compute*/
    double radLat2 = asin(sin(radLat) * cos(d / EARTHRADIUS) + cos(radLat) * sin(d / EARTHRADIUS) * cos(brng));
    double radLon2 = radLon + atan2(sin(brng) * sin(d / EARTHRADIUS) * cos(radLat), cos(d / EARTHRADIUS) - sin(radLat) * sin(radLat2));
    /*plot old pos*/
    // Serial.printf("Old lock google: https://www.google.nl/maps/@%2.12lf,%2.12lf\r\n", *lat, *lon);
    // Serial.printf("Old lock openst: https://www.openstreetmap.org/#map=19/%2.12lf/%2.12lf\r\n", *lat, *lon);
    /*convert back to degrees*/
    *lat = degre(radLat2);
    *lon = degre(radLon2);
    /*plot new pos*/
    // Serial.printf("New lock google: https://www.google.nl/maps/@%2.12lf,%2.12lf\r\n", *lat, *lon);
    // Serial.printf("New lock openst: https://www.openstreetmap.org/#map=19/%2.12lf/%2.12lf\r\n", *lat, *lon);
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
    Calculate power to thrusters
    if distance between 0.5 and 1.5 meter rotate at the slowest speed.
    else do normal rudder calculation.
*/
double push = 0;
#define ILIM 35 // Maximum interal limit (35% power)
RoboStruct CalcRudderBuoy(RoboStruct buoy)
{
    double error = smallestAngle(buoy.dirMag, buoy.tgDir);
    /*Rotate to target direction first*/
    if (buoy.tgDist > 0.5 && abs(error) > 45)
    {
        float power = map(abs(error), 45, 180, 2, 20);
        power = sin(radians(power)) * 100;
        power = (int)map(power, 13, 100, 13, buoy.maxSpeed / 2);
        power += push;
        push += 0.01;
        power = constrain(power, 0, buoy.maxSpeed);
        if (error >= 0)
        {
            buoy.speedBb = power;
            buoy.speedSb = -power;
        }
        else
        {
            buoy.speedBb = -power;
            buoy.speedSb = power;
        }
        buoy.pr = 0;
        buoy.ir = 0;
        buoy.dr = 0;
        buoy.iintergrater = 0;
        buoy.lastErrr = 0;
        buoy.lastTimer = millis();
        return buoy;
    }
    push = 0;
    /*calculate proportion thrusters*/
    /*Scale error in range for tan*/
    error = map(error, -180, 180, -80, 80);
    unsigned long now = millis();
    double timeChange = (double)(now - buoy.lastTimer);
    double dErr = (error - buoy.lastErrr) / timeChange;
    buoy.iintergrater += error * timeChange;
    if ((buoy.kir / 1000) * buoy.iintergrater > ILIM)
    {
        buoy.iintergrater = ILIM / ((buoy.kir / 1000));
    }
    if ((buoy.kir / 1000) * buoy.iintergrater < -ILIM)
    {
        buoy.iintergrater = -ILIM / ((buoy.kir / 1000));
    }
    buoy.pr = buoy.kpr * error;
    buoy.ir = (buoy.kir / 1000) * buoy.iintergrater;
    buoy.dr = buoy.kdr * dErr;
    double adj = buoy.pr + buoy.ir + buoy.dr;
    buoy.lastErrr = error;
    buoy.lastTimer = now;
    buoy.speedBb = (int)(buoy.speed * (1 + tan(radians(adj))));
    buoy.speedSb = (int)(buoy.speed * (1 - tan(radians(adj))));
    /*Sanety check*/
    buoy.speedBb = constrain(buoy.speedBb, -buoy.maxSpeed, buoy.maxSpeed);
    buoy.speedSb = constrain(buoy.speedSb, -buoy.maxSpeed, buoy.maxSpeed);
    // printf("dErr%f buoy.iintergrater:%f pr%f ir%f ", dErr, buoy.iintergrater, buoy.pr, buoy.ir);
    return buoy;
}

/*
Only used PID if the distancs is less than buoy.maxOfsetDist return BUOYMAXSPEED otherwise.
*/
RoboStruct hooverPid(RoboStruct buoy)
{
    /*Do not use the pid loop if distance is to big just go full power*/
    if (buoy.maxSpeed == 51) /*old config use this by setting max speed to 51*/
    {
        if (buoy.tgDist > buoy.maxOfsetDist)
        {
            buoy.speed = buoy.maxSpeed;
            return buoy;
        }
        if (buoy.armIntergrators == false)
        {
            buoy.armIntergrators = true;
            buoy.iintergrates = 0;
        }
    }
    else
    {
        /*Do not use the pid loop if distance is to big just go full power*/
        if (buoy.armIntergrators == false)
        {
            if (buoy.tgDist > 3)
            {
                buoy.iintergrates = 0;
                buoy.speed = buoy.maxSpeed;
                return buoy;
            }
            else
            {
                buoy.armIntergrators = true;
                buoy.iintergrates = 0;
            }
        }
    }
    /*How long since we last calculated*/
    double Output = 0;
    unsigned long now = millis();
    double timeChange = (double)(now - buoy.lastTimes);
    /*Compute all the working error variables*/
    double error = (buoy.tgDist - buoy.minOfsetDist) / 1000;
    buoy.iintergrates += (error * timeChange);
    double dErr = (error - buoy.lastErrs) / timeChange;
    /* Do not sail backwards*/
    if (buoy.iintergrates < 0)
    {
        buoy.iintergrates = 0;
    }
    /*max 70% I correction*/
    if (buoy.kis * buoy.iintergrates > 70)
    {
        buoy.iintergrates = 70 / buoy.kis;
    }
    /*Compute PID Output*/
    buoy.ps = buoy.kps * (buoy.tgDist - buoy.minOfsetDist);
    buoy.is = buoy.kis * buoy.iintergrates;
    buoy.ds = buoy.kds * dErr;
    Output = buoy.ps + buoy.is + buoy.ds;
    /* Do not sail backwards*/
    if (Output < 0)
    {
        Output = 0;
    }
    /*Remember some variables for next time*/
    buoy.lastErrs = buoy.tgDist;
    buoy.lastTimes = now;
    // printf("Output:%f dErr%f buoy.iintergrates:%f ps%f is%f ", Output, dErr, buoy.iintergrates, buoy.ps, buoy.is);
    buoy.speed = (int)Output;
    buoy.speed = (int)Output;
    return buoy;
}

/*
    Position calculations
*/
void threePointAverage(double lat1, double lon1, double lat2, double lon2, double lat3, double lon3, double *latgem, double *longem)
{
    // Calculate the average of latitude and longitude
    *latgem = (lat1 + lat2 + lat3) / 3;
    *longem = (lon1 + lon2 + lon3) / 3;
}

void twoPointAverage(double lat1, double lon1, double lat2, double lon2, double *latgem, double *longem)
{
    // Calculate the average of latitude and longitude
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

// Function to check the position relative to the wind direction
int checkWindDirection(double windDegrees, double lat, double lon, double centroidX, double centroidY)
{
    // Convert degrees to radians
    double windRadians = windDegrees * M_PI / 180.0;
    // Calculate the wind vector components
    double windX = cos(windRadians);
    double windY = sin(windRadians);

    // Vector from centroid to point
    double vectorPX = lat - centroidX;
    double vectorPY = lon - centroidY;
    // Calculate the angle between the wind vector and the vector to the point
    double angle = calculateAngle(windX, windY, vectorPX, vectorPY);
    if (angle <= 45.0)
    {
        return HEAD; // Point is within ±45 degrees of the wind direction
    }
    else
    {
        // Calculate the cross product to determine starboard or port
        double crossProduct = windX * vectorPY - windY * vectorPX;

        if (crossProduct > 0)
        {
            return PORT; // Left of the wind direction
        }
        else
        {
            return STARBOARD; // Right of the wind direction
        }
    }
}

/************************************************************************************************************************************************************************* */
// Conversion ready until here
/************************************************************************************************************************************************************************* */
void reCalcStartLine(double *lat1, double *lon1, double *lat2, double *lon2, double winddir)
{
    double latgem, longem;
    double lat, lon;
    double dist = distanceBetween(*lat1, *lon1, *lat2, *lon2);
    twoPointAverage(*lat1, *lon1, *lat2, *lon2, &latgem, &longem);
    dist = dist / 2;
    int buoypos = checkWindDirection(winddir, *lat1, *lon1, latgem, longem);
    if (buoypos == PORT)
    {
        double dir = winddir + 90;
        if (dir > 360)
        {
            dir -= 360;
        }
        lat = *lat1;
        lon = *lon1;
        adjustPositionDirDist(dir, dist, &lat, &lon);
        *lat1 = lat;
        *lon1 = lon;
        dir = winddir - 90;
        if (dir < 0)
        {
            dir += 360;
        }
        lat = *lat2;
        lon = *lon2;
        adjustPositionDirDist(dir, dist, &lat, &lon);
        *lat2 = lat;
        *lon2 = lon;
    }
    else
    {
        double dir = winddir + 90;
        if (dir > 360)
        {
            dir -= 360;
        }
        lat = *lat2;
        lon = *lon2;
        adjustPositionDirDist(dir, dist, &lat, &lon);
        *lat2 = lat;
        *lon2 = lon;
        dir = winddir - 90;
        if (dir < 0)
        {
            dir += 360;
        }
        lat = *lat1;
        lon = *lon1;
        adjustPositionDirDist(dir, dist, &lat, &lon);
        *lat1 = lat;
        *lon1 = lon;
    }
}