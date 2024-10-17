#include <Arduino.h>
#include "RoboCalc.h"
#define _GPS_EARTH_MEAN_RADIUS 6371009 // old: 6372795

pid buoy;

int PidDecodeTop(String data)
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
    buoy.speedP = numbers[1];
    buoy.speedI = numbers[2];
    buoy.speedD = numbers[3];
    buoy.speedKp = 0;
    buoy.speedKi = 0;
    buoy.speedKd = 0;
    return numbers[0];
}

int PidDecodeSub(String data)
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
    buoy.rudderP = numbers[1];
    buoy.rudderI = numbers[2];
    buoy.rudderD = numbers[3];
    buoy.rudderKp = 0;
    buoy.rudderKi = 0;
    buoy.rudderKd = 0;
    return numbers[0];
}

String PidCodeTop(void)
{
    String out = "," + String(buoy.speedP);
    out += "," + String(buoy.speedI);
    out += "," + String(buoy.speedD);
    out += "," + String(buoy.speedKp);
    out += "," + String(buoy.speedKi);
    out += "," + String(buoy.speedKd);
    return out;
}
String PidCodeSup(void)
{
    String out = "," + String(buoy.rudderP);
    out += "," + String(buoy.rudderI);
    out += "," + String(buoy.rudderD);
    out += "," + String(buoy.rudderKp);
    out += "," + String(buoy.rudderKi);
    out += "," + String(buoy.rudderKd);
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

void initRudderPid(void)
{
    // pidRudderParameters(&rudderpid.kp, &rudderpid.ki, &rudderpid.kd, true);
    buoy.rudderIintergrate = 0;
    buoy.rudderLastErr = 0;
    buoy.rudderLastTime = millis();
}

void addBeginAndEndToString(String &input)
{
    input = "$" + input + "*";
}

// Subroutine to add CRC to a string (similar to NMEA format)
void addCRCToString(String &input) // Use reference to modify the original string
{
    // Find where the checksum starts (between '$' and '*')
    int start = input.indexOf('$');
    int end = input.indexOf('*');

    // If there's no '$' or '*' in the string, return without changes
    if (start == -1 || end == -1 || end <= start)
    {
        return; // Invalid format, do nothing and return
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

double ComputeSmallestAngleDir(double heading1, double heading2)
{
    double angle = fmod(heading2 - heading1 + 360, 360); // Calculate the difference and keep it within 360 degrees
    if (angle > 180)
    {
        return angle - 360; // Angle is greater than 180, SB (turn right)
    }
    return angle; // Angle is less than or equal to 180, BB (Turn left)
}

#define ILIM 35 // Maximum interal limit (35% power)
float push = 0;
bool CalcRudderBuoy(double magheading, float tgheading, double tdistance, int speed, int *bb, int *sb)
{
    double error = ComputeSmallestAngleDir(magheading, tgheading);
    /*Rotate to target direction first*/
    if (tdistance > 0.5 && abs(error) > 45)
    {
        float power = map(abs(error), 45, 180, 2, 20);
        power = sin(radians(power)) * 100;
        power = (int)map(power, 13, 100, 13, buoy.maxSpeed / 2);
        power += push;
        push += 0.01;
        power = constrain(power, 0, buoy.maxSpeed);
        if (error >= 0)
        {
            *bb = -power;
            *sb = power;
        }
        else
        {
            *bb = power;
            *sb = -power;
        }
        initRudderPid();
        return false;
    }
    push = 0;
    /*calculate proportion thrusters*/
    /*Scale error in range for tan*/
    error = map(error, -180, 180, -80, 80);
    unsigned long now = millis();
    double timeChange = (double)(now - buoy.rudderLastTime);
    double dErr = (error - buoy.rudderLastErr) / timeChange;
    buoy.rudderIintergrate += error * timeChange;
    if ((buoy.rudderKi / 1000) * buoy.rudderIintergrate > ILIM)
    {
        buoy.rudderIintergrate = ILIM / ((buoy.rudderKi / 1000));
    }
    if ((buoy.rudderKi / 1000) * buoy.rudderIintergrate < -ILIM)
    {
        buoy.rudderIintergrate = -ILIM / ((buoy.rudderKi / 1000));
    }
    buoy.rudderP = buoy.rudderKp * error;
    buoy.rudderI = (buoy.rudderKi / 1000) * buoy.rudderIintergrate;
    buoy.rudderD = buoy.rudderKd * dErr;
    double adj = buoy.rudderP + buoy.rudderI + buoy.rudderD + buoy.mechanicCorrection;
    buoy.rudderLastErr = error;
    buoy.rudderLastTime = now;
    *bb = (int)(speed * (1 - tan(radians(adj))));
    *sb = (int)(speed * (1 + tan(radians(adj))));
    *bb = *bb;
    /*Sanety check*/
    *bb = constrain(*bb, -buoy.maxSpeed, buoy.maxSpeed);
    *sb = constrain(*sb, -buoy.maxSpeed, buoy.maxSpeed);
    return true;
}

void RouteToPoint(double lat1, double lon1, double lat2, double lon2, double *distance, double *direction)
{
    *distance = distanceBetween(lat1, lon1, lat2, lon2);
    *direction = courseTo(lat1, lon1, lat2, lon2);
}
