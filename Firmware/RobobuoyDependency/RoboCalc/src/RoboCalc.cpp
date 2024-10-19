#include <Arduino.h>
#include <math.h>
#include "RoboCalc.h"

/*some constans*/
#define _GPS_EARTH_MEAN_RADIUS 6371009 // old: 6372795
#define EARTHRADIUS 6371
#define radian(x) (x * M_PI / 180)
#define degre(x) (x * 180 / M_PI)
#define ILIM 35 // Maximum interal limit (35% power)

/*PID structs*/
// pid buoy;
pid speed;
pid rudder;

int PidDecode(String data, pid buoy)
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
    buoy.p = numbers[1];
    buoy.i = numbers[2];
    buoy.i = numbers[3];
    buoy.kp = 0;
    buoy.ki = 0;
    buoy.kd = 0;
    return numbers[0];
}

String PidData(pid buoy)
{
    String out = String(buoy.p);
    out += "," + String(buoy.i);
    out += "," + String(buoy.d);
    out += "," + String(buoy.kp);
    out += "," + String(buoy.ki);
    out += "," + String(buoy.kd);
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

void initPid(pid buoy)
{
    // pidRudderParameters(&buoy.kp, &buoy.ki, &buoy.kd, true);
    buoy.iintergrate = 0;
    buoy.lastErr = 0;
    buoy.lastTime = millis();
}

// void addBeginAndEndToString(String &input)
// {
//     input = "$" + input + "*";
// }

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

// void initCalculate(void)
// {
//     computeParameters(&buoy.minOfsetDist, &buoy.maxOfsetDist, &buoy.minSpeed, &buoy.maxSpeed, true);
// }

/*Approimate roling average deafualt 100 samples*/
double approxRollingAverage(double avg, double input)
{
    avg -= avg / 100;
    avg += input / 100;
    return avg;
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
void setparameters(int &minOfsetDist, int &maxOfsetDist, int &minSpeed, int &maxSpeed)
{
    /*
    sanety check
    */
    if (minOfsetDist < 0)
    {
        minOfsetDist = 2;
    }
    if (maxOfsetDist > 100)
    {
        maxOfsetDist = 20;
    }
    if (minSpeed < 0)
    {
        minSpeed = 0;
    }
    if (maxSpeed > 80)
    {
        maxSpeed = 80;
    }
    if (minOfsetDist >= maxOfsetDist)
    {
        maxOfsetDist = minOfsetDist + 2;
    }

    // computeParameters(&buoy.minOfsetDist, &buoy.maxOfsetDist, &buoy.minSpeed, &buoy.maxSpeed, false);
    //  Serial.printf("Stored Parameters: Minimum offset distance: %dM Maxumum offset distance: %dM, buoy minimum speed: %d%%, buoy maximum speed: %d%%\r\n", buoy.minOfsetDist, buoy.maxOfsetDist, buoy.minSpeed, buoy.maxSpeed);
}

/*
input: directon to go to, distance and start position
return: target latitude and longitude
*/
void adjustPositionDirDist(int dir, double dist, double *lat, double *lon)
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
        angle = 360 - angle; // Take the smaller angle between the two
    }
    return angle;
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

double ComputeSmallestAngleDir(double heading1, double heading2)
{
    double angle = fmod(heading2 - heading1 + 360, 360); // Calculate the difference and keep it within 360 degrees
    if (angle > 180)
    {
        return angle - 360; // Angle is greater than 180, SB (turn right)
    }
    return angle; // Angle is less than or equal to 180, BB (Turn left)
}

/*
    adjust speed Cosinus does not work here. We have to do it manually
*/
float Angle2SpeedFactor(float angle)
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
void CalcRemoteRudderBuoy(double magheading, float tgheading, int speed, int *bb, int *sb)
{
    bool dir = determineDirection(magheading, tgheading);
    double correctonAngle = smallestAngle(magheading, tgheading);
    float corr = Angle2SpeedFactor(abs(correctonAngle));
    float tbb, tsb;
    if (dir == 1)
    {
        tbb = speed + speed * (1 - corr);
        tsb = speed * corr;
    }
    else
    {
        tbb = speed * corr;
        tsb = speed + speed * (1 - corr);
    }

    double error = ComputeSmallestAngleDir(magheading, tgheading);
    tbb = (int)(speed * cos(radian(error)) * (1 - sin(radian(error))));
    tsb = (int)(speed * cos(radian(error)) * (1 - sin(radian(error)) * -1));
    *bb = (int)constrain(tbb, -60, 100);
    *sb = (int)constrain(tsb, -60, 100);
    Serial.printf("Error=%lf BB=%d SB=%d\r\n\r\n", error, *bb, *sb);
    return;
}

/*
    Calculate power to thrusters
    if distance between 0.5 and 1.5 meter rotate at the slowest speed.
    else do normal rudder calculation.
*/
float push = 0;

bool CalcRudderBuoy(double magheading, float tgheading, double tdistance, int speed, int *bb, int *sb, pid buoy)
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
        initPid(buoy);
        return false;
    }
    push = 0;
    /*calculate proportion thrusters*/
    /*Scale error in range for tan*/
    error = map(error, -180, 180, -80, 80);
    unsigned long now = millis();
    double timeChange = (double)(now - buoy.lastTime);
    double dErr = (error - buoy.lastErr) / timeChange;
    buoy.iintergrate += error * timeChange;
    if ((buoy.ki / 1000) * buoy.iintergrate > ILIM)
    {
        buoy.iintergrate = ILIM / ((buoy.ki / 1000));
    }
    if ((buoy.ki / 1000) * buoy.iintergrate < -ILIM)
    {
        buoy.iintergrate = -ILIM / ((buoy.ki / 1000));
    }
    buoy.p = buoy.kp * error;
    buoy.i = (buoy.ki / 1000) * buoy.iintergrate;
    buoy.d = buoy.kd * dErr;
    double adj = buoy.p + buoy.i + buoy.d + buoy.compassCorrection;
    buoy.lastErr = error;
    buoy.lastTime = now;
    *bb = (int)(speed * (1 - tan(radians(adj))));
    *sb = (int)(speed * (1 + tan(radians(adj))));
    *bb = *bb;
    /*Sanety check*/
    *bb = constrain(*bb, -buoy.maxSpeed, buoy.maxSpeed);
    *sb = constrain(*sb, -buoy.maxSpeed, buoy.maxSpeed);
    return true;
}

/*
Only used PID if the distancs is less than buoy.maxOfsetDist return BUOYMAXSPEED otherwise.
*/
int hooverPid(double dist, pid buoy)
{
    /*Do not use the pid loop if distance is to big just go full power*/
    if (buoy.maxSpeed == 51) /*old config use this by setting max speed to 51*/
    {
        if (dist > buoy.maxOfsetDist)
        {
            return buoy.maxSpeed;
        }
        if (speed.armIntergrator == false)
        {
            speed.armIntergrator = true;
            speed.iintergrate = 0;
        }
    }
    else
    {
        /*Do not use the pid loop if distance is to big just go full power*/
        if (buoy.armIntergrator == false)
        {
            if (dist > 3)
            {
                buoy.iintergrate = 0;
                return buoy.maxSpeed;
            }
            else
            {
                buoy.armIntergrator = true;
                buoy.iintergrate = 0;
            }
        }
    }
    /*How long since we last calculated*/
    double Output = 0;
    unsigned long now = millis();
    double timeChange = (double)(now - buoy.lastTime);
    /*Compute all the working error variables*/
    double error = (dist - buoy.minOfsetDist) / 1000;
    buoy.iintergrate += (error * timeChange);
    double dErr = (error - buoy.lastErr) / timeChange;
    /* Do not sail backwards*/
    if (buoy.iintergrate < 0)
    {
        buoy.iintergrate = 0;
    }
    /*max 70% I correction*/
    if (buoy.ki * buoy.iintergrate > 70)
    {
        buoy.iintergrate = 70 / buoy.ki;
    }
    /*Compute PID Output*/
    buoy.p = buoy.kp * (dist - buoy.minOfsetDist);
    buoy.i = buoy.ki * buoy.iintergrate;
    buoy.d = buoy.kd * dErr;
    Output = buoy.p + buoy.i + buoy.d;
    /* Do not sail backwards*/
    if (Output < 0)
    {
        Output = 0;
    }
    /*Remember some variables for next time*/
    buoy.lastErr = dist;
    buoy.lastTime = now;
    buoy.speed = (int)constrain(Output, 0, buoy.maxSpeed);
    return buoy.speed;
}
