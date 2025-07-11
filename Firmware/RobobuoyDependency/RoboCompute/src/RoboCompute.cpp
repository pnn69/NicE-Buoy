#include <arduino.h>
#include "RoboCompute.h"

bool startsWithDollar(const String &str)
{
    // Check if the string first character is '$'
    return str.charAt(0) == '$';
}

/*
    $IDr,IDs,ACK,MSG,STATUS,<data>*chksum
    decode incomming data.
    input string format $msg,STATUS,<data>*
    output parameters in to stuct type RoboStruct
    crc does not have any value
*/
void RoboDecode(String data, RoboStruct *dataStore)
{
    dataStore->cmd = -1;

    String numbers[15]; // Array to hold the decoded numbers
    int count = 0;
    String substring = data;

    // Split the string by commas
    while (substring.length() > 0 && count < 15)
    {
        int commaIndex = substring.indexOf(',');
        if (commaIndex == -1)
        {
            numbers[count++] = substring;
            break;
        }
        numbers[count++] = substring.substring(0, commaIndex);
        substring = substring.substring(commaIndex + 1);
    }

    // Parse fields
    dataStore->cmd = numbers[0].toInt();
    dataStore->status = numbers[1].toInt();

    switch (dataStore->cmd)
    {
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
        dataStore->wStd = numbers[2].toDouble(); // NOTE: This might be a bug (same index twice)
        break;

    case STORE_DECLINATION:
        dataStore->declination = numbers[2].toDouble();
        break;

    case MAXMINPWR:
    case MAXMINPWRSET:
        dataStore->maxSpeed = numbers[2].toInt();
        dataStore->minSpeed = numbers[3].toInt();
        break;

    case DIRMDIRTGDIRG:
        dataStore->dirMag = numbers[2].toDouble();
        dataStore->tgDir = numbers[2].toDouble();
        dataStore->gpsDir = numbers[2].toInt();
        break;

    case SET_DECLINATION:
        dataStore->status = SET_DECLINATION;
        break;

    case TOPDATA:
        Serial.println("TOPDATA not implemented yet");
        break;

    case ROUTTOPOINT:
    case DOCKING:
    case STOREASDOC:
    case IDELING:
    case LOCKING:
    case PING:
    case PONG:
    case CALIBRATE_MAGNETIC_COMPASS:
    case LORAACK:
        // Nothing to do
        break;

    default:
        Serial.println("RoboDecode: Unknown decode format in <" + data + ">");
        dataStore->cmd = -1;
        break;
    }
}
/*
    Encode outgoing data.
    input DATA RoboSruct and depeding on field msg
    output string format $ID,<data>*xx
    crc has to be added!
*/
String RoboCode(const RoboStruct *dataOut)
{
    String out = String(dataOut->cmd);
    out += "," + String(dataOut->status);

    if (dataOut->ack == LORAACK) // only send ack data
    {
        return out;
    }

    switch (dataOut->cmd)
    {
    case IDLE:
        out += ",0,0";
        break;

    case DOCKED:
    case LOCKED:
        out += "," + String(dataOut->tgDir, 2);
        out += "," + String(dataOut->tgDist, 2);
        out += "," + String(dataOut->tgSpeed, 2);
        out += "," + String(dataOut->wDir, 1);
        out += "," + String(dataOut->wStd, 1);
        break;

    case REMOTE:
        out += "," + String(dataOut->tgDir, 0);
        out += "," + String(dataOut->tgSpeed, 0);
        break;

    case SUBDATA:
        out += "," + String(dataOut->dirMag, 2);
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        out += "," + String(dataOut->subAccuV, 2);
        out += "," + String(dataOut->subAccuP);
        break;

    case MDIR:
        out += "," + String(dataOut->dirMag, 2);
        break;

    case GDIR:
        out += "," + String(dataOut->gpsDir, 2);
        break;

    case TDIR:
        out += "," + String(dataOut->tgDir, 2);
        break;

    case DIRSPEED:
        out += "," + String(dataOut->dirMag, 2);
        out += "," + String(dataOut->speed);
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        break;

    case TGDIRSPEED:
        out += "," + String(dataOut->tgDir, 2);
        out += "," + String(dataOut->speedSet, 0);
        break;

    case SUBSPEED:
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        out += "," + String(dataOut->speed);
        break;

    case SUBACCU:
        out += "," + String(dataOut->subAccuV, 2);
        out += "," + String(dataOut->subAccuP);
        break;

    case PIDSPEED:
    case PIDSPEEDSET:
        out += "," + String(dataOut->Kps, 3);
        out += "," + String(dataOut->Kis, 3);
        out += "," + String(dataOut->Kds, 3);
        break;

    case PIDRUDDER:
    case PIDRUDDERSET:
        out += "," + String(dataOut->Kpr, 3);
        out += "," + String(dataOut->Kir, 3);
        out += "," + String(dataOut->Kdr, 3);
        break;

    case DIRDIST:
        out += "," + String(dataOut->tgDir, 2);
        out += "," + String(dataOut->tgDist, 2);
        break;

    case CALCRUDDER:
        out += "," + String(dataOut->tgDir, 2);
        out += "," + String(dataOut->tgDist, 2);
        out += "," + String(dataOut->speedSet, 2);
        break;

    case BUOYPOS:
        out += "," + String(dataOut->lat, 8);
        out += "," + String(dataOut->lng, 8);
        out += "," + String(dataOut->dirMag, 2);
        out += "," + String(dataOut->wDir, 2);
        out += "," + String(dataOut->wStd, 2);
        out += "," + String(dataOut->topAccuP);
        out += "," + String(dataOut->subAccuP);
        out += "," + String(dataOut->gpsFix);
        out += "," + String(dataOut->gpsSat);
        break;

    case SUBPWR:
        out += "," + String(dataOut->speedSet, 0);
        out += "," + String(dataOut->speed);
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        out += "," + String(dataOut->subAccuV, 2);
        break;

    case SPBBSPSB:
        out += "," + String(dataOut->speedBb);
        out += "," + String(dataOut->speedSb);
        break;

    case SETLOCKPOS:
    case SETDOCKPOS:
        out += "," + String(dataOut->tgLat, 10);
        out += "," + String(dataOut->tgLng, 10);
        break;

    case LOCKPOS:
    case DOCKPOS:
        out += "," + String(dataOut->tgLat, 10);
        out += "," + String(dataOut->tgLng, 10);
        out += "," + String(dataOut->wDir, 1);
        out += "," + String(dataOut->wStd, 1);
        break;

    case WINDDATA:
        out += "," + String(dataOut->wDir, 1);
        out += "," + String(dataOut->wStd, 1);
        break;

    case MAXMINPWRSET:
    case MAXMINPWR:
        out += "," + String(dataOut->maxSpeed);
        out += "," + String(dataOut->minSpeed);
        break;

    case DIRMDIRTGDIRG:
    case LOCKING:
        out += "," + String(dataOut->dirMag, 0);
        out += "," + String(dataOut->tgDir, 0);
        out += "," + String(dataOut->gpsDir);
        break;

    case STORE_DECLINATION:
        out += "," + String(dataOut->declination, 2);
        break;

    case PING:
        return String(PING);

    case PONG:
        return String(PONG);

    // Cases with no output
    case DOCKING:
    case IDELING:
    case STOREASDOC:
    case CALIBRATE_MAGNETIC_COMPASS:
    case SET_DECLINATION:
        break;

    default:
        printf("RoboCode: Unknown formatter <%d>\r\n", dataOut->cmd);
        break;
    }

    return out;
}

//***************************************************************************************************
//  code rf string
//  add crc
// $IDr,IDs,ack,cmd,status,data*chk
//***************************************************************************************************
String rfCode(RoboStruct *rfOut)
{
    String rfMsg = String(rfOut->IDr, HEX);
    rfMsg += "," + String(rfOut->IDs, HEX);
    rfMsg += "," + String(rfOut->ack);
    rfMsg += "," + RoboCode(rfOut);
    rfMsg = addCRCToString(rfMsg);
    // Serial.println("coded data out<" + rfMsg + ">");
    return rfMsg;
}

//***************************************************************************************************
//   Subroutine to add CRC to a string (similar to NMEA format)
//***************************************************************************************************
String addCRCToString(String input)
{
    input.trim();           // Clean up whitespace
    input.replace(" ", ""); // remove all internal spaces
    byte crc = 0;
    for (int i = 0; i < input.length(); i++)
    {
        crc ^= (byte)input.charAt(i); // ✅ ensure unsigned XOR
    }
    char crcHex[3];
    sprintf(crcHex, "%02X", crc);
    return "$" + input + "*" + String(crcHex);
    ;
}
//***************************************************************************************************
//  decode rf string
//  $IDr,IDs,ACK,MSG,<data>*chk
//***************************************************************************************************
// rfIn = "$9*39";
void rfDeCode(String rfIn, RoboStruct *in)
{
    rfIn.trim(); // Remove whitespace

    // Set defaults
    in->IDr = -1;
    in->IDs = -1;

    // Basic structure validation
    if (!rfIn.startsWith("$") || rfIn.indexOf('*') == -1)
    {
        return; // Invalid format
    }

    // CRC check
    if (!verifyCRC(rfIn))
    {
        return;
    }

    int commaIndex;

    // Parse IDr
    commaIndex = rfIn.indexOf(',');
    if (commaIndex == -1)
        return;
    String hexString = rfIn.substring(1, commaIndex);
    in->IDr = strtoull(hexString.c_str(), NULL, 16);

    // Parse IDs
    rfIn = rfIn.substring(commaIndex + 1);
    commaIndex = rfIn.indexOf(',');
    if (commaIndex == -1)
        return;
    hexString = rfIn.substring(0, commaIndex);
    in->IDs = strtoull(hexString.c_str(), NULL, 16);

    // Parse ack
    rfIn = rfIn.substring(commaIndex + 1);
    commaIndex = rfIn.indexOf(',');
    if (commaIndex == -1)
        return;
    in->ack = rfIn.substring(0, commaIndex).toInt();

    // Parse msg and data
    rfIn = rfIn.substring(commaIndex + 1);
    int starIndex = rfIn.indexOf('*');
    if (starIndex == -1)
        return;
    String dataPart = rfIn.substring(0, starIndex);
    dataPart.replace("$", "");

    // Decode remaining data into struct
    RoboDecode(dataPart, in);
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
    // return givenCRC.equalsIgnoreCase(calculatedCRCHex);
    return givenCRC.equalsIgnoreCase(String(calculatedCRCHex));
}

/*
Add new data in buffer
Structure buf[averige][deviation][data0][datan...]
*/
void addNewSampleInBuffer(RoboWindStruct *wData, double nwdata)
{
    if (wData->ptr >= SAMPELS)
    {
        wData->ptr = 0;
    }
    wData->data[wData->ptr] = nwdata;
    wData->ptr++;
}

void averageWindVector(RoboWindStruct *wData)
{
    double sumX = 0, sumY = 0;

    for (int i = 0; i < SAMPELS; ++i)
    {
        double angleRad = radians(wData->data[i]);
        double speed = wData->speed[i];

        sumX += cos(angleRad) * speed;
        sumY += sin(angleRad) * speed;
    }

    double meanAngle = atan2(sumY, sumX);
    wData->wDir = fmod((meanAngle * 180.0 / M_PI) + 360.0, 360.0); // degrees
    wData->wSpeed = sqrt(sumX * sumX + sumY * sumY) / SAMPELS;
}

/*
compute deviation of a buffer pos
Structure buf[averige][deviation][data0][datan...]
*/
void deviationWindRose(RoboWindStruct *wData)
{
    averageWindVector(wData); // Compute mean wind direction first

    double sumSquaredCircularDiff = 0;

    for (int i = 0; i < SAMPELS; ++i)
    {
        double diff = wData->data[i] - wData->wDir;

        // Wrap difference to [-180, 180)
        if (diff > 180.0)
            diff -= 360.0;
        else if (diff < -180.0)
            diff += 360.0;

        sumSquaredCircularDiff += diff * diff;
    }

    wData->wStd = sqrt(sumSquaredCircularDiff / SAMPELS); // Standard deviation in degrees
}

void PidDecode(String data, int pid, RoboStruct buoy)
{
    int numbers[15]; // Array to hold the decoded numbers (adjust size as needed)
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
        buoy.Kps = numbers[1];
        buoy.Kis = numbers[2];
        buoy.Kis = numbers[3];
    }
    if (pid == PIDRUDDER)
    {
        buoy.Kpr = numbers[1];
        buoy.Kir = numbers[2];
        buoy.Kir = numbers[3];
    }
}

String PidEncode(int pid, RoboStruct buoy)
{
    String out = "";
    if (pid == PIDSPEED)
    {
        out = String(buoy.Kps);
        out += "," + String(buoy.Kis);
        out += "," + String(buoy.Kds);
    }
    if (pid == PIDRUDDER)
    {
        out = String(buoy.Kpr);
        out += "," + String(buoy.Kir);
        out += "," + String(buoy.Kdr);
    }
    return out;
}

double gpsgem[21][1];
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
    // Distance in meters
    return EARTH_MEAN_RADIUS * c;
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2)
{
    // Convert degrees to radians
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 = lat1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;

    // Calculate bearing
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

    double bearing = atan2(y, x) * (180.0 / M_PI); // radians to degrees

    // Normalize to 0-360 degrees
    return fmod((bearing + 360.0), 360.0);
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
void adjustPositionDirDist(double bearing_deg, double distance,
                           double lat1_deg, double lon1_deg,
                           double *lat2_deg, double *lon2_deg)
{
    double lat1 = radians(lat1_deg);
    double lon1 = radians(lon1_deg);
    double bearing = radians(bearing_deg);
    double angular_distance = distance / (EARTH_RADIUS_KM * 1000.0);

    double lat2 = asin(sin(lat1) * cos(angular_distance) +
                       cos(lat1) * sin(angular_distance) * cos(bearing));

    double lon2 = lon1 + atan2(sin(bearing) * sin(angular_distance) * cos(lat1),
                               cos(angular_distance) - sin(lat1) * sin(lat2));

    *lat2_deg = degrees(lat2);
    *lon2_deg = degrees(lon2);
    // Normalize longitude to -180...+180
    if (*lon2_deg > 180.0)
        *lon2_deg -= 360.0;
    else if (*lon2_deg < -180.0)
        *lon2_deg += 360.0;
}

double smallestAngle(double heading1, double heading2)
{
    // Calculate the difference and normalize to [0, 360)
    double angle = fmod(heading2 - heading1 + 360, 360);

    // Convert angles > 180 to negative equivalent to get shortest turn direction
    if (angle > 180)
    {
        return angle - 360; // Turn right (negative angle)
    }
    return angle; // Turn left (positive angle)
}

/*
    calculate the smallest angle between two directions
    return 1 if angle is >180
*/
bool determineDirection(double heading1, double heading2)
{
    double angle = fmod(heading2 - heading1 + 360, 360); // Normalize difference to [0, 360)
    return (angle > 180);                                // true if angle > 180 (turn right), else false
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

    tbb = (int)(buoy.speedSet * cos(radians(error)) * (1 - sin(radians(error))));
    tsb = (int)(buoy.speedSet * cos(radians(error)) * (1 - sin(radians(error)) * -1));
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
    if (buoy.Kis * buoy.errSums < 0)
    {
        buoy.errSums = 0;
    }
    // /*max 70% I correction*/
    if (buoy.Kis * buoy.errSums > 70)
    {
        buoy.errSums = 70 / buoy.Kis;
    }
    /*Compute PID Output*/
    buoy.Kps = buoy.Kps * error;
    buoy.Kis = buoy.Kis * buoy.errSums;
    buoy.Kds = buoy.Kds * dErr;
    Output = buoy.Kps + buoy.Kis + buoy.Kds;
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
void threePointAverage(struct RoboStruct p3[2], double *latgem, double *lnggem)
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
    // Normalize to [0, 360)
    windDegrees = fmod(windDegrees, 360);
    if (windDegrees < 0)
        windDegrees += 360;

    double radiansAngle = radians(windDegrees);
    *windX = cos(radiansAngle);
    *windY = sin(radiansAngle);
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
    cosAngle = fmax(-1.0, fmin(1.0, cosAngle)); // Clamp to [-1, 1]    // Return the angle in degrees
    return acos(cosAngle) * (180.0 / M_PI);
}
double calculateAngleSigned(double x1, double y1, double x2, double y2)
{
    double dotProduct = x1 * x2 + y1 * y2;
    double magnitudeA = sqrt(x1 * x1 + y1 * y1);
    double magnitudeB = sqrt(x2 * x2 + y2 * y2);
    // Calculate the cosine of the angle
    double cosAngle = dotProduct / (magnitudeA * magnitudeB);
    // Clamp the value to the range [-1, 1] to avoid NaN from acos
    cosAngle = fmax(-1.0, fmin(1.0, cosAngle)); // Clamp to [-1, 1]    // Return the angle in degrees
    double crossProduct = x1 * y2 - y1 * x2;
    double angle = atan2(crossProduct, dotProduct) * (180.0 / M_PI); // Signed angle
    return angle;
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
#define INVALID_POINT(p) ((p).tgLat == 0.0 || (p).tgLng == 0.0)

void recalcStartLine(struct RoboStruct rsl[3])
{
    double d0 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng);
    double d1 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng);
    double d2 = distanceBetween(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng);

    double midLat, midLng;
    double angleSb, angleBb;

    printf("# Winddir 0:%.2f 1:%.3f 2:%.2f \r\n", rsl[0].wDir, rsl[1].wDir, rsl[2].wDir);

    if (d0 < d1 && d0 < d2)
    {
        if (INVALID_POINT(rsl[0]) || INVALID_POINT(rsl[1]))
        {
            printf("# No data to compute with (0-1)\r\n");
            return;
        }

        twoPointAverage(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng, &midLat, &midLng);
        angleSb = fmod(rsl[0].wDir + 90.0, 360.0);
        angleBb = fmod(angleSb + 180.0, 360.0);

        adjustPositionDirDist(angleBb, d0 / 2, midLat, midLng, &rsl[0].tgLat, &rsl[0].tgLng); // PORT
        adjustPositionDirDist(angleSb, d0 / 2, midLat, midLng, &rsl[1].tgLat, &rsl[1].tgLng); // STARBOARD

        rsl[0].trackPos = PORT;
        rsl[1].trackPos = STARBOARD;
    }
    else if (d1 < d0 && d1 < d2)
    {
        if (INVALID_POINT(rsl[0]) || INVALID_POINT(rsl[2]))
        {
            printf("# No data to compute with (0-2)\r\n");
            return;
        }

        twoPointAverage(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng, &midLat, &midLng);
        angleSb = fmod(rsl[0].wDir + 90.0, 360.0);
        angleBb = fmod(angleSb + 180.0, 360.0);

        adjustPositionDirDist(angleBb, d1 / 2, midLat, midLng, &rsl[0].tgLat, &rsl[0].tgLng); // PORT
        adjustPositionDirDist(angleSb, d1 / 2, midLat, midLng, &rsl[2].tgLat, &rsl[2].tgLng); // STARBOARD

        rsl[0].trackPos = PORT;
        rsl[2].trackPos = STARBOARD;
    }
    else if (d2 < d0 && d2 < d1)
    {
        if (INVALID_POINT(rsl[1]) || INVALID_POINT(rsl[2]))
        {
            printf("# No data to compute with (1-2)\r\n");
            return;
        }

        twoPointAverage(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng, &midLat, &midLng);
        angleSb = fmod(rsl[1].wDir + 90.0, 360.0);
        angleBb = fmod(angleSb + 180.0, 360.0);

        adjustPositionDirDist(angleBb, d2 / 2, midLat, midLng, &rsl[1].tgLat, &rsl[1].tgLng); // PORT
        adjustPositionDirDist(angleSb, d2 / 2, midLat, midLng, &rsl[2].tgLat, &rsl[2].tgLng); // STARBOARD

        rsl[1].trackPos = PORT;
        rsl[2].trackPos = STARBOARD;
    }
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
void reCalcTrack(struct RoboStruct rsl[3])
{
    double d0, d1, d2;
    double startLineL, centerPont2Startline, centerPoint2Head;
    double angleSb, angleBb, angle180;
    double lat2gem, lng2gem;
    double lat3gem, lng3gem;

    // Reset track positions
    for (int i = 0; i < 3; i++)
    {
        rsl[i].trackPos = -1;
        if (rsl[i].tgLng == 0 || rsl[i].tgLat == 0)
        {
            printf("# No data to compute with\r\n");
            return;
        }
    }

    // Midpoint of three buoys
    threePointAverage(rsl, &lat3gem, &lng3gem);
    printf("midpoint =(%.12f,%.12f)\r\n", lat3gem, lng3gem);

    // Distances between buoys
    d0 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng);
    d1 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng);
    d2 = distanceBetween(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng);

    // Determine start line and midpoint of that line
    if (d0 < d1 && d0 < d2)
    {
        startLineL = d0;
        twoPointAverage(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng, &lat2gem, &lng2gem);
    }
    else if (d1 < d2 && d1 < d0)
    {
        startLineL = d1;
        twoPointAverage(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng, &lat2gem, &lng2gem);
    }
    else
    {
        startLineL = d2;
        twoPointAverage(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng, &lat2gem, &lng2gem);
    }

    // Compute wind angles to midpoint
    double b0 = computeWindAngle(rsl[0].wDir, rsl[0].tgLat, rsl[0].tgLng, lat3gem, lng3gem);
    double b1 = computeWindAngle(rsl[0].wDir, rsl[1].tgLat, rsl[1].tgLng, lat3gem, lng3gem);
    double b2 = computeWindAngle(rsl[0].wDir, rsl[2].tgLat, rsl[2].tgLng, lat3gem, lng3gem);

    printf("winddir =(%.1f)\r\n", rsl[0].wDir);

    angle180 = rsl[0].wDir + 180;
    if (angle180 > 360)
        angle180 -= 360;

    angleSb = rsl[0].wDir + 90;
    if (angleSb > 360)
        angleSb -= 360;

    angleBb = angleSb + 180;
    if (angleBb > 360)
        angleBb -= 360;

    // Determine which buoy is HEAD (closest to wind direction)
    if (b0 < b1 && b0 < b2)
    {
        centerPoint2Head = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, lat3gem, lng3gem);
        centerPont2Startline = distanceBetween(lat2gem, lng2gem, lat3gem, lng3gem);
        adjustPositionDirDist(rsl[0].wDir, centerPoint2Head, lat3gem, lng3gem, &rsl[0].tgLat, &rsl[0].tgLng); // HEAD
        adjustPositionDirDist(angle180, centerPont2Startline, lat3gem, lng3gem, &lat2gem, &lng2gem);
        adjustPositionDirDist(angleBb, startLineL / 2, lat2gem, lng2gem, &rsl[1].tgLat, &rsl[1].tgLng); // PORT
        adjustPositionDirDist(angleSb, startLineL / 2, lat2gem, lng2gem, &rsl[2].tgLat, &rsl[2].tgLng); // STARBOARD
        rsl[0].trackPos = HEAD;
        rsl[1].trackPos = PORT;
        rsl[2].trackPos = STARBOARD;
    }
    else if (b1 < b0 && b1 < b2)
    {
        centerPoint2Head = distanceBetween(rsl[1].tgLat, rsl[1].tgLng, lat3gem, lng3gem);
        centerPont2Startline = distanceBetween(lat2gem, lng2gem, lat3gem, lng3gem);
        adjustPositionDirDist(rsl[0].wDir, centerPoint2Head, lat3gem, lng3gem, &rsl[1].tgLat, &rsl[1].tgLng); // HEAD
        adjustPositionDirDist(angle180, centerPont2Startline, lat3gem, lng3gem, &lat2gem, &lng2gem);
        adjustPositionDirDist(angleBb, startLineL / 2, lat2gem, lng2gem, &rsl[0].tgLat, &rsl[0].tgLng); // PORT
        adjustPositionDirDist(angleSb, startLineL / 2, lat2gem, lng2gem, &rsl[2].tgLat, &rsl[2].tgLng); // STARBOARD
        rsl[1].trackPos = HEAD;
        rsl[0].trackPos = PORT;
        rsl[2].trackPos = STARBOARD;
    }
    else
    {
        centerPoint2Head = distanceBetween(rsl[2].tgLat, rsl[2].tgLng, lat3gem, lng3gem);
        centerPont2Startline = distanceBetween(lat2gem, lng2gem, lat3gem, lng3gem);
        adjustPositionDirDist(rsl[0].wDir, centerPoint2Head, lat3gem, lng3gem, &rsl[2].tgLat, &rsl[2].tgLng); // HEAD
        adjustPositionDirDist(angle180, centerPont2Startline, lat3gem, lng3gem, &lat2gem, &lng2gem);
        adjustPositionDirDist(angleBb, startLineL / 2, lat2gem, lng2gem, &rsl[0].tgLat, &rsl[0].tgLng); // PORT
        adjustPositionDirDist(angleSb, startLineL / 2, lat2gem, lng2gem, &rsl[1].tgLat, &rsl[1].tgLng); // STARBOARD
        rsl[2].trackPos = HEAD;
        rsl[0].trackPos = PORT;
        rsl[1].trackPos = STARBOARD;
    }
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
    double dir = 0;
    double d0 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng); // compute length p0 p1
    double d1 = distanceBetween(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng); // compute length p0 p2
    double d2 = distanceBetween(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng); // compute length p1 p2
    if (d0 < d1 && d0 < d2)
    {
        dir = calculateBearing(rsl[0].tgLat, rsl[0].tgLng, rsl[1].tgLat, rsl[1].tgLng);
        if (smallestAngle(rsl[0].wDir, dir) >= 0)
        {
            rsl[0].trackPos = PORT;
            rsl[1].trackPos = STARBOARD;
            rsl[2].trackPos = HEAD;
        }
        else
        {
            rsl[0].trackPos = STARBOARD;
            rsl[1].trackPos = PORT;
            rsl[2].trackPos = HEAD;
        }
    }
    if (d1 < d0 && d1 < d2)
    {
        dir = calculateBearing(rsl[0].tgLat, rsl[0].tgLng, rsl[2].tgLat, rsl[2].tgLng);
        if (smallestAngle(rsl[0].wDir, dir) >= 0)
        {
            rsl[0].trackPos = PORT;
            rsl[1].trackPos = HEAD;
            rsl[2].trackPos = STARBOARD;
        }
        else
        {
            rsl[0].trackPos = STARBOARD;
            rsl[1].trackPos = HEAD;
            rsl[2].trackPos = PORT;
        }
    }
    if (d2 < d0 && d2 < d1)
    {
        dir = calculateBearing(rsl[1].tgLat, rsl[1].tgLng, rsl[2].tgLat, rsl[2].tgLng);
        if (smallestAngle(rsl[0].wDir, dir) >= 0)
        {
            rsl[0].trackPos = HEAD;
            rsl[1].trackPos = PORT;
            rsl[2].trackPos = STARBOARD;
        }
        else
        {
            rsl[0].trackPos = HEAD;
            rsl[1].trackPos = STARBOARD;
            rsl[2].trackPos = PORT;
        }
    }
    printf("# dir= %.0f\r\n", dir);
    return rsl[3];
}

/************************************************************************************************************************************************************************* */
//  Store parameters
/************************************************************************************************************************************************************************* */
void AddDataToBuoyBase(RoboStruct dataIn, RoboStruct *buoyPara[3])
{
    for (int i = 0; i < 3; i++)
    {
        if (buoyPara[i] == nullptr)
            continue;
        if (dataIn.IDs == buoyPara[i]->IDs || buoyPara[i]->IDs == 0)
        {
            *buoyPara[i] = dataIn; // Store the data in the buoyPara array
            Serial.println("# Lock data stored! on pos:" + String(i) + " IDs:" + String(buoyPara[i]->IDs, HEX) + "Lat:" + String(buoyPara[i]->tgLat, 6) + "Lng:" + String(buoyPara[i]->tgLng, 6) + " Wdir:" + String(buoyPara[i]->wDir, 2));
            return;
        }
    }
    printf("# No data stored! :( \r\n");
}
/************************************************************************************************************************************************************************* */
//  Retreve parameters
/************************************************************************************************************************************************************************* */
int GetDataPosFromBuoyBase(uint64_t id, RoboStruct buoyPara[3])
{
    for (int i = 0; i < 3; i++)
    {
        if (id == buoyPara[i].IDs)
        {
            return i;
        }
    }
    printf("# No data found! :( \r\n");
    return -1;
}
