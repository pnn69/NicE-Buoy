#include "main.h"

bool AddDataToBuoyBase(RoboStruct dataIn)
{
    // Serial.print("ID to store:" + String(dataIn.mac, HEX));
    // Serial.print(" Stor lat ->Lat" + String(dataIn.lat, 8));
    if (dataIn.mac == buoyId)
    {
        memcpy(&buoyPara[0], &dataIn, sizeof(RoboStruct));
        Serial.println("LOCK data added to buoyPara[0]");
        Serial.println(String(dataIn.tgLat));
        Serial.println(String(dataIn.tgLng));
        return true;
    }
    else
    {
        for (int i = 1; i < 3; i++)
        {
            if (dataIn.mac == buoyPara[i].mac || buoyPara[i].mac == 0)
            {
                memcpy(&buoyPara[i], &dataIn, sizeof(RoboStruct));
                // printf("Copied dataIn to buoyPara[%d]\n", i);
                return true;
            }
        }
    }
    Serial.println("No data stored! :( ");
    delay(1000);
    return false;
}

RoboStruct GetDataFromBuoyBase(uint64_t id)
{
    RoboStruct out;
    out.mac = 0;
    for (int i = 0; i < 3; i++)
    {
        if (id == buoyPara[i].mac)
        {
            memcpy(&out, &buoyPara[i], sizeof(RoboStruct));
            // Serial.println("Data found on pos=" + String(i) + " ! ID" + String(out.mac, HEX) + "Lat:" + String(out.lat));
            return out;
        }
    }
    Serial.println("No data found! :( ");
    return out;
}

void printDirection(int value)
{
    switch (value)
    {
    case 1:
        Serial.print("HEAD");
        break;
    case 2:
        Serial.print("PORT");
        break;
    case 3:
        Serial.print("STARBOARD");
        break;
    default:
        Serial.print("Invalid input");
    }
}
