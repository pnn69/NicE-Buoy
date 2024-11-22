#include "main.h"

bool AddDataToBuoyBase(RoboStruct dataIn, RoboStruct buoyPara[3])
{
    if (dataIn.mac != -1 || dataIn.mac != -2)
    {
        Serial.print("#ID to store:" + String(dataIn.mac, HEX));
        Serial.print(" lat:" + String(dataIn.tgLat, 8));
        Serial.println(" Lon:" + String(dataIn.tgLng, 8));
        if (dataIn.mac == buoyId)
        {
            memcpy(&buoyPara[0], &dataIn, sizeof(RoboStruct));
            printf("LOCK data added to buoyPara[0]\r\n");
            return true;
        }
        else
        {
            for (int i = 1; i < 3; i++)
            {
                if (dataIn.mac == buoyPara[i].mac || buoyPara[i].mac == 0)
                {
                    memcpy(&buoyPara[i], &dataIn, sizeof(RoboStruct));
                    printf("#LOCK data added to buoyPara[%d]\r\n", i);
                    return true;
                }
            }
        }
    }
    Serial.println("#No data stored! :( ");
    return false;
}

RoboStruct GetDataFromBuoyBase(uint64_t id, RoboStruct buoyPara[3])
{
    RoboStruct out;
    out.mac = 0;
    for (int i = 0; i < 3; i++)
    {
        if (id == buoyPara[i].mac)
        {
            memcpy(&out, &buoyPara[i], sizeof(RoboStruct));
            return out;
        }
    }
    Serial.println("#No data found! :( ");
    return out;
}
