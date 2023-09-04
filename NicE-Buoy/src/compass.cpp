#include <Arduino.h>
#include <LSM303AGR_ACC_Sensor.h>
#include <LSM303AGR_MAG_Sensor.h>
#include "io23017.h"
#include "io.h"

#define DEV_I2C Wire // Or Wire
#define SerialPort Serial

// Components.
LSM303AGR_ACC_Sensor Acc(&DEV_I2C);
LSM303AGR_MAG_Sensor Mag(&DEV_I2C);

bool InitCompass(void)
{
    DEV_I2C.begin();
    // Initlialize components.
    Acc.begin();
    Acc.Enable();
    Acc.EnableTemperatureSensor();
    Mag.begin();
    Mag.Enable();
    return 0;
}
bool CalibrateCompass(void)
{
    static unsigned long calstamp;
    u_int lokcnt = 0;
    bool lokon = 0;
    Serial.println("Callibrating compass now!!!");
    calstamp = millis();
    int32_t magnetometer[3], min_mag[3], max_mag[3];
        for (int i = 0; i < 3; i++)
        {
            min_mag[i] = +2147483647;
            max_mag[i] = -2147483648;
        }


    while (millis() - calstamp <= 1000 * 60) // 1 minute callibrating
    {
        Mag.GetAxes(magnetometer);
        for (int i = 0; i < 3; i++)
        {
            min_mag[i] = min(min_mag[i], magnetometer[i]);
            max_mag[i] = max(max_mag[i], magnetometer[i]);
        }

        if (lokcnt++ > 250)
        {
            lokcnt = 0;
            mcp.digitalWrite(MAINSSWITCH_LEDGREEN_GPB, lokon);
            lokon = !lokon;
            mcp.digitalWrite(MAINSSWITCH_LEDRED_GPB, lokon);
            Serial.printf("Calllibration factors Compass: MaxX:%d MaxY:%d MaxZ:%d MinX:%d MinY:%d MinZ:%d\r\n", max_mag[0], max_mag[1], max_mag[2], min_mag[0], min_mag[2], min_mag[2] );
        }
    }
    // CompassCallibrationFactors(&running_max.x, &running_max.y, &running_max.z, &running_min.x, &running_min.y, &running_min.z, false); //  store callibration data
    Serial.printf("New callibration stored!!!\n\r");
    mcp.digitalWrite(MAINSSWITCH_LEDGREEN_GPB, 1);
    mcp.digitalWrite(MAINSSWITCH_LEDRED_GPB, 0);
    return 0;
}

float GetHeading(void)
{
    // Read accelerometer LSM303AGR.
    int32_t accelerometer[3];
    Acc.GetAxes(accelerometer);
    // Read temperature LSM303AGR.
    float temperature;
    Acc.GetTemperature(&temperature);
    // Read magnetometer LSM303AGR.
    int32_t magnetometer[3];
    Mag.GetAxes(magnetometer);

    // Serial.printf("Acc[mg]: x %3d,y %3d, z %3d ",accelerometer[0],accelerometer[1],accelerometer[2]);
    Serial.printf("Mag[mGauss]: x %3d, y %3d, z %3d n", magnetometer[0], magnetometer[2], magnetometer[2]);
    Serial.printf("Temp[C]: %0.1f\r\n", temperature);

    return 0;
}
float CompassAverage(float)
{
    return -1;
}
void GpsAverage(double *lat, double *lon)
{
}
