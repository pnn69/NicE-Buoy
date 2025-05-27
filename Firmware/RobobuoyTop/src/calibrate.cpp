#include <Arduino.h>
#include <RoboCompute.h>
#include "main.h"
#include "topwifi.h"

int calibratStatus = 0;
int cdir = 0;
int cspeed = 0;
unsigned long calibrateTime = 0;
double startlat = 0;
double startlng = 0;
double homelat = 0;
double homelng = 0;
double sailedDir = -1;
RoboStruct compasOffestCallicration(RoboStruct calibrate)
{

    if (calibratStatus == 0)
    {
        printf("START CALIBRATING OFFSET\r\n");
        cdir = calibrate.dirMag;
        cspeed = 50;
        calibrate.speedSet = cspeed;
        calibrate.tgDir = cdir;
        calibrate.cmd = DIRSPEED;
        calibrateTime = millis();
        xQueueSend(udpOut, (void *)&calibrate, 0); // update WiFi
        calibratStatus = 1;
    }
    // else if (calibratStatus == 1 && calibrateTime + 10 * 1000 < millis())
    else if (calibratStatus == 1 && calibrateTime + 2 * 1000 < millis())
    {
        // 52.32049143852263, 4.965444283020624 //home
        // startlat = calibrate.lat;
        // startlng = calibrate.lng;
        startlat = 52.32049143852263;
        startlng = 4.965444283020624;
        calibratStatus = 2;
    }
    // else if (calibratStatus == 3 && calibrateTime + 70 * 1000 < millis())
    else if (calibratStatus == 2 && calibrateTime + 4 * 1000 < millis())
    {
        // 52.50113909510498, 5.062448670554131 //volendam
        // sailedDir = calculateAngle(startlat, startlng, calibrate.lat, calibrate.lng);
        sailedDir = calculateAngle(startlat, startlng, 52.50113909510498, 5.062448670554131);
        cdir = smallestAngle(sailedDir, cdir);
        calibrate.cmd = STORE_DECLINATION;

        calibrate.tgDir = cdir;
        printf("error found: %d\r\n", cdir);
        // xQueueSend(udpOut, (void *)&calibrate, 0); // update WiFi
        calibrate.tgLat = homelat;
        calibrate.tgLng = homelng;
        calibrate.cmd = -1;
        calibrate.status = LOCKED;
        calibratStatus = 0;
    }

    return calibrate;
}