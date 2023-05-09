/*
https://github.com/sandeepmistry/arduino-LoRa/blob/master/examples/LoRaDuplex/LoRaDuplex.ino
http://www.lilygo.cn/prod_view.aspx?TypeId=50003&Id=1130&FId=t3:50003:3
https://github.com/Xinyuan-LilyGO/TTGO-LoRa-Series
https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/master/schematic/T3_V1.6.1.pdf
433MHz is SX1278
*/
#include <Arduino.h>
#include "compass.h"
#include "gps.h"

void setup()
{
    Serial.begin(9600);
    xTaskCreate(CompassTask, "CompassTask", 100, NULL, 1, NULL);
    xTaskCreate(CompassTask, "GpsTask", 100, NULL, 1, NULL);
}

void loop()
{
    Serial.println(GetHeading());
    delay(100);
}