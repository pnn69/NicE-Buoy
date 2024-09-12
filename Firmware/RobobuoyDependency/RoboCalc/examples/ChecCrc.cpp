#include <Arduino.h>
#include <RoboCalc.h>

void setup()
{
    Serial.begin(115200);
}
String good_msg = "$HDG,060.6*49";
String bad_msg = "$HDG,060.6*00";
void loop()
{

    Serial.print(good_msg);
    if (verifyCRC(good_msg))
    {
        Serial.println(" <CRC OK");
    }
    else
    {
        Serial.println(" <CRC NOT OK");
    }
    delay(1000);
    Serial.print(bad_msg);
    if (verifyCRC(bad_msg))
    {
        Serial.println(" <CRC OK");
    }
    else
    {
        Serial.println(" <NOT OK");
    }
    delay(1000);
}