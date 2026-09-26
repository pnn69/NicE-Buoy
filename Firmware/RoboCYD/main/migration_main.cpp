#include <Arduino.h>

extern "C" void app_main(void)
{
    // ESP-IDF owns application startup.
    // Arduino is used only as a compatibility component during migration.
    initArduino();

    Serial.begin(115200);
    delay(100);

    Serial.println();
    Serial.println("====================================");
    Serial.println(" RoboCYD ESP-IDF + Arduino test");
    Serial.println("====================================");

    String framework = "Arduino compatibility active";

    Serial.println(framework);
    Serial.printf("millis() = %lu\n", millis());

    for (;;)
    {
        Serial.printf("RoboCYD alive: %lu ms\n", millis());
        delay(1000);
    }
}