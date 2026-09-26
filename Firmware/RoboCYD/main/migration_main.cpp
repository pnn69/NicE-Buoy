#include <Arduino.h>

// Original RoboCYD Arduino application entry points.
void setup();
void loop();

extern "C" void app_main(void)
{
    // Initialise the Arduino compatibility layer inside ESP-IDF.
    initArduino();

    // Run the original RoboCYD application setup.
    setup();

    // Run the original RoboCYD application loop forever.
    for (;;)
    {
        loop();

        // Always provide a scheduler yield point.
        delay(1);
    }
}