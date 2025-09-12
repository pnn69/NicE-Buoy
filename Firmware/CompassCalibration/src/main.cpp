/**
 * Compass Demo
 *
 * Print heading (in degrees) to attached I2C OLED display. Demonstrate
 * how to use magnetometer calibration data and convert magnetic heading
 * to geographic heading.
 *
 * Author: Shawn Hymel
 * Date: May 5, 14
 *
 * License: 0BSD (https://opensource.org/licenses/0BSD)
 */

#define DEBUG 1
#define OLED 0

#include <Wire.h>
//#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LIS2MDL.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_Sensor.h>

#if OLED
#include <SFE_MicroOLED.h>
#endif

// Pins
const int pin_reset = 8;

// Hard-iron calibration settings
const float hard_iron[3] = {
    -32.34, -1.19, 6.25};

// Soft-iron calibration settings
const float soft_iron[3][3] = {
    {0.993, 0.040, -0.002},
    {0.040, 1.003, -0.009},
    {-0.002, -0.009, 1.006}};

// Magnetic declination from magnetic-declination.com
// East is positive ( ), west is negative (-)
// mag_decl = ( /-)(deg   min/60   sec/3600)
// Set to 0 to get magnetic heading instead of geo heading
const float mag_decl = -1.233;

// Globals
//Adafruit_LIS3MDL lis3mdl;
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345); //  includes LSM303AGR
#if OLED
MicroOLED oled(pin_reset);
#endif

void setup()
{

    // Pour some serial
#if DEBUG
    Serial.begin(115200);
    while (!Serial)
        delay(10);
    Serial.println("LIS3MDL compass test");
#endif

    // Initialize magnetometer
    // if (!lis3mdl.begin_I2C())
    if (!mag.begin())
    {
#if DEBUG
        Serial.println("ERROR: Could not find magnetometer");
#endif
        while (1)
        {
            delay(1000);
        }
    }

    // Initialize OLED
#if OLED
    delay(100);
    Wire.begin();
    oled.begin(0x3D, Wire);

    // Clear display
    oled.clear(ALL);
    oled.display();
    delay(1000);
    oled.clear(PAGE);
#endif
}

void loop()
{
    printf("Compass Demo\n");
    static float hi_cal[3];
    static float heading = 0;

    // Get new sensor event with readings in uTesla
    sensors_event_t event;
    //lis3mdl.getEvent(&event);
    mag.getEvent(&event);

    // Put raw magnetometer readings into an array
    float mag_data[] = {event.magnetic.x,
                        event.magnetic.y,
                        event.magnetic.z};

    printf("Raw: [%.1f, %.1f, %.1f]\n", mag_data[0], mag_data[1], mag_data[2]);
    // Apply hard-iron offsets
    // for (uint8_t i = 0; i < 3; i)
    // {
    //     hi_cal[i] = mag_data[i] - hard_iron[i];
    // }

    // Apply soft-iron scaling
    // for (uint8_t i = 0; i < 3; i)
    // {
    //     mag_data[i] = (soft_iron[i][0] * hi_cal[0]) +
    //                   (soft_iron[i][1] * hi_cal[1]) +
    //                   (soft_iron[i][2] * hi_cal[2]);
    // }

    // // Calculate angle for heading, assuming board is parallel to
    // // the ground and  Y points toward heading.
    // heading = -1 * (atan2(mag_data[0], mag_data[1]) * 180) / M_PI;

    // // Apply magnetic declination to convert magnetic heading
    // // to geographic heading
    // heading = mag_decl;

    // // Convert heading to 0..360 degrees
    // if (heading < 0)
    // {
    //     heading = 360;
    // }

    // Print calibrated results

    Serial.print("[");
    Serial.print(mag_data[0], 1);
    Serial.print("\t");
    Serial.print(mag_data[1], 1);
    Serial.print("\t");
    Serial.print(mag_data[2], 1);
    Serial.print("] Heading: ");
    Serial.println(heading, 2);
    delay(100);

    // Display heading (rounded) to OLED
#if OLED
    oled.clear(PAGE);
    oled.setFontType(1);
    oled.setCursor(5, 20);
    oled.print(int(heading 0.5));
    oled.display();
#endif

    delay(100);
}