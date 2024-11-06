#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// // Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)

bool displayOK = false;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

bool initSSD1306(void)
{
    //     // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        Serial.println(F("SSD1306 failed"));
        return false;
    }
    displayOK = true;
    Serial.println(F("SSD1306 OK"));
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(6 * 7, 10);
    display.print("NicE BUOY");
    display.setCursor(3 * 7, 20);
    display.printf("Sailing for you!");
    display.setCursor(0, 30);
    display.printf("2x adj startline");
    display.setCursor(0, 40);
    display.printf("3x adj track");
    display.setCursor(0, 50);
    display.printf("1x long DOCK");
    display.display();
    return true;
}

void udateDisplay(void)
{
    if (displayOK)
    {
        display.clearDisplay();
        display.display();
        display.printf("Hello world!");
        display.display();
    }
}
