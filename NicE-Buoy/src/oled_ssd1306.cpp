#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "general.h"
#include "LiLlora.h"
#include "gps.h"

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
        Serial.println(F("SSD1306 allocation failed"));
        return false;
    }
    displayOK = true;
    Serial.println(F("SSD1306 allocation OK"));
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(6 * 7, 10);
    display.print("NicE BUOY");
    display.setCursor(3 * 7, 20);
    display.printf("Sailing for you!");
    display.display();
    return true;
}

void speedbars(int sb, int bb)
{
#define barwide 10
    // if (sb == 0)
    //     sb = 1;
    // if (bb == 0)
    //     bb = 1;
    bb = constrain(bb, -100, 100);
    sb = constrain(sb, -100, 100);
    display.drawRect(0, 0, barwide, 64, WHITE);
    display.drawRect(128 - barwide, 0, barwide, 64, WHITE);
    if (bb <= 0)
    {
        display.fillRect(0, 32, barwide, 32 * -bb / 100, WHITE);
    }
    else
    {
        display.fillRect(0, 32 + 32 * -bb / 100, barwide, 32 * bb / 100, WHITE);
    }

    if (sb <= 0)
    {
        display.fillRect(128 - barwide, 32, barwide, 32 * -sb / 100, WHITE);
    }
    else
    {
        display.fillRect(128 - barwide, 32 + 32 * -sb / 100, barwide, 32 * sb / 100, WHITE);
    }
}

void BatPowerBarr(float perc)
{
    int fill = 0;
    fill = (SCREEN_WIDTH - barwide * 2 - 12) * perc / 100;
    display.drawRect(barwide + 7, 54, SCREEN_WIDTH - barwide * 2 - 12, 10, WHITE);
    display.fillRect(barwide + 7, 54, fill, 10, WHITE);
}

void udateDisplay(int sb, int bb, unsigned long distance, unsigned long direction, unsigned long mdirection, bool fix)
{
    if (displayOK)
    {
        display.clearDisplay();
        display.setCursor(6 * 5, 0);
        display.printf("NicE BUOY %d", buoyID);
        display.setCursor(barwide + 7, 10);
        display.printf("Stat:%d Rssi:%d", status, loraIn.rssi);
        display.setCursor(barwide + 7, 20);
        display.printf("Dist: %3ldM", distance);
        display.setCursor(barwide + 7, 30);
        display.printf("Dir:%3d HDG:%3d", (int)direction, (int)mdirection);
        display.setCursor(barwide + 7, 40);
        display.printf("%4d%%", bb);
        display.setCursor(128 - barwide - 5 * 7, 40);
        display.printf("%4d%%", sb);
        speedbars(sb, bb);
        BatPowerBarr(buoy.vperc);
        display.display();
    }
}