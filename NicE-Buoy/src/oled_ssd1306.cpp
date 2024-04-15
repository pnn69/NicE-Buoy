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

void udateDisplay(int sb, int bb, unsigned long distance, unsigned int direction, unsigned int mdirection, bool fix)
{
    if (displayOK)
    {
        display.clearDisplay();
        // putchar(x & (1u << i) ? '1' : '0');
        display.setCursor(barwide + 7, 0);
        display.printf("NicE BUOY %d", buoyID);
        display.setCursor(128 - barwide - 7 * 3 - 2, 0);
        display.printf("%d", gpsdata.nrsats);
        display.setCursor(128 - barwide - 7 - 2, 0);
        if (fix)
        {
            display.printf("*");
        }
        else
        {
            display.printf("0");
        }
        String st;
        switch (status)
        {
        case LOCKED:
            st = "LOCKED";
            break;
        case DOCKED:
            st = "DOCKED";
            break;
        case REMOTE:
            st = "REMOTE";
            break;
        case DOCKING:
            st = "DOCKING";
            break;
        case IDLE:
            st = "IDLE";
            break;
        case CALIBRATE_MAGNETIC_COMPASS:
            st = "C COMP";
            break;
        case STORE_CALIBRATE_OFFSET_MAGNETIC_COMPASS:
            st = "C OFFS";
            break;

        default:
            st = "";
            break;
        }
        display.setCursor(barwide + 7, 10);
        display.print(st);
        display.setCursor(128 - barwide - 7 * 8 - 1, 10);
        display.printf("Rssi:%04d", loraIn.rssi);
        display.setCursor(barwide + 7, 20);
        if (status == LOCKED || status == DOCKED)
        {
            display.printf("Dist:%5.1lfM", buoy.tgdistance);
            display.setCursor(128 - barwide - 28, 20);
            display.printf("d%02dM", buoy.maxOfsetDist);
        }
        display.setCursor(barwide + 7, 30);
        display.printf("Dir:%03.0lf  HDG:%3.0lf", buoy.tgdir, buoy.mheading);
        display.setCursor(barwide + 7, 40);
        display.printf("%4d%%", buoy.speedbb);
        display.printf(" D %1.0lf", gpsdata.cource);
        display.setCursor(128 / 2, 40);
        display.setCursor(128 - barwide - 5 * 7, 40);
        display.printf("%4d%%", buoy.speedsb);
        BatPowerBarr(buoy.vperc);
        speedbars(sb, bb);
        display.display();
        // }

        //         display.clearDisplay();
        //         display.setCursor(6 * 5, 0);
        //         display.printf("NicE BUOY %d", buoyID);
        //         display.setCursor(barwide + 7, 10);
        //         display.printf("Stat:%d Rssi:%d", status, loraIn.rssi);
        //         display.setCursor(barwide + 7, 20);
        //         display.printf("Dist: %3ldM", distance);
        //         display.setCursor(barwide + 7, 30);
        //         display.printf("Dir:%3d HDG:%3d", (int)direction, (int)mdirection);
        //         display.setCursor(barwide + 7, 40);
        //         display.printf("%4d%%", bb);
        //         display.setCursor(128 - barwide - 5 * 7, 40);
        //         display.printf("%4d%%", sb);
        //         speedbars(sb, bb);
        //         BatPowerBarr(buoy.vperc);
        //         display.display();
        //     }
    }
}