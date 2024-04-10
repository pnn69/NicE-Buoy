#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "general.h"

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
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.print("NicE BUOY\n\rSailing\n\rfor you!!!");
    display.display();
    display.setTextSize(1);
    return true;
}

void speedbars(int bb, int sb)
{
#define barwide 10
    if (sb == 0)
        sb = 1;
    if (bb == 0)
        bb = 1;
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
        display.fillRect(SCREEN_WIDTH - barwide, 32, barwide, 32 * -sb / 100, WHITE);
    }
    else
    {
        display.fillRect(SCREEN_WIDTH - barwide, 32 + 32 * -sb / 100, barwide, 32 * sb / 100, WHITE);
    }
}

void BatPowerBarr(float perc)
{
    int fill = 0;
    fill = (SCREEN_WIDTH - barwide * 2 - 12) * perc / 100;
    display.drawRect(barwide + 7, 54, SCREEN_WIDTH - barwide * 2 - 12, 10, WHITE);
    display.fillRect(barwide + 7, 54, fill, 10, WHITE);
}

void ShowBuoyData(int buoyID)
{
    display.clearDisplay();
    // putchar(x & (1u << i) ? '1' : '0');
    speedbars(buoy[buoyID].speedbb, buoy[buoyID].speedsb);
    display.setCursor(barwide + 7, 0);
    display.printf("NicE BUOY %d", buoyID);
    if (buoy[buoyID].fix)
    {
        display.printf("    *");
    }
    else
    {
        display.printf("    0");
    }
    String st;
    switch (buoy[buoyID].remotestatus)
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
    default:
        st = "";
        break;
    }
    display.setCursor(barwide + 7, 10);
    display.print(st);
    display.setCursor(barwide + 55, 10);
    display.printf("Rssi:%d", (int)buoy[buoyID].rssi);
    display.setCursor(barwide + 7, 20);
    if (buoy[buoyID].status == LOCKED || buoy[buoyID].status == DOCKED)
    {
        display.printf("Dist:%5.1lfM", buoy[buoyID].tgdistance);
        display.setCursor(128 - barwide - 3 * 7, 20);
        display.printf("%dM",buoy[buoyID].maxOfsetDist);
    }
    display.setCursor(barwide + 7, 30);
    display.printf("Dir:%3d HDG:%3d", (int)buoy[buoyID].tgdir, (int)buoy[buoyID].mdir);
    display.setCursor(barwide + 7, 40);
    buoy[buoyID].speedbb = constrain(buoy[buoyID].speedbb, -100, 100);
    display.printf("%4d%%", buoy[buoyID].speedbb);
    display.printf(" D %d", buoy[buoyID].gpscource);
    display.setCursor(128 / 2, 40);
    display.setCursor(128 - barwide - 5 * 7, 40);
    buoy[buoyID].speedsb = constrain(buoy[buoyID].speedsb, -100, 100);
    display.printf("%4d%%", buoy[buoyID].speedsb);
    BatPowerBarr(buoy[buoyID].percentage);
    display.display();
}

void showDip(char s, String p)
{
    display.clearDisplay();
    display.setTextSize(s);
    display.setCursor(0, 0);
    char pout[100];
    p.toCharArray(pout, p.length() + 1);
    display.println(pout);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.display();
}

void updateDisplay(void)
{
    if (displayOK)
    {
        ShowBuoyData(1);
        // display.clearDisplay();
        // display.setCursor(0, 8);
        // display.printf("B1> %d", buoy[1].tgdir);
        // display.print(" ");
        // display.printf("%d", buoy[1].tgdistance);
        // display.print(" ");
        // display.printf("%d", buoy[1].rssi);

        // display.setCursor(3 * 8, 16);
        // display.printf("BB:%d%%", buoy[1].speedbb);
        // display.setCursor(9 * 8, 16);
        // display.printf("SB:%d%%", buoy[1].speedsb);

        // display.setCursor(0, 26);
        // display.printf("B2> %d", buoy[2].tgdir);
        // display.print(" ");
        // display.printf("%d", buoy[2].tgdistance);
        // display.print(" ");
        // display.printf("%d", buoy[2].rssi);

        // display.setCursor(3 * 8, 34);
        // display.printf("BB:%d%%", buoy[2].speedbb);
        // display.setCursor(9 * 8, 34);
        // display.printf("SB:%d%%", buoy[2].speedsb);

        // display.setCursor(0, 44);
        // display.printf("B3> %d", buoy[3].tgdir);
        // display.print(" ");
        // display.printf("%d", buoy[3].tgdistance);
        // display.print(" ");
        // display.printf("%d", buoy[3].rssi);

        // display.setCursor(3 * 8, 52);
        // display.printf("BB:%d%%", buoy[3].speedbb);
        // display.setCursor(9 * 8, 52);
        // display.printf("SB:%d%%", buoy[3].speedsb);

        // display.display();
    }
}
