#include <Arduino.h>
#include <Wire.h>
#include "main.h"
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
        Serial.println(F("SSD1306 allocation failed"));
        return false;
    }
    displayOK = true;
    Serial.println(F("SSD1306 allocation OK"));
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.print(" Robobuoy\n\r\n\r Sailing\n\rfor you!!!");
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
    display.setCursor(barwide + 7, 0);
    display.printf("NicE BUOY %d", buoyID);
    display.setCursor(128 - barwide - 7 * 3 - 2, 0);
    // display.printf("%d", buoy[buoyID].nrsats);
    display.setCursor(128 - barwide - 7 - 2, 0);
    display.setCursor(barwide + 7, 10);
    // display.print(st);
    display.setCursor(128 - barwide - 7 * 8 - 1, 10);
    // display.printf("Rssi:%04d", (int)buoy[buoyID].rssi);
    // display.printf("Ki:%2.2lf", buoy[buoyID].ki);
    display.setCursor(barwide + 7, 20);
    // if (buoy[buoyID].status == LOCKED || buoy[buoyID].status == DOCKED)
    // {
    //     display.printf("Dist:%5.1lfM", buoy[buoyID].tgdistance);
    // }
    // else
    // {
    //     display.printf("KI:%1.1lf", buoy[buoyID].i);
    //     display.setCursor(128 - barwide - 7 * 8 - 1, 20);
    //     display.printf("Rssi:%04d", (int)buoy[buoyID].rssi);
    // }
    display.setCursor(barwide + 7, 30);
    // display.printf("Dir:%03.0lf  HDG:%03d", buoy[buoyID].tgdir, buoy[buoyID].mdir);
    display.setCursor(barwide + 7, 40);
    // buoy[buoyID].speedbb = constrain(buoy[buoyID].speedbb, -100, 100);
    // display.printf("%4d%%", buoy[buoyID].speedbb);
    // display.printf(" D %d", buoy[buoyID].gpscource);
    display.setCursor(128 / 2, 40);
    display.setCursor(128 - barwide - 5 * 7, 40);
    // buoy[buoyID].speedsb = constrain(buoy[buoyID].speedsb, -100, 100);
    // display.printf("%4d%%", buoy[buoyID].speedsb);
    // speedbars(buoy[buoyID].speedbb, buoy[buoyID].speedsb);
    // BatPowerBarr(buoy[buoyID].percentage);
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

void updateDisplay(String showdata)
{
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    char pout[100];
    showdata.toCharArray(pout, showdata.length() + 1);
    display.println(pout);
    display.display();
}
void updateOled(RoboStruct *data)
{
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.printf("BB%2d%%\n\r", data->speedBb);
    display.setCursor(0, 15);
    display.printf("SB%2d%%\n\r", data->speedSb);
    // display.setCursor(0, 40);
    // display.printf("%04.2f%V\n\r", data->subAccuV);
    display.setCursor(90, 0);
    if (data->status == IDLE)
    {
        display.printf("I");
    }
    if (data->status == LOCKED)
    {
        display.printf("L");
    }
    if (data->status == DOCKED)
    {
        display.printf("D");
    }
    if (data->status == REMOTE)
    {
        display.printf("R");
    }
    if (data->status == DOCKED || data->status == LOCKED)
    {
        display.setCursor(0, 30);
        display.printf("%03.1f%M\n\r", data->tgDist);
    }
    int fill = 0;
    int tmp = (int)map(data->subAccuV, 19, 25.2, 0, 100); // 4095;
    tmp = constrain(tmp, 0, 100);
    fill = (SCREEN_WIDTH - 20) * tmp / 100;
    display.drawRect(0, 54, SCREEN_WIDTH - 20, 10, WHITE);
    display.fillRect(0, 54, fill, 10, WHITE);

    display.drawRect(SCREEN_WIDTH - 19, 0, 10, 64, WHITE);
    fill = (data->speedBb / 100.0) * (SCREEN_HEIGHT / 2);
    if (fill < 0)
    {
        display.fillRect(SCREEN_WIDTH - 19, SCREEN_HEIGHT / 2, 10, fill, WHITE);
    }
    else
    {
        display.fillRect(SCREEN_WIDTH - 19, SCREEN_HEIGHT / 2 - fill, 10,fill , WHITE);
    }
    display.fillRect(SCREEN_WIDTH - 19, SCREEN_HEIGHT / 2, 10, -fill, WHITE);
    Serial.println(data->speedBb);
    Serial.println(fill);

    display.drawRect(SCREEN_WIDTH - 10, 0, 10, 64, WHITE);
    fill = (data->speedSb / 100.0) * (SCREEN_HEIGHT / 2);
    if (fill < 0)
    {
        display.fillRect(SCREEN_WIDTH - 10, SCREEN_HEIGHT / 2, 10, -fill, WHITE);
    }
    else
    {
        display.fillRect(SCREEN_WIDTH - 10, SCREEN_HEIGHT / 2 - fill, 10,fill, WHITE);
    }

    Serial.println(data->speedSb);
    Serial.println(fill);
    display.display();
}
