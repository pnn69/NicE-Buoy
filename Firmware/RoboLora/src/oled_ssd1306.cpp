#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "oled_ssd1306.h"
#include "main.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define THRUST_BAR_WIDTH 8
#define THRUST_BAR_GAP 0
#define RIGHT_MARGIN (2 * THRUST_BAR_WIDTH)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayOK = false;

// Forward declarations of helper functions
void drawCommonHeader(RoboStruct *buoy);
void drawThrustBars(int bb, int sb);
void drawBatteryBar(float voltage);

bool initSSD1306(void) {
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
        Serial.println(F("SSD1306 allocation failed"));
        return false;
    }
    displayOK = true;
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    
    // Boot Screen
    display.setTextSize(2);
    display.setCursor(10, 8);
    display.println(F("NICE BUOY"));
    display.setCursor(22, 28);
    display.println(F("SAILING"));
    display.setCursor(22, 44);
    display.println(F("FOR YOU"));
    display.display();
    
    delay(5000);
    return true;
}

// ----------------------------------------------------------------------------------
// HELPERS
// ----------------------------------------------------------------------------------

void drawCommonHeader(RoboStruct *buoy) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.printf("ID:%04X", (uint16_t)(buoy->IDs & 0xFFFF));
}

void drawThrustBars(int bb, int sb) {
    int midY = 32;
    int maxBarH = 32;
    int sbX = SCREEN_WIDTH - THRUST_BAR_WIDTH;
    int bbX = sbX - THRUST_BAR_WIDTH;

    display.drawRect(bbX, 0, THRUST_BAR_WIDTH, 64, SSD1306_WHITE);
    int bbH = (constrain(abs(bb), 0, 100) * maxBarH) / 100;
    if (bb > 0) display.fillRect(bbX + 1, midY - bbH, THRUST_BAR_WIDTH - 2, bbH, SSD1306_WHITE);
    else if (bb < 0) display.fillRect(bbX + 1, midY, THRUST_BAR_WIDTH - 2, bbH, SSD1306_WHITE);

    display.drawRect(sbX, 0, THRUST_BAR_WIDTH, 64, SSD1306_WHITE);
    int sbH = (constrain(abs(sb), 0, 100) * maxBarH) / 100;
    if (sb > 0) display.fillRect(sbX + 1, midY - sbH, THRUST_BAR_WIDTH - 2, sbH, SSD1306_WHITE);
    else if (sb < 0) display.fillRect(sbX + 1, midY, THRUST_BAR_WIDTH - 2, sbH, SSD1306_WHITE);
}

void drawBatteryBar(float voltage) {
    float minV = 19.0;
    float maxV = 25.2;
    int barW = 112; // As per spec 4.2
    int fillW = (int)((constrain(voltage, minV, maxV) - minV) * barW / (maxV - minV));
    display.drawRect(0, 60, barW, 4, SSD1306_WHITE);
    if (fillW > 0) display.fillRect(0, 60, fillW, 4, SSD1306_WHITE);
}

// ----------------------------------------------------------------------------------
// INDIVIDUAL SCREENS
// ----------------------------------------------------------------------------------

void drawIdleScreen(RoboStruct *buoy, adcDataType *adc) {
    drawCommonHeader(buoy);
    
    display.setTextSize(2);
    display.setCursor(0, 10);
    display.printf("MAG: %03.0f", buoy->dirMag);

    display.setTextSize(1);
    display.setCursor(0, 28);
    display.printf("SAT:%02d %s", buoy->gpsSat, buoy->gpsFix ? "FIX" : "No FIX");

    // Fixed Mode Indicator
    display.setTextSize(2);
    display.setCursor(96, 0);
    display.print("I");

    display.setTextSize(1);
    display.setCursor(0, 42);
    display.printf("Lat: %2.6f", buoy->lat);
    display.setCursor(0, 50);
    display.printf("Lon: %2.6f", buoy->lng);
    
    drawBatteryBar(buoy->subAccuV);
    drawThrustBars(buoy->speedBb, buoy->speedSb);
}

void drawLockScreen(RoboStruct *buoy, adcDataType *adc) {
    drawCommonHeader(buoy);
    
    display.setTextSize(2);
    display.setCursor(0, 16);
    display.printf("%03.0f: %03.0f", buoy->tgDir, buoy->dirMag);

    display.setCursor(0, 36);
    if (buoy->tgDist < 1000) display.printf("D: %3.1fm", buoy->tgDist);
    else display.printf("D: %3.1fKm", buoy->tgDist / 1000.0);

    // Fixed Mode Indicator
    display.setTextSize(2);
    char m = (buoy->status == DOCKED || buoy->status == DOCKING || buoy->status == DOC) ? 'D' : 'L';
    display.setCursor(96, 0);
    display.printf("%c", m);

    // Speed PID I-Term to the left of the mode indicator (Size 2, no decimals)
    display.setTextSize(2);
    String iStr = String((int)buoy->ip);
    
    int iX = 94 - (iStr.length() * 12);
    if (iX < 42) iX = 42; // Prevent overwriting the ID header
    display.setCursor(iX, 0);
    display.print(iStr);

    drawBatteryBar(buoy->subAccuV);
    drawThrustBars(buoy->speedBb, buoy->speedSb);
}

void drawRemoteScreen(RoboStruct *buoy, adcDataType *adc) {
    drawCommonHeader(buoy);
    
    display.setTextSize(2);
    display.setCursor(0, 16);
    display.printf("%03.0f: %03.0f", buoy->tgDir, buoy->dirMag);

    display.setCursor(0, 36);
    display.printf("SPD: %d%%", buoy->speed);

    // Fixed Mode Indicator
    display.setTextSize(2);
    display.setCursor(96, 0);
    display.print("R");

    drawBatteryBar(buoy->subAccuV);
    drawThrustBars(buoy->speedBb, buoy->speedSb);
}

// ----------------------------------------------------------------------------------
// MAIN UPDATE ROUTE
// ----------------------------------------------------------------------------------

void updateOled(RoboStruct *buoy, adcDataType *adc) {
    if (!displayOK) return;

    // Show website IP address screen on OLED for 5 seconds when triggered by physical button long press
    extern unsigned long showIpUntil;
    if (millis() < showIpUntil) {
        display.clearDisplay();
        display.setTextColor(SSD1306_WHITE);
        display.setTextSize(1);
        
        display.setCursor(0, 0);
        display.println("Web Controller IP:");
        
        // Draw controller IP in bold TextSize 1 (using double-strike rendering)
        display.setCursor(0, 10);
        IPAddress displayIp = (WiFi.getMode() == WIFI_AP) ? WiFi.softAPIP() : WiFi.localIP();
        display.print(displayIp);
        display.setCursor(1, 10);
        display.print(displayIp);
        
        display.setCursor(0, 24);
        display.println("Known Buoys:");
        
        extern RoboStruct IDs[5];
        int lineCount = 0;
        int startY = 34;
        for (int i = 0; i < 5; i++) {
            if (IDs[i].IDs != 0 && IDs[i].ip > 0) {
                char buoyLine[40];
                sprintf(buoyLine, "ID:%04X: 192.168.1.%d", (uint16_t)(IDs[i].IDs & 0xFFFF), (int)IDs[i].ip);
                
                // Draw buoy IP line in bold TextSize 1 (using double-strike rendering)
                display.setCursor(0, startY + (lineCount * 10));
                display.print(buoyLine);
                display.setCursor(1, startY + (lineCount * 10));
                display.print(buoyLine);
                
                lineCount++;
                if (lineCount >= 3) break; // Limit to 3 lines
            }
        }
        if (lineCount == 0) {
            display.setCursor(0, startY);
            display.println("None connected yet");
        }
        
        display.display();
        return;
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    switch (buoy->status) {
        case IDLE:
        case IDELING:
            drawIdleScreen(buoy, adc);
            break;

        case LOCKED:
        case LOCKING:
        case DOCKED:
        case DOCKING:
            drawLockScreen(buoy, adc);
            break;

        case REMOTE:
            drawRemoteScreen(buoy, adc);
            break;

        default:
            drawIdleScreen(buoy, adc);
            break;
    }

    display.display();
}

void showDip(char s, String p) {
    if (!displayOK) return;
    display.clearDisplay();
    display.setTextSize(s);
    display.setCursor(0, 0);
    display.println(p);
    display.display();
}
