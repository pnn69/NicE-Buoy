#include <Arduino.h>
#include <Wire.h>
#include <io23017.h>
#include <Adafruit_MCP23X17.h>
#include "io.h"
Adafruit_MCP23X17 mcp;

static bool mcp_ok = false;

bool initMCP23017(void)
{
    if (!mcp.begin_I2C())
    {
        mcp_ok = false;
        Serial.println("MCP23017 Error!");
        return 1;
    }
    else
    {
        mcp.pinMode(LEDYELLOW_GPA, OUTPUT);
        mcp.pinMode(LEDRED_GPA, OUTPUT);
        mcp.pinMode(LED1_GPA, OUTPUT);
        mcp.pinMode(LED2_GPA, OUTPUT);
        mcp.pinMode(LED3_GPA, OUTPUT);
        mcp.pinMode(SWITCH1_GPA, INPUT_PULLUP);
        mcp.pinMode(SWITCH2_GPA, INPUT_PULLUP);
        mcp.pinMode(SWITCH3_GPA, INPUT_PULLUP);
        mcp.pinMode(MAINSSWITCH_LEDGREEN_GPB, OUTPUT);
        mcp.pinMode(MAINSSWITCH_LEDRED_GPB, OUTPUT);
        mcp.pinMode(SWITCHV3V3_GPB, OUTPUT);
        mcp.pinMode(SWITCHPWRVBATT_GPB, OUTPUT);
        mcp.pinMode(SWITCH_T1_GPB, INPUT);
        mcp.pinMode(SWITCH_T2_GPB, OUTPUT);

        mcp.digitalWrite(LEDYELLOW_GPA, 0);
        mcp.digitalWrite(LEDRED_GPA, 1);
        mcp.digitalWrite(SWITCHPWRVBATT_GPB, 1);
        mcp.digitalWrite(MAINSSWITCH_LEDGREEN_GPB, 1);
        mcp.digitalWrite(MAINSSWITCH_LEDRED_GPB, 0);
        Serial.println("MCP23017 Initialised");
        mcp_ok = true;
    }
    return 0;
}