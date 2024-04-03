#ifndef io23017_H_
#define io23017_H_
#include <Adafruit_MCP23X17.h>

#define PUSHED 0
#define BUTTON_LIGHT_ON mcp.digitalWrite(SWITCHPWRVBATT_GPB, 1);
#define BUTTON_LIGHT_OFF mcp.digitalWrite(SWITCHPWRVBATT_GPB, 0);
#define SWITCH_RED_ON mcp.digitalWrite(MAINSSWITCH_LEDRED_GPB, 1);
#define SWITCH_RED_OFF mcp.digitalWrite(MAINSSWITCH_LEDRED_GPB, 0);
#define SWITCH_GRN_ON mcp.digitalWrite(MAINSSWITCH_LEDGREEN_GPB, 1);
#define LEDYELLOW_ON mcp.digitalWrite(LEDYELLOW_GPB, 0);
#define LEDYELLOW_OFF mcp.digitalWrite(LEDYELLOW_GPB, 1);
#define LEDRED_ON mcp.digitalWrite(LEDRED_GPB, 0);
#define LEDRED_OFF mcp.digitalWrite(LEDRED_GPB, 1);
#define SWITCH_GRN_OFF mcp.digitalWrite(MAINSSWITCH_LEDGREEN_GPB, 0);
#define FRONTBUTTON_READ mcp.digitalRead(SWITCH1_GPA)
#define BUTTON_LIGHT_READ mcp.digitalRead(SWITCHPWRVBATT_GPB)
#define SWITCH_GRN_READ mcp.digitalRead(MAINSSWITCH_LEDGREEN_GPB)

bool initMCP23017(void);
extern Adafruit_MCP23X17 mcp;

#endif /* io23017_H_ */
