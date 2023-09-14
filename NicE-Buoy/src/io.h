#ifndef IO_H_
#define IO_H_

#define SDA 21
#define SCL 22
#define SDA_DISP 16
#define SCL_DISP 17
#define GPSRX 33
#define GPSTX 32

#define ESC_BB_PIN 12
#define ESC_SB_PIN 14

#define VBATT 35

#define LED_PIN 25
#define LEDSTRIPPIN 26

#define SWITCH_STANDBY 36
#define SWITCH_LOCK 39
#define BAT_PIN 35

#define RADIO_SCLK_PIN 18
#define RADIO_MISO_PIN 19
#define RADIO_MOSI_PIN 23
#define RADIO_CS_PIN 5
#define RADIO_DIO0_PIN 26
#define RADIO_RST_PIN 14
#define RADIO_DIO1_PIN 12
// #define RADIO_BUSY_PIN 32

#define MCP23017_ADDR 0x20
#define SWITCH1_GPA 0
#define SWITCH2_GPA 1
#define SWITCH3_GPA 2
#define LED1_GPA 3
#define LED2_GPA 4
#define LED3_GPA 5
#define LEDYELLOW_GPA 6
#define LEDRED_GPA 7
#define MAINSSWITCH_LEDGREEN_GPB 8
#define MAINSSWITCH_LEDRED_GPB 9
#define SWITCHV3V3_GPB 10
#define SWITCHPWRVBATT_GPB 11
#define SWITCH_T1_GPB 12
#define SWITCH_T2_GPB 13

#define LoRa_frequency 433E6

// switch
#define SWITCH1 36
#define SWITCH2 39

#define BUZZERPIN 13
#endif /* IO_H_ */
