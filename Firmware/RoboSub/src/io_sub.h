/*
    Hardware Pin Definitions
*/

#ifndef IO_SUB_H_
#define IO_SUB_H_

#ifdef ROBOBUOY_V3
    // V3.0 Pin Definitions
    #define BUZZER_PIN 12
    #define LEDS_PIN 32
    #define BUTTON_PIN 17
    #define VBATT 35
    #define PWRENABLE 26
    #define COM_PIN_TX 18
    #define COM_PIN_RX 5
    #define ESC_SB_PIN 14
    #define ESC_SB_PWR_PIN 16
    #define ESC_BB_PIN 33
    #define ESC_BB_PWR_PIN 25
    #define IMON_PIN 36
#else
    // V2.0 Pin Definitions (Default)
    #define BUZZER_PIN 12
    #define LEDS_PIN 13
    #define BUTTON_PIN 32
    #define VBATT 35
    #define PWRENABLE 26
    #define COM_PIN_TX 18
    #define COM_PIN_RX 5
    #define ESC_SB_PIN 25
    #define ESC_SB_PWR_PIN 16
    #define ESC_BB_PIN 14
    #define ESC_BB_PWR_PIN 4
#endif

#endif /* IO_SUB_H_ */
