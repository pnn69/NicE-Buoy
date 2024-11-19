//  Lilygo v1.6.1
//  https://github.com/Xinyuan-LilyGO/LilyGo-LoRa-Series/blob/master/schematic/T3_V1.6.1.pdf
//  Pinout: https://www.tinytronics.nl/en/development-boards/microcontroller-boards/with-lora/lilygo-ttgo-t3-lora32-433mhz-v1.6.1-esp32

#ifndef IO_BASE_H_
#define IO_BASE_H_

#define BUTTON_PIN_SW 12
#define BUTTON_PIN_GND 14
#define LEDS_PIN 18 // fastled
#define LED_PIN 13  // 13
#define BULETINLED 25

#define RADIO_SCLK_PIN 5
#define RADIO_MISO_PIN 19
#define RADIO_MOSI_PIN 27
#define RADIO_CS_PIN 18
#define RADIO_DIO0_PIN 26
#define RADIO_RST_PIN 23
#define RADIO_DIO1_PIN 33
#define RADIO_BUSY_PIN 32

#define VBATT 1
#define VBATTSELECT 1

#endif /* IO_BASE_H_ */
