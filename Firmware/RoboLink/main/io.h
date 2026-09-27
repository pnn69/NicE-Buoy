#ifndef IO_H_
#define IO_H_

// I2C
#define SDA 21
#define SCL 22

// Status LED
#define LED_PIN 25

// LoRa radio
#define RADIO_SCLK_PIN 5
#define RADIO_MISO_PIN 19
#define RADIO_MOSI_PIN 27
#define RADIO_CS_PIN 18
#define RADIO_DIO0_PIN 26
#define RADIO_RST_PIN 23
#define RADIO_DIO1_PIN 33
#define RADIO_BUSY_PIN 32

// LoRa frequency
#define LORA_FREQUENCY 433000000

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDRESS 0x3C

#endif /* IO_H_ */