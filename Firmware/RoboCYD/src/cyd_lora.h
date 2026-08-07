#ifndef CYD_LORA_H
#define CYD_LORA_H

#include <Arduino.h>

void init_lora();
void check_lora_packets();
void send_lora_packet(const String &message);

#endif // CYD_LORA_H
