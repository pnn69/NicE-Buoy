#ifndef CYD_WIFI_H
#define CYD_WIFI_H

#include <Arduino.h>

void init_wifi_and_ota();
void handle_ota();
void udp_broadcast(const String &message);

// Broadcasters to push structured UDP/LoRa telemetry over WebSockets to browsers
void broadcast_websocket_udp(const String &payload, int rssi, const String &ip);
void broadcast_websocket_lora(const String &payload, int rssi);

// Client handlers to process incoming web requests and WebSockets loops
void handle_wifi_clients();

// Diagnostic log line, broadcast on UDP 1002 alongside the Subs' and Tops' logs. See cyd_wifi.cpp.
void cyd_log(const char *fmt, ...);

#endif // CYD_WIFI_H
