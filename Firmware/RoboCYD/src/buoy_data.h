#ifndef BUOY_DATA_H
#define BUOY_DATA_H

#include <Arduino.h>

struct BuoyData {
    String id = "";
    bool present = false;
    unsigned long last_seen_ms = 0;
    
    // Telemetry fields
    String status = "UNKNOWN";
    float mag_dir = 0;
    float gps_dir = 0;
    float tg_dir = 0;
    float tg_dist = 0;
    float wind_dir = 0;
    float wind_std = 0;
    float bb_power = 0;
    float sb_power = 0;
    float pid_i = 0;
    float pid_r = 0;
    float battery_v = 0;
    float battery_pct = 0;
    String lat = "N/A";
    String lon = "N/A";
    String gps_fix = "N/A";
    int gps_sat = 0;
    float current = 0;

    // Setup / Calibration fields (Preloaded with standard baseline defaults, matching the webpage!)
    float kpr = 1.20;
    float kir = 0.10;
    float kdr = 0.05;
    float kps = 1.50;
    float kis = 0.05;
    float kds = 0.10;
    float max_speed = 100;
    float min_speed = 10;
    float pivot_speed = 0.2;
    float compass_offset = 0;
    float hold_radius = 2.0;
    bool rev_bb = false;
    bool rev_sb = false;
    bool swap_bb_sb = false;
    bool compass_trim_enabled = false;
    int dock_app_dist = 0;
    int dock_app_dir = 0;
    bool dock_to_wp = false;
};

extern BuoyData buoys[3];
extern int selected_buoy_idx;

// Checkbox enable/disable flags for communication channels
extern bool udp_enabled;
extern bool lora_enabled;

// Global Flag to manage local screen setups
extern bool in_setup_mode;
extern bool setup_data_loaded; // True once SETUPDATA is successfully received from the buoy!

void parse_buoy_packet(const String &packetStr, const String &source);
void send_buoy_command(const String &buoy_id, int cmd_code);

// Query setup parameters from buoy exactly matching webpage GET formatting
void query_buoy_setup(const String &buoy_id);

// Save tactile setup values dynamically over UDP & LoRa channels
void send_buoy_setup(int buoy_idx);

#endif // BUOY_DATA_H
