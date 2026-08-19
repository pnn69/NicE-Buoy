#ifndef BUOY_DATA_H
#define BUOY_DATA_H

#include <Arduino.h>

struct BuoyData {
    String id = "";
    bool present = false;
    unsigned long last_seen_ms = 0;
    String ip_addr = ""; // Connection IP address (empty if LoRa only)
    int lora_rssi = -999; // LoRa received signal strength indicator (-999 if no active LoRa telemetry)
    
    // Telemetry fields
    String status = "UNKNOWN";
    float mag_dir = 0;
    float gps_dir = 0;
    float tg_dir = 0;
    float tg_dist = 0;
    float tg_speed = 0; // Target Speed for Manual Navigation Slider
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
    int dock_app_dist = 20;
    int dock_app_dir = 180;
    bool dock_to_wp = false;

    // GPS Fourier compass calibration progress, from GPS_FOURIER_STATUS (cmd 90).
    // cal_seen_ms is 0 until the first report arrives; the display uses it to decide whether it
    // has anything worth showing rather than drawing a stale run from an hour ago.
    unsigned long cal_seen_ms = 0;
    int cal_step = 0;        // gpscal_step_t: 0 idle, 1 fetch, 2 settle, 3 run, 4 store, 5 done, 6 aborted
    int cal_leg = 0;         // 0..7
    float cal_cmd_dir = 0;   // heading commanded for this leg
    float cal_dist = 0;      // metres covered on this leg
    float cal_err = 0;       // live error of the leg in progress
    float cal_last_err = 0;  // error of the last completed leg
};

extern BuoyData buoys[3];
extern int selected_buoy_idx;

// Checkbox enable/disable flags for communication channels
extern bool udp_enabled;
extern bool lora_enabled;

// Global Flag to manage local screen setups
extern bool in_setup_mode;
extern bool setup_data_loaded; // True once SETUPDATA is successfully received from the buoy!
extern bool in_mannav_mode;    // True when manually navigating the buoy!

extern unsigned long last_udp_blink_ms;      // Any incoming UDP packet
extern unsigned long last_udp_sel_blink_ms;  // UDP packet from the currently selected buoy only
extern unsigned long last_lora_blink_ms;     // LoRa packet from the currently selected buoy only
extern unsigned long last_global_lora_blink_ms;

// Outgoing traffic timestamps - the indicator dots turn RED while transmitting
extern unsigned long last_udp_tx_ms;
extern unsigned long last_lora_tx_ms;

void parse_buoy_packet(const String &packetStr, const String &source, int rssi = -999);
void send_buoy_command(const String &buoy_id, int cmd_code);

// Starts a GPS Fourier compass calibration run on the buoy's Top. Needs its own sender
// because send_buoy_command() emits an empty payload, and the still-water flag lives in
// the first payload field.
void send_gps_fourier_calibrate(const String &buoy_id, bool still_water);

// Query setup parameters from buoy exactly matching webpage GET formatting
void query_buoy_setup(const String &buoy_id);

// Save tactile setup values dynamically over UDP & LoRa channels
void send_buoy_setup(int buoy_idx);

// Send dynamic DIRDIST (Manual Navigation target direction and speed)
void send_buoy_dirdist(int buoy_idx);

#endif // BUOY_DATA_H
