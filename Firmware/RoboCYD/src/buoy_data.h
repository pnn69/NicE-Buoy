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
    // The same thing as status, unmapped: the numeric MsgType straight off the wire. Kept next to
    // the spelled out version because a frame we SEND has to carry a number, not a label - see
    // send_buoy_setlockpos(). 7 is IDLE, the state a buoy that has said nothing yet is assumed in.
    int status_code = 7;
    float mag_dir = 0;
    // The buoy's heading with the iron correction applied and NOTHING else: before the eight point
    // compass table, before the mounting offset, before the adaptive trim - the "Imag" field on the
    // wire. This is the value the table is indexed by, so it is what a calibration must capture:
    // reading it needs nothing switched off on the buoy, and it cannot be spoiled by the state of
    // the correction, the offset or the trim.
    //
    // It is also the number the operator turns the hull against. MAN CAL shows it large and refuses
    // the N mark until it reads zero, which is what anchors the run.
    //
    // mag_dir_iron_ms stays 0 until the buoy actually sends it. A buoy running firmware that
    // predates the field sends a shorter frame, and capturing 0 as a heading would quietly write a
    // nonsense table - so MAN CAL checks this before it lets anything be captured.
    float mag_dir_iron = 0;
    unsigned long mag_dir_iron_ms = 0;
    // Attitude of the hull, from TOPDATA. Wanted by MAN CAL, which draws a level in the middle of
    // the rose: heading comes from the horizontal field component, so a listing hull mixes in the
    // vertical one and that error is stored as if it were deviation. pitch_ms stays 0 until the
    // buoy actually sends it, so "not reported" can be told from "level".
    float pitch = 0;
    float roll = 0;
    unsigned long pitch_ms = 0;
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

    // The waypoint the buoy was told to hold, from LOCKPOS/DOCKPOS (cmd 21/23) - the only
    // frames that carry it. TOPDATA and BUOYPOS report where the buoy IS, never where it is
    // supposed to be. 0/0 means never reported; tg_pos_seen_ms ages it out, because the Top
    // stops beaconing the moment it leaves LOCKED/DOCKED and a stale mark is worse than none.
    double tg_lat = 0;
    double tg_lon = 0;
    unsigned long tg_pos_seen_ms = 0;
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
    // ---- guided eight point calibration, mirrored from the buoy ----------------------------
    // Every value here arrived in a CAL8_SESSION frame the buoy sent. The CYD never writes them
    // itself and never derives them, because the session belongs to the buoy: see the block
    // comment in RoboSub/src/compass.cpp. The MAN CAL screen renders these and presses buttons,
    // and that is all it does - which is the fix for a screen that used to keep its own eight
    // offsets, its own step counter and its own copy of the table maths.
    // cal8_ms stays 0 until the buoy has answered once, so "no session running" can be told apart
    // from "we have not heard yet" - the difference between a blank screen and a wrong one.
    bool cal8_active = false;
    int cal8_next = 0;
    float cal8[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    // Bit i set once direction i has been captured. This, not cal8_next, is what says whether the
    // run is complete: a direction may be re-captured, and doing so leaves the cursor where it was.
    int cal8_mask = 0;
    // Serial of the last press the buoy applied. A press this screen sends is numbered one past it,
    // so the Top's resends and the buoy's de-duplication agree on which press is which.
    int cal8_seq = 0;
    unsigned long cal8_ms = 0;

    // The "apply the compass table" switch used to live here, with a second copy recording what
    // the buoy had reported so MAN CAL could tell intent from fact. Both are gone: the table is
    // applied unconditionally, so there is nothing to intend and nothing to disagree about. The
    // SETUPDATA field is still sent, always "on", because the frame is positional.

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

// True while this buoy's reported waypoint is fresh enough to plot. RoboTop re-broadcasts
// LOCKPOS every 5 s for as long as it holds station, so four missed beacons means it stopped.
bool buoy_has_waypoint(const BuoyData &b);

void parse_buoy_packet(const String &packetStr, const String &source, int rssi = -999);
// ack defaults to GETACK (3), which puts the packet in RoboTop's LoRa retransmit table
// (loratop.cpp: retry = 5, resent until the target answers). Pass INF (6) for anything the
// target cannot acknowledge - a REBOOT sent as GETACK is retried five times and reboots the
// buoy five times.
void send_buoy_command(const String &buoy_id, int cmd_code, int ack = 3);

// Hands a buoy a new waypoint to sail to and lock onto (SETLOCKPOS, cmd 20) - the same command the
// Top uses to push computed start line ends to the other buoy. Needs its own sender because the
// coordinates live in the payload, which send_buoy_command() leaves empty. status_code is echoed
// back in the status field the way the dashboard does it; the receiving Top overrides it with
// LOCKED either way.
void send_buoy_setlockpos(const String &buoy_id, int status_code, double lat, double lon);

// Query setup parameters from buoy exactly matching webpage GET formatting
void query_buoy_setup(const String &buoy_id);

// Save tactile setup values dynamically over UDP & LoRa channels
void send_buoy_setup(int buoy_idx);

// One press of the guided eight point calibration. action is a cal8_action_t: 0 begin, 1 set,
// 2 save, 3 cancel. ack SET (2) performs it; ack GET (1) only asks for the state, which is how the
// screen finds out about a run someone started from a web page. Either way the buoy answers with
// the state and it lands in BuoyData::cal8_*.
//
// leg names the direction a SET is for, and seq numbers the press. Both are needed and neither is
// enough alone: any direction can be captured at any time, so the leg says which slot, and the
// press is resent by the Top until the buoy confirms it, so the serial says whether an arriving
// copy is new. Number a SET with BuoyData::cal8_seq + 1. Both ignored for the other three actions.
// See CAL8_SESSION in RoboCompute.h.
void send_buoy_cal8(const String &buoy_id, int action, int leg = 0, int ack = 2, int seq = 0);

// Send dynamic DIRDIST (Manual Navigation target direction and speed)
void send_buoy_dirdist(int buoy_idx);


// XOR checksum for the $...*CRC envelope every frame on this network has to carry - rfDeCode()
// drops anything without it. Declared here because senders live outside buoy_data.cpp too.

// ---------------------------------------------------------------------------------------------
//  Delivery confirmation for the commands that matter
// ---------------------------------------------------------------------------------------------
// The CYD used to be write-only. Every command went out once over LoRa and once over UDP and that
// was the end of it: no acknowledgement was read, nothing was retried, and no screen ever found
// out whether a buoy had heard anything. track_settings_compute() painted "COMPUTING..." BEFORE
// touching the radio, so the banner appeared whether the press landed or vanished - which is
// exactly how an ALIGN STARTLINE could look like it had worked while the fleet never moved.
//
// RoboTop now answers a unicast GETACK/SET with an ACK frame (see ackOverLora() there), so there
// is finally something to wait for. This tracks ONE outstanding command - the operator presses one
// button at a time - resends it until it is acknowledged, and latches the outcome for the screen.
//
// Deliberately NOT applied to every command. Only the ones where a silent loss is expensive and a
// repeat is harmless are registered: see send_buoy_command(). Remote steering must never be
// retried, because a stale repeat would fight the operator's next input.
struct PendingCmd {
    bool active = false;
    String target_id = "";
    int cmd = 0;
    String frame = "";              // the exact envelope to put back on the air
    int attempts_left = 0;
    unsigned long next_due_ms = 0;
};

extern PendingCmd pending_cmd;

// Latched outcome of the last tracked command. Set by the ACK path / the retry timeout, and
// cleared by whichever screen reports it, so a banner cannot be missed by a slow repaint.
extern bool pending_cmd_acked;
extern bool pending_cmd_failed;

// Registers a just-sent frame as awaiting an ACK. A second call replaces the first - pressing
// START and then TRACK means the operator wants TRACK, not both.
void await_ack(const String &target_id, int cmd, const String &frame, int attempts = 3);

// Called from loop(): resends the outstanding command when its retry is due, and gives up (setting
// pending_cmd_failed) once the attempts are spent.
void service_pending_cmd();

// LoRa link quality. link_note() records one received frame's signal strength against its sender;
// service_lora_link_report() puts a minute's worth on the air, LoRa only. See buoy_data.cpp - the
// report doubles as the beacon that lets the Tops measure THIS end.
void link_note(const String &id, int rssi);
void service_lora_link_report();


uint8_t calculate_crc(const String &content);

#endif // BUOY_DATA_H
