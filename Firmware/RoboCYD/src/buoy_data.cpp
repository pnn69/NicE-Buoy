#include <Arduino.h>
#include <vector>
#include "buoy_data.h"
#include "cyd_wifi.h"
#include "cyd_lora.h"

BuoyData buoys[3];
int selected_buoy_idx = -1; // -1 for main menu, 0-2 for details

// Initialize both communication channels to enabled by default
bool udp_enabled = true;
bool lora_enabled = true;

// Setup Screen Mode on display
bool in_setup_mode = false;
bool setup_data_loaded = false; // Initialize to false
bool in_mannav_mode = false;    // Initialize to false

unsigned long last_udp_blink_ms = 0;
unsigned long last_udp_sel_blink_ms = 0;
unsigned long last_lora_blink_ms = 0;
unsigned long last_global_lora_blink_ms = 0;

unsigned long last_udp_tx_ms = 0;
unsigned long last_lora_tx_ms = 0;

uint8_t calculate_crc(const String &content) {
    uint8_t crc = 0;
    for (int i = 0; i < content.length(); i++) {
        crc ^= content[i];
    }
    return crc;
}

void parse_buoy_packet(const String &packetStr, const String &source, int rssi) {
    // If the corresponding communication channel is disabled, silently discard the message
    bool is_udp = source.startsWith("UDP");
    if (is_udp && !udp_enabled) return;
    if (!is_udp && !lora_enabled) return;

    // Robust parsing: Find the dollar sign wherever it starts (handles "D$", "UDP$", etc. dynamically!)
    int dollarIdx = packetStr.indexOf("$");
    int starIdx = packetStr.indexOf("*");
    if (dollarIdx == -1 || starIdx == -1 || starIdx <= dollarIdx) return;
    
    String cleanPacket = packetStr.substring(dollarIdx);
    starIdx = cleanPacket.indexOf("*"); // Recalculate relative to the clean start
    
    String content = cleanPacket.substring(1, starIdx);
    String crcHex = cleanPacket.substring(starIdx + 1);
    crcHex.trim();
    
    uint8_t calculated_crc = calculate_crc(content);
    uint8_t received_crc = strtol(crcHex.c_str(), NULL, 16);
    if (calculated_crc != received_crc) {
        Serial.printf("CRC error: calculated %02X, received %02X\n", calculated_crc, received_crc);
        return;
    }

    // Check communication source and prepare blink updates after verifying the sender ID
    
    // Split content by comma
    std::vector<String> fields;
    int prev_idx = 0;
    int next_idx = content.indexOf(",", prev_idx);
    while (next_idx != -1) {
        fields.push_back(content.substring(prev_idx, next_idx));
        prev_idx = next_idx + 1;
        next_idx = content.indexOf(",", prev_idx); // Correctly include prev_idx to prevent infinite loops!
    }
    fields.push_back(content.substring(prev_idx));
    
    if (fields.size() < 5) return;
    
    // Extract command code first to verify if this is an actual telemetry or setup packet
    int cmd = atoi(fields[3].c_str());
    
    // TELEMETRY: Ignore any auxiliary command echoes, except standard status updates, SETUPDATA (83)
    // and the GPS Fourier calibration progress report (90).
    if (cmd != 51 && cmd != 19 && cmd != 83 && cmd != 90) return;
    
    String sender_id = fields[1];
    sender_id.trim();
    
    // Ignore commands sent from ourselves (Display Sender ID is now 98, ignore 99 PC as well to prevent loops!)
    if (sender_id == "98" || sender_id == "99") return;
    
    // We have a verified valid incoming message from a buoy! Update blink timers dynamically:
    if (source.startsWith("UDP")) {
        last_udp_blink_ms = millis(); // Global blink on any incoming valid packet

        // Update selected buoy udp blink ONLY if a buoy is selected AND it matches the sender of the packet!
        if (selected_buoy_idx != -1 && buoys[selected_buoy_idx].id == sender_id) {
            last_udp_sel_blink_ms = millis();
        }
    } else if (source == "LoRa") {
        last_global_lora_blink_ms = millis(); // Always update global lora blink on any incoming valid packet!
        
        // Update selected buoy lora blink ONLY if a buoy is selected AND it matches the sender of the packet!
        if (selected_buoy_idx != -1 && buoys[selected_buoy_idx].id == sender_id) {
            last_lora_blink_ms = millis();
        }
    }
    
    // Search for an existing registered buoy
    int buoy_idx = -1;
    for (int i = 0; i < 3; i++) {
        if (buoys[i].id == sender_id) {
            buoy_idx = i;
            break;
        }
    }
    
    // FALLBACK FOR SETUPDATA OVER LORA:
    // If we are in Setup mode, and the command is SETUPDATA (83),
    // map the packet directly to our selected buoy index even if the sender ID was modified by a bridging module (RoboTop)!
    if (buoy_idx == -1 && cmd == 83 && in_setup_mode && selected_buoy_idx != -1) {
        buoy_idx = selected_buoy_idx;
        Serial.printf("Setup packet mapped to active buoy slot %d (fallback for bridge ID %s)\n", buoy_idx, sender_id.c_str());
    }
    
    // If not found, assign to the first empty slot
    if (buoy_idx == -1) {
        for (int i = 0; i < 3; i++) {
            if (buoys[i].id == "") {
                buoys[i].id = sender_id;
                buoy_idx = i;
                break;
            }
        }
    }
    
    if (buoy_idx == -1) return; // No slot available
    
    // Update timestamps and online presence
    buoys[buoy_idx].present = true;
    buoys[buoy_idx].last_seen_ms = millis();
    if (source.startsWith("UDP:")) {
        buoys[buoy_idx].ip_addr = source.substring(4);
    } else if (source == "LoRa") {
        buoys[buoy_idx].lora_rssi = rssi;
    }
    
    int status_code = atoi(fields[4].c_str());
    
    // Status text mapping
    if (status_code == 7) {
        buoys[buoy_idx].status = "IDLE";
        buoys[buoy_idx].bb_power = 0; // Explicitly reset thrusters to 0% when IDLE!
        buoys[buoy_idx].sb_power = 0;
    }
    else if (status_code == 12) buoys[buoy_idx].status = "LOCKING";
    else if (status_code == 13) buoys[buoy_idx].status = "LOCKED";
    else if (status_code == 15) buoys[buoy_idx].status = "DOCKING";
    else if (status_code == 16) {
        buoys[buoy_idx].status = "DOCKED";
        buoys[buoy_idx].bb_power = 0; // Reset thrusters when DOCKED
        buoys[buoy_idx].sb_power = 0;
    }
    else if (status_code == 25) buoys[buoy_idx].status = "REMOTE";
    // GPS_FOURIER_CALIBRATE: the Top is sailing its eight compass calibration legs.
    else if (status_code == 89) buoys[buoy_idx].status = "GPS CALIB";
    else buoys[buoy_idx].status = "MODE " + String(status_code);
    
    // Parse GPS_FOURIER_STATUS (CMD = 90).
    // fields[0..4] are IDr, IDs, ack, cmd, status - the payload starts at 5.
    if (cmd == 90 && fields.size() >= 11) {
        buoys[buoy_idx].cal_seen_ms = millis();
        buoys[buoy_idx].cal_step    = atoi(fields[5].c_str());
        buoys[buoy_idx].cal_leg     = atoi(fields[6].c_str());
        buoys[buoy_idx].cal_cmd_dir = atof(fields[7].c_str());
        buoys[buoy_idx].cal_dist    = atof(fields[9].c_str());
        buoys[buoy_idx].cal_err     = atof(fields[10].c_str());
        // fields[11] only exists on a Top new enough to send it.
        if (fields.size() >= 12) buoys[buoy_idx].cal_last_err = atof(fields[11].c_str());
        return; // carries no position or battery data - nothing else here applies
    }

    // Parse TOPDATA (CMD = 51)
    if (cmd == 51 && fields.size() >= 21) {
        buoys[buoy_idx].mag_dir = atof(fields[5].c_str());
        buoys[buoy_idx].gps_dir = atof(fields[6].c_str());
        buoys[buoy_idx].tg_dir = atof(fields[7].c_str());
        buoys[buoy_idx].tg_dist = atof(fields[8].c_str());
        buoys[buoy_idx].wind_dir = atof(fields[9].c_str());
        buoys[buoy_idx].wind_std = atof(fields[10].c_str());
        buoys[buoy_idx].bb_power = atof(fields[11].c_str());
        buoys[buoy_idx].sb_power = atof(fields[12].c_str());
        buoys[buoy_idx].pid_i = atof(fields[13].c_str());
        buoys[buoy_idx].pid_r = atof(fields[14].c_str());
        buoys[buoy_idx].battery_v = atof(fields[15].c_str());
        buoys[buoy_idx].battery_pct = atof(fields[16].c_str());
        buoys[buoy_idx].lat = fields[17];
        buoys[buoy_idx].lon = fields[18];
        buoys[buoy_idx].gps_fix = fields[19] == "1" ? "3D" : fields[19] == "2" ? "2D" : "NoFix";
        buoys[buoy_idx].gps_sat = atoi(fields[20].c_str());
        if (fields.size() > 21) {
            buoys[buoy_idx].current = atof(fields[21].c_str());
        }
    }
    // Parse BUOYPOS (CMD = 19)
    else if (cmd == 19 && fields.size() >= 14) {
        buoys[buoy_idx].lat = fields[5];
        buoys[buoy_idx].lon = fields[6];
        buoys[buoy_idx].mag_dir = atof(fields[7].c_str());
        buoys[buoy_idx].wind_dir = atof(fields[8].c_str());
        buoys[buoy_idx].wind_std = atof(fields[9].c_str());
        
        // fields[10] is topAccuP
        buoys[buoy_idx].battery_pct = atof(fields[11].c_str()); // subAccuP
        
        // Estimate voltage if battery_v is currently zero
        if (buoys[buoy_idx].battery_v == 0) {
            buoys[buoy_idx].battery_v = 17.0 + (8.2 * buoys[buoy_idx].battery_pct / 100.0);
        }
        
        buoys[buoy_idx].gps_fix = fields[12] == "1" ? "3D" : fields[12] == "2" ? "2D" : "NoFix";
        buoys[buoy_idx].gps_sat = atoi(fields[13].c_str());
    }
    // Parse SETUPDATA (CMD = 83) dynamically with robust length checking!
    else if (cmd == 83 && fields.size() >= 6) {
        Serial.printf("Parsing SETUPDATA for Buoy: %s (fields: %d)\n", sender_id.c_str(), fields.size());
        if (fields.size() > 5) buoys[buoy_idx].kpr = atof(fields[5].c_str());
        if (fields.size() > 6) buoys[buoy_idx].kir = atof(fields[6].c_str());
        if (fields.size() > 7) buoys[buoy_idx].kdr = atof(fields[7].c_str());
        if (fields.size() > 8) buoys[buoy_idx].kps = atof(fields[8].c_str());
        if (fields.size() > 9) buoys[buoy_idx].kis = atof(fields[9].c_str());
        if (fields.size() > 10) buoys[buoy_idx].kds = atof(fields[10].c_str());
        if (fields.size() > 11) buoys[buoy_idx].max_speed = atof(fields[11].c_str());
        if (fields.size() > 12) buoys[buoy_idx].min_speed = atof(fields[12].c_str());
        if (fields.size() > 13) buoys[buoy_idx].pivot_speed = atof(fields[13].c_str());
        if (fields.size() > 14) buoys[buoy_idx].compass_offset = atof(fields[14].c_str());
        if (fields.size() > 15) buoys[buoy_idx].hold_radius = atof(fields[15].c_str());
        if (fields.size() > 16) buoys[buoy_idx].rev_bb = (fields[16] == "1");
        if (fields.size() > 17) buoys[buoy_idx].rev_sb = (fields[17] == "1");
        if (fields.size() > 18) buoys[buoy_idx].swap_bb_sb = (fields[18] == "1");
        if (fields.size() > 19) buoys[buoy_idx].compass_trim_enabled = (fields[19] == "1");
        if (fields.size() > 20) buoys[buoy_idx].dock_app_dist = atoi(fields[20].c_str());
        if (fields.size() > 21) buoys[buoy_idx].dock_app_dir = atoi(fields[21].c_str());
        if (fields.size() > 22) buoys[buoy_idx].dock_to_wp = (fields[22] == "1");
        // Tri-state so that "0"/empty means the buoy never reported it - a plain 0 could not
        // be told apart from the zero-compression RoboTop applies to the payload.
        if (fields.size() > 23 && fields[23] != "" && fields[23] != "0")
            buoys[buoy_idx].harmonic_enabled = (fields[23] == "2");

        // Mark setup parameters as successfully loaded if this is the buoy currently active!
        if (selected_buoy_idx == buoy_idx) {
            setup_data_loaded = true;
            Serial.println("On-screen Setup Data loaded successfully from Buoy!");
        }
    }
}

void send_buoy_command(const String &buoy_id, int cmd_code, int ack) {
    // Standard command formatting: $Target,Sender,ACK,CMD,Status,Data1,Data2...*CRC
    // Use unique Display Sender ID "98" to ensure proper bi-directional LoRa routing!
    String cmdStr = buoy_id + ",98," + String(ack) + "," + String(cmd_code) + "," + String(cmd_code) + ",,,,,,";
    
    uint8_t crc = calculate_crc(cmdStr);
    char crc_buf[8];
    sprintf(crc_buf, "*%02X", crc);
    
    String finalPacket = "$" + cmdStr + String(crc_buf);
    
    Serial.printf("Broadcasting Command: %s\n", finalPacket.c_str());
    
    // Send over both LoRa and UDP!
    send_lora_packet(finalPacket);
    udp_broadcast(finalPacket);
}

void send_gps_fourier_calibrate(const String &buoy_id, bool still_water) {
    // Same envelope as send_buoy_command(), but with the still-water flag in the first payload
    // field. RoboTop reads it as fields[5]; an empty payload there decodes as 0, which is the
    // current-tolerant pair-averaged mode.
    String cmdStr = buoy_id + ",98,3,89,89," + String(still_water ? 1 : 0);

    uint8_t crc = calculate_crc(cmdStr);
    char crc_buf[8];
    sprintf(crc_buf, "*%02X", crc);

    String finalPacket = "$" + cmdStr + String(crc_buf);

    Serial.printf("Broadcasting GPS Fourier calibrate (%s): %s\n",
                  still_water ? "still water" : "pair averaged", finalPacket.c_str());

    send_lora_packet(finalPacket);
    udp_broadcast(finalPacket);
}

void query_buoy_setup(const String &buoy_id) {
    // Build query with GET (1) to guarantee that bridging modules (RoboTop) never misinterpret it as a write/SET command.
    // Use unique Display Sender ID "98" to bypass RoboTop's 0x99 PC-serial filter and force LoRa transmission of replies!
    String cmdStr = buoy_id + ",98,1,83,,,,,,,"; // ack = 1 (GET), sender = 98
    
    uint8_t crc = calculate_crc(cmdStr);
    char crc_buf[8];
    sprintf(crc_buf, "*%02X", crc);
    
    String finalPacket = "$" + cmdStr + String(crc_buf);
    
    Serial.printf("Broadcasting SETUP GET: %s\n", finalPacket.c_str());
    send_lora_packet(finalPacket);
    udp_broadcast(finalPacket);
}

void send_buoy_setup(int buoy_idx) {
    BuoyData &b = buoys[buoy_idx];
    
    // Construct standard SET command payload using SET (2) and unique Display Sender ID "98"
    char cmdPayload[256];
    sprintf(cmdPayload, "%s,98,2,83,7,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.0f,%0.0f,%0.2f,%0.0f,%0.1f,%d,%d,%d,%d,%d,%d,%d,%d",
            b.id.c_str(),
            b.kpr, b.kir, b.kdr, b.kps, b.kis, b.kds,
            b.max_speed, b.min_speed, b.pivot_speed, b.compass_offset, b.hold_radius,
            b.rev_bb ? 1 : 0, b.rev_sb ? 1 : 0, b.swap_bb_sb ? 1 : 0, b.compass_trim_enabled ? 1 : 0,
            b.dock_app_dist, b.dock_app_dir, b.dock_to_wp ? 1 : 0,
            b.harmonic_enabled ? 2 : 1);
            
    uint8_t crc = calculate_crc(cmdPayload);
    char finalPacket[320];
    sprintf(finalPacket, "$%s*%02X", cmdPayload, crc);
    
    Serial.printf("Broadcasting SETUP SAVE command: %s\n", finalPacket);
    
    // Send over both LoRa and UDP channels
    send_lora_packet(finalPacket);
    udp_broadcast(finalPacket);
}

void send_buoy_dirdist(int buoy_idx) {
    BuoyData &b = buoys[buoy_idx];
    
    // Construct standard SET command payload using unique Display Sender ID "98"
    // CMD = 25 (REMOTE), ACK = 6 (INF), status = 25 (REMOTE)
    // Data1 (tgDir) = tg_dir
    // Data2 (tgSpeed) = tg_speed
    char cmdPayload[256];
    sprintf(cmdPayload, "%s,98,6,25,25,%0.1f,%0.1f,,,,",
            b.id.c_str(), b.tg_dir, b.tg_speed);
            
    uint8_t crc = calculate_crc(cmdPayload);
    char finalPacket[320];
    sprintf(finalPacket, "$%s*%02X", cmdPayload, crc);
    
    Serial.printf("Broadcasting REMOTE command: %s\n", finalPacket);
    
    // Send over both LoRa and UDP channels
    send_lora_packet(finalPacket);
    udp_broadcast(finalPacket);
}
