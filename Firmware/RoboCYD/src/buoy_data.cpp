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

// How long a waypoint stays on the map after the last LOCKPOS/DOCKPOS that carried it.
// RoboTop beacons one every 5 s while it holds station, so this is four missed beacons -
// long enough to ride out LoRa losses, short enough that a buoy which has gone quiet or been
// switched off stops claiming a target.
#define WAYPOINT_STALE_MS 20000UL

bool buoy_has_waypoint(const BuoyData &b) {
    if (b.tg_pos_seen_ms == 0) return false;
    if (b.tg_lat == 0.0 && b.tg_lon == 0.0) return false;
    return (millis() - b.tg_pos_seen_ms) < WAYPOINT_STALE_MS;
}

uint8_t calculate_crc(const String &content) {
    uint8_t crc = 0;
    for (int i = 0; i < content.length(); i++) {
        crc ^= content[i];
    }
    return crc;
}

PendingCmd pending_cmd;
bool pending_cmd_acked = false;
bool pending_cmd_failed = false;

// How long to wait for an ACK before resending. RoboTop answers from its loop task, typically well
// inside 300 ms, so a second is generous - and it deliberately stays clear of the Top's OWN 1500 ms
// retry cadence, so the two ends are not transmitting on top of each other.
#define ACK_RETRY_MS 1000UL

void await_ack(const String &target_id, int cmd, const String &frame, int attempts) {
    pending_cmd.active = true;
    pending_cmd.target_id = target_id;
    pending_cmd.cmd = cmd;
    pending_cmd.frame = frame;
    pending_cmd.attempts_left = attempts - 1;   // the first attempt has already gone out
    pending_cmd.next_due_ms = millis() + ACK_RETRY_MS;
    pending_cmd_acked = false;
    pending_cmd_failed = false;
}

// An ACK has arrived. Matched on sender AND command: the Top puts its own MAC in the sender field
// of the reply - which is the id we addressed the command to - and names the command it is
// answering for. Anything else is somebody else's traffic.
static void note_ack(const String &sender_id, int cmd) {
    if (!pending_cmd.active) return;
    if (pending_cmd.cmd != cmd) return;
    if (pending_cmd.target_id != sender_id) return;

    Serial.printf("ACK from %s for cmd %d - delivery confirmed\n", sender_id.c_str(), cmd);
    pending_cmd.active = false;
    pending_cmd.frame = "";
    pending_cmd_acked = true;
}

void service_pending_cmd() {
    if (!pending_cmd.active) return;
    if ((long)(millis() - pending_cmd.next_due_ms) < 0) return;

    if (pending_cmd.attempts_left <= 0) {
        Serial.printf("No ACK from %s for cmd %d after all attempts - giving up\n",
                      pending_cmd.target_id.c_str(), pending_cmd.cmd);
        pending_cmd.active = false;
        pending_cmd.frame = "";
        pending_cmd_failed = true;
        return;
    }

    pending_cmd.attempts_left--;
    pending_cmd.next_due_ms = millis() + ACK_RETRY_MS;
    Serial.printf("No ACK yet for cmd %d, resending (%d attempt(s) left)\n",
                  pending_cmd.cmd, pending_cmd.attempts_left);
    send_lora_packet(pending_cmd.frame);
    udp_broadcast(pending_cmd.frame);
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

    // ACK (ack field == 4) is handled here and nowhere else, ahead of the telemetry filter below.
    // An ACK names the command it answers FOR, so its cmd reads 62, 20, ... and that filter would
    // drop every one of them. RoboCode() emits only "cmd,status" for an ACK, so there is no
    // telemetry in it to fall through to anyway.
    if (atoi(fields[2].c_str()) == 4) {
        String ack_sender = fields[1];
        ack_sender.trim();
        note_ack(ack_sender, cmd);
        return;
    }
    
    // TELEMETRY: Ignore any auxiliary command echoes, except standard status updates, SETUPDATA (83),
    // CAL8_SESSION (91), ATTITUDE (92), the interpolation table (88) and the waypoint beacons
    // LOCKPOS (21) / DOCKPOS (23).
    //
    // 91 was missing from this list from the day the guided calibration was added, so every reply
    // the buoy sent about a run was dropped here - 200 lines above the parser that wanted it. The
    // touchscreen therefore never learned that a session existed and sat on "waiting for the buoy"
    // for ever, with no way to tell that apart from a buoy that was not answering at all.
    //
    // SETLOCKPOS (20) is deliberately NOT accepted here even though it carries the same two
    // fields: it is sent BY the Top that computed a start line TO another buoy, so its sender ID
    // is the wrong buoy and the waypoint would be filed against the computer instead of the
    // recipient re-broadcasts it as LOCKPOS under its own ID anyway
    // (RoboTop/src/main.cpp, case SETLOCKPOS), which is the copy we want.
    if (cmd != 51 && cmd != 19 && cmd != 83 && cmd != 21 && cmd != 23 && cmd != 88 &&
        cmd != 91 && cmd != 92) return;

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
    // Only BUOYPOS and TOPDATA fix a buoy's IP address. Those two are broadcast by the buoy
    // itself and are never relayed onward by another Top, so the UDP source address really is
    // this buoy. Everything else can reach us second hand - a Top bridges frames it receives
    // but that are not for it, keeping the ORIGINAL sender id - and taking the address off one
    // of those files the relaying Top's IP against the originating buoy. That is how two rows
    // in the menu ended up showing the same IP.
    bool ip_is_authoritative = (cmd == 51 || cmd == 19);
    if (source.startsWith("UDP:") && ip_is_authoritative) {
        buoys[buoy_idx].ip_addr = source.substring(4);
    } else if (source == "LoRa") {
        buoys[buoy_idx].lora_rssi = rssi;
    }
    
    int status_code = atoi(fields[4].c_str());
    buoys[buoy_idx].status_code = status_code;
    
    // Status text mapping
    if (status_code == 7) {
        buoys[buoy_idx].status = "IDLE";
        buoys[buoy_idx].bb_power = 0; // Explicitly reset thrusters to 0% when IDLE!
        buoys[buoy_idx].sb_power = 0;
        // An idle buoy is not steering at anything, so drop its waypoint at once rather than
        // letting it fade out over the staleness window - the mark would otherwise sit on the
        // map for another 20 s claiming a target the buoy has already abandoned.
        buoys[buoy_idx].tg_pos_seen_ms = 0;
    }
    else if (status_code == 12) buoys[buoy_idx].status = "LOCKING";
    else if (status_code == 13) buoys[buoy_idx].status = "LOCKED";
    else if (status_code == 15) buoys[buoy_idx].status = "DOCKING";
    else if (status_code == 16) {
        buoys[buoy_idx].status = "DOCKED";
        buoys[buoy_idx].bb_power = 0; // Reset thrusters when DOCKED
        buoys[buoy_idx].sb_power = 0;
    }
    else if (status_code == 25) {
        buoys[buoy_idx].status = "REMOTE";
        buoys[buoy_idx].tg_pos_seen_ms = 0; // Hand steering - no waypoint to hold. Same reason as IDLE.
    }
    else buoys[buoy_idx].status = "MODE " + String(status_code);
    
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
        // Imag - see BuoyData::mag_dir_iron. numbers[19] in RoboCompute's TOPDATA, which is
        // fields[22] here. Appended to the frame, so an older buoy simply does not send it and
        // mag_dir_iron_ms stays 0.
        if (fields.size() > 22 && fields[22].length() > 0) {
            buoys[buoy_idx].mag_dir_iron = atof(fields[22].c_str());
            buoys[buoy_idx].mag_dir_iron_ms = millis();
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
    // Parse LOCKPOS (CMD = 21) / DOCKPOS (CMD = 23) - the waypoint the buoy is steering at.
    // Same payload for both (RoboCompute RoboCode/RoboDecode): tgLat, tgLng, wDir, wStd.
    // RoboTop broadcasts this on the lock/dock transition and then every 5 s for as long as it
    // holds station, so it doubles as a keep-alive for the mark on the map.
    else if ((cmd == 21 || cmd == 23) && fields.size() >= 7) {
        double t_lat = atof(fields[5].c_str());
        double t_lon = atof(fields[6].c_str());
        // 0/0 is what an unset target decodes to, and plotting it would drop a mark in the Gulf
        // of Guinea and blow the plot scale out to thousands of kilometres.
        if (t_lat != 0.0 || t_lon != 0.0) {
            buoys[buoy_idx].tg_lat = t_lat;
            buoys[buoy_idx].tg_lon = t_lon;
            buoys[buoy_idx].tg_pos_seen_ms = millis();
        }
        if (fields.size() >= 9) {
            buoys[buoy_idx].wind_dir = atof(fields[7].c_str());
            buoys[buoy_idx].wind_std = atof(fields[8].c_str());
        }
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
        // Field 23 used to carry the "apply the compass table" switch. The switch is gone and the
        // Sub no longer sends the field, so there is nothing to read here - and nothing to guard
        // the block below with either. Removing the assignments alone left the if() standing with
        // the next statement as its body, which quietly made setup_data_loaded conditional on a
        // field that is never sent: the Setup pages then never loaded at all.

        // Mark setup parameters as successfully loaded if this is the buoy currently active!
        if (selected_buoy_idx == buoy_idx) {
            setup_data_loaded = true;
            Serial.println("On-screen Setup Data loaded successfully from Buoy!");
        }
    }
    // Parse ATTITUDE (CMD = 92) - pitch and roll, for MAN CAL's level. UDP only, so it simply
    // never arrives over a LoRa-only link and pitch_ms stays 0 - which the screen shows as an
    // unknown level rather than a level one.
    else if (cmd == 92 && fields.size() >= 7) {
        buoys[buoy_idx].pitch = atof(fields[5].c_str());
        buoys[buoy_idx].roll = atof(fields[6].c_str());
        buoys[buoy_idx].pitch_ms = millis();
    }
    // Parse CAL8_SESSION (CMD = 91) - the state of a guided eight point calibration.
    //
    // Taken from EVERY frame, unlike the table above: this is not a value the operator is editing
    // that a stale duplicate could wipe out, it is the buoy's own step counter, and the buoy is the
    // only thing that ever changes it. The Top puts each reply on both UDP and LoRa, so the worst a
    // late duplicate can do is repaint the same step.
    else if (cmd == 91 && fields.size() >= 8) {
        buoys[buoy_idx].cal8_active = (atoi(fields[6].c_str()) != 0);
        buoys[buoy_idx].cal8_next = atoi(fields[7].c_str());
        // The eight captures are appended, so a buoy too old to send them leaves them alone rather
        // than reading as a run with everything at zero.
        if (fields.size() >= 16) {
            for (int i = 0; i < 8; i++)
                buoys[buoy_idx].cal8[i] = atof(fields[8 + i].c_str());
        }
        // The captured mask and the press serial are appended after the eight values, so a buoy
        // too old to send them leaves this end's copy alone rather than reading as an empty run.
        if (fields.size() >= 17) buoys[buoy_idx].cal8_mask = atoi(fields[16].c_str());
        if (fields.size() >= 18) buoys[buoy_idx].cal8_seq  = atoi(fields[17].c_str());
        buoys[buoy_idx].cal8_ms = millis();
        extern bool mancal_is_dirty;
        mancal_is_dirty = true;
        Serial.printf("CAL8 from %s: %s, asking for %d, mask %02X, seq %d\n", sender_id.c_str(),
                      buoys[buoy_idx].cal8_active ? "running" : "idle",
                      buoys[buoy_idx].cal8_next, buoys[buoy_idx].cal8_mask,
                      buoys[buoy_idx].cal8_seq);
    }
    // Parse STORE_INTERPOLATION_TABLE (CMD = 88) to pre-populate fourier offsets dynamically!
    else if (cmd == 88 && fields.size() >= 13) {
        Serial.printf("Parsing STORE_INTERPOLATION_TABLE (88) for Buoy: %s\n", sender_id.c_str());
        extern bool in_man_fourier_cal_mode;
        extern bool mancal_offsets_loaded;
        // Only the calibration screen, and only while it is still waiting for its first answer.
        // These frames arrive in duplicate - the Top puts every reply on BOTH UDP and LoRa, and the
        // LoRa copy can lag by seconds and carry an older table (an identity one, if the correction
        // was off when it was generated). Taking every frame that comes past meant the correct
        // values landed and were then overwritten with zeros a moment later. It also means the
        // Sub's echo of a STORE can no longer wipe out what the operator has dialled in.
        // Field 13 is the Sub saying whether this table is USABLE. When it is not, the frame
        // carries the identity table rather than the stored one, and latching it would show a
        // calibrated buoy as having no corrections at all. Tri-stated on the field count so a
        // node that predates the flag is not read as "off".
        // Any table frame from the buoy we are calibrating is proof it answered. SAVE waits on
        // this: without it, "CALIBRATION COMPLETE" was printed whether or not anything landed.
        if (selected_buoy_idx == buoy_idx) {
            extern volatile unsigned long mancal_table_echo_ms;
            mancal_table_echo_ms = millis();
        }
        bool table_usable = (fields.size() < 14) || (atoi(fields[13].c_str()) != 0);
        if (!table_usable) {
            Serial.println("Ignoring interpolation table: the Sub reports it is not usable, "
                           "so this is the identity table and not the stored one.");
        }
        else if (selected_buoy_idx == buoy_idx && in_man_fourier_cal_mode && !mancal_offsets_loaded) {
            extern float mancal_offsets[8];
            extern bool mancal_is_dirty;

            for (int i = 0; i < 8; i++) {
                float tableVal = atof(fields[5 + i].c_str());
                float offset = (i * 45) - tableVal;
                while (offset < -180.0f) offset += 360.0f;
                while (offset > 180.0f) offset -= 360.0f;
                
                mancal_offsets[i] = offset;
            }
            mancal_offsets_loaded = true;
            mancal_is_dirty = true;
            Serial.println("Loaded 8 Fourier calibration offsets from Buoy onto CYD successfully!");
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

    // Confirm delivery for the two commands whose loss is invisible to the operator.
    //
    // COMPUTESTART (62) and COMPUTETRACK (63) are carried out by the buoy that receives them, and
    // the result reaches the rest of the fleet as SETLOCKPOS BETWEEN buoys - so when the press is
    // lost, this screen sees precisely what it sees when the press arrives: nothing.
    //
    // Everything else is left alone on purpose. IDLE/LOCK/DOCK are single visible state changes an
    // operator can watch fail and simply press again, and a retry of a REMOTE or DIRDIST frame
    // would put a stale steering command on the air behind the operator's next one.
    if (cmd_code == 62 || cmd_code == 63) {
        await_ack(buoy_id, cmd_code, finalPacket);
    }
}

void send_buoy_cal8(const String &buoy_id, int action, int leg, int ack, int seq) {
    // $Target,Sender,ACK,CMD,Status,Action,Active,Next,c0..c7,Mask,Seq*CRC - RoboCompute's decoder
    // reads the action as numbers[2], counting from the CMD field, and needs more than four numbers
    // present before it reads any of them. Active, the eight captures and Mask are state the BUOY
    // reports, so what we put there is ignored on arrival; Next carries the leg this press is for
    // and Seq numbers the press.
    //
    // The eight zeroes and the mask are placeholders rather than omissions: the decoder keys on the
    // field COUNT, so a short frame would leave cal8Seq unread and every press would arrive
    // unnumbered - which reads as "no retry behind this one" and defeats the de-duplication.
    //
    // Status 7 (IDLE) rather than echoing the command, which is what send_buoy_command() does:
    // this frame is not asking the buoy to change state, only to take a calibration step.
    String cmdStr = buoy_id + ",98," + String(ack) + ",91,7," + String(action) + ",0," + String(leg)
                  + ",0,0,0,0,0,0,0,0,0," + String(seq);

    uint8_t crc = calculate_crc(cmdStr);
    char crc_buf[8];
    sprintf(crc_buf, "*%02X", crc);
    String finalPacket = "$" + cmdStr + String(crc_buf);

    Serial.printf("CAL8 %s action %d leg %d seq %d: %s\n", ack == 1 ? "GET" : "SET", action, leg,
                  seq, finalPacket.c_str());
    // UDP only, no LoRa. Calibration happens with somebody stood over the hull, so WiFi is in range
    // by definition - and putting these on the air was doing real harm, not just costing airtime.
    // Measured on the bench: one press reached the Top 35 times, because it went out on both
    // transports, the other Top relayed what it heard, and the LoRa retransmit table added more.
    // Every copy meant another frame down the half-duplex wire to the Sub, which was already
    // dropping most of what it was given - so the flood was feeding the very loss it looked like
    // noise around.
    udp_broadcast(finalPacket);
}

void send_buoy_setlockpos(const String &buoy_id, int status_code, double lat, double lon) {
    // $Target,Sender,ACK,CMD,Status,Lat,Lng*CRC - RoboCompute's RoboDecode() reads the two
    // coordinates as numbers[2] and numbers[3], counting from the CMD field.
    //
    // ACK is SET (2), not GETACK (3): GETACK puts the frame in RoboTop's LoRa retransmit table and
    // has it resent five times, and the Top already re-broadcasts the waypoint it adopted.
    //
    // 10 decimals is what the dashboard sends and what RoboCompute's formatFloat() emits for a
    // position - roughly 0.01 mm, i.e. the coordinate survives the round trip unrounded.
    String cmdStr = buoy_id + ",98,2,20," + String(status_code) + ","
                  + String(lat, 10) + "," + String(lon, 10);

    uint8_t crc = calculate_crc(cmdStr);
    char crc_buf[8];
    sprintf(crc_buf, "*%02X", crc);

    String finalPacket = "$" + cmdStr + String(crc_buf);

    Serial.printf("Broadcasting SETLOCKPOS: %s\n", finalPacket.c_str());

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
    sprintf(cmdPayload, "%s,98,2,83,7,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.0f,%0.0f,%0.2f,%0.2f,%0.1f,%d,%d,%d,%d,%d,%d,%d,%d",
            b.id.c_str(),
            b.kpr, b.kir, b.kdr, b.kps, b.kis, b.kds,
            // compassOffset goes out with 2 decimals, matching what the Sub already puts on the
            // wire (formatFloat(compassOffset, 2) in RoboCode). It used to be %0.0f, so every
            // SAVE from this screen silently rounded the buoy's offset to a whole degree - the
            // Sub's own Set as North computes a fractional one, and pressing SAVE afterwards
            // threw away up to half a degree of it.
            b.max_speed, b.min_speed, b.pivot_speed, b.compass_offset, b.hold_radius,
            b.rev_bb ? 1 : 0, b.rev_sb ? 1 : 0, b.swap_bb_sb ? 1 : 0, b.compass_trim_enabled ? 1 : 0,
            b.dock_app_dist, b.dock_app_dir, b.dock_to_wp ? 1 : 0);
            
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
