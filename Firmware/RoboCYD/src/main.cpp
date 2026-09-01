#include <Arduino.h>
#include <WiFi.h>
#include "cyd_display.h"
#include "cyd_touch.h"
#include "boot_screen.h"
#include "cyd_wifi.h"
#include "cyd_lora.h"
#include "buoy_data.h"
#include "RGBledDriver.h"

// Pick the buoy whose wind reading should drive a course computation.
//
// COMPUTESTART/COMPUTETRACK are executed by whichever buoy receives them, and that buoy squares
// the start line against ITS OWN wind direction. Sending the command to "the first buoy in the
// list" is therefore not safe: a buoy with a failed compass reports wDir/wStd as 0/0, which is
// indistinguishable from a real due-north calm, and the line silently comes out squared to north.
// Same test the wind overlay already uses to decide it has no data to draw.
static int pick_wind_reference_buoy() {
    for (int i = 0; i < 3; i++) {
        if (buoys[i].id != "" && (buoys[i].wind_dir != 0 || buoys[i].wind_std != 0)) {
            return i;
        }
    }
    return -1;
}

unsigned long lastUIUpdate = 0;
int lastKnownState = -2; // Used to trigger a static redraw on state change
bool lastSetupState = false; // Tracks setup screen state transitions
bool lastMannavState = false; // Tracks manual navigation screen state transitions
bool lastLoadedState = false; // Tracks setup data loaded transitions
bool lastManFourierCalState = false; // Tracks manual Fourier calibration screen state transitions

bool in_man_fourier_cal_mode = false;
unsigned long mancal_started_ms = 0;
int mancal_selected_leg = 0; // 0..7, or -1 while no direction has been picked yet
float mancal_offsets[8] = {0.0f};
bool mancal_is_dirty = true;
// The 8 offsets on screen are the ones the buoy is already running, not a blank slate. Until its
// answer to the STORE_INTERPOLATION_TABLE GET lands they are all zero, which is a real calibration
// value - so it has to be possible to tell "not answered yet" from "answered, and it is 0".
bool mancal_offsets_loaded = false;
unsigned long mancal_query_next_ms = 0;
int mancal_query_tries = 0;
#define MANCAL_QUERY_INTERVAL_MS 1200
#define MANCAL_QUERY_MAX_TRIES 5
// True between opening the screen and switching the buoy's harmonic correction off. The switch has
// to wait for the table to arrive: the Sub reports the table that is IN EFFECT, so once the
// correction is off it answers with the identity table and every offset would read 0.
bool mancal_harmonic_pending = false;
// Stamped by parse_buoy_packet() whenever the buoy sends its interpolation table. SAVE sends the
// table and then waits for this to move, which is the only honest way to tell the operator the
// store actually reached the buoy rather than just leaving the CYD.
volatile unsigned long mancal_table_echo_ms = 0;
// True from asking the buoy to switch its harmonic correction off until the buoy's own telemetry
// confirms it really is off. The 8 point table is defined against the RAW compass, so a capture
// taken while the correction is still running is silently wrong - it produces a table that gets
// applied on top of the one already in effect.
bool mancal_await_harmonic_off = false;
unsigned long mancal_harmonic_off_next_ms = 0;
int mancal_harmonic_off_tries = 0;
#define MANCAL_HARMONIC_RETRY_MS 1500
#define MANCAL_HARMONIC_MAX_TRIES 6
// This screen switches the harmonic correction off so the dialing works against the raw compass,
// and it must be ON again by the time the screen is left - by EITHER exit. A buoy left with it off
// answers every later table request with the identity table, so the corrections still in NVS become
// invisible and the screen reads as "this buoy has no calibration". Restoring "whatever it was on
// entry" was tried and is worse: the off state is almost always the residue of an earlier
// interrupted session, so restoring it makes the fault stick permanently.
// A finger still resting on the Setup screen's MAN CAL box (or its + button) sits right on top of
// the compass rose and the +/- row of the screen that replaces it. Without waiting for the release
// that same press is read as a direction tap and the buoy starts pivoting the moment the screen
// opens - which is exactly what entering this screen must not do.
bool mancal_await_release = false;
// Which of the eight directions have been steered to at least once this session. SAVE commits all
// eight entries in one frame, not just the one on screen, so a session that only looked at two of
// them would still overwrite the other six on the buoy with whatever happened to be in the buffer.
// The button stays locked until every direction has been visited.
bool mancal_leg_visited[8] = {false};

static int mancal_visited_count() {
    int n = 0;
    for (int i = 0; i < 8; i++) if (mancal_leg_visited[i]) n++;
    return n;
}

// mancal_selected_leg doubles as "which of the eight directions" and "none picked yet" (-1), and
// C++ will happily let that -1 index mancal_offsets: a silent write to the four bytes in front of
// the array. That is what used to command a heading of -45 degrees with nothing selected and spin
// the thrusters up. Every call site is guarded now, but the guards sit up to a hundred lines away
// from the subscripts they protect, so the next branch added inside one of those blocks would
// inherit the hazard with no compiler warning. Going through here instead makes a missed guard a
// harmless no-op on a scratch value rather than memory corruption.
static float mancal_offset_scratch = 0.0f;
static float &mancal_active_offset() {
    if (mancal_selected_leg < 0 || mancal_selected_leg > 7) {
        mancal_offset_scratch = 0.0f;
        return mancal_offset_scratch;
    }
    return mancal_offsets[mancal_selected_leg];
}

static bool mancal_all_legs_visited() {
    return mancal_visited_count() == 8;
}

// North has to be done first. Everything downstream is measured against it: SET AS NORTH folds the
// North error into compassOffset, and because the Sub applies that offset BEFORE the table lookup
// (RoboSub/src/compass.cpp:936 then :944) it shifts every other sector by the same amount. Doing
// North first means there is nothing else dialled in yet for it to invalidate. Once North has been
// visited the operator is free - take SET AS NORTH, or just leave a Fourier offset on North.
static bool mancal_leg_locked(int leg) {
    return leg != 0 && !mancal_leg_visited[0];
}

// Thrusters during this calibration session. ON is the in-water flow this screen has always done:
// picking a direction, and every -/+ press, commands the buoy to steer there and you dial until it
// points true. OFF is the shore flow - the buoy is aimed by hand, so nothing here ever commands a
// heading and SET captures the compass where it is pointed instead. Chosen on the row under the
// rose before a direction is taken; from the first direction onwards that row is the Set as North
// button and the mode is fixed, because half a table dialled and half captured is not one table.
bool mancal_thrusters_on = true;

// How stale the buoy's reported heading may be before SET refuses to capture it. RoboTop pushes
// TOPDATA at 4-10 Hz over UDP but only every 1-5 s over LoRa (RoboTop/src/main.cpp), so this has to
// clear a LoRa-only link while still being far short of the 60 s the dashboard calls "present" -
// capturing a minute-old heading would calibrate this leg against wherever the buoy used to point.
#define MANCAL_HEADING_STALE_MS 6000UL

// Every heading this screen commands goes through here, so shore mode is one test instead of six
// guards scattered across the touch handler.
static void mancal_steer(int buoy_idx) {
    if (!mancal_thrusters_on) return;
    send_buoy_dirdist(buoy_idx);
}

// The one-line complaint band under the rose, in the gap above the TG line.
static void mancal_warn(const char *msg) {
    tft.fillRect(0, 180, tft.width(), 26, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(2);
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawString(msg, tft.width() / 2, 193);
    delay(900);
    mancal_is_dirty = true;
}

// The five-across adjust row. Drawn from one place because it appears in both halves of the "has a
// direction been picked yet" split in update_mancal_dynamic(), and a second copy is a second
// geometry to keep in step with the single touch map in loop().
static void draw_mancal_adjust_row(bool leg_picked) {
    static const struct { int x; const char *label; } btn[5] = {
        { 10, "-10" }, { 55, "-1" }, { 100, "SET" }, { 145, "+1" }, { 190, "+10" }
    };
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    for (int i = 0; i < 5; i++) {
        // SET is not a third increment - it captures the compass where the buoy is aimed right
        // now - so it is coloured apart rather than sitting in the middle of the nudge buttons
        // looking like one of them. Dimmed until there is a direction for it to capture into.
        bool is_set = (i == 2);
        uint16_t bg = is_set ? (leg_picked ? TFT_BLUE : TFT_NAVY) : TFT_DARKGREY;
        tft.fillRoundRect(btn[i].x, 215, 40, 35, 5, bg);
        tft.setTextColor((is_set && !leg_picked) ? TFT_DARKGREY : TFT_WHITE, bg);
        tft.drawString(btn[i].label, btn[i].x + 20, 232);
    }
}

// The thruster switch, on the row that becomes Set as North once a direction has been taken.
// Colour rather than wording alone carries it: which way round it is has to be readable at a
// glance with the buoy in your hands.
static void draw_mancal_thruster_toggle() {
    int w = tft.width();
    uint16_t bg = mancal_thrusters_on ? TFT_RED : TFT_GREEN;
    tft.fillRoundRect(10, 260, w - 20, 35, 5, bg);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(mancal_thrusters_on ? TFT_WHITE : TFT_BLACK, bg);
    tft.setTextSize(2);
    tft.drawString(mancal_thrusters_on ? "THRUSTERS ON" : "THRUSTERS OFF", w / 2, 272);
    tft.setTextSize(1);
    tft.drawString(mancal_thrusters_on ? "in the water - tap to switch"
                                       : "on shore, by hand - tap to switch", w / 2, 288);
}

// Setup page index (0 for Page 1, 1 for Page 2)
int setup_page = 0;

// -------------------------------------------------------------------------------------------
// Setup screen slot map.
//
// The pages mirror the section headings of RoboTop's web Setup dialog one for one, in the same
// order and with the same labels, so the two interfaces can be used interchangeably. A page is
// 4 rows x 2 columns; slot = page * 8 + local, where local 0..3 is the LEFT column top to bottom
// and 4..7 the RIGHT column. An empty name is a gap: not drawn, not selectable.
//
// The GPS Fourier calibration has a page to itself instead of sharing "In-Field Calibration"
// with the other four actions, because its live progress panel needs three of the four rows.
// -------------------------------------------------------------------------------------------
#define SETUP_PAGES 6
#define SETUP_SLOTS (SETUP_PAGES * 8)

enum SetupSlot {
    S_RUD_P  = 0,  S_RUD_I  = 1,  S_RUD_D = 2,
    S_SPD_P  = 4,  S_SPD_I  = 5,  S_SPD_D = 6,
    S_MAXSPD = 8,  S_MINSPD = 9,  S_PIVOT = 10,
    S_COMPOFF = 12, S_HOLDRAD = 13,
    S_TRIMEN = 16, S_HARMONIC = 17,
    S_REVBB  = 20, S_REVSB = 21, S_SWAP = 22,
    S_APPDIST = 24, S_APPDIR = 25, S_DOCKWP = 26,
    S_DESKCAL = 32, S_SETNORTH = 33, S_REBOOT = 34,
    S_GPSSTILL = 40, S_MANCAL = 44
};

static const char *SETUP_NAMES[SETUP_SLOTS] = {
    /* 1 PID              */ "Rud P:", "Rud I:", "Rud D:", "",      "Spd P:", "Spd I:", "Spd D:", "",
    /* 2 SPEED & COMPASS  */ "MaxSpd:", "MinSpd:", "PvtSpd:", "",   "CompOff:", "HoldRad:", "", "",
    /* 3 TRIM & THRUSTERS */ "TrimEn:", "Harmonic:", "", "",        "BB Inv:", "SB Inv:", "Swap:", "",
    /* 4 DOCKING          */ "Appr Dist", "Appr Dir", "DockToWP", "", "", "", "", "",
    /* 5 CALIBRATION      */ "Desk Cal", "Set North", "Reboot", "", "", "", "", "",
    /* 6 GPS COMPASS CAL  */ "CAL STILLWTR", "", "", "",            "MAN CAL", "", "", ""
};

// Shown where the screen used to just say "SETUP" - these are the <h4> headings of the web form.
static const char *SETUP_PAGE_TITLES[SETUP_PAGES] = {
    "PID", "SPEED & COMPASS", "TRIM & THRUSTERS", "DOCKING", "CALIBRATION", "GPS COMPASS CAL"
};

// Pages holding no editable value. They must not be held behind the SETUPDATA reply the way the
// parameter pages are, and "-" means nothing on them.
static inline bool setup_is_action_page(int page) { return page == 4 || page == 5; }

static inline bool setup_slot_used(int slot) {
    return slot >= 0 && slot < SETUP_SLOTS && SETUP_NAMES[slot][0] != 0;
}

// Slots that DO something when "+" is pressed rather than holding a number.
static inline bool setup_slot_is_action(int slot) {
    return slot == S_DESKCAL || slot == S_SETNORTH || slot == S_REBOOT ||
           slot == S_GPSSTILL || slot == S_MANCAL;
}

static inline bool setup_slot_is_bool(int slot) {
    return slot == S_TRIMEN || slot == S_HARMONIC || slot == S_REVBB ||
           slot == S_REVSB  || slot == S_SWAP     || slot == S_DOCKWP;
}

static bool setup_bool_get(const BuoyData &b, int slot) {
    switch (slot) {
        case S_TRIMEN:   return b.compass_trim_enabled;
        case S_HARMONIC: return b.harmonic_enabled;
        case S_REVBB:    return b.rev_bb;
        case S_REVSB:    return b.rev_sb;
        case S_SWAP:     return b.swap_bb_sb;
        case S_DOCKWP:   return b.dock_to_wp;
        default:         return false;
    }
}

static void setup_bool_toggle(BuoyData &b, int slot) {
    switch (slot) {
        case S_TRIMEN:   b.compass_trim_enabled = !b.compass_trim_enabled; break;
        case S_HARMONIC: b.harmonic_enabled     = !b.harmonic_enabled;     break;
        case S_REVBB:    b.rev_bb               = !b.rev_bb;               break;
        case S_REVSB:    b.rev_sb               = !b.rev_sb;               break;
        case S_SWAP:     b.swap_bb_sb           = !b.swap_bb_sb;           break;
        case S_DOCKWP:   b.dock_to_wp           = !b.dock_to_wp;           break;
        default: break;
    }
}

// The value on its own, without the label - used both inside the box and in the big readout
// between "-" and "+", so the two can never disagree.
static void setup_value_text(const BuoyData &b, int slot, char *out, size_t n) {
    switch (slot) {
        case S_RUD_P:   snprintf(out, n, "%0.3f", b.kpr); break;
        case S_RUD_I:   snprintf(out, n, "%0.3f", b.kir); break;
        case S_RUD_D:   snprintf(out, n, "%0.3f", b.kdr); break;
        case S_SPD_P:   snprintf(out, n, "%0.3f", b.kps); break;
        case S_SPD_I:   snprintf(out, n, "%0.3f", b.kis); break;
        case S_SPD_D:   snprintf(out, n, "%0.3f", b.kds); break;
        case S_PIVOT:   snprintf(out, n, "%0.3f", b.pivot_speed); break;
        case S_MAXSPD:  snprintf(out, n, "%0.0f", b.max_speed); break;
        case S_MINSPD:  snprintf(out, n, "%0.0f", b.min_speed); break;
        case S_COMPOFF: snprintf(out, n, "%0.0f", b.compass_offset); break;
        case S_HOLDRAD: snprintf(out, n, "%0.1fm", b.hold_radius); break;
        case S_APPDIST: snprintf(out, n, "%dm", b.dock_app_dist); break;
        case S_APPDIR:  snprintf(out, n, "%d deg", b.dock_app_dir); break;
        default:        if (n) out[0] = 0; break;
    }
}

// Must match HOLD_RADIUS_MIN in RoboCompute.h. Spelled out again because the CYD does not link
// RoboCompute - it speaks to the buoys over the CSV wire format only. This used to allow 0.5,
// which the Sub then silently raised to 1.5 on arrival, so the Setup page showed a radius the
// buoy was not using.
#define CYD_HOLD_RADIUS_MIN 1.5f

static float setup_step(int slot) {
    switch (slot) {
        case S_RUD_I: case S_RUD_D: case S_SPD_I: case S_SPD_D: return 0.005f;
        case S_MAXSPD: case S_MINSPD:                           return 5.0f;
        case S_COMPOFF: case S_APPDIST: case S_APPDIR:          return 1.0f;
        case S_HOLDRAD:                                         return 0.5f;
        default:                                                return 0.05f;
    }
}

// One place for the "+" / "-" arithmetic and its limits, instead of the two mirrored if-chains
// this used to carry - keeping those in step by hand is what let the tap handler and the adjust
// handler disagree about which slots exist.
static void setup_adjust(BuoyData &b, int slot, bool plus) {
    float st = setup_step(slot) * (plus ? 1.0f : -1.0f);
    switch (slot) {
        case S_RUD_P: b.kpr += st; if (b.kpr < 0) b.kpr = 0; break;
        case S_RUD_I: b.kir += st; if (b.kir < 0) b.kir = 0; break;
        case S_RUD_D: b.kdr += st; if (b.kdr < 0) b.kdr = 0; break;
        case S_SPD_P: b.kps += st; if (b.kps < 0) b.kps = 0; break;
        case S_SPD_I: b.kis += st; if (b.kis < 0) b.kis = 0; break;
        case S_SPD_D: b.kds += st; if (b.kds < 0) b.kds = 0; break;
        case S_MAXSPD: b.max_speed += st;
                       if (b.max_speed < 0) b.max_speed = 0;
                       if (b.max_speed > 100) b.max_speed = 100; break;
        case S_MINSPD: b.min_speed += st;   // may legitimately be negative
                       if (b.min_speed < -100) b.min_speed = -100;
                       if (b.min_speed > 100) b.min_speed = 100; break;
        case S_PIVOT:  b.pivot_speed += st;
                       if (b.pivot_speed < 0) b.pivot_speed = 0;
                       if (b.pivot_speed > 1.0) b.pivot_speed = 1.0; break;
        case S_COMPOFF: b.compass_offset += st;
                        if (b.compass_offset < -180) b.compass_offset = -180;
                        if (b.compass_offset > 180) b.compass_offset = 180; break;
        case S_HOLDRAD: b.hold_radius += st;
                        if (b.hold_radius < CYD_HOLD_RADIUS_MIN) b.hold_radius = CYD_HOLD_RADIUS_MIN;
                        if (b.hold_radius > 10.0) b.hold_radius = 10.0; break;
        case S_APPDIST: b.dock_app_dist += (int)st;
                        if (b.dock_app_dist < 0) b.dock_app_dist = 0; break;
        case S_APPDIR:  b.dock_app_dir += (int)st;
                        if (b.dock_app_dir < 0) b.dock_app_dir += 360;
                        if (b.dock_app_dir >= 360) b.dock_app_dir -= 360; break;
        default: break;
    }
}
int lastSetupPage = -1; // Used to track page swaps

// Retry state for the SETUPDATA request.
//
// The request is a single unacknowledged packet, and so is the reply, over a path with four
// separate chances to drop it: CYD -> LoRa/UDP -> RoboTop -> half duplex serial -> Sub, and all
// the way back. Asking once meant any single loss left "LOADING DATA FROM BUOY..." on screen
// forever, with leaving and re-entering the screen as the only way to ask again.
#define SETUP_QUERY_INTERVAL_MS 2500UL
#define SETUP_QUERY_MAX_TRIES   8
unsigned long setup_query_next_ms = 0;
int setup_query_tries = 0;

// Screen transition lock-out timestamp to prevent touch propagation leakages
unsigned long last_transition_ms = 0;

// State caches to avoid redrawing menu buttons unnecessarily (Portrait size)
String last_drawn_ids[3] = {"", "", ""};
int last_drawn_present[3] = {-2, -2, -2}; // -2 uninitialized, -1 empty, 0 offline, 1 online

// Store old arrow angles globally to erase them cleanly on updates (Compass Windrose)
float old_mag_dir = -999.0;
float old_tg_dir = -999.0;
float old_wind_dir = -999.0;

// Touch Calibration Mode Variables
#include <Preferences.h>
bool in_calibration_mode = false;
bool in_track_settings_mode = false;
int cal_state = 0;
int cal_rx1 = 0, cal_ry1 = 0;
int cal_rx2 = 0, cal_ry2 = 0;
uint16_t temp_minx = 300, temp_maxx = 3800, temp_miny = 260, temp_maxy = 3800;
unsigned long save_screen_start_ms = 0;
unsigned long cal_started_ms = 0;
unsigned long last_left_touch_time = 0;
unsigned long last_right_touch_time = 0;
unsigned long simultaneous_touch_start = 0;
bool both_touched_previously = false;
unsigned long cal_touch_start = 0;
unsigned long last_touch_active_ms = 0;

void draw_calibration_screen();
void handle_touch_calibration();
void start_touch_calibration();
void draw_resting_ui();
void update_radar_map_dynamic();
void draw_track_buttons(bool force);
void draw_track_lock_hint();
void draw_line_len_label(const char *text, uint16_t color);
void draw_mancal_static();
void update_mancal_dynamic();
void draw_mancal_offset_strip();
void send_mancal_table_to_sub(int buoy_idx);
void enter_man_fourier_cal(int buoy_idx);
void service_mancal_entry();

// --- TRACK SETTINGS screen geometry (240x320 Portrait) ---
// The plot that used to be its own BUOY MAP page now IS this page: the two align buttons and the
// MAP button that led to it are gone, and the strip below the plot carries the six controls
// instead. Everything around the plot is size 2 text (12x16 px, max 20 characters per line).
#define MAP_CX          120  // Plot centre X
#define MAP_CY          140  // Plot centre Y
#define MAP_R            72  // Plot half-width: X 48..192, Y 68..212
#define MAP_HEADER_Y     30  // Range / Demo Mode banner (top of size 2 text)

// The start line length and the wind sit on the same row as the N cardinal, left and right of it,
// rather than on two legend lines under the plot - that strip is the button area now. The row is
// the band between the range banner and the top of the plot.
#define MAP_NROW_Y       66  // Text baseline, same as the N cardinal (MAP_CY - MAP_R - 2)
#define MAP_NROW_TOP     48  // Top of the band that is cleared on every refresh
#define MAP_NROW_H       19  // Height of that band
#define MAP_NROW_LEFT_R 100  // Right edge of the line length label (the N glyph starts at 114)
#define MAP_NROW_RIGHT_L 140 // Left edge of the wind label

// Two rows of buttons under the S cardinal, which ends at y=230.
#define TS_ROW1_Y       234  // - / START / TRACK / +
#define TS_ROW2_Y       273  // BACK / EXECUTE
#define TS_BTN_H         35

#define TS_MINUS_X        6
#define TS_MINUS_W       40
#define TS_START_X       51
#define TS_START_W       66
#define TS_TRACK_X      122
#define TS_TRACK_W       66
#define TS_PLUS_X       193
#define TS_PLUS_W        41
#define TS_BACK_X         6
#define TS_BACK_W       110
#define TS_EXEC_X       124
#define TS_EXEC_W       110

// One press of - or + moves the start line by this much, the same step the dashboard's start line
// panel uses (START_LINE_STEP_M in data/index.js).
#define TS_LINE_STEP_M  5.0f

// Text inset for the menu buoy buttons, which span X 10..230. Size 2 text is 12 px per
// character. The top row is spelled "Buoy1:b7a5b578" with no spaces around the colon: at
// 14 chars it is 168 px and ends at x=182, well clear of the one-character status badge
// that sits at 214..226 hard against the right inset. Spelled out as "Buoy n: <id>" it ran
// to 192 px and left only 8 px of daylight, so the badge painted over the tail of the id.
#define BTN_PAD_L        14  // Left text edge
#define BTN_PAD_R        14  // Right text edge, measured from the screen width

// Furthest a waypoint may be from the fleet centre and still pull the plot range out to fit it.
// A real course mark is tens of metres away; anything past this is a fault, so the plot keeps
// the fleet readable and pegs the mark to the rim instead.
#define MAP_WAYPOINT_MAX_M 300.0f

// Colour for a traffic indicator dot. Transmitting wins over receiving, so an outgoing
// command always shows up as RED even if telemetry is streaming in at the same time.
// 500 ms, comfortably longer than the 250 ms UI refresh tick so a one-shot command is never missed
#define TRAFFIC_TX_BLINK_MS 500

uint16_t traffic_dot_color(unsigned long tx_ms, unsigned long rx_ms, unsigned long rx_window, uint16_t rx_color) {
    unsigned long now = millis();

    if (tx_ms != 0 && now - tx_ms < TRAFFIC_TX_BLINK_MS) return TFT_RED;
    if (rx_ms != 0 && now - rx_ms < rx_window) return rx_color;

    return TFT_BLACK;
}

// Helper function to draw beautifully tapered, thick compass arrows with distinct arrowheads at their tips
void draw_compass_arrow(int cx, int cy, int L, float angle_deg, uint16_t color) {
    float theta = angle_deg * PI / 180.0;
    
    // Tip of the arrow
    int tip_x = cx + sin(theta) * L;
    int tip_y = cy - cos(theta) * L;
    
    // Base of the arrowhead (8 pixels down from tip)
    int base_x = cx + sin(theta) * (L - 8);
    int base_y = cy - cos(theta) * (L - 8);
    
    // Wings of the arrowhead (4 pixels offset perpendicular to the base)
    float perp_theta = theta + PI / 2.0;
    int wing_l_x = base_x + sin(perp_theta) * 4;
    int wing_l_y = base_y - cos(perp_theta) * 4;
    
    int wing_r_x = base_x - sin(perp_theta) * 4;
    int wing_r_y = base_y + cos(perp_theta) * 4;
    
    // Base of the shaft at the center (1.5 pixels offset perpendicular)
    int shaft_l_x = cx + sin(perp_theta) * 1.5;
    int shaft_l_y = cy - cos(perp_theta) * 1.5;
    
    int shaft_r_x = cx - sin(perp_theta) * 1.5;
    int shaft_r_y = cy + cos(perp_theta) * 1.5;
    
    // Draw thick shaft as a filled triangle/polygon or just three lines
    tft.fillTriangle(shaft_l_x, shaft_l_y, shaft_r_x, shaft_r_y, base_x, base_y, color);
    
    // Draw arrowhead
    tft.fillTriangle(wing_l_x, wing_l_y, wing_r_x, wing_r_y, tip_x, tip_y, color);
}

// Track selected parameter index on the 16-parameter SETUP screen on display
int selected_param_idx = 0; // Defaults to Rudder P (0)

void reset_button_draw_cache() {
    for (int i = 0; i < 3; i++) {
        last_drawn_ids[i] = "RESET"; // Force mismatch
        last_drawn_present[i] = -2;   // Force mismatch
    }
}

// Set whenever the setup screen is repainted from scratch, so the calibration panel knows its
// cached contents are no longer on the glass.
bool gps_cal_panel_dirty = true;

void draw_setup_static() {
    gps_cal_panel_dirty = true;
    int w = tft.width();
    int h = tft.height();
    int idx = selected_buoy_idx;
    
    tft.fillScreen(TFT_BLACK);
    
    // Header title
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(TC_DATUM);
    // The heading names the section, exactly as the web Setup form does, so the page you are
    // on is obvious - and so that the page that commands the buoy to sail for half an hour is
    // never labelled the same as the parameter pages.
    // No buoy ID here: the nav screen this was opened from already shows it as its own
    // header, so repeating it only costs width. Longest title is 16 chars = 192 px at size 2.
    // Width at text size 2 is 12 px per character on a 240 px screen, so the longer of the
    // two is 18 chars = 216 px and clears both edges by 12 px.
    tft.drawString(SETUP_PAGE_TITLES[setup_page % SETUP_PAGES], w / 2, 5);
    
    tft.drawFastHLine(15, 27, w - 30, TFT_WHITE);
    
    // Draw Single large Plus/Minus Adjustment Buttons Row (Y: 195 to 225)
    // If not loaded yet, these buttons are drawn disabled (Dark Grey) to protect the buoy NVM!
    // On the ACTIONS page "+" starts the selected calibration and "-" has no meaning at all, so
    // they are gated separately. Elsewhere both follow the setup-loaded guard as before.
    bool adjPlusUsable = setup_data_loaded || setup_is_action_page(setup_page);
    bool adjMinusUsable = setup_data_loaded && !setup_is_action_page(setup_page);
    uint16_t adjMinusColor = adjMinusUsable ? TFT_RED : TFT_DARKGREY;
    uint16_t adjMinusText = adjMinusUsable ? TFT_WHITE : TFT_LIGHTGREY;
    tft.fillRoundRect(15, 195, 60, 30, 4, adjMinusColor);
    tft.setTextColor(adjMinusText, adjMinusColor);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("-", 45, 210);
    
    uint16_t adjPlusColor = adjPlusUsable ? TFT_GREEN : TFT_DARKGREY;
    uint16_t adjPlusText = adjPlusUsable ? TFT_BLACK : TFT_LIGHTGREY;
    tft.fillRoundRect(165, 195, 60, 30, 4, adjPlusColor);
    tft.setTextColor(adjPlusText, adjPlusColor);
    tft.drawString("+", 195, 210);
    
    tft.drawFastHLine(15, 230, w - 30, TFT_WHITE);
    
    // Control buttons at the bottom: BACK (Blue), PAGE Toggle (Orange), & SAVE (Green or Grey depending on loaded state!)
    tft.setFreeFont(&FreeSansBold9pt7b); // Use beautiful bold GFX font!
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    
    tft.fillRoundRect(10, 235, 70, 35, 4, TFT_BLUE);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.drawString("BACK", 45, 252);
    
    tft.fillRoundRect(85, 235, 70, 35, 4, TFT_ORANGE);
    tft.setTextColor(TFT_BLACK, TFT_ORANGE);
    char pg_buf[16];
    sprintf(pg_buf, "PG %d/%d", setup_page + 1, SETUP_PAGES);
    tft.drawString(pg_buf, 120, 252);
    
    uint16_t saveBtnColor = setup_data_loaded ? TFT_GREEN : TFT_DARKGREY;
    uint16_t saveTextColor = setup_data_loaded ? TFT_BLACK : TFT_LIGHTGREY;
    tft.fillRoundRect(160, 235, 70, 35, 4, saveBtnColor);
    tft.setTextColor(saveTextColor, saveBtnColor);
    tft.drawString(setup_data_loaded ? "SAVE" : "WAIT", 195, 252);
    
    tft.setFreeFont(NULL); // Restore default font
}

void draw_nav_static() {
    int w = tft.width();
    int h = tft.height();
    int idx = selected_buoy_idx;

    tft.fillScreen(TFT_BLACK);

    // update_nav_dynamic() leaves a 40 pixel text padding behind. Left set, the opaque
    // background band around the W and E labels bites a chunk out of the compass circle.
    tft.setTextPadding(0);

    // Header title
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(TC_DATUM);
    tft.drawString(buoys[idx].id, w / 2, 5);
    
    tft.drawFastHLine(15, 27, w - 30, TFT_WHITE);
    
    // Draw Compass Rose Circle at center (120, 100)
    tft.drawCircle(120, 100, 45, TFT_WHITE);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("N", 120, 48);
    tft.drawString("S", 120, 152);
    tft.drawString("W", 67, 100);
    tft.drawString("E", 173, 100);
    
    // Draw Speedbar Outlines (Left BB, Right SB)
    tft.drawRect(15, 58, 15, 100, TFT_WHITE); // BB
    tft.drawRect(210, 58, 15, 100, TFT_WHITE); // SB
    
    tft.setTextDatum(BC_DATUM);
    tft.setTextSize(2); // Increased speedbar labels to font size 2!
    tft.drawString("BB", 22, 50);
    tft.drawString("SB", 217, 50);
    
    tft.drawFastHLine(15, 166, w - 30, TFT_WHITE);
    
    // Draw Static Voltage Bar outline (Y: 172) - bar still spans 17V to 25V, without the end labels
    tft.drawRect(50, 172, 140, 10, TFT_WHITE);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);

    tft.drawFastHLine(15, 202, w - 30, TFT_WHITE);
    
    // Draw Static Status header on the Left Column (X < 120)
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString("Status:", 15, 204);
    
    // Draw Static Stacked UDP / LoRa checkboxes on the Right Column (X >= 120, UDP at Y: 204, LoRa lowered to Y: 222!)
    tft.drawRect(125, 204, 11, 11, TFT_WHITE); // UDP box
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("UDP Enabled", 141, 204);
    
    tft.drawRect(125, 222, 11, 11, TFT_WHITE); // LoRa box (lowered to Y: 222 for supreme readability)
    tft.drawString("LoRa Enabled", 141, 222);
    
    // Row 1 Buttons (LOCK, DOCK, IDLE) at Y: 240 to 275 (height 35)
    tft.setFreeFont(&FreeSansBold9pt7b); // Use beautiful bold GFX font!
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    
    tft.fillRoundRect(10, 240, 70, 35, 4, TFT_DARKGREEN);
    tft.setTextColor(TFT_WHITE, TFT_DARKGREEN);
    tft.drawString("LOCK", 45, 257);
    
    tft.fillRoundRect(85, 240, 70, 35, 4, TFT_YELLOW);
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
    tft.drawString("DOCK", 120, 257);
    
    tft.fillRoundRect(160, 240, 70, 35, 4, TFT_MAROON);
    tft.setTextColor(TFT_WHITE, TFT_MAROON);
    tft.drawString("IDLE", 195, 257);
    
    // Row 2 Buttons (BACK, MANNAV, SETUP) at Y: 280 to 315 (height 35)
    tft.fillRoundRect(10, 280, 70, 35, 4, TFT_BLUE);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.drawString("BACK", 45, 297);
    
    tft.fillRoundRect(85, 280, 70, 35, 4, TFT_ORANGE);
    tft.setTextColor(TFT_BLACK, TFT_ORANGE);
    tft.drawString("MAN", 120, 297);
    
    tft.fillRoundRect(160, 280, 70, 35, 4, TFT_DARKGREY);
    tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
    tft.drawString("SETUP", 195, 297);
    
    tft.setFreeFont(NULL); // Restore default font
}

void draw_mannav_static() {
    int w = tft.width();
    int h = tft.height();
    int idx = selected_buoy_idx;

    tft.fillScreen(TFT_BLACK);

    // Same stale padding hazard as draw_nav_static(): clear it before the compass labels
    tft.setTextPadding(0);

    // Header title
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("MANUAL: " + buoys[idx].id, w / 2, 5);
    
    tft.drawFastHLine(15, 28, w - 30, TFT_WHITE);
    
    // Draw Compass Rose Circle at center (120, 95) with radius 45
    tft.drawCircle(120, 95, 45, TFT_WHITE);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("N", 120, 43);
    tft.drawString("S", 120, 147);
    tft.drawString("W", 67, 95);
    tft.drawString("E", 173, 95);
    
    // Draw Speedbar Outlines (Left BB, Right SB)
    tft.drawRect(15, 58, 15, 100, TFT_WHITE); // BB
    tft.drawRect(210, 58, 15, 100, TFT_WHITE); // SB
    
    tft.setTextDatum(BC_DATUM);
    tft.setTextSize(2); // Increased speedbar labels to font size 2!
    tft.drawString("BB", 22, 50);
    tft.drawString("SB", 217, 50);
    
    // Draw Static Voltage Bar outline (Y: 175) - Shifted down to prevent speedbar overlap!
    tft.drawRect(50, 175, 140, 10, TFT_WHITE);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("17V", 25, 176);
    tft.drawString("25V", 197, 176);
    
    // --- Slider 1 (Target Dir) static layout (Y: 210) ---
    // Minus/Decrease Button on Left (X: 15 to 50, Y: 200 to 225)
    tft.fillRoundRect(15, 200, 35, 25, 4, TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("-", 32, 212);
    
    // Plus/Increase Button on Right (X: 190 to 225, Y: 200 to 225)
    tft.fillRoundRect(190, 200, 35, 25, 4, TFT_GREEN);
    tft.setTextColor(TFT_BLACK, TFT_GREEN);
    tft.drawString("+", 207, 212);
    
    // Track line (X: 60 to 180, width 120)
    tft.drawRoundRect(60, 210, 120, 6, 3, TFT_DARKGREY);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("Direction", 60, 197);
    
    // --- Slider 2 (Speed) static layout (Y: 250) ---
    // Minus/Decrease Button on Left (X: 15 to 50, Y: 240 to 265)
    tft.fillRoundRect(15, 240, 35, 25, 4, TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("-", 32, 252);
    
    // Plus/Increase Button on Right (X: 190 to 225, Y: 240 to 265)
    tft.fillRoundRect(190, 240, 35, 25, 4, TFT_GREEN);
    tft.setTextColor(TFT_BLACK, TFT_GREEN);
    tft.drawString("+", 207, 252);
    
    // Track line (X: 60 to 180, width 120)
    tft.drawRoundRect(60, 250, 120, 6, 3, TFT_DARKGREY);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("Speed", 60, 237);
    
    // Bottom Buttons: BACK (left) and IDLE (right) at Y: 275 to 310 (height 35)
    tft.fillRoundRect(15, 275, 100, 35, 4, TFT_BLUE);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("BACK", 65, 292);
    
    tft.fillRoundRect(125, 275, 100, 35, 4, TFT_MAROON);
    tft.setTextColor(TFT_WHITE, TFT_MAROON);
    tft.drawString("IDLE", 175, 292);
}

void update_mannav_dynamic() {
    int w = tft.width();
    int h = tft.height();
    int idx = selected_buoy_idx;
    BuoyData &b = buoys[idx];
    
    // State caches for mannav screen
    static int last_buoy_idx = -1;
    static float last_battery_v = -1.0;
    static float last_tg_dir = -999;
    static float last_tg_speed = -999;
    static float last_mag_dir = -999;
    static float last_bb_power = -999;
    static float last_sb_power = -999;
    static bool was_mannav_mode = false;
    static bool was_setup_mode = false;
    
    if (selected_buoy_idx != last_buoy_idx || in_mannav_mode != was_mannav_mode || in_setup_mode != was_setup_mode) {
        last_buoy_idx = selected_buoy_idx;
        was_mannav_mode = in_mannav_mode;
        was_setup_mode = in_setup_mode;
        
        last_battery_v = -1.0;
        last_tg_dir = -999;
        last_tg_speed = -999;
        last_mag_dir = -999;
        last_bb_power = -999;
        last_sb_power = -999;
    }
    
    char buf[128];
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    
    // --- 1. Redraw dynamic arrows inside windrose circle (center 120, 95) ---
    if (b.mag_dir != last_mag_dir || b.tg_dir != last_tg_dir) {
        // Erase old arrows
        if (last_mag_dir != -999.0) draw_compass_arrow(120, 95, 42, last_mag_dir, TFT_BLACK);
        if (last_tg_dir != -999.0) draw_compass_arrow(120, 95, 36, last_tg_dir, TFT_BLACK);
        
        last_mag_dir = b.mag_dir;
        last_tg_dir = b.tg_dir;
        
        // Draw new arrows
        draw_compass_arrow(120, 95, 42, last_mag_dir, TFT_GREEN);
        draw_compass_arrow(120, 95, 36, last_tg_dir, TFT_RED);
        
        // Redraw center pivot dot
        tft.fillCircle(120, 95, 3, TFT_WHITE);
    }
    
    // Print Magnetic Direction (Mag) text on the bottom-right of the windrose circle (Y: 130)
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.setTextDatum(TR_DATUM);
    tft.setTextPadding(44); // Overwrite old text in single pass!
    sprintf(buf, "Mag:%0.0f", b.mag_dir);
    tft.drawString(buf, 206, 130);
    
    // Save caches
    last_mag_dir = b.mag_dir;
    
    // --- 2. Update Dynamic Voltage Bar (Y: 175-185) ---
    if (b.battery_v != last_battery_v) {
        last_battery_v = b.battery_v;
        tft.fillRect(51, 176, 138, 8, TFT_BLACK);
        float v = b.battery_v;
        if (v < 17.0) v = 17.0;
        if (v > 25.0) v = 25.0;
        int fill_w = map(v * 10, 170, 250, 0, 138);
        
        uint16_t barColor = TFT_GREEN;
        if (b.battery_v < 19.5) barColor = TFT_RED;
        else if (b.battery_v < 22.0) barColor = TFT_YELLOW;
        
        tft.fillRect(51, 176, fill_w, 8, barColor);
    }
    
    // --- 3. Update BB and SB Speedbars ---
    int mid_y = 108;
    if (b.bb_power != last_bb_power) {
        last_bb_power = b.bb_power;
        tft.fillRect(16, 59, 13, 98, TFT_BLACK); // Clear inner area (Y: 59-157)
        if (b.bb_power > 0) {
            int fill_h = (b.bb_power * 49) / 100;
            tft.fillRect(16, mid_y - fill_h, 13, fill_h, TFT_GREEN);
        } else if (b.bb_power < 0) {
            int fill_h = (-b.bb_power * 49) / 100;
            tft.fillRect(16, mid_y, 13, fill_h, TFT_RED);
        }
        tft.drawFastHLine(15, mid_y, 15, TFT_DARKGREY); // Reset centerline
    }
    
    if (b.sb_power != last_sb_power) {
        last_sb_power = b.sb_power;
        tft.fillRect(211, 59, 13, 98, TFT_BLACK); // Clear inner area
        if (b.sb_power > 0) {
            int fill_h = (b.sb_power * 49) / 100;
            tft.fillRect(211, mid_y - fill_h, 13, fill_h, TFT_GREEN);
        } else if (b.sb_power < 0) {
            int fill_h = (-b.sb_power * 49) / 100;
            tft.fillRect(211, mid_y, 13, fill_h, TFT_RED);
        }
        tft.drawFastHLine(210, mid_y, 15, TFT_DARKGREY); // Reset centerline
    }
    
    // Print BB and SB percentage text below speedbars using text padding to eliminate flicker
    tft.setTextSize(2); // Increased speedbar percentage text to font size 2!
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TC_DATUM);
    tft.setTextPadding(45); // Overwrite old text in single pass!
    
    sprintf(buf, "%0.0f%%", b.bb_power);
    tft.drawString(buf, 22, 162);
    
    sprintf(buf, "%0.0f%%", b.sb_power);
    tft.drawString(buf, 217, 162);
    
    // --- 4. Update Sliders (ONLY draw tracks and circular thumbs!) ---
    // TG Dir Slider (Y: 210)
    if (b.tg_dir != last_tg_dir) {
        last_tg_dir = b.tg_dir;
        // Clear slider track width area (X: 55 to 185, Y: 201 to 221)
        tft.fillRect(55, 201, 130, 20, TFT_BLACK);
        tft.drawRoundRect(60, 210, 120, 6, 3, TFT_DARKGREY);
        
        // Draw new thumb (X: 60 to 180)
        int thumb_x = 60 + (b.tg_dir * 120) / 360;
        tft.fillCircle(thumb_x, 213, 6, TFT_RED);
    }
    
    // Speed Slider (Y: 250)
    if (b.tg_speed != last_tg_speed) {
        last_tg_speed = b.tg_speed;
        // Clear slider track area (X: 55 to 185, Y: 241 to 261)
        tft.fillRect(55, 241, 130, 20, TFT_BLACK);
        tft.drawRoundRect(60, 250, 120, 6, 3, TFT_DARKGREY);
        
        // Draw new thumb (X: 60 to 180)
        float denom = (2.0 * b.max_speed);
        float pct = (denom == 0) ? 0.5 : (b.tg_speed - (-b.max_speed)) / denom;
        int thumb_x = 60 + pct * 120;
        tft.fillCircle(thumb_x, 253, 6, TFT_GREEN);
    }
    
    tft.setTextPadding(0);
    
    // Redraw the main white circle boundary ON TOP of all elements to prevent overlap gaps!
    tft.drawCircle(120, 95, 45, TFT_WHITE);
    tft.fillCircle(120, 95, 3, TFT_WHITE);
    
    // --- 5. Update Blinking Telemetry Indicators (Top-Right, stacked vertically, LoRa on top of UDP) ---
    // Both only blink for traffic belonging to THIS buoy; red while transmitting.
    uint16_t udpDotColor = traffic_dot_color(last_udp_tx_ms, last_udp_sel_blink_ms, 100, TFT_GREEN);
    tft.fillCircle(230, 20, 4, udpDotColor);

    uint16_t loraDotColor = traffic_dot_color(last_lora_tx_ms, last_lora_blink_ms, 300, TFT_CYAN);
    tft.fillCircle(230, 8, 4, loraDotColor);
}

// =================================================================================================
// Start line length, as dialled on this screen.
//
// Nothing in the system stores a "line length": recalcStartLine() on the Top keeps whatever
// distance the two buoys already are apart and only swings the line square to the wind. So the
// length here is MEASURED off the fleet, and changing it means physically repositioning both end
// buoys - which is what EXECUTE does, by pushing each of them a new SETLOCKPOS.
//
// track_line_cur_m is re-measured on every 1 Hz plot refresh. track_line_tgt_m is what - and +
// have dialled it to, and is < 0 while no change is pending. Keeping the pending figure separate
// is the whole point: the buoys creep towards their targets constantly, so a number that ticks
// while you are trying to set it is unusable.
// =================================================================================================
static float track_line_cur_m = -1.0f;  // Live measurement, < 0 when there is no line to measure
static float track_line_tgt_m = -1.0f;  // Dialled by - / +, < 0 when nothing is pending
static int track_line_a = -1;           // The two buoys forming the line, indices into buoys[]
static int track_line_b = -1;

// How many buoys the controller currently has on the air. Drives the greying out of the buttons:
// a start line needs two ends, and a track needs the third buoy for the upwind mark.
static int count_buoys_present() {
    int n = 0;
    for (int i = 0; i < 3; i++) {
        if (buoys[i].id != "" && buoys[i].present) n++;
    }
    return n;
}

// How many buoys are actually holding a lock position.
//
// ALIGN squares the start line through the two ends' LOCK positions, so the Top refuses the whole
// operation below two of them - and against an unlocked buoy it would be computing from whatever
// stale target that buoy still carries. LOCKING counts: the buoy owns a real lock position and is
// on its way to it. DOCKING/DOCKED do not - a buoy going home is not part of a course.
// LOCKING (12) counts as locked as well as LOCKED (13): the buoy owns a real lock position and is
// on its way to it. DOCKING/DOCKED deliberately do not - a buoy going home is not part of a course.
static bool buoy_is_locked(int i) {
    if (buoys[i].id == "" || !buoys[i].present) return false;
    return (buoys[i].status_code == 12 || buoys[i].status_code == 13);
}

static int count_buoys_locked() {
    int n = 0;
    for (int i = 0; i < 3; i++) {
        if (buoy_is_locked(i)) n++;
    }
    return n;
}

// Where a buoy's end of the line is. Its commanded waypoint if it still has a fresh one, otherwise
// its own fix. Preferring the waypoint keeps the geometry clean: a buoy holding station wanders
// inside its hold radius, and measuring off that wobble would make the length - and the bearing
// EXECUTE re-uses - jitter by a couple of metres between refreshes.
static bool buoy_line_point(const BuoyData &b, double &out_lat, double &out_lon) {
    if (buoy_has_waypoint(b) && b.tg_lat != 0 && b.tg_lon != 0) {
        out_lat = b.tg_lat;
        out_lon = b.tg_lon;
        return true;
    }
    if (b.lat == "N/A" || b.lat == "" || atof(b.lat.c_str()) == 0) return false;
    out_lat = atof(b.lat.c_str());
    out_lon = atof(b.lon.c_str());
    return true;
}

// Great circle distance in metres. get_relative_meters() would do for the short legs involved, but
// this pairs with track_bearing_to()/track_project() below and the three have to agree.
static double track_distance_m(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0, rad = PI / 180.0;
    double dLat = (lat2 - lat1) * rad, dLon = (lon2 - lon1) * rad;
    double a = sin(dLat / 2) * sin(dLat / 2)
             + cos(lat1 * rad) * cos(lat2 * rad) * sin(dLon / 2) * sin(dLon / 2);
    return 2 * R * atan2(sqrt(a), sqrt(1 - a));
}

static double track_bearing_to(double lat1, double lon1, double lat2, double lon2) {
    const double rad = PI / 180.0;
    double y = sin((lon2 - lon1) * rad) * cos(lat2 * rad);
    double x = cos(lat1 * rad) * sin(lat2 * rad)
             - sin(lat1 * rad) * cos(lat2 * rad) * cos((lon2 - lon1) * rad);
    return fmod(atan2(y, x) * 180.0 / PI + 360.0, 360.0);
}

static void track_project(double lat, double lon, double bearing_deg, double dist_m,
                          double &out_lat, double &out_lon) {
    const double R = 6371000.0, rad = PI / 180.0;
    double d = dist_m / R;
    double brg = bearing_deg * rad;
    double la = lat * rad, lo = lon * rad;
    double la2 = asin(sin(la) * cos(d) + cos(la) * sin(d) * cos(brg));
    double lo2 = lo + atan2(sin(brg) * sin(d) * cos(la), cos(d) - sin(la) * sin(la2));
    out_lat = la2 / rad;
    out_lon = fmod(lo2 / rad + 540.0, 360.0) - 180.0;
}

// Sailing convention for the buoy colours: starboard hand GREEN, port hand RED, the upwind HEAD
// mark BLUE. Roles only exist once the WHOLE fleet is locked - until then a course has not been
// laid out, so there are no sides to be on and every buoy present is simply green.
//
// Derived exactly the way RoboCompute::recalcStartLine() derives them, so the colours agree with
// what the buoys will actually be told to do: the two lying closest together are the start line,
// the odd one out is the upwind HEAD mark, and the starboard end is whichever end lies towards
// wDir + 90 from the middle of the line.
//
// Anything missing - a buoy without a fix, or no wind reading anywhere in the fleet - leaves every
// role NONE. Green for "present, no course" is honest; guessing a side from incomplete data and
// painting a buoy red would not be.
#define ROLE_NONE       0
#define ROLE_HEAD       1
#define ROLE_PORT       2
#define ROLE_STARBOARD  3

static void compute_buoy_roles(int roles[3]) {
    for (int i = 0; i < 3; i++) roles[i] = ROLE_NONE;

    int present = count_buoys_present();
    if (present < 2 || count_buoys_locked() != present) return;

    double lat[3] = {0, 0, 0}, lon[3] = {0, 0, 0};
    int idx[3], n = 0;
    for (int i = 0; i < 3; i++) {
        if (buoys[i].id == "" || !buoys[i].present) continue;
        if (!buoy_line_point(buoys[i], lat[i], lon[i])) return; // no position, no geometry
        idx[n++] = i;
    }
    if (n != present) return;

    int wind_idx = pick_wind_reference_buoy();
    if (wind_idx < 0) return; // Without wind there is no port or starboard side to speak of
    double wdir = buoys[wind_idx].wind_dir;

    int a = idx[0], b = idx[1];
    if (n == 3) {
        double best = -1;
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                double d = track_distance_m(lat[idx[i]], lon[idx[i]], lat[idx[j]], lon[idx[j]]);
                if (best < 0 || d < best) { best = d; a = idx[i]; b = idx[j]; }
            }
        }
        for (int i = 0; i < n; i++) {
            if (idx[i] != a && idx[i] != b) roles[idx[i]] = ROLE_HEAD;
        }
    }

    double mid_lat = (lat[a] + lat[b]) / 2.0;
    double mid_lon = (lon[a] + lon[b]) / 2.0;
    double sb_bearing = fmod(wdir + 90.0, 360.0);
    // Smallest signed separation from the starboard bearing, folded to 0..180.
    double da = fabs(fmod(track_bearing_to(mid_lat, mid_lon, lat[a], lon[a]) - sb_bearing + 540.0, 360.0) - 180.0);
    double db = fabs(fmod(track_bearing_to(mid_lat, mid_lon, lat[b], lon[b]) - sb_bearing + 540.0, 360.0) - 180.0);
    if (da <= db) { roles[a] = ROLE_STARBOARD; roles[b] = ROLE_PORT; }
    else          { roles[a] = ROLE_PORT;      roles[b] = ROLE_STARBOARD; }
}

// Fill colour for a buoy, by role. An OFFLINE buoy is greyed out rather than reddened - red means
// "port hand" here, and a dead buoy must not be mistaken for one that is holding a station.
static uint16_t buoy_role_color(int idx, const int roles[3]) {
    if (buoys[idx].id == "" || !buoys[idx].present) return TFT_DARKGREY;
    switch (roles[idx]) {
        case ROLE_HEAD:      return TFT_BLUE;
        case ROLE_STARBOARD: return TFT_GREEN;
        case ROLE_PORT:      return TFT_RED;
        default:             break;
    }
    // No role to show - either the fleet is not fully locked, or there is no wind to take sides
    // against. YELLOW for a buoy that is not holding a station, green once it is. That makes the
    // one thing you want at a glance before a start readable without counting badges: anything
    // yellow is still not on station.
    return buoy_is_locked(idx) ? TFT_GREEN : TFT_YELLOW;
}

static uint16_t buoy_role_text_color(uint16_t bg) {
    return (bg == TFT_GREEN || bg == TFT_YELLOW) ? TFT_BLACK : TFT_WHITE;
}

// True when both ends of the line are known, i.e. when - / + / EXECUTE have something to work on.
// Filled in by update_radar_map_dynamic() as a side effect of plotting the line.
static bool track_line_measured() {
    return track_line_cur_m > 0 && track_line_a >= 0 && track_line_b >= 0;
}

// One button. Greyed out by filling it dark rather than by hiding it - the layout has to stay put
// so the operator's finger lands in the same place whether or not the third buoy is on the air.
static void draw_track_button(int x, int y, int w, const char *label, uint16_t fill,
                              uint16_t text_col, bool enabled) {
    uint16_t bg = enabled ? fill : TFT_DARKGREY;
    uint16_t fg = enabled ? text_col : 0x4208; // Barely-there grey: reads as "not now", not as text
    tft.fillRoundRect(x, y, w, TS_BTN_H, 5, bg);
    tft.setFreeFont(&FreeSansBold9pt7b);
    tft.setTextPadding(0);
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(fg, bg);
    tft.drawString(label, x + w / 2, y + TS_BTN_H / 2);
    tft.setFreeFont(NULL);
}

// The six controls under the plot. Repainted only when something about them actually changed - at
// the 1 Hz refresh an unconditional repaint of six rounded rects reads as a flicker.
//
//   -  START  TRACK  +      - and + dial the line 5 m shorter/longer, START squares it to the
//   BACK      EXECUTE       wind (needs 2 buoys), TRACK lays out the full course (needs 3), and
//                           EXECUTE commits whatever - and + have dialled.
void draw_track_buttons(bool force) {
    int fleet = count_buoys_present();
    int locked = count_buoys_locked();
    bool can_len   = (fleet >= 2) && track_line_measured();
    // START and TRACK need LOCKED buoys, not merely present ones - see count_buoys_locked().
    bool can_start = (locked >= 2);
    bool can_track = (locked >= 3);
    bool can_exec  = can_len && track_line_tgt_m > 0
                     && lroundf(track_line_tgt_m) != lroundf(track_line_cur_m);

    static int last_key = -1;
    int key = (can_len ? 1 : 0) | (can_start ? 2 : 0) | (can_track ? 4 : 0) | (can_exec ? 8 : 0);
    // The EXECUTE caption carries the dialled length, so that has to be in the cache key as well
    key |= (can_exec ? ((int)lroundf(track_line_tgt_m) << 4) : 0);
    if (!force && key == last_key) return;
    last_key = key;

    draw_track_button(TS_MINUS_X, TS_ROW1_Y, TS_MINUS_W, "-",     TFT_BROWN,   TFT_WHITE, can_len);
    draw_track_button(TS_START_X, TS_ROW1_Y, TS_START_W, "START", TFT_SKYBLUE, TFT_BLACK, can_start);
    draw_track_button(TS_TRACK_X, TS_ROW1_Y, TS_TRACK_W, "TRACK", TFT_BLUE,    TFT_WHITE, can_track);
    draw_track_button(TS_PLUS_X,  TS_ROW1_Y, TS_PLUS_W,  "+",     TFT_BROWN,   TFT_WHITE, can_len);

    draw_track_button(TS_BACK_X, TS_ROW2_Y, TS_BACK_W, "BACK", TFT_DARKGREY, TFT_WHITE, true);

    char exec_label[16];
    if (can_exec) snprintf(exec_label, sizeof(exec_label), "GO %dM", (int)lroundf(track_line_tgt_m));
    else          snprintf(exec_label, sizeof(exec_label), "EXECUTE");
    draw_track_button(TS_EXEC_X, TS_ROW2_Y, TS_EXEC_W, exec_label, TFT_GREEN, TFT_BLACK, can_exec);
}

// Move both ends of the line symmetrically about its current midpoint, keeping its present
// bearing. Length only - squaring the line to the wind is what START does.
//
// SETLOCKPOS is the same command the Top uses to push computed start line ends to the other buoy
// (RoboTop, case SENDTRACK): the receiver stores the point, sails to it and locks.
static void track_line_execute() {
    if (!track_line_measured() || track_line_tgt_m <= 0) return;

    double alat, alon, blat, blon;
    if (!buoy_line_point(buoys[track_line_a], alat, alon)) return;
    if (!buoy_line_point(buoys[track_line_b], blat, blon)) return;

    double mid_lat = (alat + blat) / 2.0;
    double mid_lon = (alon + blon) / 2.0;
    double brg_a = track_bearing_to(mid_lat, mid_lon, alat, alon);
    double brg_b = track_bearing_to(mid_lat, mid_lon, blat, blon);
    double half = track_line_tgt_m / 2.0;

    double pa_lat, pa_lon, pb_lat, pb_lon;
    track_project(mid_lat, mid_lon, brg_a, half, pa_lat, pa_lon);
    track_project(mid_lat, mid_lon, brg_b, half, pb_lat, pb_lon);

    send_buoy_setlockpos(buoys[track_line_a].id, buoys[track_line_a].status_code, pa_lat, pa_lon);
    send_buoy_setlockpos(buoys[track_line_b].id, buoys[track_line_b].status_code, pb_lat, pb_lon);

    // The dialled figure is the line now. Clearing the pending value hands the left hand label back
    // to the live measurement, which walks to the new length as the two buoys motor out to it.
    track_line_tgt_m = -1.0f;
}

// Show what - / + have dialled, in the buttons' own colour so it reads as "pending", not as the
// measurement. Only the label is repainted: replotting the whole map on every press is both slower
// and visibly flickery when a button is held down and repeats.
static void track_settings_show_pending() {
    char buf[12];
    snprintf(buf, sizeof(buf), "%0.0fM", track_line_tgt_m);
    draw_line_len_label(buf, TFT_ORANGE);
    draw_track_buttons(false);
}

// The strip between the bottom of the plot and the first row of buttons carries EITHER the S
// cardinal or, while fewer than two buoys are locked, the LOCK 2 BUOYS FIRST warning. There is no
// room for both - the plot ends at 212, the S glyph runs to 230 and the buttons start at 234 - and
// the warning is the more useful of the two while there is no course to look at.
//
// Out here it has the full 240 px to work with, so it fits on one line: 18 characters at 12 px per
// character is 216 px, centred with 12 px to spare either side. Inside the plot it would not have,
// which is why it used to be stacked over two lines in the middle of the map.
//
// Both branches repaint the whole strip, so this is also what puts the S back when the fleet
// locks - the static screen draw is not re-run on a status change.
void draw_track_lock_hint() {
    bool warn = (count_buoys_locked() < 2);

    tft.fillRect(0, MAP_CY + MAP_R, 240, TS_ROW1_Y - (MAP_CY + MAP_R) - 1, TFT_BLACK);
    tft.setFreeFont(NULL);
    tft.setTextPadding(0);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);

    if (warn) {
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.drawString("LOCK 2 BUOYS FIRST", MAP_CX, MAP_CY + MAP_R + 10);
    } else {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.drawString("S", MAP_CX, MAP_CY + MAP_R + 10);
    }
}

// A one line message across the middle of the plot, for the moment between pressing a button and
// the fleet acting on it. It is painted straight over the plot rectangle, which the next refresh
// wipes anyway, so nothing has to undo it.
static void track_settings_banner(const char *text, uint16_t color) {
    tft.fillRect(MAP_CX - MAP_R, MAP_CY - 12, MAP_R * 2 + 1, 24, TFT_BLACK);
    tft.setFreeFont(NULL);
    tft.setTextPadding(0);
    tft.setTextColor(color, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(text, MAP_CX, MAP_CY);
}

// START and TRACK both come down to one command sent to one buoy: COMPUTESTART (62) squares the
// start line to the wind, COMPUTETRACK (63) lays out the whole course.
//
// The command is executed by whichever buoy receives it, and that buoy squares the line against
// ITS OWN wind. Sending it to "the first buoy in the list" is therefore not safe: a buoy with a
// failed compass reports wDir/wStd as 0/0, which is indistinguishable from a real due-north calm,
// and the line silently comes out squared to north. Hence pick_wind_reference_buoy().
static void track_settings_compute(int cmd_code) {
    track_settings_banner("COMPUTING...", TFT_YELLOW);

    int wind_idx = pick_wind_reference_buoy();
    if (wind_idx >= 0) {
        send_buoy_command(buoys[wind_idx].id, cmd_code);
    } else {
        track_settings_banner("NO WIND DATA!", TFT_RED);
    }

    // The buoys are about to be given new targets, so nothing dialled before this still applies.
    track_line_tgt_m = -1.0f;

    delay(800);
    last_transition_ms = millis();
    reset_button_draw_cache();
    draw_resting_ui();
}

// Commit the eight captured directions to the buoy and leave the screen.
//
// Reached from two places now - the small SAVE in the footer while the table is still being built,
// and the big SAVE NEW FOURIER TABLE that takes over the bottom of the screen once all eight are
// in. One body, so the two cannot drift apart.
static void mancal_save_table_and_exit(int buoy_index) {
    BuoyData &b = buoys[buoy_index];
    const int selected_buoy_idx = buoy_index;   // the body below was written against this name
    // Locked until every direction has been visited - see the button drawing.
    if (!mancal_all_legs_visited()) {
        tft.fillRect(0, 180, tft.width(), 26, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.setTextSize(2);
        tft.setTextColor(TFT_ORANGE, TFT_BLACK);
        char warn_buf[32];
        sprintf(warn_buf, "%d OF 8 DONE", mancal_visited_count());
        tft.drawString(warn_buf, tft.width() / 2, 193);
        delay(900);
        mancal_is_dirty = true;
        delay(20);
        return;
    }

    // Commit the eight entries, then WAIT for the buoy to echo the table
    // back. The Sub answers every store with the table it now holds, so that
    // echo is proof it reached NVS - and it beeps at the same moment. This
    // used to print CALIBRATION COMPLETE unconditionally, which said exactly
    // as much whether the buoy had heard it or not.
    tft.fillRect(0, 195, tft.width(), 100, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(2);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.drawString("SAVING...", tft.width() / 2, 230);

    mancal_table_echo_ms = 0;
    send_mancal_table_to_sub(selected_buoy_idx);

    unsigned long wait_until = millis() + 4000;
    while (mancal_table_echo_ms == 0 && (long)(millis() - wait_until) < 0) {
        handle_wifi_clients();
        check_lora_packets();
        delay(20);
    }
    bool stored = (mancal_table_echo_ms != 0);
    
    // Put the correction and the trim back, whichever way this went.
    b.harmonic_enabled = true;
    b.compass_trim_enabled = true;
    send_buoy_setup(selected_buoy_idx);
    mancal_await_harmonic_off = false;
    
    // Send IDLE command immediately to shut down thrusters/motors after calibration!
    send_buoy_command(b.id, 8);
    
    tft.fillRect(0, 195, tft.width(), 100, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(2);
    if (stored) {
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.drawString("TABLE STORED", tft.width() / 2, 225);
        tft.setTextSize(1);
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        tft.drawString("buoy confirmed and beeped", tft.width() / 2, 248);
    } else {
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawString("NO REPLY", tft.width() / 2, 225);
        tft.setTextSize(1);
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        tft.drawString("NOT saved - try again", tft.width() / 2, 248);
    }
    delay(stored ? 1200 : 2500);
    
    in_man_fourier_cal_mode = false;
    in_setup_mode = true;
    last_transition_ms = millis();
    reset_button_draw_cache();
    draw_resting_ui();
}

void draw_resting_ui() {
    int w = tft.width();
    int h = tft.height();
    
    tft.fillScreen(TFT_BLACK);
    
    if (selected_buoy_idx == -1) {
        if (in_track_settings_mode) {
            // --- Static Track Settings Screen: the buoy map, with the course controls under it ---
            tft.setTextPadding(0); // Stale padding from the nav screens would eat into the labels below
            tft.setTextColor(TFT_CYAN, TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextDatum(TC_DATUM);
            tft.drawString("TRACK SETTINGS", w / 2, 4);

            tft.drawFastHLine(15, 26, w - 30, TFT_WHITE);

            int cx = MAP_CX, cy = MAP_CY, r_max = MAP_R;

            // Draw crosshairs (plain N-up plot, no windrose rings)
            tft.drawFastHLine(cx - r_max - 5, cy, r_max * 2 + 10, TFT_DARKGREY);
            tft.drawFastVLine(cx, cy - r_max, r_max * 2 + 10, TFT_DARKGREY);

            // Draw cardinal direction markers, clear of the plot area on all four sides
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextDatum(BC_DATUM);
            tft.drawString("N", cx, cy - r_max - 2);
            tft.setTextDatum(TC_DATUM);
            tft.drawString("S", cx, cy + r_max + 2);
            tft.setTextDatum(MR_DATUM);
            tft.drawString("W", cx - r_max - 4, cy);
            tft.setTextDatum(ML_DATUM);
            tft.drawString("E", cx + r_max + 4, cy);

            // Plot the buoys and the start line straight away instead of waiting for the next
            // refresh tick. This also fills in the length and wind labels either side of the N.
            update_radar_map_dynamic();
            draw_track_lock_hint();

            // Buttons last: they read the length the plot just measured to decide what is live.
            draw_track_buttons(true);
        } else {
            // --- Static Menu Screen (240x320 Portrait) ---
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextDatum(TC_DATUM);
            tft.drawString("ROBOBUOY", w / 2, 15);
            tft.drawString("CONTROLLER", w / 2, 40);
            
            tft.drawFastHLine(15, 70, w - 30, TFT_WHITE);
            
            // Draw TRACK SETTINGS Button (Y: 210 to 245, X: 10 to 230) - as big as BUOY buttons and BLUE!
            tft.fillRoundRect(10, 210, w - 20, 35, 5, TFT_BLUE);
            tft.setTextColor(TFT_WHITE, TFT_BLUE);
            tft.setTextSize(2);
            tft.setTextDatum(MC_DATUM);
            tft.drawString("TRACK SETTINGS", w / 2, 227);
            
            // Draw Calibrate Touch Button centered (Y: 250 to 278, X: 30 to 210)
            tft.fillRoundRect(30, 250, 180, 28, 4, TFT_DARKGREY);
            tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
            tft.setTextSize(1);
            tft.setTextDatum(MC_DATUM);
            tft.drawString("CALIBRATE TOUCH", w / 2, 264);
            
            tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextDatum(BC_DATUM);
            
            // Draw LoRa status on its own line above the IP address
            tft.drawString("LoRa: 433M", w / 2, h - 22);
            
            IPAddress ip;

            if (WiFi.getMode() & WIFI_MODE_AP)
            {
                ip = WiFi.softAPIP();
            }
            else
            {
                ip = WiFi.localIP();
            }

            char ip_buf[48];
            sprintf(ip_buf, "IP: %s", ip.toString().c_str());
            tft.drawString(ip_buf, w / 2, h - 6);
        }
    } else {
        if (in_man_fourier_cal_mode) {
            // --- Static Manual Fourier Calibration Screen ---
            draw_mancal_static();
        } else if (in_setup_mode) {
            // --- Static Setup screen on Display ---
            draw_setup_static();
        } else if (in_mannav_mode) {
            // --- Static Manual Navigation Screen ---
            draw_mannav_static();
        } else {
            // --- Static Navigation/Compass Screen (Opened directly on tapping buoy!) ---
            draw_nav_static();
        }
    }
}

// Live GPS Fourier calibration progress, drawn on the ACTIONS page below the two start buttons.
// Fed by GPS_FOURIER_STATUS (cmd 90); see parse_buoy_packet() in buoy_data.cpp.
// gpscal_abort_t from RoboCompute.h, spelled out for the panel. The Top sends only the number.
static const char *gps_cal_abort_text(int code) {
    switch (code) {
        case 1:  return "no GPS fix";
        case 2:  return "Sub silent on serial";
        case 3:  return "GPS fix lost";
        case 4:  return "lost serial to Sub";
        case 5:  return "compass frozen";
        case 6:  return "drifted too far";
        case 7:  return "Sub sent no table";
        case 8:  return "never settled on heading";
        case 9:  return "leg too unstable";
        case 10: return "leg too slow";
        case 11: return "Sub refused table";
        case 12: return "Sub never confirmed";
        default: return "reason not reported";
    }
}

void draw_gps_cal_panel(BuoyData &b) {
    int w = tft.width();

    // The Top only reports while a run is active, so silence means no run. 15 s is comfortably
    // longer than the 5 s LoRa report interval, so a single dropped frame does not blank the panel.
    bool live = (b.cal_seen_ms != 0) && (millis() - b.cal_seen_ms < 15000);
    // An abort is the one report you must not miss, and it is also the last one the Top ever
    // sends - so the 15 s liveness window used to erase it and leave the panel reading "No
    // calibration running", which is exactly what a command that never arrived looks like.
    // Hold it on the glass until something else happens.
    bool aborted = (b.cal_seen_ms != 0) && (b.cal_step == 6);
    if (aborted) live = true;

    static int   c_idx = -1;
    static bool  c_live = false;
    static int   c_step = -1, c_leg = -1;
    static float c_dir = -999, c_dist = -999, c_err = -999, c_last = -999;
    static int c_abort = -1;
    if (!gps_cal_panel_dirty && c_idx == selected_buoy_idx && c_live == live &&
        c_step == b.cal_step && c_leg == b.cal_leg && c_dir == b.cal_cmd_dir &&
        c_dist == b.cal_dist && c_err == b.cal_err && c_last == b.cal_last_err &&
        c_abort == b.cal_abort) {
        return;
    }
    c_abort = b.cal_abort;
    gps_cal_panel_dirty = false;
    c_idx = selected_buoy_idx; c_live = live;
    c_step = b.cal_step; c_leg = b.cal_leg; c_dir = b.cal_cmd_dir;
    c_dist = b.cal_dist; c_err = b.cal_err; c_last = b.cal_last_err;

    tft.fillRect(8, 71, w - 16, 118, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
    char buf[64];
    if (!live) {
        // Draw selection helper text on the left
        tft.setTextDatum(TL_DATUM);
        tft.setTextSize(1);
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        tft.drawString("Tap a dot to", 14, 80);
        tft.drawString("select starting", 14, 94);
        tft.drawString("direction of", 14, 108);
        tft.drawString("the first leg.", 14, 122);
        
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.drawString("Calibration will", 14, 144);
        tft.drawString("proceed CW.", 14, 158);

        // Draw the 8 dots in a circle on the right side
        int cx = 165;
        int cy = 130;
        int r_dots = 30;
        int r_arrow = 48;

        // Draw outer clockwise circular arrow
        tft.drawCircle(cx, cy, r_arrow, TFT_DARKGREY);
        tft.drawLine(cx + r_arrow, cy, cx + r_arrow - 4, cy - 5, TFT_DARKGREY);
        tft.drawLine(cx + r_arrow, cy, cx + r_arrow + 4, cy - 5, TFT_DARKGREY);

        // Draw the 8 dots
        for (int i = 0; i < 8; i++) {
            int angle_deg = i * 45;
            float angle_rad = angle_deg * (float)M_PI / 180.0f;
            int x = cx + (int)(r_dots * sin(angle_rad));
            int y = cy - (int)(r_dots * cos(angle_rad));

            bool is_selected = (angle_deg == (int)b.cal_start_heading);
            if (is_selected) {
                // Draw selected dot in bright green with a white ring
                tft.fillCircle(x, y, 5, TFT_GREEN);
                tft.drawCircle(x, y, 7, TFT_WHITE);
            } else {
                // Draw unselected dot in dark grey
                tft.fillCircle(x, y, 3, TFT_DARKGREY);
                tft.drawCircle(x, y, 4, TFT_BLACK);
            }

            // Draw directional labels (N, NE, E, SE, S, SW, W, NW) outside the dots
            int r_label = r_dots + 11;
            int lx = cx + (int)(r_label * sin(angle_rad));
            int ly = cy - (int)(r_label * cos(angle_rad));
            
            tft.setTextSize(1);
            tft.setTextColor(is_selected ? TFT_GREEN : TFT_DARKGREY, TFT_BLACK);
            tft.setTextDatum(MC_DATUM);
            
            const char* label = "";
            switch (angle_deg) {
                case 0:   label = "N"; break;
                case 45:  label = "NE"; break;
                case 90:  label = "E"; break;
                case 135: label = "SE"; break;
                case 180: label = "S"; break;
                case 225: label = "SW"; break;
                case 270: label = "W"; break;
                case 315: label = "NW"; break;
            }
            tft.drawString(label, lx, ly);
        }

        // Show start heading in center
        tft.setTextSize(1);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        char s_buf[16];
        sprintf(s_buf, "%03d", (int)b.cal_start_heading);
        tft.drawString("START", cx, cy - 6);
        tft.drawString(s_buf, cx, cy + 6);

        // Restore global TFT state
        tft.setTextDatum(MC_DATUM);
        tft.setTextSize(1);
        return;
    }

    // gpscal_step_t from RoboCompute.h
    const char *phase;
    uint16_t phaseCol = TFT_YELLOW;
    switch (b.cal_step) {
        case 1:  phase = "READING TABLE"; break;
        case 2:  phase = "SETTLING";      break;
        case 3:  phase = "MEASURING";     break;
        case 4:  phase = "STORING TABLE"; break;
        case 5:  phase = "DONE";    phaseCol = TFT_GREEN; break;
        case 6:  phase = "ABORTED"; phaseCol = TFT_RED;   break;
        default: phase = "IDLE";    phaseCol = TFT_DARKGREY; break;
    }

    tft.setTextSize(2);
    tft.setTextColor(phaseCol, TFT_BLACK);
    tft.drawString(phase, 14, 78);

    if (b.cal_step == 6) {
        // Distance, live error and the progress bar all describe a leg that is no longer being
        // sailed. The reason is the only thing worth the space.
        tft.setTextSize(1);
        tft.setTextColor(TFT_RED, TFT_BLACK);
        tft.drawString(gps_cal_abort_text(b.cal_abort), 14, 104);
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        sprintf(buf, "stopped on leg %d of 8", b.cal_leg + 1);
        tft.drawString(buf, 14, 124);
        tft.drawString("fix the cause, then run it again", 14, 144);
        tft.setTextDatum(MC_DATUM);
        return;
    }

    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    sprintf(buf, "Leg %d of 8   commanded %03.0f deg", b.cal_leg + 1, b.cal_cmd_dir);
    tft.drawString(buf, 14, 104);

    sprintf(buf, "Sailed %0.0f m of 100 m", b.cal_dist);
    tft.drawString(buf, 14, 122);

    // Progress bar for the current leg
    int barW = (int)((b.cal_dist / 100.0f) * (w - 32));
    if (barW < 0) barW = 0;
    if (barW > w - 32) barW = w - 32;
    tft.drawRect(14, 134, w - 32, 8, TFT_DARKGREY);
    tft.fillRect(15, 135, barW ? barW - 2 : 0, 6, TFT_CYAN);

    // The live error is held at 0 by the Top until the leg has run 10 m, because below that the
    // displacement is inside GPS noise and the bearing is meaningless. Say so rather than showing
    // a confident 0.0.
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    if (b.cal_step == 3 && b.cal_dist < 10.0f) {
        tft.drawString("Live error: settling...", 14, 152);
    } else {
        sprintf(buf, "Live error: %+0.1f deg", b.cal_err);
        tft.drawString(buf, 14, 152);
    }

    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    if (b.cal_leg > 0 || b.cal_step >= 4) {
        sprintf(buf, "Last completed leg: %+0.1f deg", b.cal_last_err);
        tft.drawString(buf, 14, 170);
    } else {
        tft.drawString("No leg completed yet", 14, 170);
    }

    // Restore what the rest of update_setup_dynamic() expects. TFT_eSPI datum is global state and
    // this function is called from the middle of that routine.
    tft.setTextDatum(MC_DATUM);
    tft.setTextSize(1);
}

void update_setup_dynamic() {
    int w = tft.width();
    int idx = selected_buoy_idx;
    BuoyData &b = buoys[idx];
    char buf[64];
    
    // Page 4 carries no setup values at all, only the calibration actions and their progress, so
    // it must not be held behind the SETUPDATA reply the way the parameter pages are.
    if (!setup_data_loaded && !setup_is_action_page(setup_page)) {
        // Display beautiful loading overlay while awaiting the NMEA response packet from buoy
        tft.setTextSize(2);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        char lbuf[48];
        bool gaveUp = (setup_query_tries >= SETUP_QUERY_MAX_TRIES);
        if (gaveUp) {
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.drawString("NO REPLY", w / 2, 85);
            tft.drawString("FROM BUOY", w / 2, 115);
        } else {
            tft.drawString("LOADING DATA", w / 2, 85);
            tft.drawString("FROM BUOY...", w / 2, 115);
        }
        
        tft.setTextSize(1);
        tft.setTextColor(gaveUp ? TFT_ORANGE : TFT_DARKGREY, TFT_BLACK);
        tft.setTextPadding(200); // the attempt counter shortens as it climbs; pad so it self-erases
        if (gaveUp) {
            tft.drawString("Tap BACK, then SETUP to retry", w / 2, 160);
        } else {
            sprintf(lbuf, "Awaiting reply - attempt %d of %d", setup_query_tries, SETUP_QUERY_MAX_TRIES);
            tft.drawString(lbuf, w / 2, 160);
        }
        tft.setTextPadding(0);
        return;
    }
    
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    
    // Draw the 8 grid boxes of the current page (4 rows x 2 columns). Slots with no name are
    // gaps in the layout and are skipped entirely - drawing them used to leave a stray value in
    // the empty boxes at the bottom of the docking page.
    char vbuf[24];
    for (int i = 0; i < 8; i++) {
        int r = i % 4;       // Row index (0 to 3)
        int c = i / 4;       // Col index (0 to 1)
        int x = (c == 0) ? 10 : 122;
        int y = 35 + r * 36; // Double-height boxes! (32px height, 4px gap = 36px offset)

        int global_idx = setup_page * 8 + i;
        if (!setup_slot_used(global_idx)) continue;

        // Highlight selected parameter box in Yellow, otherwise draw in Dark Grey
        uint16_t boxColor = (global_idx == selected_param_idx) ? TFT_YELLOW : TFT_DARKGREY;
        uint16_t textColor = (global_idx == selected_param_idx) ? TFT_YELLOW : TFT_WHITE;

        tft.drawRoundRect(x, y, 108, 32, 4, boxColor);
        tft.setTextColor(textColor, TFT_BLACK);

        if (setup_slot_is_bool(global_idx)) {
            bool on = setup_bool_get(b, global_idx);
            tft.setTextColor(on ? TFT_GREEN : textColor, TFT_BLACK);
            sprintf(buf, "%s %s", SETUP_NAMES[global_idx], on ? "YES" : "NO");
            tft.drawString(buf, x + 54, y + 16);
        } else if (global_idx == S_GPSSTILL || global_idx == S_MANCAL) {
            // Deliberately two separate entries rather than one box with a toggle: picking the
            // wrong mode is the one way this calibration can leave the compass worse than it
            // found it, so the choice has to be explicit at the moment you start it.
            tft.setTextColor(global_idx == S_GPSSTILL ? TFT_GREEN : TFT_ORANGE, TFT_BLACK);
            tft.drawString(SETUP_NAMES[global_idx], x + 54, y + 16);
        } else if (global_idx == S_REBOOT) {
            tft.setTextColor(TFT_ORANGE, TFT_BLACK);
            tft.drawString(SETUP_NAMES[global_idx], x + 54, y + 16);
        } else if (setup_slot_is_action(global_idx)) {
            tft.drawString(SETUP_NAMES[global_idx], x + 54, y + 16);
        } else {
            setup_value_text(b, global_idx, vbuf, sizeof(vbuf));
            sprintf(buf, "%s %s", SETUP_NAMES[global_idx], vbuf);
            tft.drawString(buf, x + 54, y + 16);
        }
    }

    if (setup_page == 5) draw_gps_cal_panel(b);
    
    // Draw currently selected value in big text in center of adjustment row (Y: 195 to 225)
    tft.setTextSize(2);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    if (setup_is_action_page(setup_page)) {
        // Clear the strip between the two buttons (they occupy x 15..75 and x 165..225).
        tft.fillRect(78, 196, 84, 28, TFT_BLACK);
        bool armed = setup_slot_is_action(selected_param_idx);
        tft.setTextDatum(MC_DATUM); // set, not inherited - see the note above draw_gps_cal_panel()
        tft.setTextSize(1); // "TAP + TO RUN" at size 2 is 144 px and the gap is 84
        uint16_t col = TFT_DARKGREY;
        if (armed) {
            col = (selected_param_idx == S_GPSSTILL) ? TFT_GREEN
                : (selected_param_idx == S_MANCAL || selected_param_idx == S_REBOOT) ? TFT_ORANGE
                : TFT_YELLOW;
        }
        tft.setTextColor(col, TFT_BLACK);
        tft.drawString(armed ? "TAP + TO RUN" : "SELECT ACTION", 120, 210);
    } else if (setup_slot_is_bool(selected_param_idx)) {
        tft.drawString(setup_bool_get(b, selected_param_idx) ? "YES" : "NO", 115, 210);
    } else {
        setup_value_text(b, selected_param_idx, buf, sizeof(buf));
        tft.drawString(buf, 115, 210);
    }
    
    // Draw real-time traffic dots (UDP and LoRa) in the top-right corner of the setup page header (Y: 13)
    uint16_t udpDotColor = traffic_dot_color(last_udp_tx_ms, last_udp_sel_blink_ms, 100, TFT_GREEN);
    tft.fillCircle(218, 13, 4, udpDotColor);
    uint16_t loraDotColor = traffic_dot_color(last_lora_tx_ms, last_lora_blink_ms, 300, TFT_CYAN);
    tft.fillCircle(232, 13, 4, loraDotColor);
}

void update_nav_dynamic() {
    int w = tft.width();
    int h = tft.height();
    int idx = selected_buoy_idx;
    BuoyData &b = buoys[idx];
    
    // State caches to completely eliminate steady-state flicker on every refresh
    static int last_buoy_idx = -1;
    static float last_bb_power = -999;
    static float last_sb_power = -999;
    static float last_battery_v = -1.0;
    static String last_nav_status = "";
    static bool was_mannav_mode = false;
    static bool was_setup_mode = false;
    static String last_gps_fix = "";
    
    // Reset caches on buoy selection or screen mode change
    if (selected_buoy_idx != last_buoy_idx || in_mannav_mode != was_mannav_mode || in_setup_mode != was_setup_mode) {
        last_buoy_idx = selected_buoy_idx;
        was_mannav_mode = in_mannav_mode;
        was_setup_mode = in_setup_mode;
        
        last_bb_power = -999;
        last_sb_power = -999;
        last_battery_v = -1.0;
        last_nav_status = "";
        last_gps_fix = "";
        
        old_mag_dir = -999.0;
        old_tg_dir = -999.0;
        old_wind_dir = -999.0;
    }
    
    // Clear all dynamic text fields ONCE when transition into/out of IDLE status occurs
    if (b.status != last_nav_status) {
        last_nav_status = b.status;
        tft.fillRect(34, 45, 60, 12, TFT_BLACK);     // Clear top-left (Dis)
        tft.fillRect(150, 45, 56, 25, TFT_BLACK);    // Clear top-right (Wnd & Std)
        tft.fillRect(34, 135, 60, 12, TFT_BLACK);    // Clear bottom-left (Tg)
        tft.fillRect(150, 135, 56, 12, TFT_BLACK);   // Clear bottom-right (Mag)
    }

    // --- 1. Update Speedbars (BB Bow & SB Stern) ---
    int mid_y = 108;
    
    // Only redraw speedbars on value change to prevent constant high-frequency flickering
    if (b.bb_power != last_bb_power) {
        last_bb_power = b.bb_power;
        tft.fillRect(16, 59, 13, 98, TFT_BLACK); // Clear inner area (Lowered to starting Y: 59)
        if (b.bb_power > 0) {
            int fill_h = (b.bb_power * 49) / 100;
            tft.fillRect(16, mid_y - fill_h, 13, fill_h, TFT_GREEN);
        } else if (b.bb_power < 0) {
            int fill_h = (-b.bb_power * 49) / 100;
            tft.fillRect(16, mid_y, 13, fill_h, TFT_RED);
        }
        tft.drawFastHLine(15, mid_y, 15, TFT_DARKGREY); // Reset centerline
    }
    
    if (b.sb_power != last_sb_power) {
        last_sb_power = b.sb_power;
        tft.fillRect(211, 59, 13, 98, TFT_BLACK); // Clear inner area
        if (b.sb_power > 0) {
            int fill_h = (b.sb_power * 49) / 100;
            tft.fillRect(211, mid_y - fill_h, 13, fill_h, TFT_GREEN);
        } else if (b.sb_power < 0) {
            int fill_h = (-b.sb_power * 49) / 100;
            tft.fillRect(211, mid_y, 13, fill_h, TFT_RED);
        }
        tft.drawFastHLine(210, mid_y, 15, TFT_DARKGREY); // Reset centerline
    }
    
    // Allocate 128 bytes on the stack for buffer formatting (fixes stack smashing watchdog reboots!)
    char buf[128];
    
    // Print BB and SB percentage text below speedbars using text padding to eliminate flicker
    tft.setTextSize(2); // Increased speedbar percentage text to font size 2!
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TC_DATUM);
    tft.setTextPadding(45); // Increased padding to 45px for size 2 text!
    
    sprintf(buf, "%0.0f%%", b.bb_power);
    tft.drawString(buf, 22, 162); // Lowered to Y: 162
    
    sprintf(buf, "%0.0f%%", b.sb_power);
    tft.drawString(buf, 217, 162); // Lowered to Y: 162
    
    // --- 2. Update Trigonometric Compass Rose Arrows ---
    float current_tg = (b.status != "IDLE") ? b.tg_dir : -999.0;
    float current_wind = (b.status != "IDLE") ? b.wind_dir : -999.0;

    if (b.mag_dir != old_mag_dir || current_tg != old_tg_dir || current_wind != old_wind_dir) {
        // Erase old thick arrows first in TFT_BLACK to prevent trails!
        if (old_mag_dir != -999.0) draw_compass_arrow(120, 100, 42, old_mag_dir, TFT_BLACK);
        if (old_tg_dir != -999.0) draw_compass_arrow(120, 100, 36, old_tg_dir, TFT_BLACK);
        if (old_wind_dir != -999.0) draw_compass_arrow(120, 100, 30, old_wind_dir, TFT_BLACK);
        
        // Save new angles for the next erasure cycle
        old_mag_dir = b.mag_dir;
        old_tg_dir = current_tg;
        old_wind_dir = current_wind;
        
        // Draw the new wide dynamic arrows
        draw_compass_arrow(120, 100, 42, old_mag_dir, TFT_GREEN);
        if (old_tg_dir != -999.0) draw_compass_arrow(120, 100, 36, old_tg_dir, TFT_RED);
        if (old_wind_dir != -999.0) draw_compass_arrow(120, 100, 30, old_wind_dir, TFT_CYAN);
        
        // Redraw center pivot dot to prevent overlap gaps
        tft.fillCircle(120, 100, 3, TFT_WHITE);
    }
    
    // --- 3. Update Cockpit Telemetry Fields around Compass Rose ---
    tft.setTextSize(1);
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);

    if (b.status != "IDLE") {
        // Top-Left: Target Distance (Dis) aligned with 4-pixel padding to Left Speedbar
        tft.setTextDatum(TL_DATUM);
        tft.setTextPadding(50); // Reduced padding to 50px (smaller footprint, no circle overlap!)
        sprintf(buf, "Dis:%0.1fm", b.tg_dist);
        tft.drawString(buf, 34, 45);
        
        // Top-Right: Wind Direction and Standard Deviation (Wnd & Std) aligned perfectly flush against Right Speedbar (X: 206)
        tft.setTextDatum(TR_DATUM);
        tft.setTextPadding(44); // Reduced padding to 44px
        sprintf(buf, "Wnd:%0.0f", b.wind_dir);
        tft.drawString(buf, 206, 45);
        
        tft.setTextPadding(44); // Reduced padding to 44px
        sprintf(buf, "Std:%0.0f", b.wind_std);
        tft.drawString(buf, 206, 58);
        
        // Bottom-Left: Target Direction (TgD) aligned with 4-pixel padding to Left Speedbar
        tft.setTextDatum(TL_DATUM);
        tft.setTextPadding(50); // Reduced padding to 50px
        sprintf(buf, "Tg:%0.0f", b.tg_dir);
        tft.drawString(buf, 34, 135);
    }
    
    // Bottom-Right: Magnetic direction (Mag) aligned perfectly flush against Right Speedbar (X: 206) - ALWAYS drawn!
    tft.setTextDatum(TR_DATUM);
    tft.setTextPadding(44); // Reduced padding to 44px
    sprintf(buf, "Mag:%0.0f", b.mag_dir);
    tft.drawString(buf, 206, 135);
    
    // --- 4. Update Dynamic Voltage Bar ---
    // Only redraw the voltage bar if the voltage value actually changed
    if (b.battery_v != last_battery_v) {
        last_battery_v = b.battery_v;
        // Clear inner area
        tft.fillRect(51, 173, 138, 8, TFT_BLACK);
        
        float v = b.battery_v;
        if (v < 17.0) v = 17.0; // Min limit scaled to 17V!
        if (v > 25.0) v = 25.0;
        // Map voltage range [17.0V to 25.0V] into [0 to 138] pixels
        int fill_w = map(v * 10, 170, 250, 0, 138);
        
        // Dynamic color coding
        uint16_t barColor = TFT_GREEN;
        if (b.battery_v < 19.5) barColor = TFT_RED;
        else if (b.battery_v < 22.0) barColor = TFT_YELLOW;
        
        tft.fillRect(51, 173, fill_w, 8, barColor);
    }
    
    // Print current numeric value and load current below the bar at Y = 184 in BIGGER FONT (Size 2!)
    tft.setTextSize(2); // BIGGER FONT!
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TC_DATUM);
    tft.setTextPadding(w - 20); // Center-aligned, pads both left and right!
    sprintf(buf, "%0.1fV  %0.0f%%  %0.1fA", b.battery_v, b.battery_pct, b.current);
    tft.drawString(buf, w / 2, 184);
    
    // --- 5. Update Status on Left Column in BIGGER FONT (Size 2!) - Colored CYAN to match top NAV! ---
    tft.setTextColor(TFT_CYAN, TFT_BLACK); // Changed to TFT_CYAN to match top NAV header exactly!
    tft.setTextSize(2); // BIGGER FONT!
    tft.setTextDatum(TL_DATUM);
    tft.setTextPadding(110);
    sprintf(buf, "%-8s", b.status.c_str());
    tft.drawString(buf, 15, 214);
    
    tft.setTextPadding(0); // Reset padding
    
    // --- 6. Draw Ticked Status of Stacked UDP / LoRa Checkboxes (UDP at Y: 204 & LoRa lowered to Y: 222!) ---
    if (udp_enabled) {
        tft.fillRect(127, 206, 7, 7, TFT_GREEN);
    } else {
        tft.fillRect(127, 206, 7, 7, TFT_BLACK);
    }
    
    if (lora_enabled) {
        tft.fillRect(127, 224, 7, 7, TFT_GREEN); // Checkbox lowered to Y: 222 (fill offset to 224)
    } else {
        tft.fillRect(127, 224, 7, 7, TFT_BLACK);
    }
    
    // --- 7. Update Blinking Telemetry Indicators (Top-Right, UDP sits left of LoRa) ---
    // Both only blink for traffic belonging to THIS buoy; red while transmitting.
    uint16_t udpDotColor = traffic_dot_color(last_udp_tx_ms, last_udp_sel_blink_ms, 100, TFT_GREEN);
    tft.fillCircle(212, 13, 4, udpDotColor);

    uint16_t loraDotColor = traffic_dot_color(last_lora_tx_ms, last_lora_blink_ms, 300, TFT_CYAN);
    tft.fillCircle(230, 13, 4, loraDotColor);

    // --- 8. Update Live LoRa RSSI in Top-Left Corner of Header (Y: 5, X: 5) ---
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextSize(2); // One step bigger (Size 2!)
    tft.setTextDatum(TL_DATUM);
    tft.setTextPadding(55); // Expanded padding (55px) to fully cover -111 or -120 without ghosting!
    if (b.lora_rssi != -999) {
        sprintf(buf, "%d", b.lora_rssi);
    } else {
        sprintf(buf, "---");
    }
    tft.drawString(buf, 5, 5);
    
    // --- 9. Dynamic Redraw of LOCK and DOCK buttons based on GPS Fix ---
    if (b.gps_fix != last_gps_fix) {
        last_gps_fix = b.gps_fix;
        bool has_fix = (b.gps_fix == "3D" || b.gps_fix == "2D");
        
        tft.setFreeFont(&FreeSansBold9pt7b); // Use bold GFX font
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        
        if (has_fix) {
            // Draw active colored buttons
            tft.fillRoundRect(10, 240, 70, 35, 4, TFT_DARKGREEN);
            tft.setTextColor(TFT_WHITE, TFT_DARKGREEN);
            tft.drawString("LOCK", 45, 257);
            
            tft.fillRoundRect(85, 240, 70, 35, 4, TFT_YELLOW);
            tft.setTextColor(TFT_BLACK, TFT_YELLOW);
            tft.drawString("DOCK", 120, 257);
        } else {
            // Draw disabled grey buttons
            tft.fillRoundRect(10, 240, 70, 35, 4, TFT_DARKGREY);
            tft.setTextColor(TFT_LIGHTGREY, TFT_DARKGREY);
            tft.drawString("LOCK", 45, 257);
            
            tft.fillRoundRect(85, 240, 70, 35, 4, TFT_DARKGREY);
            tft.setTextColor(TFT_LIGHTGREY, TFT_DARKGREY);
            tft.drawString("DOCK", 120, 257);
        }
        
        tft.setFreeFont(NULL); // Restore default font
    }
}

// One-letter status badge for a menu button: I idle, L locked, D docked.
//
// LOCKING and DOCKING report the letter they are heading for, but settle_out comes back false
// so the caller can colour them differently. That distinction earns its keep: a buoy that
// cannot get a GPS fix sits in LOCKING indefinitely (RoboTop only promotes it to LOCKED once
// gpsFix is true), and a badge that read a plain "L" there would claim it was holding station
// when it has not even started. Anything else still gets a letter rather than a blank.
static char status_badge(const String &status, bool &settled_out) {
    settled_out = true;
    if (status == "IDLE")      return 'I';
    if (status == "LOCKED")    return 'L';
    if (status == "DOCKED")    return 'D';

    settled_out = false;
    if (status == "LOCKING")   return 'L';
    if (status == "DOCKING")   return 'D';
    if (status == "REMOTE")    return 'R';
    if (status == "GPS CALIB") return 'C';
    return '?';
}

void update_dynamic_ui() {
    int w = tft.width();
    int h = tft.height();
    unsigned long now = millis();
    
    // Check if buoys went offline (> 60 seconds since last transmission)
    for (int i = 0; i < 3; i++) {
        if (buoys[i].id != "") {
            if (now - buoys[i].last_seen_ms > 60000) { // 60 seconds timeout
                buoys[i].present = false;
            }
        }
    }
    
    if (selected_buoy_idx == -1) {
        if (in_track_settings_mode) {
            // Track Settings Screen: replot the buoy markers and the start line, then let the
            // buttons pick up any change in what is possible (a third buoy arriving enables
            // TRACK, the fleet going quiet greys the lot out).
            // Refresh at 1 Hz (not the 250 ms UI tick) to keep the markers from flickering.
            static unsigned long last_radar_update_ms = 0;
            if (now - last_radar_update_ms > 1000) {
                last_radar_update_ms = now;
                update_radar_map_dynamic();
                draw_track_lock_hint();
                draw_track_buttons(false);
            }
            return;
        }
        
        // --- Menu Screen (Optimized with State Caching to prevent redraw flicker) ---
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        
        // Colours follow the course, not the slot: green until the fleet is locked, then starboard
        // green / port red / head blue. Resolved once for all three buttons - the roles are decided
        // by the fleet as a whole, so working them out per button would be three times the work for
        // the same answer.
        int roles[3];
        compute_buoy_roles(roles);

        for (int i = 0; i < 3; i++) {
            int y = 75 + i * 45; // Compact spacing starting at Y: 75!
            int current_present = (buoys[i].id == "") ? -1 : (buoys[i].present ? 1 : 0);
            uint16_t btn_col = buoy_role_color(i, roles);
            uint16_t txt_col = buoy_role_text_color(btn_col);
            
            static String last_drawn_ips[3] = {"", "", ""};
            // The status badge changes on its own, with the ID, presence and IP all unchanged -
            // without it in the cache key the letter would be painted once and then never
            // updated again.
            static String last_drawn_status[3] = {"", "", ""};
            // Kept OUT of the cache key on purpose. The RSSI changes with every LoRa packet, and
            // putting it in the key would repaint the whole 40 px button at packet rate - which
            // is the flicker the caching exists to avoid. It gets its own small repaint below.
            static String last_drawn_rssi[3] = {"", "", ""};
            // The colour changes when the fleet locks, or when the wind swings the line round, with
            // the id, presence, IP and status all unchanged - so it has to be in the cache key too,
            // or a buoy would keep whatever colour it was first painted.
            static uint16_t last_drawn_col[3] = {0, 0, 0};

            // Only draw/redraw if the ID, online presence, IP address or status changed!
            if (buoys[i].id != last_drawn_ids[i] || current_present != last_drawn_present[i] ||
                buoys[i].ip_addr != last_drawn_ips[i] || buoys[i].status != last_drawn_status[i] ||
                btn_col != last_drawn_col[i]) {
                last_drawn_col[i] = btn_col;
                last_drawn_ids[i] = buoys[i].id;
                last_drawn_present[i] = current_present;
                last_drawn_ips[i] = buoys[i].ip_addr;
                last_drawn_status[i] = buoys[i].status;
                
                // Clear only this button's area first (height 40!)
                tft.fillRect(10, y, w - 20, 40, TFT_BLACK);
                
                if (current_present == -1) {
                    // Empty slot
                    tft.drawRoundRect(10, y, w - 20, 40, 5, TFT_DARKGREY);
                    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
                    tft.setTextSize(2);
                    tft.drawString("Buoy" + String(i+1) + ": [Waiting]", w / 2, y + 20);
                } else if (current_present == 1) {
                    // Present: green, or the role colour once the whole fleet is locked
                    tft.fillRoundRect(10, y, w - 20, 40, 5, btn_col);
                    tft.setTextColor(txt_col, btn_col);
                    
                    // Two rows of size 2 text in the 40 px button, each with a left and a right
                    // field. Everything is aligned to the button edges rather than centred, so
                    // the four columns line up down the whole list and can be read at a glance.
                    //   top:    Buoy n: <id>        <status>
                    //   bottom: <ip / LoRa only>    <rssi>
                    tft.setTextSize(2);

                    tft.setTextDatum(TL_DATUM);
                    tft.drawString("Buoy" + String(i+1) + ":" + buoys[i].id, BTN_PAD_L, y + 3);

                    bool settled = true;
                    char badge = status_badge(buoys[i].status, settled);
                    // Normal text colour for a state the buoy has actually reached, a contrasting
                    // one while it is still on its way there. Red reads as "not yet" on a green
                    // button but vanishes on a red one, so the port hand button uses yellow.
                    uint16_t badge_col = settled ? txt_col
                                                 : ((btn_col == TFT_GREEN || btn_col == TFT_YELLOW)
                                                        ? TFT_RED : TFT_YELLOW);
                    tft.setTextColor(badge_col, btn_col);
                    tft.setTextDatum(TR_DATUM);
                    tft.drawString(String(badge), w - BTN_PAD_R, y + 3);
                    tft.setTextColor(txt_col, btn_col);

                    String conn_str = (buoys[i].ip_addr == "") ? "LoRa only" : buoys[i].ip_addr;
                    tft.setTextDatum(TL_DATUM);
                    tft.drawString(conn_str, BTN_PAD_L, y + 20);

                    // The RSSI is painted below, outside this cached block, because it moves with
                    // every LoRa packet. Force that to happen now: this branch has just wiped the
                    // whole button green, so whatever it last drew is gone from the glass.
                    last_drawn_rssi[i] = "\x01";

                    // Leave the datum as this branch always used to, so the [Waiting] / [Offline]
                    // rows below still land where they did before.
                    tft.setTextDatum(TC_DATUM);
                } else {
                    // Offline: greyed out, NOT red. Red is the port hand mark on this screen now,
                    // and a dead buoy must not read as one that is holding a station.
                    tft.fillRoundRect(10, y, w - 20, 40, 5, TFT_DARKGREY);
                    tft.setTextColor(TFT_LIGHTGREY, TFT_DARKGREY);
                    tft.setTextSize(2);
                    tft.drawString("Buoy" + String(i+1) + ": [Offline]", w / 2, y + 20);
                }
            }

            // LoRa RSSI, bottom right, repainted on its own so a changing signal level does not
            // drag the whole button through a clear-and-redraw. Blank when the buoy has not been
            // heard over LoRa at all: lora_rssi is -999 then, and printing that would read as a
            // measurement rather than the absence of one.
            if (current_present == 1) {
                String rssi_str = (buoys[i].lora_rssi == -999) ? "" : String(buoys[i].lora_rssi);
                if (rssi_str != last_drawn_rssi[i]) {
                    last_drawn_rssi[i] = rssi_str;
                    // Expanded clearing rectangle (56px width) to guarantee full erasure of -111
                    tft.fillRect(w - BTN_PAD_R - 56, y + 19, 56, 17, btn_col);
                    tft.setTextColor(txt_col, btn_col);
                    tft.setTextSize(2);
                    tft.setTextDatum(TR_DATUM);
                    tft.drawString(rssi_str, w - BTN_PAD_R, y + 20);
                    tft.setTextDatum(TC_DATUM);
                }
            }
        }

        // Both traffic indicators share one right-hand column (X: 230), each vertically
        // centred on its own status line: LoRa on the "LoRa: 433M" line, UDP on the IP line.
        // RED while transmitting, cyan / green on reception.
        uint16_t globalLoraDotColor = traffic_dot_color(last_lora_tx_ms, last_global_lora_blink_ms, 300, TFT_CYAN);
        tft.fillCircle(230, h - 29, 4, globalLoraDotColor);

        uint16_t globalUdpDotColor = traffic_dot_color(last_udp_tx_ms, last_udp_blink_ms, 100, TFT_GREEN);
        tft.fillCircle(230, h - 13, 4, globalUdpDotColor);
    } else {
        if (in_man_fourier_cal_mode) {
            // --- Manual Fourier Calibration Screen ---
            update_mancal_dynamic();
        } else if (in_setup_mode) {
            // --- Setup Dynamic values on Display ---
            update_setup_dynamic();
        } else if (in_mannav_mode) {
            // --- Manual Navigation Screen ---
            update_mannav_dynamic();
        } else {
            // --- Navigation Screen (Direct dashboard!) ---
            update_nav_dynamic();
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Setup RGB LED
    initRGBled();
    ChangeRGBColor(RGB_COLOR_2); // Start with a Green LED

    // Initialize display and touch controllers
    init_display();
    init_touch();

    // The ROBOBUOY splash owns the screen for the rest of setup(). Both radios report their
    // progress into its status band instead of drawing their own text screens, so there is one
    // picture from power-on until the dashboard appears rather than three in a row.
    boot_screen_begin();

    // Setup WiFi, OTA & UDP listener
    boot_screen_status("WIFI");
    init_wifi_and_ota();

    // Initialize LoRa radio using MicroSD card SPI pins
    boot_screen_status("LORA");
    init_lora();

    // Initialize standard random seed using ESP32's onboard hardware True Random Number Generator (TRNG)
    // to avoid ADC2/Wi-Fi hardware driver conflicts and crashes!
    randomSeed(esp_random());

    // Leave the address up for the dwell rather than a "READY" that says nothing: whether we
    // joined home or became Robo_WiFi, this is the one line worth reading off the splash.
    String addr = (WiFi.getMode() & WIFI_MODE_AP) ? WiFi.softAPIP().toString()
                                                  : WiFi.localIP().toString();
    boot_screen_status(addr.c_str());

    // Holds the splash until it has been up long enough to read, then goes inert - after this
    // every boot_screen_* call is a no-op and the screen belongs to the dashboard.
    boot_screen_end();

    // Initially draw the resting menu dashboard
    draw_resting_ui();
}

void loop() {
    handle_ota();

    if (in_calibration_mode) {
        handle_touch_calibration();
        delay(20);
        return;
    }

    handle_wifi_clients();

    // Process incoming LoRa telemetry packets
    check_lora_packets();

    // Trigger dynamic interface redraws on screen state transitions
    if (selected_buoy_idx != lastKnownState || in_setup_mode != lastSetupState || in_mannav_mode != lastMannavState || in_man_fourier_cal_mode != lastManFourierCalState || setup_data_loaded != lastLoadedState || setup_page != lastSetupPage) {
        lastKnownState = selected_buoy_idx;
        lastSetupState = in_setup_mode;
        lastMannavState = in_mannav_mode;
        lastManFourierCalState = in_man_fourier_cal_mode;
        lastLoadedState = setup_data_loaded;
        lastSetupPage = setup_page;
        reset_button_draw_cache(); // Clear button draw cache to force complete redraw
        draw_resting_ui();
    }

    // Process and sense touchscreen button touches (MAPPED for 240x320 Portrait!)
    int touchX, touchY;
    if (get_touch_point(touchX, touchY)) {
        Serial.printf("Touch at: X=%d, Y=%d\n", touchX, touchY);
        
        // CRITICAL DEBOUNCE: Ignore all touch inputs for 500ms after any screen transition
        // to prevent touch event leakage/propagation and accidental button clicks!
        if (millis() - last_transition_ms < 500) {
            delay(20);
            return;
        }
        
        if (selected_buoy_idx == -1) {
            if (in_track_settings_mode) {
                // --- TRACK SETTINGS SCREEN TOUCH INTERACTION ---
                // Row 1: -  START  TRACK  +      Row 2: BACK  EXECUTE
                // Every branch re-tests the same condition draw_track_buttons() greys the button
                // on, so a tap that lands on a dark button does nothing at all rather than firing
                // a course computation the fleet is too small to carry out.
                bool row1 = (touchY >= TS_ROW1_Y && touchY <= TS_ROW1_Y + TS_BTN_H);
                bool row2 = (touchY >= TS_ROW2_Y && touchY <= TS_ROW2_Y + TS_BTN_H);
                int fleet = count_buoys_present();
                int locked = count_buoys_locked();

                // MINUS: shorten the start line by 5 m
                if (row1 && touchX >= TS_MINUS_X && touchX <= TS_MINUS_X + TS_MINUS_W) {
                    if (fleet >= 2 && track_line_measured()) {
                        float base = (track_line_tgt_m > 0) ? track_line_tgt_m : track_line_cur_m;
                        track_line_tgt_m = max(TS_LINE_STEP_M, base - TS_LINE_STEP_M);
                        track_settings_show_pending();
                        delay(150); // One step per tap: the touch is polled far faster than this
                    }
                }
                // PLUS: lengthen the start line by 5 m
                else if (row1 && touchX >= TS_PLUS_X && touchX <= TS_PLUS_X + TS_PLUS_W) {
                    if (fleet >= 2 && track_line_measured()) {
                        float base = (track_line_tgt_m > 0) ? track_line_tgt_m : track_line_cur_m;
                        track_line_tgt_m = base + TS_LINE_STEP_M;
                        track_settings_show_pending();
                        delay(150);
                    }
                }
                // START: square the start line to the wind (COMPUTESTART, needs both end buoys)
                else if (row1 && touchX >= TS_START_X && touchX <= TS_START_X + TS_START_W) {
                    if (locked >= 2) {
                        track_settings_compute(62);
                    }
                }
                // TRACK: lay out the whole course (COMPUTETRACK, needs the third buoy as the mark)
                else if (row1 && touchX >= TS_TRACK_X && touchX <= TS_TRACK_X + TS_TRACK_W) {
                    if (locked >= 3) {
                        track_settings_compute(63);
                    }
                }
                // BACK: a pending +/- that was never executed is dropped, not remembered
                else if (row2 && touchX >= TS_BACK_X && touchX <= TS_BACK_X + TS_BACK_W) {
                    track_line_tgt_m = -1.0f;
                    in_track_settings_mode = false;
                    last_transition_ms = millis();
                    reset_button_draw_cache();
                    draw_resting_ui();
                }
                // EXECUTE: move both ends of the line out to the dialled length
                else if (row2 && touchX >= TS_EXEC_X && touchX <= TS_EXEC_X + TS_EXEC_W) {
                    if (track_line_measured() && track_line_tgt_m > 0
                        && lroundf(track_line_tgt_m) != lroundf(track_line_cur_m)) {
                        track_settings_banner("MOVING...", TFT_YELLOW);
                        track_line_execute();
                        delay(800);
                        last_transition_ms = millis();
                        reset_button_draw_cache();
                        draw_resting_ui();
                    }
                }
            } else {
                // --- MAIN MENU INTERACTION ---
                // Button 1: Y 75 to 115, X 10 to 230
                if (touchY >= 75 && touchY <= 115 && touchX >= 10 && touchX <= 230) {
                    if (buoys[0].id != "" && buoys[0].present) {
                        selected_buoy_idx = 0;
                        last_transition_ms = millis();
                        ChangeRGBColor(RGB_COLOR_3); // Shift LED to blue indicating view
                    }
                }
                // Button 2: Y 120 to 160, X 10 to 230
                else if (touchY >= 120 && touchY <= 160 && touchX >= 10 && touchX <= 230) {
                    if (buoys[1].id != "" && buoys[1].present) {
                        selected_buoy_idx = 1;
                        last_transition_ms = millis();
                        ChangeRGBColor(RGB_COLOR_3);
                    }
                }
                // Button 3: Y 165 to 205, X 10 to 230
                else if (touchY >= 165 && touchY <= 205 && touchX >= 10 && touchX <= 230) {
                    if (buoys[2].id != "" && buoys[2].present) {
                        selected_buoy_idx = 2;
                        last_transition_ms = millis();
                        ChangeRGBColor(RGB_COLOR_3);
                    }
                }
                // TRACK SETTINGS Button: Y 210 to 245, X 10 to 230
                else if (touchY >= 210 && touchY <= 245 && touchX >= 10 && touchX <= 230) {
                    unsigned long now = millis();
                    static unsigned long right_touch_start = 0;
                    if (right_touch_start == 0) {
                        right_touch_start = now;
                    } else if (now - right_touch_start >= 300) {
                        right_touch_start = 0;
                        in_track_settings_mode = true;
                        last_transition_ms = now;
                        reset_button_draw_cache();
                        draw_resting_ui();
                    }
                }
                // CALIBRATE TOUCH Button: Y 246 to 320, X 10 to 230 (uses the entire bottom screen area for maximum sensitivity!)
                else if (touchY >= 246 && touchY <= 320 && touchX >= 10 && touchX <= 230) {
                    last_transition_ms = millis();
                    Serial.println("Calibration button tapped! Entering calibration screen.");
                    start_touch_calibration();
                }
            }
        } else {
            if (in_man_fourier_cal_mode) {
                // Symmetrically enforce touch lockout delay to prevent touch propagation leakage!
                if (millis() - mancal_started_ms < 800) {
                    return;
                }
                // The press that opened this screen may still be down, and the MAN CAL box and its
                // + button sit on top of the compass rose and the +/- row. Ignore everything until
                // the glass has been let go of once, or that press steers the buoy on arrival.
                if (mancal_await_release) {
                    return;
                }

                // --- MANUAL FOURIER CALIBRATION SCREEN TOUCH INTERACTION ---
                
                // 1. Compass Rose Dot Tap (Y: 40 to 180, covers the circle)
                if (touchY >= 40 && touchY <= 180) {
                    // Steering before the buoy has told us its table, and before its harmonic
                    // correction is off, would be dialed against a compass that is about to shift.
                    if (mancal_harmonic_pending) {
                        delay(20);
                        return;
                    }
                    int cx = 120;
                    int cy = 110;
                    int r_dots = 38;
                    for (int i = 0; i < 8; i++) {
                        int angle_deg = i * 45;
                        float angle_rad = angle_deg * (float)M_PI / 180.0f;
                        int dot_x = cx + (int)(r_dots * sin(angle_rad));
                        int dot_y = cy - (int)(r_dots * cos(angle_rad));
                        
                        int dx = touchX - dot_x;
                        int dy = touchY - dot_y;
                        if (dx*dx + dy*dy <= 18*18) { // 18-pixel touch catchment radius
                            if (mancal_leg_locked(i)) {
                                tft.fillRect(0, 180, tft.width(), 26, TFT_BLACK);
                                tft.setTextDatum(MC_DATUM);
                                tft.setTextSize(2);
                                tft.setTextColor(TFT_ORANGE, TFT_BLACK);
                                tft.drawString("SET NORTH FIRST", tft.width() / 2, 193);
                                delay(900);
                                mancal_is_dirty = true;
                                delay(20);
                                return;
                            }
                            mancal_selected_leg = i;
                            mancal_leg_visited[i] = true;
                            mancal_is_dirty = true;
                            
                            // Immediately command the buoy to steer to this target angle with
                            // speed = 0. The correction this direction already carries goes in
                            // straight away, the same way the +/- buttons apply it - steering to
                            // the bare angle would make the first 1-degree tap swing the buoy by
                            // the whole stored offset.
                            BuoyData &b = buoys[selected_buoy_idx];
                            b.tg_dir = angle_deg - mancal_offsets[i];
                            while (b.tg_dir < 0.0f) b.tg_dir += 360.0f;
                            while (b.tg_dir >= 360.0f) b.tg_dir -= 360.0f;
                            b.tg_speed = 0.0f;
                            mancal_steer(selected_buoy_idx);

                            Serial.printf("Mancal selected leg: %d (%d deg)\n", i, angle_deg);
                            break;
                        }
                    }
                }
                // 2. Large Minus / Plus Adjustment Buttons (Generous Y: 200 to 255, no gaps!)
                else if (touchY >= 200 && touchY <= 255) {
                    // Nothing to adjust before a direction is chosen. Without this the branches
                    // below index mancal_offsets[-1] and send a REMOTE heading of -45 degrees, so
                    // a stray press on this row spun the thrusters up on a screen that is supposed
                    // to be doing nothing yet.
                    if (mancal_selected_leg < 0) {
                        tft.fillRect(0, 180, tft.width(), 26, TFT_BLACK);
                        tft.setTextDatum(MC_DATUM);
                        tft.setTextSize(2);
                        tft.setTextColor(TFT_ORANGE, TFT_BLACK);
                        tft.drawString("PICK A DIRECTION", tft.width() / 2, 193);
                        delay(500);
                        mancal_is_dirty = true;
                        delay(20);
                        return;
                    }

                    BuoyData &b = buoys[selected_buoy_idx];
                    float target_angle = mancal_selected_leg * 45.0f;

                    if (touchX >= 10 && touchX <= 52) {
                        // MINUS 10 degrees
                        mancal_active_offset() -= 10.0f;
                        if (mancal_active_offset() < -180.0f) mancal_active_offset() = -180.0f;
                        
                        // Ultra-fast surgical text update (avoid full-screen redraw)
                        tft.fillRect(40, 192, tft.width() - 80, 22, TFT_BLACK);
                        tft.setTextSize(2);
                        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                        tft.setTextDatum(MC_DATUM);
                        char off_buf[16];
                        sprintf(off_buf, "%+0.0f deg", mancal_active_offset());
                        tft.drawString(off_buf, tft.width() / 2, 198);
                        
                        // Calculate target angle and apply offset to shift the steering target in real-time
                        b.tg_dir = target_angle - mancal_active_offset();
                        while (b.tg_dir < 0.0f) b.tg_dir += 360.0f;
                        while (b.tg_dir >= 360.0f) b.tg_dir -= 360.0f;
                        b.tg_speed = 0.0f;
                        mancal_steer(selected_buoy_idx);
                        
                        // Send the updated offset in real-time to the buoy (using command 78)
                        send_man_fourier_calibrate(b.id, mancal_selected_leg, mancal_active_offset());
                    }
                    else if (touchX >= 53 && touchX <= 97) {
                        // MINUS 1 degree
                        mancal_active_offset() -= 1.0f;
                        if (mancal_active_offset() < -180.0f) mancal_active_offset() = -180.0f;
                        
                        // Ultra-fast surgical text update (avoid full-screen redraw)
                        tft.fillRect(40, 192, tft.width() - 80, 22, TFT_BLACK);
                        tft.setTextSize(2);
                        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                        tft.setTextDatum(MC_DATUM);
                        char off_buf[16];
                        sprintf(off_buf, "%+0.0f deg", mancal_active_offset());
                        tft.drawString(off_buf, tft.width() / 2, 198);
                        
                        // Calculate target angle and apply offset to shift the steering target in real-time
                        b.tg_dir = target_angle - mancal_active_offset();
                        while (b.tg_dir < 0.0f) b.tg_dir += 360.0f;
                        while (b.tg_dir >= 360.0f) b.tg_dir -= 360.0f;
                        b.tg_speed = 0.0f;
                        mancal_steer(selected_buoy_idx);
                        
                        // Send the updated offset in real-time to the buoy (using command 78)
                        send_man_fourier_calibrate(b.id, mancal_selected_leg, mancal_active_offset());
                    }
                    else if (touchX >= 98 && touchX <= 142) {
                        // SET. The buoy is pointing at this direction right now - aimed by hand on
                        // the shore, or steered there in the water - so whatever its compass reads
                        // IS the reading for this leg. That is exactly the relation the table is
                        // read back through in parse_buoy_packet() (offset = i*45 - tableVal), with
                        // the live heading standing in for the stored one, and the relation
                        // RoboSub/src/compass.cpp:1003 fits its Fourier curve to. Wrapped, not
                        // clamped like the nudge buttons: those clamp to stop key-repeat running
                        // away, but a captured heading is a real bearing and has to fold at 180.
                        if (mancal_harmonic_pending) {
                            // The correction is still ON until the buoy answers the table GET, so
                            // mag_dir is a corrected heading, not the raw one the table is defined
                            // against. Dialling through that window is harmless; capturing is not.
                            mancal_warn("WAIT - READING TABLE");
                            delay(20);
                            return;
                        }
                        if (millis() - b.last_seen_ms > MANCAL_HEADING_STALE_MS) {
                            mancal_warn("NO LIVE HEADING");
                            delay(20);
                            return;
                        }

                        if (mancal_await_harmonic_off || b.harmonic_reported) {
                            // mag_dir is still a CORRECTED heading. Capturing it writes a table on
                            // top of the correction already running, and the buoy then applies
                            // both - the exact fault that silently ruined a full calibration.
                            mancal_warn("WAIT - CORRECTION ON");
                            delay(20);
                            return;
                        }

                        // Press feedback: the value it changes is 30 pixels away and the confirm
                        // banner takes a moment.
                        tft.fillRoundRect(100, 215, 40, 35, 5, TFT_WHITE);
                        tft.setTextColor(TFT_BLACK, TFT_WHITE);
                        tft.setTextSize(1);
                        tft.setTextDatum(MC_DATUM);
                        tft.drawString("SET", 120, 232);

                        float captured = b.mag_dir;
                        float off = target_angle - captured;
                        while (off < -180.0f) off += 360.0f;
                        while (off > 180.0f) off -= 360.0f;
                        mancal_active_offset() = off;

                        send_man_fourier_calibrate(b.id, mancal_selected_leg, off);
                        // Nothing should carry on steering to the target this leg held before the
                        // capture, and on shore the buoy is in the operator's hands.
                        send_buoy_command(b.id, 8);

                        Serial.printf("MANCAL: SET leg %d (%d deg) from live heading %.1f -> corr %+.0f\n",
                                      mancal_selected_leg, (int)target_angle, captured, off);

                        char set_buf[32];
                        sprintf(set_buf, "SET FROM MAG %0.0f", captured);
                        tft.fillRect(0, 180, tft.width(), 26, TFT_BLACK);
                        tft.setTextDatum(MC_DATUM);
                        tft.setTextSize(2);
                        tft.setTextColor(TFT_GREEN, TFT_BLACK);
                        tft.drawString(set_buf, tft.width() / 2, 193);
                        delay(600);
                        mancal_is_dirty = true;
                        delay(20);
                        return;
                    }
                    else if (touchX >= 143 && touchX <= 187) {
                        // PLUS 1 degree
                        mancal_active_offset() += 1.0f;
                        if (mancal_active_offset() > 180.0f) mancal_active_offset() = 180.0f;
                        
                        // Ultra-fast surgical text update (avoid full-screen redraw)
                        tft.fillRect(40, 192, tft.width() - 80, 22, TFT_BLACK);
                        tft.setTextSize(2);
                        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                        tft.setTextDatum(MC_DATUM);
                        char off_buf[16];
                        sprintf(off_buf, "%+0.0f deg", mancal_active_offset());
                        tft.drawString(off_buf, tft.width() / 2, 198);
                        
                        // Calculate target angle and apply offset to shift the steering target in real-time
                        b.tg_dir = target_angle - mancal_active_offset();
                        while (b.tg_dir < 0.0f) b.tg_dir += 360.0f;
                        while (b.tg_dir >= 360.0f) b.tg_dir -= 360.0f;
                        b.tg_speed = 0.0f;
                        mancal_steer(selected_buoy_idx);
                        
                        // Send the updated offset in real-time to the buoy (using command 78)
                        send_man_fourier_calibrate(b.id, mancal_selected_leg, mancal_active_offset());
                    }
                    else if (touchX >= 188 && touchX <= 230) {
                        // PLUS 10 degrees
                        mancal_active_offset() += 10.0f;
                        if (mancal_active_offset() > 180.0f) mancal_active_offset() = 180.0f;
                        
                        // Ultra-fast surgical text update (avoid full-screen redraw)
                        tft.fillRect(40, 192, tft.width() - 80, 22, TFT_BLACK);
                        tft.setTextSize(2);
                        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                        tft.setTextDatum(MC_DATUM);
                        char off_buf[16];
                        sprintf(off_buf, "%+0.0f deg", mancal_active_offset());
                        tft.drawString(off_buf, tft.width() / 2, 198);
                        
                        // Calculate target angle and apply offset to shift the steering target in real-time
                        b.tg_dir = target_angle - mancal_active_offset();
                        while (b.tg_dir < 0.0f) b.tg_dir += 360.0f;
                        while (b.tg_dir >= 360.0f) b.tg_dir -= 360.0f;
                        b.tg_speed = 0.0f;
                        mancal_steer(selected_buoy_idx);
                        
                        // Send the updated offset in real-time to the buoy (using command 78)
                        send_man_fourier_calibrate(b.id, mancal_selected_leg, mancal_active_offset());
                    }

                    // The four branches above each patch the big CORR readout in place to stay
                    // snappy; the table row under the rose has to follow the same value.
                    draw_mancal_offset_strip();
                }
                // 3a. SAVE NEW FOURIER TABLE - drawn over the North/hint band once all eight
                //     directions are in, so it has to be tested before that row.
                else if (mancal_all_legs_visited() && touchY >= 256 && touchY <= 294) {
                    mancal_save_table_and_exit(selected_buoy_idx);
                }
                // 3. Thruster switch / North Button Row (Generous Y: 256 to 299, no gaps!)
                else if (touchY >= 256 && touchY <= 299) {
                    BuoyData &b = buoys[selected_buoy_idx];
                    // The row spans X 10 to 230 whatever it is carrying.
                    if (touchX < 10 || touchX > 230) {
                        delay(20);
                        return;
                    }

                    // Before a direction is picked this row is the thruster switch, which is the
                    // only point in the session it can be changed - see mancal_thrusters_on. After
                    // that it is Set as North on the North leg, and an inert hint on the other
                    // seven.
                    if (mancal_selected_leg == -1) {
                        mancal_thrusters_on = !mancal_thrusters_on;
                        Serial.printf("MANCAL: thrusters %s\n",
                                      mancal_thrusters_on ? "ON (in the water)" : "OFF (on shore, aim by hand)");
                        // Stop it here rather than at the first SET: switching to shore mode is
                        // the operator saying they are about to pick the buoy up.
                        if (!mancal_thrusters_on) send_buoy_command(b.id, 8);
                        mancal_is_dirty = true;
                        delay(150);
                        return;
                    }
                    if (mancal_selected_leg != 0) {
                        delay(20);
                        return;
                    }

                    // SET AS NORTH now spans the row: X 10 to 230
                    {
                        Serial.println("MANCAL: Click registered on Set as North!");

                        // This reads the live heading now, so it needs the two preconditions SET
                        // needs. Checked before the highlight so a refusal does not leave the row
                        // painted white as though it had done something.
                        if (mancal_harmonic_pending) {
                            mancal_warn("WAIT - READING TABLE");
                            delay(20);
                            return;
                        }
                        if (millis() - b.last_seen_ms > MANCAL_HEADING_STALE_MS) {
                            mancal_warn("NO LIVE HEADING");
                            delay(20);
                            return;
                        }

                        // Zero-latency visual feedback (highlight in white)
                        tft.fillRoundRect(10, 260, tft.width() - 20, 35, 5, TFT_WHITE);
                        tft.setTextColor(TFT_BLACK, TFT_WHITE);
                        tft.setTextSize(2);
                        tft.drawString("Set as North...", tft.width() / 2, 277);

                        // The buoy is pointing north right now, so whatever its compass reads IS
                        // the north error - fold that into the mounting offset. This used to add
                        // mancal_offsets[0] instead, which meant the button did nothing at all
                        // unless a correction had already been dialled or SET first: it banked an
                        // error rather than measuring one, so "point it north and press" - the
                        // thing its name promises - left the compass exactly where it was.
                        //
                        // It is the same arithmetic the Sub does in its own case SET_AS_NORTH
                        // (compassOffset - dirMag, RoboSub/src/main.cpp:546), and in the in-water
                        // flow it comes to the same number as the old line: with the buoy holding
                        // the heading it was dialled to, -mag_dir IS mancal_offsets[0]. Measured
                        // rather than assumed, so it is only as good as how settled the buoy is.
                        // Sending that command instead would be the tidier fix, but it cannot be
                        // reached: RoboSub/src/main.cpp:25 defines SET_AS_NORTH as the macro 125
                        // AFTER including RoboCompute.h, where the enumerator is 87, so the Sub
                        // listens on a number no node on this network ever sends.
                        if (mancal_await_harmonic_off || b.harmonic_reported) {
                            // mag_dir is still a CORRECTED heading. Capturing it writes a table on
                            // top of the correction already running, and the buoy then applies
                            // both - the exact fault that silently ruined a full calibration.
                            mancal_warn("WAIT - CORRECTION ON");
                            delay(20);
                            return;
                        }

                        float captured_north = b.mag_dir;
                        float north_err = 0.0f - captured_north;
                        while (north_err < -180.0f) north_err += 360.0f;
                        while (north_err > 180.0f) north_err -= 360.0f;

                        b.compass_offset = b.compass_offset + north_err;
                        while (b.compass_offset < -180) b.compass_offset += 360;
                        while (b.compass_offset > 180) b.compass_offset -= 360;

                        Serial.printf("MANCAL: SET AS NORTH from live heading %.1f -> compass offset %+.2f\n",
                                      captured_north, b.compass_offset);

                        // Reset the North fourier correction back to 0.0f (it is now absorbed globally!)
                        mancal_offsets[0] = 0.0f;

                        // Send SETUPDATA update right away
                        send_buoy_setup(selected_buoy_idx);

                        // Send steering command again to refresh/re-evaluate the buoy control loop
                        b.tg_dir = 0.0f;
                        b.tg_speed = 0.0f;
                        mancal_steer(selected_buoy_idx);

                        delay(100); // Super-snappy highlight feel

                        tft.fillRect(0, 195, tft.width(), 100, TFT_BLACK);
                        tft.setTextDatum(MC_DATUM);
                        tft.setTextSize(2);
                        tft.setTextColor(TFT_GREEN, TFT_BLACK);
                        tft.drawString("GLOBAL NORTH SET", tft.width() / 2, 230);
                        tft.setTextSize(1);
                        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
                        char north_buf[32];
                        sprintf(north_buf, "was MAG %0.0f", captured_north);
                        tft.drawString(north_buf, tft.width() / 2, 252);
                        delay(400); // Snappy success banner display
                        mancal_is_dirty = true;
                    }
                }
                // 4. Footer Buttons (Y: 300 to 320)
                //    With all eight captured the footer is one full-width CANCEL, so the old
                //    X 10..115 half-width test would leave the right half dead. Same handler.
                else if (touchY >= 300 && touchY <= 320) {
                    BuoyData &b = buoys[selected_buoy_idx];
                    // BACK / CANCEL (X: 10 to 115, or the whole row once SAVE has taken the band above)
                    if (touchX >= 10 && (touchX <= 115 || mancal_all_legs_visited())) {
                        // Leave with the harmonic correction ON. Entering this screen switches
                        // it off so the dialing works on the raw compass; walking away and leaving
                        // it off makes every later visit read the identity table, so the
                        // corrections still in NVS become invisible.
                        b.harmonic_enabled = true;
                        b.compass_trim_enabled = true;   // switched off on entry, see above
                        send_buoy_setup(selected_buoy_idx);
                        mancal_harmonic_pending = false;
                        mancal_await_harmonic_off = false;
                        delay(150);

                        // Send IDLE command immediately to shut down thrusters/motors!
                        send_buoy_command(b.id, 8);

                        in_man_fourier_cal_mode = false;
                        in_setup_mode = true;
                        last_transition_ms = millis();
                        reset_button_draw_cache();
                        draw_resting_ui();
                    }
                    // SAVE ALL & EXIT (X: 125 to 230)
                    else if (touchX >= 125 && touchX <= 230) {

                        mancal_save_table_and_exit(selected_buoy_idx);
                    }
                }
            } else if (in_setup_mode) {
                // --- SETUP SCREEN TOUCH INTERACTION ---
                // 1. Grid Parameter Selection (Y: 35 to 189, covers 4 rows x 2 columns)
                // With only 4 rows, each box is 32 pixels high with a 4px gap (symmetrical 36px steps),
                // completely eliminating row touch overlaps and resistive jitter shifts!
                if (touchY >= 35 && touchY <= 189) {
                    if (setup_page == 5 && touchY >= 71) {
                        // Handle GPS Cal Start direction dot touch!
                        BuoyData &b = buoys[selected_buoy_idx];
                        bool live = (b.cal_seen_ms != 0) && (millis() - b.cal_seen_ms < 15000);
                        bool aborted = (b.cal_seen_ms != 0) && (b.cal_step == 6);
                        if (aborted) live = true;

                        if (!live) {
                            int cx = 165;
                            int cy = 130;
                            int r_dots = 30;
                            for (int i = 0; i < 8; i++) {
                                int angle_deg = i * 45;
                                float angle_rad = angle_deg * (float)M_PI / 180.0f;
                                int dot_x = cx + (int)(r_dots * sin(angle_rad));
                                int dot_y = cy - (int)(r_dots * cos(angle_rad));
                                
                                int dx = touchX - dot_x;
                                int dy = touchY - dot_y;
                                if (dx*dx + dy*dy <= 18*18) { // 18-pixel touch catchment radius
                                    b.cal_start_heading = angle_deg;
                                    extern bool gps_cal_panel_dirty;
                                    gps_cal_panel_dirty = true;
                                    Serial.printf("Selected GPS Cal Start Heading: %d deg\n", angle_deg);
                                    break;
                                }
                            }
                        }
                    } else {
                        int r = (touchY - 35) / 36;
                        if (r < 0) r = 0;
                        if (r > 3) r = 3;
                        int c = (touchX < 120) ? 0 : 1;
                        
                        int local_idx = c * 4 + r; // 0 to 7
                        int tapped_idx = setup_page * 8 + local_idx; // Map to 0-15 based on page!
                        
                        if (setup_slot_used(tapped_idx)) {
                            // Change focus selection
                            selected_param_idx = tapped_idx;

                            BuoyData &b = buoys[selected_buoy_idx];
                            if (selected_param_idx == S_MANCAL) {
                                // Instantly trigger Manual Fourier Calibration!
                                enter_man_fourier_cal(selected_buoy_idx);
                                reset_button_draw_cache();
                                draw_resting_ui();
                                return;
                            }
                            else if (setup_slot_is_bool(selected_param_idx)) {
                                // Boolean Toggles: Symmetrical instant toggles on tap!
                                setup_bool_toggle(b, selected_param_idx);
                            } else if (selected_param_idx == S_SETNORTH && setup_data_loaded) {
                                // "SET NORTH" acts as a quick instant trigger. It only rewrites the
                                // compass offset, so it must not run before the buoy has told us what
                                // that offset currently is - otherwise it computes from a zero.
                                b.compass_offset = b.compass_offset - b.mag_dir;
                                while (b.compass_offset < -180) b.compass_offset += 360;
                                while (b.compass_offset > 180) b.compass_offset -= 360;
                                // Jump to the compass page and highlight the offset so the computed
                                // value is actually visible, then SAVE sends it.
                                setup_page = S_COMPOFF / 8;
                                selected_param_idx = S_COMPOFF;
                            }
                        }
                    }
                }
                // 2. Large Plus / Minus Button Clicks (Y: 190 to 230)
                else if (touchY >= 190 && touchY <= 230) {
                    BuoyData &b = buoys[selected_buoy_idx];
                    
                    // The action pages hold no editable parameters. Only their own boxes
                    // respond here; anything else still selected from another page must be left
                    // alone rather than edited invisibly.
                    bool adjArmed = !setup_is_action_page(setup_page) ||
                                    setup_slot_is_action(selected_param_idx);

                    if (adjArmed && touchX >= 10 && touchX <= 75) {
                        // MINUS - no meaning on an action page
                        if (!setup_slot_is_action(selected_param_idx)) {
                            if (setup_slot_is_bool(selected_param_idx)) {
                                setup_bool_toggle(b, selected_param_idx); // toggle on minus too
                            } else {
                                setup_adjust(b, selected_param_idx, false);
                            }
                        }
                    }
                    else if (adjArmed && touchX >= 155 && touchX <= 230) {
                        // PLUS
                        if (setup_slot_is_bool(selected_param_idx)) {
                            setup_bool_toggle(b, selected_param_idx); // toggle on plus as well!
                        } else if (selected_param_idx == S_GPSSTILL) {
                            // Two-step on purpose: tapping the box on the action page only
                            // selects it, and this is the confirming press. Unlike every other
                            // entry here it commands the buoy to sail for half an hour, so it must
                            // not be reachable with a single stray touch.
                            send_gps_fourier_calibrate(b.id, true, b.cal_start_heading);
                            tft.fillRect(0, 60, tft.width(), 120, TFT_BLACK);
                            tft.setTextDatum(MC_DATUM);
                            tft.setTextSize(2);
                            tft.setTextColor(TFT_GREEN, TFT_BLACK);
                            tft.drawString("GPS CAL STARTED", tft.width() / 2, 100);
                            tft.setTextSize(1);
                            tft.setTextColor(TFT_WHITE, TFT_BLACK);
                            tft.drawString("still water - per leg", tft.width() / 2, 130);
                            tft.drawString("8 legs, about 30 minutes", tft.width() / 2, 148);
                            delay(1500);
                            // This overlay painted over the panel's area. The grid boxes are
                            // redrawn unconditionally on the next pass, but the panel is cached,
                            // so without this it would decide nothing had changed and leave the
                            // overlay on the glass until the first report happened to differ.
                            extern bool gps_cal_panel_dirty;
                            gps_cal_panel_dirty = true;
                        } else if (selected_param_idx == S_MANCAL) {
                            // Symmetrical confirmation delay/press for Manual Cal transition:
                            tft.fillRect(0, 60, tft.width(), 120, TFT_BLACK);
                            tft.setTextDatum(MC_DATUM);
                            tft.setTextSize(2);
                            tft.setTextColor(TFT_ORANGE, TFT_BLACK);
                            tft.drawString("MANUAL CAL START", tft.width() / 2, 100);
                            tft.setTextSize(1);
                            tft.setTextColor(TFT_WHITE, TFT_BLACK);
                            tft.drawString("pivots to 8 target sectors", tft.width() / 2, 130);
                            tft.drawString("and stores 1-deg manual steps", tft.width() / 2, 148);
                            delay(1500);

                            // Same entry sequence as tapping the box, table query included - it
                            // was missing here, which is why coming in this way showed a table of
                            // zeros instead of the corrections the buoy is running.
                            enter_man_fourier_cal(selected_buoy_idx);
                        } else if (selected_param_idx == S_DESKCAL) {
                            // Same two-step. 60 s of figure-of-eight on the bench; harmless
                            // afloat but it throws away the running compass calibration.
                            send_buoy_command(b.id, 27, 6); // CALIBRATE_MAGNETIC_COMPASS, ack=INF
                            tft.fillRect(0, 60, tft.width(), 120, TFT_BLACK);
                            tft.setTextDatum(MC_DATUM);
                            tft.setTextSize(2);
                            tft.setTextColor(TFT_RED, TFT_BLACK);
                            tft.drawString("DESK CAL STARTED", tft.width() / 2, 100);
                            tft.setTextSize(1);
                            tft.setTextColor(TFT_WHITE, TFT_BLACK);
                            tft.drawString("turn the buoy in figures of 8", tft.width() / 2, 130);
                            tft.drawString("for 60 seconds", tft.width() / 2, 148);
                            delay(1500);
                        } else if (selected_param_idx == S_REBOOT) {
                            // INF, not GETACK: a rebooting buoy never answers, so GETACK
                            // leaves the packet in the LoRa retry table and reboots it five
                            // times over.
                            send_buoy_command(b.id, 85, 6); // REBOOT
                            tft.fillRect(0, 60, tft.width(), 120, TFT_BLACK);
                            tft.setTextDatum(MC_DATUM);
                            tft.setTextSize(2);
                            tft.setTextColor(TFT_ORANGE, TFT_BLACK);
                            tft.drawString("REBOOTING BUOY", tft.width() / 2, 110);
                            delay(1500);
                        } else if (selected_param_idx == S_SETNORTH) {
                            if (setup_data_loaded) {
                                b.compass_offset = b.compass_offset - b.mag_dir;
                                while (b.compass_offset < -180) b.compass_offset += 360;
                                while (b.compass_offset > 180) b.compass_offset -= 360;
                                setup_page = S_COMPOFF / 8;
                                selected_param_idx = S_COMPOFF;
                            }
                        } else {
                            setup_adjust(b, selected_param_idx, true);
                        }
                    }
                    reset_button_draw_cache(); // Force complete update redraw
                }
                
                // 3. Bottom Control Buttons (Y: 232 to 275) - BACK (left), PAGE (center), SAVE (right)
                if (touchY >= 232 && touchY <= 275) {
                    // BACK Button (X: 5 to 80) - Return, Discard changes
                    if (touchX >= 5 && touchX <= 80) {
                        in_setup_mode = false;
                        last_transition_ms = millis();
                        reset_button_draw_cache();
                        draw_resting_ui();
                    }
                    // PAGE Toggle Button (X: 81 to 155) - Toggles Setup Page 1, 2 & 3!
                    else if (touchX >= 81 && touchX <= 155) {
                        setup_page = (setup_page + 1) % SETUP_PAGES;
                        last_transition_ms = millis();
                        reset_button_draw_cache();
                        draw_resting_ui();
                    }
                    // SAVE Button (X: 156 to 235) - Broadcast & Return! Only active if setup data is loaded!
                    else if (touchX >= 156 && touchX <= 235 && setup_data_loaded) {
                        tft.fillRect(10, 162, 220, 54, TFT_BLACK);
                        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                        tft.setTextSize(2);
                        tft.setTextDatum(MC_DATUM);
                        tft.drawString("SAVING SETUP...", tft.width() / 2, 190);
                        
                        send_buoy_setup(selected_buoy_idx); // Broadcast SET SETUPDATA
                        delay(800);
                        
                        in_setup_mode = false;
                        last_transition_ms = millis();
                        reset_button_draw_cache();
                        draw_resting_ui();
                    }
                }
            } else if (in_mannav_mode) {
                // --- MANUAL NAVIGATION SCREEN TOUCH INTERACTION ---
                // 1. Compass Rose Tap (Y: 35 to 145) - Tap on circle sets a new target direction
                if (touchY >= 35 && touchY <= 145) {
                    float dx = touchX - 120;
                    float dy = touchY - 95;
                    float dist = sqrt(dx*dx + dy*dy);
                    if (dist >= 15 && dist <= 75) {
                        float angle_rad = atan2(dx, -dy);
                        float angle_deg = angle_rad * 180.0 / PI;
                        if (angle_deg < 0) angle_deg += 360.0;
                        
                        buoys[selected_buoy_idx].tg_dir = angle_deg;
                        send_buoy_dirdist(selected_buoy_idx);
                        reset_button_draw_cache();
                    }
                }
                // 2. Slider 1 (TG Dir) Controls (Y: 195 to 225) - Aligned with Y: 210 track!
                else if (touchY >= 195 && touchY <= 225) {
                    if (touchX >= 10 && touchX <= 55) {
                        // Left Minus Button Tap: Decrease by 5 degrees
                        buoys[selected_buoy_idx].tg_dir -= 5.0;
                        if (buoys[selected_buoy_idx].tg_dir < 0) buoys[selected_buoy_idx].tg_dir += 360.0;
                        send_buoy_dirdist(selected_buoy_idx);
                        reset_button_draw_cache();
                    } else if (touchX >= 185 && touchX <= 230) {
                        // Right Plus Button Tap: Increase by 5 degrees
                        buoys[selected_buoy_idx].tg_dir += 5.0;
                        if (buoys[selected_buoy_idx].tg_dir >= 360.0) buoys[selected_buoy_idx].tg_dir -= 360.0;
                        send_buoy_dirdist(selected_buoy_idx);
                        reset_button_draw_cache();
                    } else if (touchX >= 58 && touchX <= 182) {
                        // Center Track Drag (X: 60 to 180, width 120px)
                        float pct = (float)(touchX - 60) / 120.0;
                        if (pct < 0.0) pct = 0.0;
                        if (pct > 1.0) pct = 1.0;
                        float new_tg_dir = pct * 360.0;
                        
                        buoys[selected_buoy_idx].tg_dir = new_tg_dir;
                        send_buoy_dirdist(selected_buoy_idx);
                        reset_button_draw_cache();
                    }
                }
                // 3. Slider 2 (Speed) Controls (Y: 235 to 265) - Aligned with Y: 250 track!
                else if (touchY >= 235 && touchY <= 265) {
                    float max_spd = buoys[selected_buoy_idx].max_speed;
                    if (touchX >= 10 && touchX <= 55) {
                        // Left Minus Button Tap: Decrease by 5%
                        buoys[selected_buoy_idx].tg_speed -= 5.0;
                        if (buoys[selected_buoy_idx].tg_speed < -max_spd) buoys[selected_buoy_idx].tg_speed = -max_spd;
                        send_buoy_dirdist(selected_buoy_idx);
                        reset_button_draw_cache();
                    } else if (touchX >= 185 && touchX <= 230) {
                        // Right Plus Button Tap: Increase by 5%
                        buoys[selected_buoy_idx].tg_speed += 5.0;
                        if (buoys[selected_buoy_idx].tg_speed > max_spd) buoys[selected_buoy_idx].tg_speed = max_spd;
                        send_buoy_dirdist(selected_buoy_idx);
                        reset_button_draw_cache();
                    } else if (touchX >= 58 && touchX <= 182) {
                        // Center Track Drag (X: 60 to 180, width 120px)
                        float pct = (float)(touchX - 60) / 120.0;
                        if (pct < 0.0) pct = 0.0;
                        if (pct > 1.0) pct = 1.0;
                        float new_tg_speed = -max_spd + pct * (2.0 * max_spd);
                        
                        buoys[selected_buoy_idx].tg_speed = new_tg_speed;
                        send_buoy_dirdist(selected_buoy_idx);
                        reset_button_draw_cache();
                    }
                }
                // 4. BACK & IDLE Buttons (Y: 270 to 320)
                else if (touchY >= 270 && touchY <= 320) {
                    if (touchX >= 10 && touchX <= 115) {
                        // BACK Button: Return to standard navigation page
                        in_mannav_mode = false;
                        last_transition_ms = millis();
                        reset_button_draw_cache();
                        draw_resting_ui();
                    } else if (touchX >= 120 && touchX <= 230) {
                        // IDLE Button: Stop the motors, show overlay, and synchronize speed setpoint to 0.0%
                        tft.fillRect(10, 170, 220, 60, TFT_BLACK);
                        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                        tft.setTextSize(2);
                        tft.setTextDatum(MC_DATUM);
                        tft.drawString("SENDING IDLE...", tft.width() / 2, 195);
                        
                        send_buoy_command(buoys[selected_buoy_idx].id, 8); // Send IDLE (8)
                        delay(600);
                        
                        // Set local manual speed setpoint back to 0.0%
                        buoys[selected_buoy_idx].tg_speed = 0.0;
                        
                        reset_button_draw_cache();
                        draw_resting_ui();
                    }
                }
            } else {
                // --- NAVIGATION PAGE INTERACTION (Directly when selected!) ---
                // Checkboxes touch checking (Y: 198 to 238, X >= 120) - STACKED & SPACED OUT!
                if (touchY >= 198 && touchY <= 238 && touchX >= 120) {
                    if (touchY <= 215) {
                        udp_enabled = !udp_enabled;
                        reset_button_draw_cache();
                        draw_resting_ui();
                    } else {
                        lora_enabled = !lora_enabled;
                        reset_button_draw_cache();
                        draw_resting_ui();
                    }
                }
                // Row 1 Control Buttons: LOCK, DOCK, IDLE (Y: 235 to 277)
                else if (touchY >= 235 && touchY <= 277) {
                    BuoyData &b = buoys[selected_buoy_idx];
                    bool has_fix = (b.gps_fix == "3D" || b.gps_fix == "2D");
                    
                    if (touchX >= 5 && touchX <= 82) {
                        if (!has_fix) {
                            Serial.println("LOCK command ignored: No GPS Fix on Top!");
                            return; // Ignore touch completely if no fix!
                        }
                        tft.fillRect(10, 170, 220, 60, TFT_BLACK);
                        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                        tft.setTextSize(2);
                        tft.setTextDatum(MC_DATUM);
                        tft.drawString("SENDING LOCK...", tft.width() / 2, 195);
                        send_buoy_command(buoys[selected_buoy_idx].id, 12); // Send LOCK (12)
                        delay(600);
                        draw_resting_ui();
                    } else if (touchX >= 83 && touchX <= 157) {
                        if (!has_fix) {
                            Serial.println("DOCK command ignored: No GPS Fix on Top!");
                            return; // Ignore touch completely if no fix!
                        }
                        tft.fillRect(10, 170, 220, 60, TFT_BLACK);
                        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                        tft.setTextSize(2);
                        tft.setTextDatum(MC_DATUM);
                        tft.drawString("SENDING DOCK...", tft.width() / 2, 195);
                        send_buoy_command(buoys[selected_buoy_idx].id, 15); // Send DOCK (15)
                        delay(600);
                        draw_resting_ui();
                    } else if (touchX >= 158 && touchX <= 235) {
                        tft.fillRect(10, 170, 220, 60, TFT_BLACK);
                        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                        tft.setTextSize(2);
                        tft.setTextDatum(MC_DATUM);
                        tft.drawString("SENDING IDLE...", tft.width() / 2, 195);
                        send_buoy_command(buoys[selected_buoy_idx].id, 8); // Send IDLE (8)
                        delay(600);
                        draw_resting_ui();
                    }
                }
                // Row 2 Actions: BACK, MANNAV, SETUP (Y: 278 to 320)
                else if (touchY >= 278 && touchY <= 320) {
                    if (touchX >= 5 && touchX <= 80) {
                        selected_buoy_idx = -1; // BACK Button: Directly return to main menu!
                        last_transition_ms = millis();
                        ChangeRGBColor(RGB_COLOR_2); // Back to green status LED
                    } else if (touchX >= 81 && touchX <= 155) {
                        // MANNAV Button clicked! Enter manual navigation mode!
                        in_mannav_mode = true;
                        
                        // Initialize manual steering setpoints
                        buoys[selected_buoy_idx].tg_speed = 0.0;
                        buoys[selected_buoy_idx].tg_dir = buoys[selected_buoy_idx].mag_dir;
                        
                        // Broadcast initial TGDIRSPEED command immediately to synchronize state!
                        send_buoy_dirdist(selected_buoy_idx);
                        
                        last_transition_ms = millis();
                        reset_button_draw_cache();
                        draw_resting_ui();
                    } else if (touchX >= 156 && touchX <= 235) {
                        // SETUP Button clicked! Enter setup mode!
                        in_setup_mode = true;
                        setup_page = 0; // Default to page 1 on entry
                        setup_data_loaded = false; // Reset loaded flag until buoy actually responds!
                        last_transition_ms = millis();
                        
                        // Send query to buoy to fetch its PID / calibration coefficients exactly matching webpage GET formatting
                        // This is attempt 1; loop() repeats it until the buoy answers.
                        setup_query_tries = 1;
                        setup_query_next_ms = millis() + SETUP_QUERY_INTERVAL_MS;
                        query_buoy_setup(buoys[selected_buoy_idx].id);
                        reset_button_draw_cache();
                        draw_resting_ui();
                    }
                }
            }
        }
    } else {
        // The glass has been let go of, so the press that opened the calibration screen can no
        // longer be mistaken for a tap on it.
        mancal_await_release = false;

        // No touch active: reset holding timer
        if (both_touched_previously) {
            both_touched_previously = false;
            // Erase the green holding progress bar
            tft.fillRect(30, 280, 180, 2, TFT_BLACK);
        }
        if (cal_touch_start != 0) {
            cal_touch_start = 0;
            // Erase the green holding progress bar
            tft.fillRect(30, 280, 180, 2, TFT_BLACK);
            
            // Redraw the button back to normal (TFT_DARKGREY)
            tft.fillRoundRect(30, 250, 180, 28, 4, TFT_DARKGREY);
            tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
            tft.setTextSize(1);
            tft.setTextDatum(MC_DATUM);
            tft.drawString("CALIBRATE TOUCH", tft.width() / 2, 264);
        }
    }

    // Re-ask for the setup data until it arrives. Nothing along the request path acknowledges or
    // retransmits, so without this a single dropped packet strands the screen on its loading
    // overlay. Stops once the data lands or after SETUP_QUERY_MAX_TRIES, so a buoy that is simply
    // not there does not get polled forever.
    if (in_setup_mode && !setup_data_loaded &&
        selected_buoy_idx >= 0 && selected_buoy_idx < 3 &&
        buoys[selected_buoy_idx].id.length() > 0 &&
        setup_query_tries < SETUP_QUERY_MAX_TRIES &&
        (long)(millis() - setup_query_next_ms) >= 0)
    {
        setup_query_tries++;
        setup_query_next_ms = millis() + SETUP_QUERY_INTERVAL_MS;
        Serial.printf("SETUPDATA not received, retry %d of %d\n", setup_query_tries, SETUP_QUERY_MAX_TRIES);
        query_buoy_setup(buoys[selected_buoy_idx].id);
    }

    // Same story for the calibration table: the GET is fire-and-forget, and a lost one leaves the
    // screen showing eight zeros, which reads as "this buoy has no corrections" rather than "the
    // answer never came". Keep asking until it lands, then switch the correction off.
    service_mancal_entry();

    // Refresh dynamic screen details every 250 milliseconds
    if (millis() - lastUIUpdate > 250) {
        lastUIUpdate = millis();
        update_dynamic_ui();
    }

    delay(20);
}

void draw_calibration_screen() {
    tft.fillScreen(TFT_BLACK);
    
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("TOUCH SCREEN", tft.width() / 2, 110);
    tft.drawString("CALIBRATION", tft.width() / 2, 140);
    
    tft.setTextSize(1);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    
    if (cal_state == 0) {
        tft.drawString("Touch the red target in", tft.width() / 2, 180);
        tft.drawString("the TOP-LEFT corner", tft.width() / 2, 200);
        
        // Target at (20, 20)
        tft.drawCircle(20, 20, 8, TFT_RED);
        tft.fillCircle(20, 20, 3, TFT_RED);
        tft.drawFastHLine(5, 20, 30, TFT_WHITE);
        tft.drawFastVLine(20, 5, 30, TFT_WHITE);
    } else if (cal_state == 1) {
        tft.drawString("Touch the red target in", tft.width() / 2, 180);
        tft.drawString("the BOTTOM-RIGHT corner", tft.width() / 2, 200);
        
        // Target at (220, 300)
        tft.drawCircle(220, 300, 8, TFT_RED);
        tft.fillCircle(220, 300, 3, TFT_RED);
        tft.drawFastHLine(205, 300, 30, TFT_WHITE);
        tft.drawFastVLine(220, 285, 30, TFT_WHITE);
    }
}

void handle_touch_calibration() {
    // Ignore all touches for the first 800ms after entering calibration
    // to allow the user to lift their finger from the menu button!
    if (millis() - cal_started_ms < 800) {
        return;
    }

    if (cal_state == 2) {
        unsigned long elapsed = millis() - save_screen_start_ms;
        if (elapsed >= 3000) {
            // Discard calibration and return
            Serial.println("Calibration timeout! Discarding changes.");
            
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextDatum(MC_DATUM);
            tft.drawString("CALIBRATION DISCARDED", tft.width() / 2, 120);
            tft.setTextSize(1);
            tft.drawString("Timed out after 3 seconds", tft.width() / 2, 160);
            
            delay(2000);
            
            // Reload old calibration bounds to discard the active changes
            Preferences prefs;
            prefs.begin("touch-cal", true);
            ts_minx = prefs.getUShort("minx", 300);
            ts_maxx = prefs.getUShort("maxx", 3800);
            ts_miny = prefs.getUShort("miny", 260);
            ts_maxy = prefs.getUShort("maxy", 3800);
            prefs.end();
            apply_calibration(ts_minx, ts_maxx, ts_miny, ts_maxy);
            
            in_calibration_mode = false;
            selected_buoy_idx = -1;
            lastKnownState = -2; // Force complete Menu Screen redraw
            return;
        } else {
            // Draw progress/countdown bar or text
            int remaining_sec = 3 - (elapsed / 1000);
            char timer_buf[32];
            sprintf(timer_buf, "Returning in %ds...", remaining_sec);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setTextSize(1);
            tft.setTextDatum(MC_DATUM);
            tft.drawString(timer_buf, tft.width() / 2, 260);
        }
    }

    int rx = 0, ry = 0;
    if (get_raw_touch_point(rx, ry)) {
        Serial.printf("Cal Touch: rawX=%d, rawY=%d\n", rx, ry);
        if (cal_state == 0) {
            cal_rx1 = rx;
            cal_ry1 = ry;
            cal_state = 1;
            draw_calibration_screen();
        } else if (cal_state == 1) {
            cal_rx2 = rx;
            cal_ry2 = ry;
            
            // Symmetrical math for calculating actual calibration values
            float S_x = (float)(220 - 20) / (cal_ry2 - cal_ry1);
            int cal_miny = cal_ry1 - (20 - 1) / S_x;
            int cal_maxy = cal_miny + 239 / S_x;
            
            float S_y = (float)(300 - 20) / (cal_rx2 - cal_rx1);
            int cal_maxx = cal_rx1 - (20 - 1) / S_y;
            int cal_minx = cal_maxx + 319 / S_y;
            
            // Constrain results to valid physical boundaries
            temp_minx = constrain(cal_minx, 50, 1000);
            temp_maxx = constrain(cal_maxx, 2500, 4000);
            temp_miny = constrain(cal_miny, 50, 1000);
            temp_maxy = constrain(cal_maxy, 2500, 4000);
            
            // Apply bounds immediately to active driver so user can touch the Green Save button!
            apply_calibration(temp_minx, temp_maxx, temp_miny, temp_maxy);
            
            // Draw Save Screen!
            cal_state = 2;
            save_screen_start_ms = millis();
            
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextDatum(MC_DATUM);
            tft.drawString("SAVE CALIBRATION?", tft.width() / 2, 60);
            
            tft.setTextSize(1);
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.drawString("Touch the green button to save", tft.width() / 2, 100);
            tft.drawString("Otherwise, it will discard and", tft.width() / 2, 120);
            tft.drawString("exit in 3 seconds.", tft.width() / 2, 140);
            
            // Draw a big green SAVE button
            tft.fillRoundRect(30, 180, 180, 45, 6, TFT_GREEN);
            tft.setTextColor(TFT_BLACK, TFT_GREEN);
            tft.setTextSize(2);
            tft.drawString("SAVE", tft.width() / 2, 202);
            
        } else if (cal_state == 2) {
            // We are in state 2 (Save confirmation screen).
            // Translate raw touch coordinates to calibrated coordinates using temporary bounds to detect the SAVE button press!
            int touchX = map(ry, temp_miny, temp_maxy, 1, 240);
            int touchY = map(rx, temp_maxx, temp_minx, 1, 320);
            touchX = constrain(touchX, 1, 240);
            touchY = constrain(touchY, 1, 320);
            
            Serial.printf("Save screen touch mapped: X=%d, Y=%d\n", touchX, touchY);
            
            // Save button: X: 30 to 210, Y: 180 to 225
            if (touchX >= 30 && touchX <= 210 && touchY >= 180 && touchY <= 225) {
                // Apply bounds and save permanently!
                ts_minx = temp_minx;
                ts_maxx = temp_maxx;
                ts_miny = temp_miny;
                ts_maxy = temp_maxy;
                
                apply_calibration(ts_minx, ts_maxx, ts_miny, ts_maxy);
                
                Preferences prefs;
                prefs.begin("touch-cal", false);
                prefs.putUShort("minx", ts_minx);
                prefs.putUShort("maxx", ts_maxx);
                prefs.putUShort("miny", ts_miny);
                prefs.putUShort("maxy", ts_maxy);
                prefs.end();
                
                Serial.printf("Saved Touch Calibration permanently: minx=%d, maxx=%d, miny=%d, maxy=%d\n", ts_minx, ts_maxx, ts_miny, ts_maxy);
                
                // Success Screen Feedback
                tft.fillScreen(TFT_BLACK);
                tft.setTextColor(TFT_GREEN, TFT_BLACK);
                tft.setTextSize(2);
                tft.setTextDatum(MC_DATUM);
                tft.drawString("CALIBRATION SAVED!", tft.width() / 2, 110);
                
                tft.setTextSize(1);
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
                char buf[64];
                sprintf(buf, "X: %d-%d  Y: %d-%d", ts_minx, ts_maxx, ts_miny, ts_maxy);
                tft.drawString(buf, tft.width() / 2, 170);
                
                delay(2000);
                
                in_calibration_mode = false;
                selected_buoy_idx = -1;
                lastKnownState = -2; // Force complete menu screen redraw
            }
        }
    }
}

void start_touch_calibration() {
    in_calibration_mode = true;
    cal_state = 0;
    cal_rx1 = 0; cal_ry1 = 0;
    cal_rx2 = 0; cal_ry2 = 0;
    cal_started_ms = millis(); // Set transition timestamp for lockout delay!
    draw_calibration_screen();
}

// Helper function to calculate relative distance in meters between two GPS coordinates
void get_relative_meters(double lat1, double lon1, double lat2, double lon2, float &out_dx, float &out_dy) {
    double avg_lat = (lat1 + lat2) * 0.5;
    double lat_rad = avg_lat * PI / 180.0;
    
    double meters_per_deg_lat = 111139.0;
    double meters_per_deg_lon = 111320.0 * cos(lat_rad);
    
    out_dx = (lon2 - lon1) * meters_per_deg_lon;
    out_dy = (lat2 - lat1) * meters_per_deg_lat;
}

// Draw the start line as a thick magenta bar between two plotted buoys (dots are drawn on top afterwards)
static void draw_start_line_segment(int x1, int y1, int x2, int y2) {
    tft.drawLine(x1, y1, x2, y2, TFT_MAGENTA);
    tft.drawLine(x1, y1 - 1, x2, y2 - 1, TFT_MAGENTA);
    tft.drawLine(x1, y1 + 1, x2, y2 + 1, TFT_MAGENTA);
    tft.drawLine(x1 - 1, y1, x2 - 1, y2, TFT_MAGENTA);
    tft.drawLine(x1 + 1, y1, x2 + 1, y2, TFT_MAGENTA);
}

// The start line length and the wind flank the N cardinal, on the row between the range banner and
// the top of the plot. Each band is cleared in full before its label is drawn: the text background
// alone cannot erase a previous, longer string, and neither label is inside the plot rectangle that
// update_radar_map_dynamic() wipes. The datum is BR/BL so both sit on the N glyph's baseline.
// Size 2 text is 12 px per character, so the left label has room for 8 and the right for 8.
void draw_line_len_label(const char *text, uint16_t color) {
    tft.fillRect(0, MAP_NROW_TOP, MAP_NROW_LEFT_R + 1, MAP_NROW_H, TFT_BLACK);
    tft.setFreeFont(NULL);
    tft.setTextPadding(0);
    tft.setTextColor(color, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(BR_DATUM);
    tft.drawString(text, MAP_NROW_LEFT_R, MAP_NROW_Y);
}

static void draw_wind_label(const char *text, uint16_t color) {
    tft.fillRect(MAP_NROW_RIGHT_L, MAP_NROW_TOP, 240 - MAP_NROW_RIGHT_L, MAP_NROW_H, TFT_BLACK);
    tft.setFreeFont(NULL);
    tft.setTextPadding(0);
    tft.setTextColor(color, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(BL_DATUM);
    tft.drawString(text, MAP_NROW_RIGHT_L, MAP_NROW_Y);
}


// Draw one buoy marker. No B1/B2/B3 label: the dots are already colour coded
// (B1 green, B2 orange, B3 cyan) and the label only added clutter to a small plot.
static void draw_buoy_marker(int x, int y, uint16_t color, int slot) {
    tft.fillCircle(x, y, 5, color);
    tft.drawCircle(x, y, 5, TFT_WHITE);

    // The dots used to be one fixed colour each (B1 green, B2 orange, B3 cyan) and needed no
    // label. They now carry the COURSE colour instead, so three unlocked buoys are three identical
    // green dots - the number is what tells them apart.
    if (slot < 0) return;
    tft.setFreeFont(NULL);
    tft.setTextPadding(0);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextDatum(BL_DATUM);
    tft.drawString(String(slot + 1), x + 7, y - 5);
}

// Draw the waypoint a buoy was told to hold: a hollow ring with a cross through it, in that
// buoy's own colour. Hollow versus the filled buoy dot is the whole distinction - "where it
// should be" against "where it is" - so the two never need labels to tell apart.
// pegged marks a target that had to be clamped to the plot rim because it lies outside the
// range the plot covers; the extra outer ring stops it reading as a mark just off the fleet.
static void draw_waypoint_marker(int x, int y, uint16_t color, bool pegged) {
    tft.drawCircle(x, y, 4, color);
    tft.drawFastHLine(x - 7, y, 15, color);
    tft.drawFastVLine(x, y - 7, 15, color);
    if (pegged) tft.drawCircle(x, y, 7, color);
}

// Dashed leader from a buoy to its waypoint. This is the "how far off am I" line: read its
// length against the EDGE range in the banner. Dashed so it cannot be confused with the solid
// magenta start line, and drawn before the buoy dots so those stay on top.
static void draw_offset_line(int x1, int y1, int x2, int y2, uint16_t color) {
    int dx = x2 - x1;
    int dy = y2 - y1;
    int steps = max(abs(dx), abs(dy));
    if (steps <= 0) return;
    for (int s = 0; s <= steps; s++) {
        if (((s / 3) & 1) != 0) continue; // 3 pixels on, 3 off
        tft.drawPixel(x1 + (dx * s) / steps, y1 + (dy * s) / steps, color);
    }
}

// Print the offset in metres next to its waypoint, inside the plot, rather than in a legend
// row: the strip below the plot is already taken by the S cardinal and the two legends, and
// the number is easier to read against the leader it belongs to anyway. Size 1 (6x8 px) keeps
// three of them legible without crowding the marks.
static void draw_offset_label(int x, int y, float metres, uint16_t color) {
    char buf[10];
    if (metres >= 9999.0f)      snprintf(buf, sizeof(buf), ">9km");
    else if (metres >= 999.5f)  snprintf(buf, sizeof(buf), "%0.1fk", metres / 1000.0f);
    else if (metres >= 99.5f)   snprintf(buf, sizeof(buf), "%0.0fm", metres);
    else                        snprintf(buf, sizeof(buf), "%0.1fm", metres);

    int text_w = strlen(buf) * 6;

    // Offer the label the space below-right of the mark, then flip it to the other side of
    // whichever plot edge it would otherwise run past. The plot rectangle is wiped on every
    // refresh, so anything drawn outside it would leave a trail.
    int lx = x + 9;
    int ly = y + 6;
    if (lx + text_w > MAP_CX + MAP_R - 1) lx = x - 9 - text_w;
    if (ly + 8 > MAP_CY + MAP_R - 1)      ly = y - 14;
    lx = constrain(lx, MAP_CX - MAP_R + 1, MAP_CX + MAP_R - 1 - text_w);
    ly = constrain(ly, MAP_CY - MAP_R + 1, MAP_CY + MAP_R - 1 - 8);

    tft.setFreeFont(NULL);
    tft.setTextPadding(0);
    tft.setTextColor(color, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
    tft.drawString(buf, lx, ly);
}

// Draw the wind on the radar: an arrow parked on the upwind rim, blowing inwards towards the centre.
// wind_dir follows the RoboCompute convention - the compass bearing the wind blows FROM
// (recalcStartLine() places the HEAD mark in the wDir direction, i.e. upwind).
static void draw_wind_overlay(int cx, int cy) {
    // Use the first buoy that actually reports wind. A buoy that has never sent a wind
    // field reads 0/0, which is indistinguishable from a real due-north calm, so treat that as no data.
    int wind_idx = -1;
    for (int i = 0; i < 3; i++) {
        if (buoys[i].id != "" && (buoys[i].wind_dir != 0 || buoys[i].wind_std != 0)) {
            wind_idx = i;
            break;
        }
    }

    if (wind_idx == -1) {
        draw_wind_label("---", TFT_DARKGREY);
        return;
    }

    float wdir = buoys[wind_idx].wind_dir;
    float theta = wdir * PI / 180.0;
    float s = sin(theta), c = cos(theta);

    // Tail on the upwind rim, tip pointing inwards: the arrow flies with the wind.
    // Both ends stay inside the plot clear rectangle so the arrow is wiped on every refresh.
    int tail_x = cx + (int)((MAP_R - 4) * s);
    int tail_y = cy - (int)((MAP_R - 4) * c);
    int tip_x  = cx + (int)((MAP_R - 32) * s);
    int tip_y  = cy - (int)((MAP_R - 32) * c);

    // 3 pixel wide shaft
    tft.drawLine(tail_x, tail_y, tip_x, tip_y, TFT_YELLOW);
    tft.drawLine(tail_x + 1, tail_y, tip_x + 1, tip_y, TFT_YELLOW);
    tft.drawLine(tail_x - 1, tail_y, tip_x - 1, tip_y, TFT_YELLOW);
    tft.drawLine(tail_x, tail_y + 1, tip_x, tip_y + 1, TFT_YELLOW);
    tft.drawLine(tail_x, tail_y - 1, tip_x, tip_y - 1, TFT_YELLOW);

    // Arrowhead: base sits 10 pixels back up-wind of the tip, wings 6 pixels to either side
    int base_x = cx + (int)((MAP_R - 22) * s);
    int base_y = cy - (int)((MAP_R - 22) * c);
    int wing1_x = base_x + (int)(6 * c);
    int wing1_y = base_y + (int)(6 * s);
    int wing2_x = base_x - (int)(6 * c);
    int wing2_y = base_y - (int)(6 * s);
    tft.fillTriangle(tip_x, tip_y, wing1_x, wing1_y, wing2_x, wing2_y, TFT_YELLOW);

    // Right of the N, so 8 characters at size 2. Direction, then the spread when there is one:
    // "145 -3" is 145 degrees give or take 3, the same pair the compass screen shows as Wnd/Std.
    char buf[16];
    if (buoys[wind_idx].wind_std != 0) {
        sprintf(buf, "%03.0f -%0.0f", wdir, buoys[wind_idx].wind_std);
    } else {
        sprintf(buf, "%03.0f", wdir);
    }
    draw_wind_label(buf, TFT_YELLOW);
}

// Dynamically draw buoy markers on the radar map screen in real-time
void update_radar_map_dynamic() {
    int w = tft.width();
    int h = tft.height();
    int cx = MAP_CX, cy = MAP_CY, r_max = MAP_R;

    int plot_roles[3];
    compute_buoy_roles(plot_roles);

    // The navigation screens leave a text padding width behind; clear it or every
    // label below paints a wide black bar over the radar grid.
    tft.setFreeFont(NULL);
    tft.setTextPadding(0);

    // 1. The plot is centred on the geometric centre of the fleet: with 2 fixed buoys that is
    //    the middle of the start line, with 3 it is the centre of the triangle (same as
    //    RoboCompute's threePointAverage), with 1 it is simply that buoy.
    int valid_count = 0;
    double lat_orig = 0, lon_orig = 0;
    double b_lat[3], b_lon[3];
    bool b_valid[3] = {false, false, false};

    for (int i = 0; i < 3; i++) {
        if (buoys[i].id != "" && buoys[i].lat != "N/A" && buoys[i].lat != "" && atof(buoys[i].lat.c_str()) != 0) {
            b_lat[i] = atof(buoys[i].lat.c_str());
            b_lon[i] = atof(buoys[i].lon.c_str());
            b_valid[i] = true;

            lat_orig += b_lat[i];
            lon_orig += b_lon[i];
            valid_count++;
        }
    }

    if (valid_count > 0) {
        lat_orig /= valid_count;
        lon_orig /= valid_count;
    }

    // 1b. The waypoint each buoy is steering at, and how far it currently is from it. Needs the
    //     buoy's own fix as well as the target, because the whole point is to draw the gap
    //     between the two - a target with no buoy to compare it against says nothing.
    double wp_lat[3], wp_lon[3];
    bool wp_valid[3] = {false, false, false};
    float wp_off_m[3] = {0, 0, 0};

    for (int i = 0; i < 3; i++) {
        if (!b_valid[i] || !buoy_has_waypoint(buoys[i])) continue;
        wp_lat[i] = buoys[i].tg_lat;
        wp_lon[i] = buoys[i].tg_lon;
        wp_valid[i] = true;

        float dx = 0, dy = 0;
        get_relative_meters(b_lat[i], b_lon[i], wp_lat[i], wp_lon[i], dx, dy);
        wp_off_m[i] = sqrt(dx * dx + dy * dy);
    }

    // Clear only the plot area (X: 48 to 192, Y: 68 to 212) first to prevent trails!
    tft.fillRect(cx - r_max, cy - r_max, r_max * 2 + 1, r_max * 2 + 1, TFT_BLACK);

    // Redraw the N-up crosshair inside the cleared region (no windrose rings)
    tft.drawFastHLine(cx - r_max, cy, r_max * 2, TFT_DARKGREY);
    tft.drawFastVLine(cx, cy - r_max, r_max * 2, TFT_DARKGREY);

    // Wind first, so the start line and the buoy dots are drawn on top of the arrow.
    // Covers the GPS, Demo and no-telemetry paths alike.
    draw_wind_overlay(cx, cy);

    if (valid_count == 0) {
        // Nothing below this point produces a real length, and a stale one would leave - / + live
        // over a plot that is either mocked up or empty.
        track_line_cur_m = -1.0f;
        track_line_a = -1;
        track_line_b = -1;
        track_line_tgt_m = -1.0f;

        // Check if any buoy is active/present
        bool any_present = false;
        for (int i = 0; i < 3; i++) {
            if (buoys[i].id != "") {
                any_present = true;
                break;
            }
        }
        
        if (any_present) {
            // No GPS fix yet, but buoys are present! Let's show a beautiful Demo Mode plot so they can verify!
            tft.fillRect(0, MAP_HEADER_Y, 240, 16, TFT_BLACK);
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextDatum(TC_DATUM);
            tft.drawString("DEMO (no fix)", cx, MAP_HEADER_Y);

            // Mock positions: a typical course layout, B1/B2 on the start line, B3 as the upwind HEAD mark
            int mock_offsets_x[3] = {-36, 36, 0};
            int mock_offsets_y[3] = {32, 32, -45};
            uint16_t buoy_colors[3];
            for (int i = 0; i < 3; i++) buoy_colors[i] = buoy_role_color(i, plot_roles);

            int mock_x[3], mock_y[3];
            bool mock_valid[3] = {false, false, false};
            int mock_count = 0, sum_x = 0, sum_y = 0;

            for (int i = 0; i < 3; i++) {
                if (buoys[i].id != "") {
                    mock_valid[i] = true;
                    sum_x += mock_offsets_x[i];
                    sum_y += mock_offsets_y[i];
                    mock_count++;
                }
            }

            // Centre the mock plot on the middle of whatever is present, exactly like the GPS path does
            for (int i = 0; i < 3; i++) {
                if (mock_valid[i]) {
                    mock_x[i] = cx + mock_offsets_x[i] - sum_x / mock_count;
                    mock_y[i] = cy + mock_offsets_y[i] - sum_y / mock_count;
                }
            }

            // Start line first, so the buoy dots end up on top of it
            int ma = -1, mb = -1;
            long best_d2 = -1;
            for (int i = 0; i < 3; i++) {
                for (int j = i + 1; j < 3; j++) {
                    if (!mock_valid[i] || !mock_valid[j]) continue;
                    long ddx = mock_x[i] - mock_x[j];
                    long ddy = mock_y[i] - mock_y[j];
                    long d2 = ddx * ddx + ddy * ddy;
                    if (best_d2 < 0 || d2 < best_d2) { best_d2 = d2; ma = i; mb = j; }
                }
            }
            if (ma != -1) {
                draw_start_line_segment(mock_x[ma], mock_y[ma], mock_x[mb], mock_y[mb]);
                draw_line_len_label("DEMO", TFT_MAGENTA);
            } else {
                draw_line_len_label("--", TFT_DARKGREY);
            }

            for (int i = 0; i < 3; i++) {
                if (mock_valid[i]) {
                    draw_buoy_marker(mock_x[i], mock_y[i], buoy_colors[i], i);
                }
            }
            return;
        } else {
            // No buoy has a valid GPS fix yet and no buoy is active!
            tft.fillRect(0, MAP_HEADER_Y, 240, 16, TFT_BLACK);
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextDatum(MC_DATUM);
            tft.drawString("AWAITING", cx, cy - 12);
            tft.drawString("TELEMETRY", cx, cy + 12);
            draw_line_len_label("--", TFT_DARKGREY);
            return;
        }
    }
    
    // 2. Scan all buoys to find the maximum relative distance (dx or dy) from the origin to dynamically scale the plot
    float max_dist = 10.0; // Default min scale of 10 meters
    for (int i = 0; i < 3; i++) {
        if (b_valid[i]) {
            float dx, dy;
            get_relative_meters(lat_orig, lon_orig, b_lat[i], b_lon[i], dx, dy);
            float d = sqrt(dx*dx + dy*dy);
            if (d > max_dist) max_dist = d;
        }
    }

    // Waypoints widen the range too, so a buoy that is well off station still has its mark on
    // screen - but only out to MAP_WAYPOINT_MAX_M. A target further away than that is a fault
    // rather than a course (RoboTop's dock position leaking into tgLat puts it kilometres away),
    // and zooming out to fit it would squash the whole fleet into one pixel. Those are clamped
    // to the rim by the constrain below and drawn with the pegged ring instead.
    for (int i = 0; i < 3; i++) {
        if (wp_valid[i]) {
            float dx, dy;
            get_relative_meters(lat_orig, lon_orig, wp_lat[i], wp_lon[i], dx, dy);
            float d = sqrt(dx*dx + dy*dy);
            if (d > max_dist && d <= MAP_WAYPOINT_MAX_M) max_dist = d;
        }
    }

    // Determine the optimal discrete scale range (25m, 50m, 100m, 250m, 500m, 1000m)
    float scale_range = 50.0;
    if (max_dist > 500.0) scale_range = 1000.0;
    else if (max_dist > 250.0) scale_range = 500.0;
    else if (max_dist > 100.0) scale_range = 250.0;
    else if (max_dist > 50.0) scale_range = 100.0;
    else if (max_dist > 25.0) scale_range = 50.0;
    else scale_range = 25.0;
    
    // Scale factor: pixels per meter (MAP_R pixels from the centre to the edge = scale_range meters)
    float S = (float)MAP_R / scale_range;

    // Print the dynamic range scale on screen (cleared in full, a shorter string cannot erase a longer one)
    tft.fillRect(0, MAP_HEADER_Y, 240, 16, TFT_BLACK);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(TC_DATUM);
    char scale_buf[24];
    sprintf(scale_buf, "EDGE = %0.0fm", scale_range);
    tft.drawString(scale_buf, cx, MAP_HEADER_Y);

    // 3. Convert every valid buoy position into screen pixels.
    //    Same colours as the menu buttons: green until the fleet is locked, then starboard green,
    //    port red, head blue. That costs the plot its old fixed green/orange/cyan per slot, which
    //    was the only thing telling the dots apart - so draw_buoy_marker() labels them now.
    uint16_t buoy_colors[3];
    for (int i = 0; i < 3; i++) buoy_colors[i] = buoy_role_color(i, plot_roles);

    int plot_x[3], plot_y[3];

    for (int i = 0; i < 3; i++) {
        if (b_valid[i]) {
            float dx = 0, dy = 0;
            get_relative_meters(lat_orig, lon_orig, b_lat[i], b_lon[i], dx, dy);

            // Map to screen pixels relative to center
            plot_x[i] = cx + (int)(dx * S);
            plot_y[i] = cy - (int)(dy * S); // Invert Y because screen coordinates increase downwards

            // Constrain plot coordinates within the radar boundaries
            plot_x[i] = constrain(plot_x[i], cx - r_max + 2, cx + r_max - 2);
            plot_y[i] = constrain(plot_y[i], cy - r_max + 2, cy + r_max - 2);
        }
    }

    // 3b. Same conversion for the waypoints. A mark that lands outside the plot is clamped to
    //     the rim rather than dropped, so "off station, that way" still reads at a glance;
    //     wp_pegged records that it was clamped, and the metre label keeps the true figure.
    int wp_x[3], wp_y[3];
    bool wp_pegged[3] = {false, false, false};

    for (int i = 0; i < 3; i++) {
        if (!wp_valid[i]) continue;

        float dx = 0, dy = 0;
        get_relative_meters(lat_orig, lon_orig, wp_lat[i], wp_lon[i], dx, dy);

        int px = cx + (int)(dx * S);
        int py = cy - (int)(dy * S);

        wp_x[i] = constrain(px, cx - r_max + 2, cx + r_max - 2);
        wp_y[i] = constrain(py, cy - r_max + 2, cy + r_max - 2);
        wp_pegged[i] = (wp_x[i] != px || wp_y[i] != py);
    }

    // 4. Draw the start line between the two buoys that are closest together.
    // Same rule RoboCompute::recalcStartLine() applies: the shortest leg is the start line,
    // the remaining buoy is the HEAD (upwind) mark.
    int sl_a = -1, sl_b = -1;
    float sl_dist = 0;

    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 3; j++) {
            if (!b_valid[i] || !b_valid[j]) continue;

            float dx = 0, dy = 0;
            get_relative_meters(b_lat[i], b_lon[i], b_lat[j], b_lon[j], dx, dy);
            float d = sqrt(dx * dx + dy * dy);

            if (sl_a == -1 || d < sl_dist) {
                sl_dist = d;
                sl_a = i;
                sl_b = j;
            }
        }
    }

    if (sl_a != -1) {
        // Line first, so the buoy dots are drawn on top of it
        draw_start_line_segment(plot_x[sl_a], plot_y[sl_a], plot_x[sl_b], plot_y[sl_b]);

        // The figure left of the N is the length the - / + buttons work on, so it is measured off
        // the two ends' WAYPOINTS where they have one rather than off the dots plotted above. The
        // dots wander inside the hold radius; the waypoints do not, and EXECUTE has to be able to
        // put the line back where it was but longer. The two coincide once the buoys arrive.
        //
        // Without both end points there is a line to look at but nothing to reposition, so the
        // length is still shown and track_line_a/b are left clear - which is what greys out
        // - / + / EXECUTE rather than letting them fire on a target that cannot be computed.
        double alat, alon, blat, blon;
        if (buoy_line_point(buoys[sl_a], alat, alon) && buoy_line_point(buoys[sl_b], blat, blon)) {
            track_line_cur_m = (float)track_distance_m(alat, alon, blat, blon);
            track_line_a = sl_a;
            track_line_b = sl_b;
        } else {
            track_line_cur_m = sl_dist;
            track_line_a = -1;
            track_line_b = -1;
            track_line_tgt_m = -1.0f;
        }

        // A pending change takes over the label until EXECUTE commits it or BACK drops it.
        char legend[12];
        if (track_line_tgt_m > 0) {
            snprintf(legend, sizeof(legend), "%0.0fM", track_line_tgt_m);
            draw_line_len_label(legend, TFT_ORANGE);
        } else {
            snprintf(legend, sizeof(legend), "%0.0fM", track_line_cur_m);
            draw_line_len_label(legend, TFT_MAGENTA);
        }
    } else {
        track_line_cur_m = -1.0f;
        track_line_a = -1;
        track_line_b = -1;
        track_line_tgt_m = -1.0f;
        draw_line_len_label("--", TFT_DARKGREY);
    }

    // 4b. Each buoy's waypoint and the gap to it, under the buoy dots so the live position
    //     always wins where the two overlap - which is exactly what "on station" looks like.
    for (int i = 0; i < 3; i++) {
        if (!wp_valid[i]) continue;
        draw_offset_line(plot_x[i], plot_y[i], wp_x[i], wp_y[i], buoy_colors[i]);
        draw_waypoint_marker(wp_x[i], wp_y[i], buoy_colors[i], wp_pegged[i]);
        draw_offset_label(wp_x[i], wp_y[i], wp_off_m[i], buoy_colors[i]);
    }

    // 5. Plot each valid active buoy on top of the start line
    for (int i = 0; i < 3; i++) {
        if (b_valid[i]) {
            draw_buoy_marker(plot_x[i], plot_y[i], buoy_colors[i], i);
        }
    }
}

/**
 * @brief Opens the Manual Fourier Calibration screen for a buoy.
 *
 * Both ways into the screen - tapping the MAN CAL box and confirming it with + - run this, so the
 * two cannot drift apart again. The order of the three commands is what makes the screen open with
 * the buoy's own corrections on it instead of a row of zeros:
 *
 *   1. IDLE first. The screen steers nothing until a direction is picked, so whatever the buoy was
 *      doing has to stop here rather than at the first tap.
 *   2. Ask for the interpolation table WHILE the harmonic correction is on - switching it on
 *      first, unconditionally. The Sub answers with the table that is in effect, not the one in NVS (see
 *      STORE_INTERPOLATION_TABLE in RoboSub/src/main.cpp), so with the correction off it reports
 *      the identity table and every offset reads 0 - the reason the stored values never showed up.
 *   3. Switch the correction off only once that answer is in, so the dialing that follows works on
 *      the raw compass. That last step is finished by loop(), see mancal_harmonic_pending.
 *
 * Both ways out of the screen - BACK / CANCEL and SAVE ALL & EXIT - leave the correction ON.
 */
void enter_man_fourier_cal(int buoy_idx) {
    BuoyData &b = buoys[buoy_idx];

    in_man_fourier_cal_mode = true;
    in_setup_mode = false;
    mancal_selected_leg = -1; // -1 means NO active steering yet!
    for (int i = 0; i < 8; i++) mancal_offsets[i] = 0.0f;
    for (int i = 0; i < 8; i++) mancal_leg_visited[i] = false;
    mancal_offsets_loaded = false;
    mancal_is_dirty = true;
    // Default to the flow this screen has always had, so a session that never touches the switch
    // behaves exactly as before.
    mancal_thrusters_on = true;

    // 1. Stop the thrusters. Entering this screen never commands a heading.
    send_buoy_command(b.id, 8);
    delay(200); // Prevent serial queue collisions and allow IDLE to process!

    // 2. Switch the correction ON before reading. Unconditionally: a buoy whose correction is off
    //    answers with the identity table, and b.harmonic_enabled is only a cached copy that can be
    //    stale (anything that changes the flag by another route - the Sub's own web page, another
    //    client - does not reach this screen). Its table is still in NVS either way, only the
    //    reporting of it depends on the flag, so this makes the stored corrections readable again
    //    even on a buoy an earlier session left switched off.
    b.harmonic_enabled = true;
    send_buoy_setup(buoy_idx);
    delay(400); // let it take effect before the table is asked for

    // 3. Query the buoy for its currently in-use 8-point fourier table (Command 88)
    send_buoy_command(b.id, 88, 1); // 1 = GET
    mancal_query_tries = 1;
    mancal_query_next_ms = millis() + MANCAL_QUERY_INTERVAL_MS;
    mancal_harmonic_pending = true;

    // Both timers start now, not before the delay above, or half the lockout is already gone.
    mancal_started_ms = millis();
    mancal_await_release = true;
    last_transition_ms = millis();
}

/**
 * @brief Finishes the entry sequence once the buoy has reported its table.
 *
 * Kept out of enter_man_fourier_cal() so waiting for the answer does not block the UI. Retries the
 * query, and gives up after MANCAL_QUERY_MAX_TRIES rather than leaving the screen unusable - a buoy
 * that never answers still has to be dialable, it just starts from an unknown baseline.
 */
void service_mancal_entry() {
    // Chase the switch-off until the buoy's own telemetry confirms it. One lost frame used to be
    // enough to ruin a whole calibration with no sign that anything was wrong.
    if (in_man_fourier_cal_mode && mancal_await_harmonic_off &&
        selected_buoy_idx >= 0 && selected_buoy_idx < 3) {
        BuoyData &hb = buoys[selected_buoy_idx];
        // hb.harmonic_reported, NOT hb.harmonic_enabled: the latter is our own intent, which
        // this code sets to false itself when it sends the switch-off, so testing it confirmed
        // nothing and let captures run against a still-corrected compass.
        if (!hb.harmonic_reported && hb.harmonic_reported_ms != 0) {
            mancal_await_harmonic_off = false;   // the BUOY says off - captures may proceed
            mancal_is_dirty = true;
        } else if ((long)(millis() - mancal_harmonic_off_next_ms) >= 0 &&
                   mancal_harmonic_off_tries < MANCAL_HARMONIC_MAX_TRIES) {
            mancal_harmonic_off_next_ms = millis() + MANCAL_HARMONIC_RETRY_MS;
            mancal_harmonic_off_tries++;
            Serial.printf("Harmonic correction still ON, re-sending switch-off %d of %d\n",
                          mancal_harmonic_off_tries, MANCAL_HARMONIC_MAX_TRIES);
            hb.harmonic_enabled = false;
            hb.compass_trim_enabled = false;
            send_buoy_setup(selected_buoy_idx);
            // A GET, so the buoy answers with what it is ACTUALLY running. Without this nothing
            // ever refreshes harmonic_reported and the wait could not end either way.
            query_buoy_setup(hb.id);
        }
    }

    if (!in_man_fourier_cal_mode || !mancal_harmonic_pending) return;
    if (selected_buoy_idx < 0 || selected_buoy_idx >= 3) return;
    BuoyData &b = buoys[selected_buoy_idx];
    if (b.id.length() == 0) return;

    bool give_up = (mancal_query_tries >= MANCAL_QUERY_MAX_TRIES &&
                    (long)(millis() - mancal_query_next_ms) >= 0);

    if (mancal_offsets_loaded || give_up) {
        if (give_up && !mancal_offsets_loaded) {
            Serial.println("Fourier table never arrived - starting manual calibration from zero.");
            // Zero is the baseline we are going to dial from, so stop calling it unknown - from
            // here on the readouts are honest again.
            mancal_offsets_loaded = true;
        }
        // Only now: with the correction off the compass is raw, which is what the +/- dialing and
        // the table written on SAVE are both defined against.
        // Ask for the correction to be switched off - but do NOT declare it done. The flag
        // rides inside a full SETUPDATA SET, and if that frame is lost or refused anywhere along
        // the way the buoy carries on correcting while this screen believes it is not. That is not
        // a theoretical risk: it happened, all eight points were captured against a corrected
        // compass, and nothing said a word because this used to clear the flag right here.
        // Adaptive trim off as well as the harmonic correction. It is a bias added AFTER the
        // correction curve and it drifts on its own - one of these buoys was carrying 4.1 degrees
        // of it - so leaving it on puts a moving error into every one of the eight captures.
        // Both go back on when the screen is left, by either exit.
        b.harmonic_enabled = false;
        b.compass_trim_enabled = false;
        send_buoy_setup(selected_buoy_idx);
        mancal_harmonic_pending = false;
        query_buoy_setup(b.id);
        mancal_await_harmonic_off = true;
        mancal_harmonic_off_next_ms = millis() + MANCAL_HARMONIC_RETRY_MS;
        mancal_harmonic_off_tries = 0;
        mancal_is_dirty = true;
        return;
    }

    if ((long)(millis() - mancal_query_next_ms) >= 0) {
        mancal_query_tries++;
        mancal_query_next_ms = millis() + MANCAL_QUERY_INTERVAL_MS;
        Serial.printf("Fourier table not received, retry %d of %d\n", mancal_query_tries, MANCAL_QUERY_MAX_TRIES);
        send_buoy_command(b.id, 88, 1); // 1 = GET
    }
}

/**
 * @brief Draws the whole 8-point correction table as one row under the compass rose.
 *
 * The big CORR readout only ever shows the direction being worked on, so without this the other
 * seven corrections the buoy is already running are invisible - and a screen that shows nothing
 * reads as a buoy with no calibration. Cells are in rose order, N first and clockwise, so a cell
 * lines up with the dot above it.
 */
void draw_mancal_offset_strip() {
    // Sits in the band between the rose's direction labels (which end at y 165) and the TG line
    // that starts at y 178.
    const int y = 173;
    tft.fillRect(0, 167, tft.width(), 12, TFT_BLACK);

    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);

    if (!mancal_offsets_loaded) {
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.drawString("reading table from buoy...", tft.width() / 2, y);
        return;
    }

    for (int i = 0; i < 8; i++) {
        char buf[8];
        sprintf(buf, "%+0.0f", mancal_offsets[i]);
        // Green = being worked on, light = confirmed this session, dim = still the value that came
        // off the buoy and not yet visited. That last distinction is what SAVE is waiting for.
        uint16_t col = (i == mancal_selected_leg) ? TFT_GREEN
                     : (mancal_leg_visited[i] ? TFT_LIGHTGREY : TFT_DARKGREY);
        tft.setTextColor(col, TFT_BLACK);
        tft.drawString(buf, 15 + i * 30, y);
    }
}

void draw_mancal_static() {
    int w = tft.width();
    tft.fillScreen(TFT_BLACK);
    
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("MAN COMPASS CAL", w / 2, 15);
    
    tft.drawFastHLine(15, 45, w - 30, TFT_WHITE);
    
    mancal_is_dirty = true;
}

void send_mancal_table_to_sub(int buoy_idx) {
    BuoyData &b = buoys[buoy_idx];
    // Target target, Sender 98, ACK SET (2), CMD 88 (STORE_INTERPOLATION_TABLE), status
    // followed by the 8 computed table values
    char cmdPayload[256];
    float table[8];
    for (int i = 0; i < 8; i++) {
        float expected = i * 45.0f;
        float off = mancal_offsets[i];
        float val = expected - off;
        while (val < 0.0f) val += 360.0f;
        while (val >= 360.0f) val -= 360.0f;
        table[i] = val;
    }
    
    sprintf(cmdPayload, "%s,98,2,88,7,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f,%0.2f",
            b.id.c_str(), table[0], table[1], table[2], table[3], table[4], table[5], table[6], table[7]);
            
    // Wrapped like every other command this firmware sends. Without it the frame goes out bare,
    // and rfDeCode() (RoboCompute.cpp:607) drops anything that does not start with $ and carry a
    // valid *CRC before it looks at a single field - so on both the Top's UDP and LoRa paths this
    // was discarded on arrival and SAVE ALL & EXIT never delivered the table at all.
    uint8_t crc = calculate_crc(cmdPayload);
    char finalPacket[320];
    sprintf(finalPacket, "$%s*%02X", cmdPayload, crc);

    send_lora_packet(finalPacket);
    udp_broadcast(finalPacket);
    Serial.printf("Transmitted STORE_INTERPOLATION_TABLE (88) SET to Buoy: %s\n", b.id.c_str());
}

void update_mancal_dynamic() {
    int w = tft.width();
    BuoyData &b = buoys[selected_buoy_idx];
    
    // Draw the 8 dots in a circle (MOVED UP by shifting cy from 125 to 110!)
    int cx = 120;
    int cy = 110;
    int r_dots = 38;
    
    // Static drawing elements that we only redraw if dirty
    if (mancal_is_dirty) {
        tft.fillRect(10, 48, w - 20, 130, TFT_BLACK); // Clear circle area
        tft.fillRect(0, 180, w, 140, TFT_BLACK); // Cleanly clear lower labels, buttons and footers to prevent overlaps!
        
        // Draw static MAG and CORR labels in tiny text above the values
        tft.setTextSize(1);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        
        tft.setTextDatum(TL_DATUM);
        tft.drawString("MAG", 15, 48);
        
        tft.setTextDatum(TR_DATUM);
        tft.drawString("CORR", w - 15, 48);

        // The mode, kept visible for the whole session. Once a direction is picked the switch
        // below is replaced by the Set as North button, and without this there would be nothing
        // on screen saying whether a -/+ press moves the buoy or only edits a number.
        // Left of centre, not on it: the rose's "N" label sits at (120, 59) and is drawn after
        // this with an opaque background, so a centred tag would have its bottom rows eaten.
        tft.setTextDatum(TL_DATUM);
        tft.setTextColor(mancal_thrusters_on ? TFT_RED : TFT_GREEN, TFT_BLACK);
        tft.drawString(mancal_thrusters_on ? "THR ON" : "THR OFF", 62, 48);
        
        // Draw dots and directional labels
        for (int i = 0; i < 8; i++) {
            int angle_deg = i * 45;
            float angle_rad = angle_deg * (float)M_PI / 180.0f;
            int x = cx + (int)(r_dots * sin(angle_rad));
            int y = cy - (int)(r_dots * cos(angle_rad));
            
            bool is_selected = (i == mancal_selected_leg);
            bool is_locked = mancal_leg_locked(i);
            uint16_t dimmed = tft.color565(55, 55, 55);
            if (is_selected) {
                tft.fillCircle(x, y, 6, TFT_GREEN);
                tft.drawCircle(x, y, 8, TFT_WHITE);
            } else if (is_locked) {
                // Barely there: these cannot be picked until North has been done.
                tft.fillCircle(x, y, 3, dimmed);
            } else {
                tft.fillCircle(x, y, 4, TFT_DARKGREY);
                tft.drawCircle(x, y, 5, TFT_BLACK);
                // North, still waiting to be pressed - ring it so it reads as the way in.
                if (i == 0 && !mancal_leg_visited[0]) {
                    tft.drawCircle(x, y, 8, TFT_YELLOW);
                }
            }
            
            int r_label = r_dots + 13;
            int lx = cx + (int)(r_label * sin(angle_rad));
            int ly = cy - (int)(r_label * cos(angle_rad));
            
            tft.setTextSize(1);
            tft.setTextColor(is_selected ? TFT_GREEN
                             : is_locked ? dimmed
                             : (i == 0 && !mancal_leg_visited[0]) ? TFT_YELLOW
                             : TFT_LIGHTGREY, TFT_BLACK);
            tft.setTextDatum(MC_DATUM);
            
            const char* label = "";
            switch (angle_deg) {
                case 0:   label = "N"; break;
                case 45:  label = "NE"; break;
                case 90:  label = "E"; break;
                case 135: label = "SE"; break;
                case 180: label = "S"; break;
                case 225: label = "SW"; break;
                case 270: label = "W"; break;
                case 315: label = "NW"; break;
            }
            tft.drawString(label, lx, ly);
        }
    }
    
    // The whole table under the rose. Repainted on its own rather than every pass, or the row
    // would flicker at the 4 Hz refresh rate.
    static bool last_strip_loaded = false;
    static int last_strip_leg = -2;
    if (mancal_is_dirty || last_strip_loaded != mancal_offsets_loaded || last_strip_leg != mancal_selected_leg) {
        last_strip_loaded = mancal_offsets_loaded;
        last_strip_leg = mancal_selected_leg;
        draw_mancal_offset_strip();
    }

    // Dynamic compass arrow of buoy's actual mag heading
    static float last_draw_mag_dir = -999.0f;
    if (mancal_is_dirty || b.mag_dir != last_draw_mag_dir) {
        // Erase old arrow
        if (last_draw_mag_dir != -999.0f) {
            draw_compass_arrow(cx, cy, r_dots - 8, last_draw_mag_dir, TFT_BLACK);
        }
        // Draw new arrow in Green representing what the buoy's compass reports
        draw_compass_arrow(cx, cy, r_dots - 8, b.mag_dir, TFT_GREEN);
        
        // Draw center pivot dot ON TOP of the arrow so it is never obscured or overwritten!
        tft.fillCircle(cx, cy, 3, TFT_WHITE);
        
        last_draw_mag_dir = b.mag_dir;
    }
    
    // Draw raw magnetic heading and correction factor in top-left and top-right of screen (Y: 60)
    // Clear old values first to prevent character ghosting
    tft.fillRect(10, 58, 48, 18, TFT_BLACK); 
    tft.fillRect(w - 58, 58, 48, 18, TFT_BLACK);
    
    tft.setTextSize(2); // Large bold numbers!
    
    // Left-aligned Mag heading value
    tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);
    char mag_buf[16];
    sprintf(mag_buf, "%0.0f", b.mag_dir);
    tft.drawString(mag_buf, 15, 60);
    
    // Right-aligned Correction factor value
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    tft.setTextDatum(TR_DATUM);
    char corr_buf[16];
    if (!mancal_offsets_loaded) {
        // Not "0" - that is a calibration value in its own right, and printing it before the buoy
        // has answered claims a correction we have not been told about yet.
        sprintf(corr_buf, "?");
    } else if (mancal_selected_leg == -1) {
        sprintf(corr_buf, "-");
    } else {
        sprintf(corr_buf, "%+0.0f", mancal_active_offset());
    }
    tft.drawString(corr_buf, w - 15, 60);
    
    // Draw the Port (BB) and Starboard (SB) motor power speedbars dynamically centered at Y: 110
    static float last_bb_mancal = -999.0f;
    static float last_sb_mancal = -999.0f;
    int mid_y = 110;
    
    if (mancal_is_dirty || b.bb_power != last_bb_mancal) {
        last_bb_mancal = b.bb_power;
        tft.fillRect(15, mid_y - 30, 12, 60, TFT_BLACK); // Clear BB speedbar area
        tft.drawFastHLine(15, mid_y, 12, TFT_DARKGREY); // Reset centerline
        if (b.bb_power > 0) {
            int fill_h = (b.bb_power * 30) / 100;
            tft.fillRect(16, mid_y - fill_h, 10, fill_h, TFT_GREEN);
        } else if (b.bb_power < 0) {
            int fill_h = (-b.bb_power * 30) / 100;
            tft.fillRect(16, mid_y, 10, fill_h, TFT_RED);
        }
    }
    
    if (mancal_is_dirty || b.sb_power != last_sb_mancal) {
        last_sb_mancal = b.sb_power;
        tft.fillRect(213, mid_y - 30, 12, 60, TFT_BLACK); // Clear SB speedbar area
        tft.drawFastHLine(213, mid_y, 12, TFT_DARKGREY); // Reset centerline
        if (b.sb_power > 0) {
            int fill_h = (b.sb_power * 30) / 100;
            tft.fillRect(214, mid_y - fill_h, 10, fill_h, TFT_GREEN);
        } else if (b.sb_power < 0) {
            int fill_h = (-b.sb_power * 30) / 100;
            tft.fillRect(214, mid_y, 10, fill_h, TFT_RED);
        }
    }
    
    // Draw state texts, buttons if dirty
    if (mancal_is_dirty) {
        if (mancal_selected_leg == -1) {
            // TARGET text (Setpoint) - "TG: TAP DIRECTION"
            tft.setTextColor(TFT_CYAN, TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextDatum(MC_DATUM);
            tft.drawString("TG: TAP NORTH (N)", w / 2, 186);
            
            // Current Offset Text - "CORR: -" in size 2
            tft.setTextSize(2);
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            tft.drawString("CORR: -", w / 2, 203);
            
            // Nudge buttons plus SET; see draw_mancal_adjust_row().
            draw_mancal_adjust_row(false);
            
            // Nothing is picked yet, so this row is the thruster switch. The prompt it used to
            // carry is not lost: the TG line above already reads "TAP NORTH (N)" and North is the
            // only dot on the rose that is ringed and pressable.
            draw_mancal_thruster_toggle();
        } else {
            // TARGET text (Setpoint) - Larger and Light Blue (TFT_CYAN)
            tft.setTextColor(TFT_CYAN, TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextDatum(MC_DATUM);
            char lbl_buf[32];
            const char* dirs[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
            sprintf(lbl_buf, "TG: %s (%d deg)", dirs[mancal_selected_leg], mancal_selected_leg * 45);
            tft.drawString(lbl_buf, w / 2, 186);
            
            // Current Offset Text (In large size 2 text, colored Yellow)
            tft.setTextSize(2);
            tft.setTextColor(TFT_YELLOW, TFT_BLACK);
            char off_buf[32];
            sprintf(off_buf, "CORR: %+0.0f deg", mancal_active_offset());
            tft.drawString(off_buf, w / 2, 203);
            
            // Nudge buttons plus SET; see draw_mancal_adjust_row().
            draw_mancal_adjust_row(true);
            
            // North Button Row (Y: 260 to 295). There is no STORE here: it wrote the same full
            // 8-point table as SAVE ALL & EXIT, so it was a second commit path that bypassed the
            // "all 8 visited" gate while its label implied it only stored the current direction.
            if (mancal_selected_leg == 0) {
                // SET AS NORTH button, now spanning the row on its own
                tft.fillRoundRect(10, 260, w - 20, 35, 5, TFT_ORANGE);
                tft.setTextColor(TFT_BLACK, TFT_ORANGE);
                tft.setTextSize(2);
                tft.drawString("Set as North", w / 2, 277);
            } else {
                // Nothing to press for the other directions - say what happens instead, so the
                // empty band does not read as a button that failed to draw.
                tft.setTextSize(1);
                tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
                tft.drawString(mancal_thrusters_on ? "dial with -/+   SAVE writes all 8"
                                                  : "aim it, then SET   SAVE writes all 8", w / 2, 277);
            }
        }
        
        // Once every direction has been captured there is nothing left to dial, so the whole
        // bottom of the screen becomes the decision: commit the table, or throw it away. A 38 px
        // green bar is hard to miss and hard to hit by accident, which is what you want for the
        // one press that overwrites the buoy's calibration.
        //
        // "SAVE NEW FOURIER TABLE" is 22 characters - 264 px at size 2, wider than the screen - so
        // it is stacked over two lines rather than shrunk to size 1 and lost.
        if (mancal_all_legs_visited()) {
            tft.fillRect(0, 254, w, 66, TFT_BLACK);

            tft.fillRoundRect(10, 256, w - 20, 38, 5, TFT_GREEN);
            tft.setTextColor(TFT_BLACK, TFT_GREEN);
            tft.setTextSize(2);
            tft.setTextDatum(MC_DATUM);
            tft.drawString("SAVE NEW", w / 2, 267);
            tft.drawString("FOURIER TABLE", w / 2, 285);

            tft.fillRoundRect(10, 298, w - 20, 20, 4, TFT_BLUE);
            tft.setTextColor(TFT_WHITE, TFT_BLUE);
            tft.setTextSize(1);
            tft.drawString("CANCEL - discard and leave", w / 2, 308);

            mancal_is_dirty = false;
            uint16_t udpDot = traffic_dot_color(last_udp_tx_ms, last_udp_sel_blink_ms, 100, TFT_GREEN);
            tft.fillCircle(218, 15, 4, udpDot);
            uint16_t loraDot = traffic_dot_color(last_lora_tx_ms, last_lora_blink_ms, 300, TFT_CYAN);
            tft.fillCircle(232, 15, 4, loraDot);
            return;
        }

        // Footer Control Buttons (Y: 300 to 320)
        tft.setTextSize(1);
        tft.fillRoundRect(10, 300, 105, 20, 4, TFT_BLUE);
        tft.setTextColor(TFT_WHITE, TFT_BLUE);
        tft.drawString("BACK / CANCEL", 62, 310);
        
        // SAVE writes all eight entries at once, so it stays locked until every direction has
        // been steered to at least once - committing a table with sectors that were never looked
        // at would overwrite good ones on the buoy with untouched buffer values. The count tells
        // the operator how many are still to do rather than just refusing the press.
        if (mancal_all_legs_visited()) {
            tft.fillRoundRect(125, 300, 105, 20, 4, TFT_DARKGREY);
            tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
            tft.drawString("SAVE ALL & EXIT", 177, 310);
        } else {
            tft.fillRoundRect(125, 300, 105, 20, 4, TFT_BLACK);
            tft.drawRoundRect(125, 300, 105, 20, 4, TFT_DARKGREY);
            tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
            char save_buf[24];
            sprintf(save_buf, "SAVE %d/8", mancal_visited_count());
            tft.drawString(save_buf, 177, 310);
        }
    }
    
    mancal_is_dirty = false;
    
    // Draw real-time traffic dots (UDP and LoRa) in the top-right corner of the header (Y: 15)
    uint16_t udpDotColor = traffic_dot_color(last_udp_tx_ms, last_udp_sel_blink_ms, 100, TFT_GREEN);
    tft.fillCircle(218, 15, 4, udpDotColor); // Shifted right to 218!
    uint16_t loraDotColor = traffic_dot_color(last_lora_tx_ms, last_lora_blink_ms, 300, TFT_CYAN);
    tft.fillCircle(232, 15, 4, loraDotColor); // Shifted right to 232!
}