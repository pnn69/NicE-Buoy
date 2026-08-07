#include <Arduino.h>
#include <WiFi.h>
#include "cyd_display.h"
#include "cyd_touch.h"
#include "cyd_wifi.h"
#include "cyd_lora.h"
#include "buoy_data.h"
#include "RGBledDriver.h"

unsigned long lastUIUpdate = 0;
int lastKnownState = -2; // Used to trigger a static redraw on state change
bool lastSetupState = false; // Tracks setup screen state transitions
bool lastMannavState = false; // Tracks manual navigation screen state transitions
bool lastLoadedState = false; // Tracks setup data loaded transitions

// Setup page index (0 for Page 1, 1 for Page 2)
int setup_page = 0;
int lastSetupPage = -1; // Used to track page swaps

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
int cal_state = 0;
int cal_rx1 = 0, cal_ry1 = 0;
int cal_rx2 = 0, cal_ry2 = 0;

void draw_calibration_screen();
void handle_touch_calibration();
void start_touch_calibration();

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

void draw_setup_static() {
    int w = tft.width();
    int h = tft.height();
    int idx = selected_buoy_idx;
    
    tft.fillScreen(TFT_BLACK);
    
    // Header title
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("SETUP: " + buoys[idx].id, w / 2, 5);
    
    tft.drawFastHLine(15, 27, w - 30, TFT_WHITE);
    
    // Draw Single large Plus/Minus Adjustment Buttons Row (Y: 195 to 225)
    // If not loaded yet, these buttons are drawn disabled (Dark Grey) to protect the buoy NVM!
    uint16_t adjMinusColor = setup_data_loaded ? TFT_RED : TFT_DARKGREY;
    uint16_t adjMinusText = setup_data_loaded ? TFT_WHITE : TFT_LIGHTGREY;
    tft.fillRoundRect(15, 195, 60, 30, 4, adjMinusColor);
    tft.setTextColor(adjMinusText, adjMinusColor);
    tft.setTextSize(2);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("-", 45, 210);
    
    uint16_t adjPlusColor = setup_data_loaded ? TFT_GREEN : TFT_DARKGREY;
    uint16_t adjPlusText = setup_data_loaded ? TFT_BLACK : TFT_LIGHTGREY;
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
    sprintf(pg_buf, "PG %d/3", setup_page + 1);
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
    
    // Header title
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("NAV: " + buoys[idx].id, w / 2, 5);
    
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
    
    // Draw Static Voltage Bar outlines and limits (Y: 172) - Min limit updated to 17V!
    tft.drawRect(50, 172, 140, 10, TFT_WHITE);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.setTextSize(1);
    tft.setTextDatum(TL_DATUM);
    tft.drawString("17V", 25, 173);
    tft.drawString("25V", 197, 173);
    
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
    
    // Header title
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(TC_DATUM);
    tft.drawString("MANUAL: " + buoys[idx].id, w / 2, 5);
    
    tft.drawFastHLine(15, 27, w - 30, TFT_WHITE);
    
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
    
    // --- 5. Update Blinking Telemetry Indicators (Top-Left for UDP, Top-Right for LoRa) ---
    uint16_t udpDotColor = (millis() - last_udp_blink_ms < 300) ? TFT_GREEN : TFT_BLACK;
    tft.fillCircle(15, 13, 4, udpDotColor);
    
    uint16_t loraDotColor = (millis() - last_lora_blink_ms < 300) ? TFT_CYAN : TFT_BLACK;
    tft.fillCircle(225, 13, 4, loraDotColor);
}

void draw_resting_ui() {
    int w = tft.width();
    int h = tft.height();
    
    tft.fillScreen(TFT_BLACK);
    
    if (selected_buoy_idx == -1) {
        // --- Static Menu Screen (240x320 Portrait) ---
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setTextSize(2);
        tft.setTextDatum(TC_DATUM);
        tft.drawString("ROBOBUOY", w / 2, 15);
        tft.drawString("CONTROLLER", w / 2, 40);
        
        tft.drawFastHLine(15, 70, w - 30, TFT_WHITE);
        
        // Draw Calibrate Touch Button (Y: 245 to 275, X: 30 to 210)
        tft.fillRoundRect(30, 245, 180, 28, 4, TFT_DARKGREY);
        tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
        tft.setTextSize(1);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("CALIBRATE TOUCH", w / 2, 259);
        
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.setTextSize(2);
        tft.setTextDatum(BC_DATUM);
        
        // Draw LoRa status on its own line above the IP address
        tft.drawString("LoRa: 433M", w / 2, h - 28);
        
        // Draw local IP address on its own line below the LoRa status
        IPAddress ip = WiFi.localIP();
        char ip_buf[48];
        sprintf(ip_buf, "IP: %s", ip.toString().c_str());
        tft.drawString(ip_buf, w / 2, h - 10);
    } else {
        if (in_setup_mode) {
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

void update_setup_dynamic() {
    int w = tft.width();
    int idx = selected_buoy_idx;
    BuoyData &b = buoys[idx];
    char buf[64];
    
    if (!setup_data_loaded) {
        // Display beautiful loading overlay while awaiting the NMEA response packet from buoy
        tft.setTextSize(2);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setTextDatum(MC_DATUM);
        tft.drawString("LOADING DATA", w / 2, 85);
        tft.drawString("FROM BUOY...", w / 2, 115);
        
        tft.setTextSize(1);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.drawString("Awaiting LoRa/UDP packet", w / 2, 160);
        return;
    }
    
    tft.setTextSize(1);
    tft.setTextDatum(MC_DATUM);
    
    // Array of 16 parameter names symmetrically grouped
    String names[24] = {
        "Rud P:", "Rud I:", "Rud D:", "MaxSpd:",
        "Spd P:", "Spd I:", "Spd D:", "MinSpd:",
        "PvtSpd:", "HoldRad:", "BB Inv:", "Swap:",
        "CompOff:", "SetNorth", "SB Inv:", "TrimEn:",
        "Appr Dist", "Appr Dir", "DockToWP", "",
        "", "", "", ""
    };
    
    // Draw 8 grid boxes for the current Setup Page (4 rows x 2 columns)
    for (int i = 0; i < 8; i++) {
        int r = i % 4;       // Row index (0 to 3)
        int c = i / 4;       // Col index (0 to 1)
        int x = (c == 0) ? 10 : 122;
        int y = 35 + r * 36; // Double-height boxes! (32px height, 4px gap = 36px offset)
        
        int global_idx = setup_page * 8 + i;
        
        // Highlight selected parameter box in Yellow, otherwise draw in Dark Grey
        uint16_t boxColor = (global_idx == selected_param_idx) ? TFT_YELLOW : TFT_DARKGREY;
        uint16_t textColor = (global_idx == selected_param_idx) ? TFT_YELLOW : TFT_WHITE;
        
        tft.drawRoundRect(x, y, 108, 32, 4, boxColor);
        tft.setTextColor(textColor, TFT_BLACK);
        
        if (global_idx == 13) {
            tft.drawString("SET NORTH", x + 54, y + 16);
        } else if (global_idx == 10) {
            tft.setTextColor(b.rev_bb ? TFT_GREEN : textColor, TFT_BLACK);
            sprintf(buf, "BB Inv: %s", b.rev_bb ? "YES" : "NO");
            tft.drawString(buf, x + 54, y + 16);
        } else if (global_idx == 14) {
            tft.setTextColor(b.rev_sb ? TFT_GREEN : textColor, TFT_BLACK);
            sprintf(buf, "SB Inv: %s", b.rev_sb ? "YES" : "NO");
            tft.drawString(buf, x + 54, y + 16);
        } else if (global_idx == 11) {
            tft.setTextColor(b.swap_bb_sb ? TFT_GREEN : textColor, TFT_BLACK);
            sprintf(buf, "Swap: %s", b.swap_bb_sb ? "YES" : "NO");
            tft.drawString(buf, x + 54, y + 16);
        } else if (global_idx == 15) {
            tft.setTextColor(b.compass_trim_enabled ? TFT_GREEN : textColor, TFT_BLACK);
            sprintf(buf, "TrimEn: %s", b.compass_trim_enabled ? "YES" : "NO");
            tft.drawString(buf, x + 54, y + 16);
        } else if (global_idx == 16) {
            sprintf(buf, "App Dist: %d m", b.dock_app_dist);
            tft.drawString(buf, x + 54, y + 16);
        } else if (global_idx == 17) {
            sprintf(buf, "App Dir: %d deg", b.dock_app_dir);
            tft.drawString(buf, x + 54, y + 16);
        } else if (global_idx == 18) {
            tft.setTextColor(b.dock_to_wp ? TFT_GREEN : textColor, TFT_BLACK);
            sprintf(buf, "DockToWP: %s", b.dock_to_wp ? "YES" : "NO");
            tft.drawString(buf, x + 54, y + 16);
        } else {
            if (global_idx == 9) {
                sprintf(buf, "%s %0.1fm", names[global_idx].c_str(), b.hold_radius);
            } else if (global_idx == 3 || global_idx == 7 || global_idx == 12) {
                float val = (global_idx == 3) ? b.max_speed : (global_idx == 7) ? b.min_speed : b.compass_offset;
                sprintf(buf, "%s %0.0f", names[global_idx].c_str(), val);
            } else {
                float val = (global_idx == 0) ? b.kpr : (global_idx == 1) ? b.kir : (global_idx == 2) ? b.kdr :
                            (global_idx == 4) ? b.kps : (global_idx == 5) ? b.kis : (global_idx == 6) ? b.kds : b.pivot_speed;
                sprintf(buf, "%s %0.3f", names[global_idx].c_str(), val);
            }
            tft.drawString(buf, x + 54, y + 16);
        }
    }
    
    // Draw currently selected value in big text in center of adjustment row (Y: 195 to 225)
    tft.setTextSize(2);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    if (selected_param_idx == 13) {
        tft.drawString("TAP +", 115, 210);
    } else if (selected_param_idx == 10) {
        tft.drawString(b.rev_bb ? "YES" : "NO", 115, 210);
    } else if (selected_param_idx == 14) {
        tft.drawString(b.rev_sb ? "YES" : "NO", 115, 210);
    } else if (selected_param_idx == 11) {
        tft.drawString(b.swap_bb_sb ? "YES" : "NO", 115, 210);
    } else if (selected_param_idx == 15) {
        tft.drawString(b.compass_trim_enabled ? "YES" : "NO", 115, 210);
    } else if (selected_param_idx == 18) {
        tft.drawString(b.dock_to_wp ? "YES" : "NO", 115, 210);
    } else {
        if (selected_param_idx == 9) {
            sprintf(buf, "%0.1fm", b.hold_radius);
        } else if (selected_param_idx == 16) {
            sprintf(buf, "%dm", b.dock_app_dist);
        } else if (selected_param_idx == 17) {
            sprintf(buf, "%d deg", b.dock_app_dir);
        } else if (selected_param_idx == 3 || selected_param_idx == 7 || selected_param_idx == 12) {
            float val = (selected_param_idx == 3) ? b.max_speed : (selected_param_idx == 7) ? b.min_speed : b.compass_offset;
            sprintf(buf, "%0.0f", val);
        } else {
            float val = (selected_param_idx == 0) ? b.kpr : (selected_param_idx == 1) ? b.kir : (selected_param_idx == 2) ? b.kdr :
                        (selected_param_idx == 4) ? b.kps : (selected_param_idx == 5) ? b.kis : (selected_param_idx == 6) ? b.kds : b.pivot_speed;
            sprintf(buf, "%0.3f", val);
        }
        tft.drawString(buf, 115, 210);
    }
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
    
    // Reset caches on buoy selection or screen mode change
    if (selected_buoy_idx != last_buoy_idx || in_mannav_mode != was_mannav_mode || in_setup_mode != was_setup_mode) {
        last_buoy_idx = selected_buoy_idx;
        was_mannav_mode = in_mannav_mode;
        was_setup_mode = in_setup_mode;
        
        last_bb_power = -999;
        last_sb_power = -999;
        last_battery_v = -1.0;
        last_nav_status = "";
        
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
    
    // --- 7. Update Blinking Telemetry Indicators (Top-Left for UDP, Top-Right for LoRa) ---
    uint16_t udpDotColor = (millis() - last_udp_blink_ms < 300) ? TFT_GREEN : TFT_BLACK;
    tft.fillCircle(15, 13, 4, udpDotColor);
    
    uint16_t loraDotColor = (millis() - last_lora_blink_ms < 300) ? TFT_CYAN : TFT_BLACK;
    tft.fillCircle(225, 13, 4, loraDotColor);
}

void update_dynamic_ui() {
    int w = tft.width();
    int h = tft.height();
    unsigned long now = millis();
    
    // Check if buoys went offline (> 10 seconds since last transmission)
    for (int i = 0; i < 3; i++) {
        if (buoys[i].id != "") {
            if (now - buoys[i].last_seen_ms > 10000) { // 10 seconds timeout
                buoys[i].present = false;
            }
        }
    }
    
    if (selected_buoy_idx == -1) {
        // --- Menu Screen (Optimized with State Caching to prevent redraw flicker) ---
        tft.setTextSize(2);
        tft.setTextDatum(MC_DATUM);
        
        for (int i = 0; i < 3; i++) {
            int y = 80 + i * 55;
            int current_present = (buoys[i].id == "") ? -1 : (buoys[i].present ? 1 : 0);
            
            static String last_drawn_ips[3] = {"", "", ""};
            
            // Only draw/redraw if the ID, online presence, or IP address changed!
            if (buoys[i].id != last_drawn_ids[i] || current_present != last_drawn_present[i] || buoys[i].ip_addr != last_drawn_ips[i]) {
                last_drawn_ids[i] = buoys[i].id;
                last_drawn_present[i] = current_present;
                last_drawn_ips[i] = buoys[i].ip_addr;
                
                // Clear only this button's area first
                tft.fillRect(10, y, w - 20, 45, TFT_BLACK);
                
                if (current_present == -1) {
                    // Empty slot
                    tft.drawRoundRect(10, y, w - 20, 45, 5, TFT_DARKGREY);
                    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
                    tft.drawString("Buoy " + String(i+1) + ": [Waiting]", w / 2, y + 22);
                } else if (current_present == 1) {
                    // Present: Green button
                    tft.fillRoundRect(10, y, w - 20, 45, 5, TFT_GREEN);
                    tft.setTextColor(TFT_BLACK, TFT_GREEN);
                    
                    // Draw name/ID at top half, and IP address / LoRa Only at bottom half (Both in bold font size 2!)
                    tft.setTextSize(2);
                    tft.setTextDatum(TC_DATUM);
                    tft.drawString("Buoy " + String(i+1) + ": " + buoys[i].id, w / 2, y + 5);
                    
                    tft.setTextSize(2); // Increased connection text to font size 2!
                    String conn_str = (buoys[i].ip_addr == "") ? "LoRa only" : buoys[i].ip_addr;
                    tft.drawString(conn_str, w / 2, y + 24);
                } else {
                    // Offline: Red button
                    tft.fillRoundRect(10, y, w - 20, 45, 5, TFT_RED);
                    tft.setTextColor(TFT_WHITE, TFT_RED);
                    tft.drawString("Buoy " + String(i+1) + ": [Offline]", w / 2, y + 22);
                }
            }
        }
    } else {
        if (in_setup_mode) {
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

    // Setup WiFi, OTA & UDP listener
    init_wifi_and_ota();

    // Initialize LoRa radio using MicroSD card SPI pins
    init_lora();

    // Initialize standard random seed using ESP32's onboard hardware True Random Number Generator (TRNG)
    // to avoid ADC2/Wi-Fi hardware driver conflicts and crashes!
    randomSeed(esp_random());

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
    if (selected_buoy_idx != lastKnownState || in_setup_mode != lastSetupState || in_mannav_mode != lastMannavState || setup_data_loaded != lastLoadedState || setup_page != lastSetupPage) {
        lastKnownState = selected_buoy_idx;
        lastSetupState = in_setup_mode;
        lastMannavState = in_mannav_mode;
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
            // --- MAIN MENU INTERACTION ---
            // Button 1: Y 80 to 125, X 10 to 230
            if (touchY >= 80 && touchY <= 125 && touchX >= 10 && touchX <= 230) {
                if (buoys[0].id != "" && buoys[0].present) {
                    selected_buoy_idx = 0;
                    last_transition_ms = millis();
                    ChangeRGBColor(RGB_COLOR_3); // Shift LED to blue indicating view
                }
            }
            // Button 2: Y 135 to 180, X 10 to 230
            else if (touchY >= 135 && touchY <= 180 && touchX >= 10 && touchX <= 230) {
                if (buoys[1].id != "" && buoys[1].present) {
                    selected_buoy_idx = 1;
                    last_transition_ms = millis();
                    ChangeRGBColor(RGB_COLOR_3);
                }
            }
            // Button 3: Y 190 to 235, X 10 to 230
            else if (touchY >= 190 && touchY <= 235 && touchX >= 10 && touchX <= 230) {
                if (buoys[2].id != "" && buoys[2].present) {
                    selected_buoy_idx = 2;
                    last_transition_ms = millis();
                    ChangeRGBColor(RGB_COLOR_3);
                }
            }
            // Calibration Button: Y 245 to 275, X 30 to 210
            else if (touchY >= 245 && touchY <= 275 && touchX >= 30 && touchX <= 210) {
                last_transition_ms = millis();
                start_touch_calibration();
            }
        } else {
            if (in_setup_mode) {
                // --- SETUP SCREEN TOUCH INTERACTION ---
                // 1. Grid Parameter Selection (Y: 35 to 180, covers 4 rows x 2 columns)
                // With only 4 rows, each box is 32 pixels high with a 4px gap (symmetrical 36px steps),
                // completely eliminating row touch overlaps and resistive jitter shifts!
                if (touchY >= 35 && touchY <= 180) {
                    int r = (touchY - 35) / 36;
                    if (r < 0) r = 0;
                    if (r > 3) r = 3;
                    int c = (touchX < 120) ? 0 : 1;
                    
                    int local_idx = c * 4 + r; // 0 to 7
                    int tapped_idx = setup_page * 8 + local_idx; // Map to 0-15 based on page!
                    
                    if (tapped_idx >= 0 && tapped_idx <= 18) {
                        // Change focus selection
                        selected_param_idx = tapped_idx;
                        
                        // "SET NORTH" (index 13) acts as a quick instant trigger!
                        if (selected_param_idx == 13) {
                            BuoyData &b = buoys[selected_buoy_idx];
                            b.compass_offset = b.compass_offset - b.mag_dir;
                            while (b.compass_offset < -180) b.compass_offset += 360;
                            while (b.compass_offset > 180) b.compass_offset -= 360;
                            selected_param_idx = 12; // Auto focus to Compass Offset (index 12) to see the computed value
                        }
                        // Boolean Toggles: Symmetrical instant toggles on tap!
                        else if (selected_param_idx == 10) {
                            buoys[selected_buoy_idx].rev_bb = !buoys[selected_buoy_idx].rev_bb;
                        } else if (selected_param_idx == 14) {
                            buoys[selected_buoy_idx].rev_sb = !buoys[selected_buoy_idx].rev_sb;
                        } else if (selected_param_idx == 11) {
                            buoys[selected_buoy_idx].swap_bb_sb = !buoys[selected_buoy_idx].swap_bb_sb;
                        } else if (selected_param_idx == 15) {
                            buoys[selected_buoy_idx].compass_trim_enabled = !buoys[selected_buoy_idx].compass_trim_enabled;
                        }
                        } else if (selected_param_idx == 18) {
                            buoys[selected_buoy_idx].dock_to_wp = !buoys[selected_buoy_idx].dock_to_wp;
                    }
                }
                // 2. Large Plus / Minus Button Clicks (Y: 190 to 230)
                else if (touchY >= 190 && touchY <= 230) {
                    BuoyData &b = buoys[selected_buoy_idx];
                    
                    // Determine custom step size based on highlighted parameter
                    float step = 0.05;
                    if (selected_param_idx == 1 || selected_param_idx == 2 || selected_param_idx == 5 || selected_param_idx == 6) {
                        step = 0.005; // Kir, Kdr, Kis, Kds step size
                    } else if (selected_param_idx == 3 || selected_param_idx == 7) {
                        step = 5.0;   // MaxSpeed, MinSpeed step size
                    } else if (selected_param_idx == 12 || selected_param_idx == 16 || selected_param_idx == 17) {
                        step = 1.0;   // Compass Offset, Appr Dist and Appr Dir step size
                    } else if (selected_param_idx == 9) {
                        step = 0.5;   // Hold Radius step size
                    }
                    
                    if (touchX >= 10 && touchX <= 75) {
                        // MINUS
                        Serial.printf("CYD Touch: MINUS for param %d, step=%0.3f (current dist=%d, dir=%d)\n", selected_param_idx, step, b.dock_app_dist, b.dock_app_dir);
                        if (selected_param_idx == 0) { b.kpr -= step; if (b.kpr < 0) b.kpr = 0; }
                        else if (selected_param_idx == 1) { b.kir -= step; if (b.kir < 0) b.kir = 0; }
                        else if (selected_param_idx == 2) { b.kdr -= step; if (b.kdr < 0) b.kdr = 0; }
                        else if (selected_param_idx == 3) { b.max_speed -= step; if (b.max_speed < 0) b.max_speed = 0; }
                        else if (selected_param_idx == 4) { b.kps -= step; if (b.kps < 0) b.kps = 0; }
                        else if (selected_param_idx == 5) { b.kis -= step; if (b.kis < 0) b.kis = 0; }
                        else if (selected_param_idx == 6) { b.kds -= step; if (b.kds < 0) b.kds = 0; }
                        else if (selected_param_idx == 7) { b.min_speed -= step; if (b.min_speed < -100) b.min_speed = -100; } // Allowed to go negative!
                        else if (selected_param_idx == 8) { b.pivot_speed -= step; if (b.pivot_speed < 0) b.pivot_speed = 0; }
                        else if (selected_param_idx == 9) { b.hold_radius -= step; if (b.hold_radius < 0.5) b.hold_radius = 0.5; }
                        else if (selected_param_idx == 10) { b.rev_bb = !b.rev_bb; } // Toggle boolean on minus tap as well!
                        else if (selected_param_idx == 11) { b.swap_bb_sb = !b.swap_bb_sb; }
                        else if (selected_param_idx == 12) { b.compass_offset -= step; if (b.compass_offset < -180) b.compass_offset = -180; }
                        else if (selected_param_idx == 14) { b.rev_sb = !b.rev_sb; }
                        else if (selected_param_idx == 15) { b.compass_trim_enabled = !b.compass_trim_enabled; }
                        else if (selected_param_idx == 16) { b.dock_app_dist -= (int)step; if (b.dock_app_dist < 0) b.dock_app_dist = 0; }
                        else if (selected_param_idx == 17) { b.dock_app_dir -= (int)step; if (b.dock_app_dir < 0) b.dock_app_dir += 360; }
                        else if (selected_param_idx == 18) { b.dock_to_wp = !b.dock_to_wp; }
                    }
                    else if (touchX >= 155 && touchX <= 230) {
                        // PLUS
                        Serial.printf("CYD Touch: PLUS for param %d, step=%0.3f (current dist=%d, dir=%d)\n", selected_param_idx, step, b.dock_app_dist, b.dock_app_dir);
                        if (selected_param_idx == 0) { b.kpr += step; }
                        else if (selected_param_idx == 1) { b.kir += step; }
                        else if (selected_param_idx == 2) { b.kdr += step; }
                        else if (selected_param_idx == 3) { b.max_speed += step; if (b.max_speed > 100) b.max_speed = 100; }
                        else if (selected_param_idx == 4) { b.kps += step; }
                        else if (selected_param_idx == 5) { b.kis += step; }
                        else if (selected_param_idx == 6) { b.kds += step; }
                        else if (selected_param_idx == 7) { b.min_speed += step; if (b.min_speed > 100) b.min_speed = 100; }
                        else if (selected_param_idx == 8) { b.pivot_speed += step; if (b.pivot_speed > 1.0) b.pivot_speed = 1.0; }
                        else if (selected_param_idx == 9) { b.hold_radius += step; if (b.hold_radius > 10.0) b.hold_radius = 10.0; }
                        else if (selected_param_idx == 10) { b.rev_bb = !b.rev_bb; } // Toggle boolean on plus tap as well!
                        else if (selected_param_idx == 11) { b.swap_bb_sb = !b.swap_bb_sb; }
                        else if (selected_param_idx == 12) { b.compass_offset += step; if (b.compass_offset > 180) b.compass_offset = 180; }
                        else if (selected_param_idx == 13) {
                            b.compass_offset = b.compass_offset - b.mag_dir;
                            while (b.compass_offset < -180) b.compass_offset += 360;
                            while (b.compass_offset > 180) b.compass_offset -= 360;
                            selected_param_idx = 12;
                        }
                        else if (selected_param_idx == 14) { b.rev_sb = !b.rev_sb; }
                        else if (selected_param_idx == 15) { b.compass_trim_enabled = !b.compass_trim_enabled; }
                        else if (selected_param_idx == 16) { b.dock_app_dist += (int)step; }
                        else if (selected_param_idx == 17) { b.dock_app_dir += (int)step; if (b.dock_app_dir >= 360) b.dock_app_dir -= 360; }
                        else if (selected_param_idx == 18) { b.dock_to_wp = !b.dock_to_wp; }
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
                        setup_page = (setup_page + 1) % 3; // Cycle through 0, 1, 2
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
                    if (touchX >= 5 && touchX <= 82) {
                        tft.fillRect(10, 170, 220, 60, TFT_BLACK);
                        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
                        tft.setTextSize(2);
                        tft.setTextDatum(MC_DATUM);
                        tft.drawString("SENDING LOCK...", tft.width() / 2, 195);
                        send_buoy_command(buoys[selected_buoy_idx].id, 12); // Send LOCK (12)
                        delay(600);
                        draw_resting_ui();
                    } else if (touchX >= 83 && touchX <= 157) {
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
                        query_buoy_setup(buoys[selected_buoy_idx].id);
                        reset_button_draw_cache();
                        draw_resting_ui();
                    }
                }
            }
        }
    }

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
            ts_minx = constrain(cal_minx, 50, 1000);
            ts_maxx = constrain(cal_maxx, 2500, 4000);
            ts_miny = constrain(cal_miny, 50, 1000);
            ts_maxy = constrain(cal_maxy, 2500, 4000);
            
            // Apply bounds immediately to active driver
            apply_calibration(ts_minx, ts_maxx, ts_miny, ts_maxy);
            
            // Save to non-volatile storage
            Preferences prefs;
            prefs.begin("touch-cal", false);
            prefs.putUShort("minx", ts_minx);
            prefs.putUShort("maxx", ts_maxx);
            prefs.putUShort("miny", ts_miny);
            prefs.putUShort("maxy", ts_maxy);
            prefs.end();
            
            Serial.printf("Saved Touch Calibration: minx=%d, maxx=%d, miny=%d, maxy=%d\n", ts_minx, ts_maxx, ts_miny, ts_maxy);
            
            // Success Screen Feedback
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_GREEN, TFT_BLACK);
            tft.setTextSize(2);
            tft.setTextDatum(MC_DATUM);
            tft.drawString("CALIBRATION SUCCESS!", tft.width() / 2, 100);
            
            tft.setTextSize(1);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            char buf[64];
            sprintf(buf, "X bounds: %d to %d", ts_minx, ts_maxx);
            tft.drawString(buf, tft.width() / 2, 170);
            sprintf(buf, "Y bounds: %d to %d", ts_miny, ts_maxy);
            tft.drawString(buf, tft.width() / 2, 190);
            
            delay(3000);
            
            // Exit calibration mode
            in_calibration_mode = false;
            selected_buoy_idx = -1;
            lastKnownState = -2; // Force complete menu screen redraw
        }
    }
}

void start_touch_calibration() {
    in_calibration_mode = true;
    cal_state = 0;
    cal_rx1 = 0; cal_ry1 = 0;
    cal_rx2 = 0; cal_ry2 = 0;
    draw_calibration_screen();
}