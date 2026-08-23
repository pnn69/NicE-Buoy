#include <Arduino.h>
#include <SPIFFS.h>
#include "boot_screen.h"
#include "cyd_display.h"
#include "cyd_wifi.h"   // handle_ota(), pumped during the dwell

//***************************************************************************************************
//      The ROBOBUOY splash
//***************************************************************************************************
// The artwork is a raw RGB565 blob in SPIFFS rather than a C array in flash, and that is a
// deliberate trade. app0 is 1280 KiB and the firmware already fills three quarters of it; a
// 240x320 image is 150 KiB, which is half of what is left. SPIFFS has 1408 KiB and holds barely
// 130 KiB of web assets, so the picture is free there and costs the app partition nothing.
//
// Regenerate the blob with tools/make_boot_logo.py after editing the PNG.
static const char *LOGO_PATH = "/boot_logo.565";

static const int PANEL_W = 240;
static const int PANEL_H = 320;
static const size_t LOGO_BYTES = (size_t)PANEL_W * PANEL_H * 2;

// The artwork's own content stops at row 282 and the design is black below it, so this band is
// ours to draw in without covering anything up.
static const int BAND_TOP = 283;
static const int STATUS_Y = BAND_TOP + 9;   // 292
static const int ICON_Y   = BAND_TOP + 27;  // 310

// Rows the ellipsis animation repaints. Kept clear of the artwork above and the icons below.
static const int STATUS_CLEAR_TOP = BAND_TOP + 2;
static const int STATUS_CLEAR_H   = 14;

// How long the splash owns the screen. Measured from boot_screen_begin(), NOT from the end of
// setup(), so the seconds WiFi and the radio spend starting up are counted towards it rather than
// added to it: the logo is up for ten seconds total, with both radios coming up underneath it,
// and the dwell below only pays out whatever is left over.
static const unsigned long MIN_VISIBLE_MS = 10000;

static bool active = false;
static unsigned long started_ms = 0;
static char status_label[24] = "";
static unsigned long last_tick_ms = 0;
static int ellipsis = 0;

static uint16_t col_gray()  { return tft.color565(0x60, 0x60, 0x60); }
static uint16_t col_green() { return tft.color565(0x00, 0xFF, 0x40); }
static uint16_t col_blue()  { return tft.color565(0x00, 0x88, 0xFF); }

// Repaint the status line only. Everything under STATUS_CLEAR_TOP is black in the artwork, so a
// plain fillRect is enough - no need to re-read the image.
static void draw_status_line()
{
    char buf[32];
    snprintf(buf, sizeof(buf), "%s%.*s", status_label, ellipsis, "...");

    tft.fillRect(0, STATUS_CLEAR_TOP, PANEL_W, STATUS_CLEAR_H, TFT_BLACK);
    tft.setTextFont(1);
    tft.setTextSize(1);
    tft.setTextPadding(0);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(buf, PANEL_W / 2, STATUS_Y);
}

static void draw_indicator(int cx, const char *label, bool label_left, uint16_t colour)
{
    tft.fillCircle(cx, ICON_Y, 4, colour);
    tft.setTextFont(1);
    tft.setTextSize(1);
    tft.setTextPadding(0);
    tft.setTextColor(col_gray(), TFT_BLACK);
    tft.setTextDatum(label_left ? ML_DATUM : MR_DATUM);
    tft.drawString(label, label_left ? cx + 10 : cx - 10, ICON_Y);
}

// Fall back to something legible if the blob is missing - a freshly flashed board that has had
// the firmware uploaded but not the filesystem image is the normal way to hit this.
static void draw_placeholder()
{
    tft.fillScreen(TFT_BLACK);
    tft.setTextFont(1);
    tft.setTextSize(3);
    tft.setTextPadding(0);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(col_blue(), TFT_BLACK);
    tft.drawString("ROBOBUOY", PANEL_W / 2, PANEL_H / 2 - 20);
    tft.setTextSize(1);
    tft.setTextColor(col_gray(), TFT_BLACK);
    tft.drawString("no /boot_logo.565", PANEL_W / 2, PANEL_H / 2 + 20);
    tft.drawString("upload the filesystem image", PANEL_W / 2, PANEL_H / 2 + 34);
}

static void draw_logo()
{
    // SPIFFS is mounted here rather than waiting for init_wifi_and_ota(), because the splash has
    // to be on the screen before WiFi starts blocking. Mounting twice is a no-op.
    if (!SPIFFS.begin(true)) {
        Serial.println("[boot] SPIFFS mount failed - splash falls back to text");
        draw_placeholder();
        return;
    }

    File f = SPIFFS.open(LOGO_PATH, "r");
    if (!f || f.size() != LOGO_BYTES) {
        Serial.printf("[boot] %s missing or wrong size (%u, expected %u)\r\n",
                      LOGO_PATH, f ? (unsigned)f.size() : 0u, (unsigned)LOGO_BYTES);
        if (f) {
            f.close();
        }
        draw_placeholder();
        return;
    }

    // Streamed in bands: a whole 150 KiB frame will not fit in RAM next to WiFi, and 16 rows is
    // 7.5 KiB - small enough to come off the heap without a fuss, big enough that the SPI setup
    // cost per push disappears.
    const int BAND_ROWS = 16;
    uint16_t *band = (uint16_t *)malloc((size_t)PANEL_W * BAND_ROWS * sizeof(uint16_t));
    if (!band) {
        Serial.println("[boot] no heap for the splash band buffer");
        f.close();
        draw_placeholder();
        return;
    }

    // The blob is little-endian RGB565, i.e. native uint16_t order; swapBytes puts it on the wire
    // the way the panel wants it.
    bool saved_swap = tft.getSwapBytes();
    tft.setSwapBytes(true);
    tft.startWrite();
    for (int y = 0; y < PANEL_H; y += BAND_ROWS) {
        int rows = min(BAND_ROWS, PANEL_H - y);
        size_t want = (size_t)PANEL_W * rows * sizeof(uint16_t);
        if (f.read((uint8_t *)band, want) != want) {
            Serial.println("[boot] short read on the splash image");
            break;
        }
        tft.pushImage(0, y, PANEL_W, rows, band);
    }
    tft.endWrite();
    tft.setSwapBytes(saved_swap);

    free(band);
    f.close();
}

void boot_screen_begin()
{
    active = true;
    started_ms = millis();
    last_tick_ms = started_ms;
    ellipsis = 0;
    status_label[0] = '\0';

    draw_logo();

    // Both radios start out down; each init reports in as it finishes.
    draw_indicator(40, "WIFI", true, col_gray());
    draw_indicator(200, "LORA", false, col_gray());
    boot_screen_status("STARTING");
}

void boot_screen_status(const char *text)
{
    if (!active) {
        return;
    }
    strncpy(status_label, text ? text : "", sizeof(status_label) - 1);
    status_label[sizeof(status_label) - 1] = '\0';
    ellipsis = 0;
    draw_status_line();
}

void boot_screen_tick()
{
    if (!active) {
        return;
    }
    unsigned long now = millis();
    if (now - last_tick_ms < 250) {
        return;
    }
    last_tick_ms = now;
    ellipsis = (ellipsis + 1) & 3;
    draw_status_line();
}

void boot_screen_wifi(BootWifiState state)
{
    if (!active) {
        return;
    }
    uint16_t colour = col_gray();
    if (state == BOOT_WIFI_CLIENT) {
        colour = col_green();
    } else if (state == BOOT_WIFI_AP) {
        colour = col_blue();
    }
    draw_indicator(40, "WIFI", true, colour);
}

void boot_screen_lora(bool up)
{
    if (!active) {
        return;
    }
    draw_indicator(200, "LORA", false, up ? col_blue() : TFT_RED);
}

void boot_screen_end()
{
    if (!active) {
        return;
    }
    while (millis() - started_ms < MIN_VISIBLE_MS) {
        // Keep OTA alive through the dwell. At three seconds this did not matter; at ten it does -
        // a board that boots into a bad state would otherwise be deaf to a rescue upload for the
        // whole splash, which is exactly when you want to reach it.
        handle_ota();
        boot_screen_tick();
        delay(20);
    }
    active = false;
}
