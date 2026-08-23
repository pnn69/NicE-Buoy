#!/usr/bin/env python
"""Convert the ROBOBUOY startup artwork into the raw RGB565 blob the splash reads from SPIFFS.

    python tools/make_boot_logo.py

Reads  Doc/robobuoy_startup_lvgl_reference/Startup_logo.png  (1024x1536)
Writes data/boot_logo.565                                    (240x320, 153600 bytes)

Why a blob and not a C array: app0 is 1280 KiB and the firmware already uses ~975 KiB of it, so
150 KiB of image would eat half the remaining headroom. SPIFFS has 1408 KiB and holds ~130 KiB of
web assets, so the picture is free there. src/boot_screen.cpp streams it back in 16-row bands.

The source is 2:3 and the panel is 3:4, so rather than letterboxing we crop an exact 3:4 window
out of the source. CROP_TOP is chosen so the artwork (which ends at source row ~1365) lands above
panel row 283, leaving the black band that boot_screen.cpp draws its status line and its WiFi /
LoRa indicators into. If you change the artwork, re-check that band with --report.
"""

import os
import sys

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

SRC = os.path.join(ROOT, "Doc", "robobuoy_startup_lvgl_reference", "Startup_logo.png")
DST = os.path.join(ROOT, "data", "boot_logo.565")

PANEL_W, PANEL_H = 240, 320
CROP_TOP = 155

# Must match BAND_TOP in src/boot_screen.cpp.
BAND_TOP = 283


def main():
    im = Image.open(SRC).convert("RGB")
    sw, sh = im.size

    crop_h = sw * PANEL_H // PANEL_W
    if CROP_TOP + crop_h > sh:
        sys.exit("crop window runs off the bottom of %dx%d - lower CROP_TOP" % (sw, sh))

    im = im.crop((0, CROP_TOP, sw, CROP_TOP + crop_h))
    im = im.resize((PANEL_W, PANEL_H), Image.LANCZOS)
    px = im.load()

    # Little-endian RGB565, i.e. native uint16_t order on the ESP32. boot_screen.cpp reads it
    # straight into a uint16_t buffer and lets tft.setSwapBytes(true) put it on the wire.
    out = bytearray()
    for y in range(PANEL_H):
        for x in range(PANEL_W):
            r, g, b = px[x, y]
            v = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
            out += bytes((v & 0xFF, v >> 8))

    with open(DST, "wb") as f:
        f.write(out)
    print("wrote %s (%d bytes, %dx%d RGB565)" % (DST, len(out), PANEL_W, PANEL_H))

    # The status band has to stay clear of the artwork, or the ellipsis animation will chew a
    # rectangle out of the picture every 250 ms.
    last = max(
        (y for y in range(PANEL_H) if max(max(px[x, y]) for x in range(0, PANEL_W, 2)) > 24),
        default=0,
    )
    print("artwork ends at panel row %d; boot_screen.cpp draws from row %d" % (last, BAND_TOP))
    if last >= BAND_TOP:
        print("WARNING: artwork overlaps the status band - raise CROP_TOP or BAND_TOP")


if __name__ == "__main__":
    main()
