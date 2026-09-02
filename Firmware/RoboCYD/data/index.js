// MsgType definitions matching Python's MsgType
const MsgType = {
    GET: 1,
    SET: 2,
    GETACK: 3,
    ACK: 4,
    NAC: 5,
    INF: 6,
    IDLE: 7,
    IDLING: 8,
    PING: 9,
    PONG: 10,
    ERROR: 11,
    LOCKING: 12,
    LOCKED: 13,
    LOCK_POS: 14,
    DOCKING: 15,
    DOCKED: 16,
    DOC: 17,
    STOREASDOC: 18,
    BUOYPOS: 19,
    SETLOCKPOS: 20,
    LOCKPOS: 21,
    SETDOCKPOS: 22,
    DOCKPOS: 23,
    UNLOCK: 24,
    REMOTE: 25,
    REMOTING: 26,
    DIRDIST: 47,
    TOPDATA: 51,
    PIDRUDDER: 55,
    PIDRUDDERSET: 56,
    PIDSPEED: 57,
    PIDSPEEDSET: 58,
    MAXMINPWR: 68,
    MAXMINPWRSET: 69,
    STORE_COMPASS_OFFSET: 75,
    INFIELD_CALIBRATE: 77,
    INFIELD_OFFSET_CALIBRATE: 78,
    RESET_RUDDER_PID: 79,
    RESET_SPEED_PID: 80,
    RESET_SPEED_RUD_PID: 81,
    WAKEUP: 82,
    SETUPDATA: 83,
    ADAPTIVE_TRIM: 84,
    COMPUTESTART: 62,
    COMPUTETRACK: 63,
    REBOOT: 85,
    // Guided eight point compass calibration. Payload: action, active, next, then the eight
    // captures. See CAL8_SESSION in RoboCompute.h - the rules live on the buoy, not here.
    CAL8_SESSION: 91
};

// What a CAL8_SESSION SET is asking for.
const Cal8 = { BEGIN: 0, SET: 1, SAVE: 2, CANCEL: 3 };

// ---------------------------------------------------------------------------------------------
// MAN CAL - guided eight point compass calibration, from the dashboard.
//
// This panel holds no calibration state. The session lives on the buoy (see the block comment in
// RoboSub/src/compass.cpp), arrives here in CAL8_SESSION frames, and every button is one frame back.
// It used to keep its own eight offsets, its own step counter and its own copy of the table
// arithmetic - one of five such copies across the firmwares, which is how a corrected heading ended
// up stored as if it were a raw one. It also switched the buoy's correction off on entry and
// assumed that had worked; on a lost frame all eight points were captured against a live correction
// and nothing said a word.
//
// North is step one and defines the reference: it lands in the buoy's compass offset, so entry 0 is
// 0 by construction and north reads true with the correction on and with it off. Captures take
// Imag, before the table and before the trim, so nothing has to be switched off. Nothing is written
// until SAVE, which means a run started here can be finished on the touchscreen or on the buoy's own
// page, and closing this panel costs nothing.
// ---------------------------------------------------------------------------------------------
let mancalWebActiveIndex = null;

// known:false means the buoy has not answered yet - which is NOT the same as "no run in progress",
// and every button stays locked until it is settled. Guessing idle here is what would let a press
// wipe out captures somebody else made from another screen.
let mancalWebSession = { known: false, active: false, next: 0, mask: 0, seq: 0,
                         captured: Array(8).fill(0) };
// The buoy's iron-only heading, and the press this panel is waiting to see confirmed.
let mancalWebImag = null;
let mancalWebPendSeq = 0;
let mancalWebPendLeg = -1;
let mancalWebPendMs = 0;

// The offsets the buoy is already applying, from its answer to a table GET. Shown while no run is
// going, so the panel opens on what the buoy is really using rather than on eight zeros - and zero
// is a real calibration value, so "not answered yet" has to look different from "answered, and it
// is 0".
const mancalWebOffsets = Array(8).fill(0);
let mancalWebOffsetsLoaded = false;

// Where the thrusters are being aimed, relative to the leg being asked for. Steering only: no part
// of this reaches the table, and SET captures whatever the compass reads at that moment.

let mancalWebQueryTimer = null;
let mancalWebQueryTries = 0;
const MANCAL_WEB_QUERY_INTERVAL_MS = 1200;
const MANCAL_WEB_QUERY_MAX_TRIES = 5;
// Helper function to provide beautiful active-state visual click feedback for webpage buttons
function flashButtonFeedback(element, activeBgColor = "#22c55e", activeTextColor = "black", duration = 150) {
    if (!element) return;
    const originalBg = element.style.backgroundColor;
    const originalColor = element.style.color;
    
    element.style.backgroundColor = activeBgColor;
    element.style.color = activeTextColor;
    
    setTimeout(() => {
        element.style.backgroundColor = originalBg;
        element.style.color = originalColor;
    }, duration);
}

// Buoy State Configuration (stores 3 buoys)
const buoys = Array.from({ length: 3 }, (_, i) => ({
    index: i,
    id: null, // Hex ID assigned dynamically from incoming messages
    ip: null, // Dynamic IP discovered from UDP packets
    title: `Buoy ${i + 1} (Waiting...)`,
    udpEnabled: true,
    loraEnabled: true,
    lastUdpTime: 0,
    lastLoraTime: 0,
    loraRssi: null,
    udpRssi: null,
    lastUdpContent: "",
    lastLoraContent: "",
    data: {},
    lockBtnOverrideUntil: 0,
    lockBtnOverrideText: "",
    dockBtnOverrideUntil: 0,
    dockBtnOverrideText: "",
    statusLabelOverrideUntil: 0,
    statusLabelOverrideText: ""
}));

// Other Discovered Devices (not assigned to the 3 main slots)
const otherDevices = {};

// Web Serial Connection State (Disabled/Removed)
let serialPort = null;

// WebSocket Connection State
let socket = null;
let reconnectTimer = null;

// Setup Modal State
let activeSetupBuoyIndex = null;
let setupCheckTimer = null;
let setupCheckRetries = 0;

// Initialize on page load
window.addEventListener("load", () => {
    // The map page hides <main> and runs its own bootstrap - the cards are not in play there.
    if (isMapView) return;
    initWebSockets();
    initUIEventListeners();
    initBuoyDetailViews();
    
    // Autodiscover IP for WebSocket
    const wsUrlInput = document.getElementById("ws-url");
    if (wsUrlInput) {
        const host = window.location.hostname || "192.168.1.165";
        wsUrlInput.value = `ws://${host}:81`;
    }
    
    // Draw initial empty gauges
    for (let i = 0; i < 3; i++) {
        drawThrustBar(document.getElementById(`bb-bar-${i}`), 0);
        drawThrustBar(document.getElementById(`sb-bar-${i}`), 0);
        drawWindrose(document.getElementById(`windrose-${i}`), "N/A", "N/A", "N/A", "N/A", "-", "N/A", "N/A", "N/A", "-");
    }

    // Start UI update loop (every 500ms)
    setInterval(updateGUI, 500);

    // Auto-connect WebSocket if loaded from a device IP
    if (window.location.hostname) {
        setTimeout(connectWebSocket, 500);
    }
});

// Calculate XOR CRC for NMEA sentences
function calculateCRC(content) {
    let crc = 0;
    for (let i = 0; i < content.length; i++) {
        crc ^= content.charCodeAt(i);
    }
    return crc;
}

// Log messages with timestamp, auto-scrolling, and log capping to 200 lines
function logMessage(message, source) {
    const timestamp = new Date().toLocaleTimeString();
    let prefix = "";
    if (source.includes("OUT")) prefix = "TX ";
    else if (source.includes("IN")) prefix = "RX ";
    
    const entry = `[${timestamp}] ${prefix}${message}\n`;
    const isLora = source.toUpperCase().includes("LORA");
    const logEl = document.getElementById(isLora ? "lora-log" : "udp-log");
    
    if (logEl) {
        logEl.textContent += entry;
        // Scroll to bottom
        logEl.scrollTop = logEl.scrollHeight;
        
        // Cap lines at 200
        const lines = logEl.textContent.split("\n");
        if (lines.length > 200) {
            logEl.textContent = lines.slice(lines.length - 200).join("\n");
        }
    }
}

// Send command over both Serial and/or WebSocket
async function sendCommand(targetId, baseCommand, useSerial = true, useWs = true) {
    const crc = calculateCRC(baseCommand);
    const fullMessage = `$${baseCommand}*${crc.toString(16).toUpperCase().padStart(2, '0')}\r\n`;
    
    let sentSerial = false;
    
    // Send over Web Serial
    if (useSerial && serialPort && serialPort.writable) {
        try {
            const encoder = new TextEncoder();
            const writer = serialPort.writable.getWriter();
            await writer.write(encoder.encode(fullMessage));
            writer.releaseLock();
            logMessage(fullMessage.trim(), "LORA OUT");
            sentSerial = true;
        } catch (e) {
            console.error("Serial write error:", e);
            logMessage(`SERIAL TX ERROR: ${e.message}`, "LORA OUT");
        }
    }
    
    // Send over WebSocket (acts as the control bridge to RoboLora hardware, which transmits over LoRa/UDP)
    if (useWs && socket && socket.readyState === WebSocket.OPEN) {
        try {
            socket.send(fullMessage);
            logMessage(fullMessage.trim(), "UDP OUT");
        } catch (e) {
            console.error("WS write error:", e);
            logMessage(`WS TX ERROR: ${e.message}`, "UDP OUT");
        }
    }
    
    return sentSerial;
}

// Helper to send default status cmd
function sendStatusCmd(targetId, cmdId) {
    return sendCommand(targetId, `${targetId},99,3,${cmdId},${cmdId}`);
}

// --- Manual Fourier Calibration: reading the buoy's existing table -------------------------------

const MANCAL_WEB_DIRS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
const MANCAL_WEB_LONG = ["NORTH", "NORTH EAST", "EAST", "SOUTH EAST",
                         "SOUTH", "SOUTH WEST", "WEST", "NORTH WEST"];

const mancalWebRunning = () => mancalWebSession.known && mancalWebSession.active;
const mancalWebCaptured = i => (mancalWebSession.mask & (1 << i)) !== 0;
const mancalWebCount = () => {
    let n = 0;
    for (let i = 0; i < 8; i++) if (mancalWebCaptured(i)) n++;
    return n;
};
// The MASK, not the cursor. Re-capturing an earlier direction leaves the cursor where it was, so
// counting on it would keep SAVE hidden after a redo of an otherwise finished run.
const mancalWebComplete = () => mancalWebRunning() && mancalWebCount() >= 8;
const mancalWebLeg = () => Math.min(7, Math.max(0, mancalWebRunning() ? mancalWebSession.next : 0));

// How far Imag may be from zero when N is captured. The same number the buoy enforces
// (CAL8_ANCHOR_TOL_DEG in RoboSub/src/compass.cpp).
const MANCAL_WEB_ANCHOR_TOL = 2;
function mancalWebOnAnchor() {
    if (mancalWebImag === null) return false;
    let off = mancalWebImag;
    while (off > 180) off -= 360;
    while (off < -180) off += 360;
    return Math.abs(off) <= MANCAL_WEB_ANCHOR_TOL;
}

// A capture that has gone out and not yet come back. The next press would carry the same serial and
// be dropped as a duplicate, so the panel holds rather than let the mark look dead.
function mancalWebPending() {
    if (mancalWebPendLeg < 0) return false;
    if (mancalWebSession.seq === mancalWebPendSeq) { mancalWebPendLeg = -1; return false; }
    if (Date.now() - mancalWebPendMs > 5000) { mancalWebPendLeg = -1; return false; }
    return true;
}

function mancalWebShow(id, on) {
    const el = document.getElementById(id);
    if (el) el.style.display = on ? "block" : "none";
}

/**
 * Paints the whole panel from the buoy's reported session: the rose as a progress display, the
 * eight-cell row, the step wording, and which of the four buttons is available. One function, so
 * the display and the click handlers cannot end up disagreeing about which step is current.
 */
function renderMancalWeb() {
    const known = mancalWebSession.known;
    const running = mancalWebRunning();
    const complete = mancalWebComplete();
    const leg = mancalWebLeg();
    const done = running ? mancalWebCount() : 0;

    // The marks ARE the buttons: the one being asked for, and any already in, which a tap replaces.
    document.querySelectorAll(".mancal-web-dot").forEach(btn => {
        const i = parseInt(btn.getAttribute("data-leg"));
        const captured = running && mancalWebCaptured(i);
        const asking = running && i === leg;
        btn.disabled = !running || (!asking && !captured);
        btn.style.cursor = btn.disabled ? "default" : "pointer";
        btn.title = asking ? (i === 0 ? "turn the hull to Imag 0, then tap" : "tap when the jig is on this mark")
                  : captured ? "captured - tap to redo" : "";
        btn.style.backgroundColor = asking ? "#3b82f6" : captured ? "#14532d" : "#334155";
        btn.style.color = asking ? "white" : captured ? "#86efac" : "#94a3b8";
        btn.style.border = asking ? "2px solid white"
                         : captured ? "1px solid #22c55e" : "1px solid #475569";
        btn.style.opacity = (running && !captured && !asking) ? "0.45" : "1";
    });

    // The eight cells: the captures so far during a run, otherwise the offsets in use.
    for (let i = 0; i < 8; i++) {
        const cell = document.getElementById("mancal-web-tbl-" + i);
        if (!cell) continue;
        let text = "?", colour = "#e2e8f0", opacity = "1";
        if (running) {
            if (mancalWebCaptured(i)) {
                // The deviation from the reference direction - "this hull reads 6 degrees high on
                // east" - which is the number that means something to the operator.
                let dev = mancalWebSession.captured[i] - i * 45;
                while (dev > 180) dev -= 360;
                while (dev < -180) dev += 360;
                text = (dev >= 0 ? "+" : "") + dev.toFixed(0) + "\u00b0";
            } else if (i === leg) {
                text = "<<";
                colour = "#22c55e";
            } else {
                text = "-";
                opacity = "0.4";
            }
        } else if (mancalWebOffsetsLoaded) {
            const off = mancalWebOffsets[i];
            text = (off >= 0 ? "+" : "") + off + "\u00b0";
        } else {
            opacity = "0.4";
        }
        cell.textContent = text;
        cell.style.color = colour;
        cell.style.opacity = opacity;
    }

    const stepEl = document.getElementById("mancal-web-step");
    const askEl = document.getElementById("mancal-web-ask");
    const hintEl = document.getElementById("mancal-web-hint");
    const statusEl = document.getElementById("mancal-web-table-status");
    const legEl = document.getElementById("mancal-web-offset-val");

    if (legEl) legEl.textContent = (running && !complete) ? ("next " + MANCAL_WEB_DIRS[leg]) : "-";

    // The anchor gate. Red until the hull is on the compass's own zero, green once it is.
    const imagEl2 = document.getElementById("mancal-web-imag");
    if (imagEl2) {
        imagEl2.textContent = (mancalWebImag === null) ? "--" : Math.round(mancalWebImag) + "\u00b0";
        imagEl2.style.color = (mancalWebImag === null) ? "#64748b"
                            : (mancalWebOnAnchor() ? "#22c55e" : "#ef4444");
    }

    mancalWebShow("mancal-web-begin", known && !running);
    mancalWebShow("mancal-web-set", running && !complete);
    mancalWebShow("mancal-web-exit", complete);
    mancalWebShow("mancal-web-cancel", running);

    if (!known) {
        if (stepEl) stepEl.textContent = "waiting for the buoy";
        if (askEl) { askEl.textContent = "--"; askEl.style.color = "#94a3b8"; }
        if (hintEl) hintEl.textContent = "asking what its calibration is doing...";
        if (statusEl) statusEl.textContent = "Reading the table from the buoy...";
    } else if (!running) {
        if (stepEl) stepEl.textContent = "not started";
        if (askEl) { askEl.textContent = "Ready"; askEl.style.color = "#e2e8f0"; }
        if (hintEl) hintEl.textContent = "Press START, then turn the hull to Imag 0 and tap N";
        if (statusEl) statusEl.textContent = mancalWebOffsetsLoaded
            ? "The corrections the buoy is applying now"
            : "Reading the table from the buoy...";
    } else if (!complete) {
        if (stepEl) stepEl.textContent = `${done} of 8 captured`;
        if (leg === 0) {
            const ok = mancalWebOnAnchor();
            if (askEl) {
                askEl.textContent = ok ? "On zero - tap N" : "Turn to Imag 0";
                askEl.style.color = ok ? "#22c55e" : "#ef4444";
            }
            if (hintEl) hintEl.textContent = "N anchors the run and is only accepted within "
                                           + `${MANCAL_WEB_ANCHOR_TOL} degrees of zero`;
        } else {
            if (askEl) {
                askEl.textContent = `Index the jig to ${MANCAL_WEB_LONG[leg]}`;
                askEl.style.color = "#38bdf8";
            }
            if (hintEl) hintEl.textContent = "turn the fixture to its next mark, then tap "
                                           + MANCAL_WEB_DIRS[leg];
        }
        const setBtn = document.getElementById("mancal-web-set");
        if (setBtn) setBtn.textContent = "CAPTURE " + MANCAL_WEB_DIRS[leg];
        if (statusEl) statusEl.textContent = `Captured so far - ${done} of 8`;
    } else {
        if (stepEl) stepEl.textContent = "all eight captured";
        if (askEl) { askEl.textContent = "Ready to save"; askEl.style.color = "#22c55e"; }
        if (hintEl) hintEl.textContent = "nothing on the buoy has changed yet";
        if (statusEl) statusEl.textContent = "SAVE writes the table - the mounting angle is "
                                           + "Set as North's job";
    }
}

/**
 * One press, one frame. No local bookkeeping: the buoy answers with the resulting state and the
 * panel repaints, so a lost frame costs one repeated press and can never leave this screen a
 * direction ahead of the hull.
 */
function mancalWebPress(action, leg, seq) {
    if (mancalWebActiveIndex === null) return;
    const b = buoys[mancalWebActiveIndex];
    if (!b || !b.id) return;
    const currStatus = b.data.Status || "7";
    // Payload: action, the Active field (state the BUOY owns, ignored on arrival), the leg this
    // press is for, then the eight captures and the mask as placeholders, then the serial.
    //
    // The placeholders are not padding for its own sake: RoboCompute's decoder keys on the field
    // COUNT, so a short frame leaves the serial unread and the press arrives unnumbered - which
    // means "no retry behind this one" and switches the de-duplication off.
    //
    // The leg says WHICH direction, because any direction already captured may be captured again
    // and the buoy cannot infer the target from its own cursor. The serial says WHETHER this is a
    // new press: the Top resends until the buoy confirms, the serial hop down to the Sub drops much
    // of what is sent, and a repeat landing after the fixture has moved on would otherwise
    // overwrite a good capture. See CAL8_SESSION in RoboCompute.h.
    leg = (leg === undefined) ? mancalWebSession.next : leg;
    seq = seq || 0;
    sendCommand(b.id, `${b.id},99,${MsgType.SET},${MsgType.CAL8_SESSION},${currStatus},`
                    + `${action},0,${leg},0,0,0,0,0,0,0,0,0,${seq}`);
}

// Capture one direction, numbering the press one past the last the buoy applied.
function mancalWebCapture(leg) {
    if (!mancalWebRunning()) return;
    if (leg !== mancalWebSession.next && !mancalWebCaptured(leg)) return;
    if (mancalWebPending()) return;
    if (leg === 0 && !mancalWebOnAnchor()) {
        alert("Imag is not on zero.\n\nTurn the hull until the Imag reading goes green - within "
            + MANCAL_WEB_ANCHOR_TOL + " degrees of zero - then tap N. That reading is the "
            + "compass's own zero and it is what anchors the whole run.");
        return;
    }
    if (mancalWebCaptured(leg) &&
        !confirm(`Re-capture ${MANCAL_WEB_LONG[leg]}?\n\nThe reading stored for it is replaced `
               + `by whatever the compass says now.`)) return;

    let seq = mancalWebSession.seq + 1;
    // 0 means "unnumbered press" to the buoy and is always applied, so the counter steps over it
    // rather than wrapping onto it - see cal8NextSeq() on the Sub.
    if (seq > 0xFFFF) seq = 1;
    mancalWebPendSeq = seq;
    mancalWebPendLeg = leg;
    mancalWebPendMs = Date.now();
    mancalWebPress(Cal8.SET, leg, seq);
}

// mancalWebTurn() used to live here: it pivoted the hull towards leg * 45 as a corrected heading.
// The directions are stops on a mechanical fixture now, 45 degrees apart by construction and
// indexed round from wherever it was aligned with Imag = 0, so a true heading of leg * 45 points
// nowhere in particular. Turning the buoy is the fixture's job.

function mancalWebPollState() {
    if (mancalWebQueryTimer) {
        clearTimeout(mancalWebQueryTimer);
        mancalWebQueryTimer = null;
    }
    if (mancalWebActiveIndex === null) return;

    const b = buoys[mancalWebActiveIndex];
    if (!b || !b.id) return;

    if ((mancalWebSession.known && mancalWebOffsetsLoaded) ||
        mancalWebQueryTries >= MANCAL_WEB_QUERY_MAX_TRIES) {
        if (!mancalWebSession.known) {
            logMessage(`Buoy ${b.id.toUpperCase()}: no answer about its calibration state - `
                     + `the buttons stay locked rather than guessing no run is in progress.`, "UDP IN");
        }
        if (!mancalWebOffsetsLoaded) {
            logMessage(`Buoy ${b.id.toUpperCase()}: no answer to the table request - the idle row `
                     + `reads unknown.`, "UDP IN");
        }
        renderMancalWeb();
        return;
    }

    mancalWebQueryTries++;
    const currStatus = b.data.Status || "7";
    if (!mancalWebSession.known) {
        // ack GET asks without changing anything.
        sendCommand(b.id, `${b.id},99,${MsgType.GET},${MsgType.CAL8_SESSION},${currStatus},0,0,0`);
    }
    if (!mancalWebOffsetsLoaded) sendCommand(b.id, `${b.id},99,${MsgType.GET},88,,,,,,,`);
    mancalWebQueryTimer = setTimeout(mancalWebPollState, MANCAL_WEB_QUERY_INTERVAL_MS);
}

// WebSockets (Acts as a Bridge to UDP server or back to ESP32 Web Server)
function initWebSockets() {
    const connBtn = document.getElementById("ws-connect-btn");
    connBtn.addEventListener("click", () => {
        if (socket) {
            disconnectWebSocket();
        } else {
            connectWebSocket();
        }
    });
}

function connectWebSocket() {
    const url = document.getElementById("ws-url").value.trim();
    if (!url) return;
    
    logMessage(`Connecting to WS: ${url}...`, "UDP");
    socket = new WebSocket(url);
    
    const statusEl = document.getElementById("ws-status");
    const connBtn = document.getElementById("ws-connect-btn");
    
    socket.onopen = () => {
        statusEl.textContent = "Connected";
        statusEl.className = "status-indicator status-connected";
        connBtn.textContent = "Disconnect WS";
        logMessage(`Connected to WebSocket`, "UDP IN");
    };
    
    socket.onmessage = (event) => {
        try {
            let raw = event.data.trim();
            if (raw) {
                let source = "UDP";
                let senderIp = null;
                let loraRssi = null;
                let udpRssi = null;
                if (raw.startsWith("LORA:")) {
                    raw = raw.substring(5);
                    source = "LoRa";
                    
                    // Check if there's an RSSI prefix in the LoRa message, e.g. "-78:$..."
                    const colonIdx = raw.indexOf(":$");
                    if (colonIdx !== -1) {
                        const rssiStr = raw.substring(0, colonIdx);
                        const parsedRssi = parseInt(rssiStr, 10);
                        if (!isNaN(parsedRssi)) {
                            loraRssi = parsedRssi;
                        }
                        raw = raw.substring(colonIdx + 1); // Extract "$IDr..."
                    }
                } else if (raw.startsWith("UDP:")) {
                    raw = raw.substring(4);
                    source = "UDP";
                    
                    // Check for RSSI and IP prefix, e.g. "-65:192.168.1.78:$..."
                    const firstColon = raw.indexOf(":");
                    if (firstColon !== -1) {
                        const firstPart = raw.substring(0, firstColon);
                        const secondPart = raw.substring(firstColon + 1);
                        const secondColon = secondPart.indexOf(":$");
                        if (secondColon !== -1) {
                            // We have both RSSI and IP!
                            const parsedRssi = parseInt(firstPart, 10);
                            if (!isNaN(parsedRssi)) {
                                udpRssi = parsedRssi;
                            }
                            senderIp = secondPart.substring(0, secondColon);
                            raw = secondPart.substring(secondColon + 1);
                        } else {
                            // Only IP (legacy/fallback)
                            const colonIdx = raw.indexOf(":$");
                            if (colonIdx !== -1) {
                                senderIp = raw.substring(0, colonIdx);
                                raw = raw.substring(colonIdx + 1);
                            }
                        }
                    }
                }
                
                // Log message to the correct panel (LORA LOG or UDP LOG)
                logMessage(raw, source + " IN");
                
                // Handle raw NMEA formats
                if (raw.startsWith("$")) {
                    parseMessage(raw, source, senderIp, loraRssi, udpRssi);
                } else {
                    // Fallback: parse as JSON if it's not an NMEA sentence
                    try {
                        const data = JSON.parse(raw);
                        handleJsonFallback(data);
                    } catch (e) {}
                }
            }
        } catch (e) {
            console.error("Error in onmessage:", e);
            logMessage(`JS ERROR: ${e.message}`, "UDP IN");
        }
    };
    
    socket.onclose = () => {
        logMessage("WebSocket connection closed", "UDP");
        cleanupWS();
    };
    
    socket.onerror = (e) => {
        logMessage(`WebSocket error: ${e.message || "Unknown"}`, "UDP");
    };
}

function disconnectWebSocket() {
    if (socket) {
        socket.close();
    }
    cleanupWS();
}

function cleanupWS() {
    socket = null;
    const statusEl = document.getElementById("ws-status");
    const connBtn = document.getElementById("ws-connect-btn");
    statusEl.textContent = "Disconnected";
    statusEl.className = "status-indicator status-disconnected";
    connBtn.textContent = "Connect WS";
}

// Fallback JSON handling for legacy index.js WebSocket formats
function handleJsonFallback(stuff) {
    // Treat legacy STATUS1 format as Buoy 1 updates
    const b = buoys[0];
    b.id = b.id || "0001";
    b.title = "Buoy 1";
    
    if (stuff["STATUS1"] !== undefined) {
        const val = stuff["STATUS1"];
        b.data["Status"] = val === 2 ? MsgType.LOCKED : val === 6 ? MsgType.REMOTE : val === 7 ? MsgType.DOCKED : MsgType.IDLE;
    }
    if (stuff["speed1"] !== undefined) b.data["Speed"] = stuff["speed1"];
    if (stuff["ddir1"] !== undefined) b.data["Target Dir"] = stuff["ddir1"];
    if (stuff["tgdir1"] !== undefined) b.data["Target Dir"] = stuff["tgdir1"];
    if (stuff["mdir1"] !== undefined) b.data["Magnetic Dir"] = stuff["mdir1"];
    if (stuff["tgdistance1"] !== undefined) b.data["Target Dist"] = stuff["tgdistance1"];
    if (stuff["speedbb1"] !== undefined) b.data["Bow Thruster (BB)"] = stuff["speedbb1"];
    if (stuff["speedsb1"] !== undefined) b.data["Stern Thruster (SB)"] = stuff["speedsb1"];
    
    b.lastUdpTime = Date.now();
    updateBuoyData(b.id, b.data);
}

// Parse NMEA message strings
function parseMessage(message, source, senderIp = null, loraRssi = null, udpRssi = null) {
    if (!message.startsWith('$') || !message.includes('*')) return;
    
    try {
        const parts = message.substring(1).split('*');
        const content = parts[0];
        const crcStr = parts[1];
        
        const calculatedCrc = calculateCRC(content);
        const receivedCrc = parseInt(crcStr, 16);
        
        if (calculatedCrc !== receivedCrc) {
            logMessage(`CRC ERROR in message: ${message}`, source + " IN");
            return;
        }
        
        const fields = content.split(',').map(f => f === "" ? "0" : f);
        if (fields.length < 5) return;
        
        // Protocol: $Target,Sender,ACK,CMD,Status,...
        const targetId = fields[0].toLowerCase();
        const senderId = fields[1].toLowerCase();
        
        // Echo prevention: if sender is 99 (us), ignore
        if (senderId === "99") return;
        
        let targetBuoy = null;
        
        // Find existing match by either target or sender ID (ignoring broadcast target IDs like 0 or 1)
        const targetInt = parseInt(targetId, 16);
        const isBroadcastTarget = (targetInt === 0 || targetInt === 1);
        for (let b of buoys) {
            if (b.id !== null && (b.id === senderId || (!isBroadcastTarget && b.id === targetId))) {
                targetBuoy = b;
                break;
            }
        }
        
        // If no match, assign to the first empty buoy slot
        if (!targetBuoy && /^[0-9a-fA-F]+$/.test(senderId) && senderId.length >= 4) {
            for (let b of buoys) {
                if (b.id === null) {
                    b.id = senderId;
                    b.title = `Buoy: ${senderId}`;
                    targetBuoy = b;
                    document.getElementById(`buoy-title-${b.index}`).textContent = b.title;
                    break;
                }
            }
        }
        
        if (!targetBuoy) {
            // It is an "other device" (not assigned to the 3 main slots)
            if (/^[0-9a-fA-F]+$/.test(senderId) && senderId.length >= 4) {
                const now = Date.now();
                if (!otherDevices[senderId]) {
                    otherDevices[senderId] = {
                        id: senderId,
                        ip: null,
                        lastSeen: now,
                        lastLoraTime: 0,
                        lastUdpTime: 0,
                        loraRssi: null
                    };
                }
                const dev = otherDevices[senderId];
                dev.lastSeen = now;
                if (source === "UDP" && senderIp) {
                    dev.ip = senderIp;
                    dev.lastUdpTime = now;
                }
                if (source === "LoRa") {
                    dev.lastLoraTime = now;
                    if (loraRssi !== null) {
                        dev.loraRssi = loraRssi;
                    }
                }
                renderOtherDevices();
            }
            return;
        }
        const buoyId = targetBuoy.id;
        
        // Save the IP address of the buoy - but ONLY off a frame that cannot have been relayed.
        //
        // Any UDP packet used to set this, which is wrong for a buoy we only hear through another
        // Top. A Top bridges frames it cannot handle itself between LoRa and UDP WITHOUT rewriting
        // the sender ID, so a relayed frame still says "from buoy B" while the UDP packet carrying
        // it came from the RELAY's address. Both buoys then showed the same IP - which is how
        // b7a5b578 came to display 192.168.1.71, the address of the other buoy's Top, while it was
        // sitting on a different subnet entirely.
        //
        // TOPDATA and BUOYPOS are the discriminator: both are broadcast by the buoy itself and
        // are never forwarded on by another Top, so one arriving over UDP really did come straight
        // from that buoy. A buoy we reach solely by relay now shows no IP at all, which is honest -
        // we genuinely do not know its address. Same rule the CYD firmware already applies in
        // parse_buoy_packet(); this page was the half that never got it.
        //
        // fields[3] is read directly rather than using the `cmd` further down: this has to stay
        // ahead of the UDP/LoRa filtering returns that sit between here and there.
        const ipCmd = parseInt(fields[3]);
        if (source === "UDP" && senderIp &&
            (ipCmd === MsgType.TOPDATA || ipCmd === MsgType.BUOYPOS)) {
            targetBuoy.ip = senderIp;
        }
        
        // UDP filtering
        if (source === "UDP" && !targetBuoy.udpEnabled) return;
        
        // LoRa filtering
        if (source === "LoRa" && !targetBuoy.loraEnabled) return;
        
        const now = Date.now();
        if (source === "LoRa") {
            targetBuoy.lastLoraTime = now;
            targetBuoy.lastLoraContent = content;
            if (loraRssi !== null) {
                targetBuoy.loraRssi = loraRssi;
            }
        } else {
            targetBuoy.lastUdpTime = now;
            targetBuoy.lastUdpContent = content;
            if (udpRssi !== null) {
                targetBuoy.udpRssi = udpRssi;
            }
        }
        
        // Dual-source filtering: if UDP is actively coming in, ignore LoRa updates for the display
        if (source === "LoRa" && (now - targetBuoy.lastUdpTime < 5000)) {
            return;
        }
        
        const cmd = parseInt(fields[3]);
        const ack = fields[2];
        const status = fields[4];
        
        let parsedData = {
            Timestamp: new Date().toLocaleTimeString(),
            IP: source === "LoRa" ? "LoRa" : "UDP",
            ACK: ack,
            Status: status
        };
        
        const isInfoPacket = (ack === "6");
        
        if (cmd === MsgType.TOPDATA && isInfoPacket && fields.length >= 21) {
            Object.assign(parsedData, {
                "Magnetic Dir": fields[5], "GPS Dir": fields[6], "Target Dir": fields[7], "Target Dist": fields[8],
                "Wind Dir": fields[9], "Wind StdDev": fields[10], "Bow Thruster (BB)": fields[11], "Stern Thruster (SB)": fields[12],
                "PID I-term": fields[13], "PID R-term": fields[14], "Sub Battery V": fields[15], "Sub Battery %": fields[16],
                "Latitude (Lat)": fields[17], "Longitude (Lon)": fields[18], "GPS Fix": fields[19], "GPS Satellites": fields[20],
                "Current": fields[21] || "0",
                // Imag - the heading after the iron correction but before the compass table and
                // before the trim, which is the value a calibration point is taken from. Appended
                // to the frame, so an older buoy simply does not send it.
                "Imag": (fields.length > 22 && fields[22] !== "") ? fields[22] : undefined
            });
        } else if (cmd === MsgType.DIRDIST && fields.length >= 7) {
            Object.assign(parsedData, {
                "Target Dir": fields[5], 
                "Target Dist": fields[6]
            });
        } else if (cmd === MsgType.BUOYPOS && fields.length >= 14) {
            Object.assign(parsedData, {
                "Latitude (Lat)": fields[5], "Longitude (Lon)": fields[6], "Magnetic Dir": fields[7], "Wind Dir": fields[8],
                "Bow Thruster (BB)": fields[10], "Stern Thruster (SB)": fields[11]
            });
        } else if ((cmd === MsgType.PIDRUDDER || cmd === MsgType.PIDRUDDERSET) && isInfoPacket && fields.length >= 8) {
            Object.assign(parsedData, {
                "Kpr": fields[5], "Kir": fields[6], "Kdr": fields[7]
            });
        } else if ((cmd === MsgType.PIDSPEED || cmd === MsgType.PIDSPEEDSET) && isInfoPacket && fields.length >= 8) {
            Object.assign(parsedData, {
                "Kps": fields[5], "Kis": fields[6], "Kds": fields[7]
            });
        } else if ((cmd === MsgType.MAXMINPWR || cmd === MsgType.MAXMINPWRSET) && isInfoPacket && fields.length >= 7) {
            Object.assign(parsedData, {
                "maxSpeed": fields[5], "minSpeed": fields[6], "pivotSpeed": fields[7] || "0.2"
            });
        } else if (cmd === MsgType.SETUPDATA && isInfoPacket && fields.length >= 14) {
            Object.assign(parsedData, {
                "Kpr": fields[5], "Kir": fields[6], "Kdr": fields[7], "Kps": fields[8], "Kis": fields[9], "Kds": fields[10],
                "maxSpeed": fields[11], "minSpeed": fields[12], "pivotSpeed": fields[13], "compassOffset": fields[14] || "0",
                "holdRad": fields[15] || "2.0",
                "revBB": fields[16] || "0", "revSB": fields[17] || "0", "swap_BB_SB": fields[18] || "0",
                "compass_trim_enabled": fields[19] || "0",
                "dockAppDist": fields[20] || "20",
                "dockAppDir": fields[21] || "180",
                "dockToWP": fields[22] || "0"
            });
            // Field 23 is tri-state: 1 = off, 2 = on, empty/0 = the buoy did not report it.
            // RoboTop compresses an all-zero token to "", so a plain 0 could not be told apart
            // from a field an older node never sent - leave the last known value alone then.
            if (fields.length > 23 && fields[23] !== "" && fields[23] !== "0") {
                parsedData["harmonic_enabled"] = (fields[23] === "2") ? "1" : "0";
            }
        } else if (cmd === MsgType.ADAPTIVE_TRIM && fields.length >= 6) {
            Object.assign(parsedData, {
                "compass_trim": fields[5],
                "compass_trim_enabled": fields[6]
            });
        } else if (cmd === MsgType.CAL8_SESSION && fields.length >= 8) {
            // The state of a guided calibration run. Taken from EVERY frame, unlike the table
            // below: this is the buoy's own step counter and only the buoy ever changes it, so the
            // worst a late duplicate off the other radio can do is repaint the same step.
            if (mancalWebActiveIndex !== null && buoys[mancalWebActiveIndex].id === buoyId) {
                mancalWebSession.known = true;
                mancalWebSession.active = fields[6] !== "0" && fields[6] !== "";
                mancalWebSession.next = parseInt(fields[7]) || 0;
                // The mask and the serial are appended after the eight captures, so a buoy too old
                // to send them leaves this end's copy alone rather than reading as an empty run.
                if (fields.length >= 17) mancalWebSession.mask = parseInt(fields[16]) || 0;
                if (fields.length >= 18) mancalWebSession.seq = parseInt(fields[17]) || 0;
                if (fields.length >= 16) {
                    for (let i = 0; i < 8; i++) {
                        mancalWebSession.captured[i] = parseFloat(fields[8 + i]) || 0;
                    }
                }
                renderMancalWeb();
                logMessage(`Buoy ${buoyId.toUpperCase()}: calibration `
                         + `${mancalWebSession.active ? "running" : "idle"}, `
                         + `asking for ${mancalWebSession.next}, `
                         + `${mancalWebCount()} of 8 in`, "UDP IN");
            }
        } else if (cmd === 88 && fields.length >= 13) {
            // Received 8-point fourier interpolation table from Sub
            console.log(`WS RX Command 88 for buoy ${buoyId.toUpperCase()}:`, fields);
            // Only while the screen is still waiting for its first answer. These frames arrive in
            // duplicate - the Top puts every reply on BOTH UDP and LoRa - and the LoRa copy can lag
            // by seconds and carry an older table, which would overwrite the good values with zeros
            // (and, after a STORE, would wipe out what the operator has dialled in).
            if (mancalWebActiveIndex !== null && buoys[mancalWebActiveIndex].id === buoyId
                && !mancalWebOffsetsLoaded) {
                for (let i = 0; i < 8; i++) {
                    const tableVal = parseFloat(fields[5 + i]);
                    let offset = (i * 45) - tableVal;
                    while (offset < -180) offset += 360;
                    while (offset > 180) offset -= 360;
                    
                    mancalWebOffsets[i] = Math.round(offset);
                }

                mancalWebOffsetsLoaded = true;
                renderMancalWeb();

                // One of the two answers we were waiting for: check whether the other is in too,
                // instead of sitting out the rest of the retry interval.
                mancalWebPollState();

                logMessage(`Loaded 8 Fourier offsets from Buoy ${buoyId.toUpperCase()}: [${mancalWebOffsets.join(", ")}]`, "UDP IN");
            }
        }
        
        updateBuoyData(buoyId, parsedData);
    } catch (e) {
        console.error("Parse error:", e);
    }
}

// Update local buoy datasets
function updateBuoyData(buoyId, parsedData) {
    const buoy = buoys.find(b => b.id === buoyId);
    if (buoy) {
        Object.assign(buoy.data, parsedData);
    }
}

// Drawing Bow and Stern Thruster Bars
function drawThrustBar(canvas, value) {
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, 20, 180);
    
    // Draw Background
    ctx.fillStyle = "#1e293b";
    ctx.fillRect(0, 0, 20, 180);
    
    // Draw Center Line (Zero mark)
    ctx.beginPath();
    ctx.moveTo(0, 90);
    ctx.lineTo(20, 90);
    ctx.strokeStyle = "#475569";
    ctx.lineWidth = 2;
    ctx.stroke();
    
    // Draw fill bar
    if (value !== 0 && !isNaN(value)) {
        ctx.fillStyle = value < 0 ? "#ef4444" : "#22c55e"; // Red for negative, green for positive
        const barHeight = (value * 0.9); // Limit height to 90px max (100 * 0.9 = 90)
        ctx.fillRect(0, 90, 20, -barHeight);
    }
}

// Draw Arrow helper for Canvas Compass
function drawArrow(ctx, fromX, fromY, toX, toY, color, width) {
    ctx.beginPath();
    ctx.moveTo(fromX, fromY);
    ctx.lineTo(toX, toY);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.stroke();

    // Arrowhead calculations
    const angle = Math.atan2(toY - fromY, toX - fromX);
    ctx.beginPath();
    ctx.moveTo(toX, toY);
    ctx.lineTo(toX - 10 * Math.cos(angle - Math.PI / 6), toY - 10 * Math.sin(angle - Math.PI / 6));
    ctx.lineTo(toX - 10 * Math.cos(angle + Math.PI / 6), toY - 10 * Math.sin(angle + Math.PI / 6));
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.fill();
}

// Render circular Windrose with target, wind, mag, gps arrows
function drawWindrose(canvas, targetDir, magDir, windDir, gpsDir, targetDist, windDirVal, windStdVal, magDirVal, targetDirLabel) {
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, 200, 200);

    // Draw background circle
    ctx.beginPath();
    ctx.arc(100, 100, 80, 0, 2 * Math.PI);
    ctx.strokeStyle = "#475569";
    ctx.lineWidth = 1;
    ctx.stroke();

    // Draw cardinal labels N, E, S, W
    const directions = [
        { angle: 0, label: "N" },
        { angle: 90, label: "E" },
        { angle: 180, label: "S" },
        { angle: 270, label: "W" }
    ];
    ctx.font = "bold 11px Arial";
    ctx.fillStyle = "#1e293b";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    
    directions.forEach(d => {
        const rad = (d.angle - 90) * Math.PI / 180;
        const x = 100 + 90 * Math.cos(rad);
        const y = 100 + 90 * Math.sin(rad);
        ctx.fillText(d.label, x, y);
    });

    // Helper to draw a specific arrow at a given angle
    function drawAngleArrow(angleVal, length, color, width) {
        if (angleVal !== "N/A" && angleVal !== "nan" && angleVal !== null && angleVal !== undefined && !isNaN(parseFloat(angleVal))) {
            const rad = (parseFloat(angleVal) - 90) * Math.PI / 180;
            const toX = 100 + length * Math.cos(rad);
            const toY = 100 + length * Math.sin(rad);
            drawArrow(ctx, 100, 100, toX, toY, color, width);
        }
    }

    // Draw Arrows
    drawAngleArrow(targetDir, 80, "#ef4444", 4);  // Target (Red)
    drawAngleArrow(magDir, 70, "#22c55e", 3);     // Mag (Green)
    drawAngleArrow(windDir, 60, "#3b82f6", 2);    // Wind (Blue)
    drawAngleArrow(gpsDir, 50, "#000000", 3);     // GPS (Black)
}

// GUI Periodic Update Loop (handles network state, overrides, gauges and parameters)
function updateGUI() {
    renderOtherDevices();
    const now = Date.now();
    let activeCount = 0;
    
    buoys.forEach((b, i) => {
        const d = b.data;
        const hasId = b.id !== null;
        
        if (hasId) activeCount++;
        
        // Dynamically update the card header title and IP address
        const titleEl = document.getElementById(`buoy-title-${i}`);
        const ipEl = document.getElementById(`buoy-ip-${i}`);
        if (hasId) {
            titleEl.textContent = `Buoy: ${b.id}`;
            if (b.ip) {
                if (ipEl) {
                    ipEl.innerHTML = `<a href="http://${b.ip}/" target="_blank" style="color: #fbbf24; text-decoration: underline; cursor: pointer;">${b.ip}</a>`;
                }
            } else {
                if (ipEl) ipEl.innerHTML = "";
            }
        } else {
            titleEl.textContent = `Buoy ${i + 1} (Waiting...)`;
            if (ipEl) ipEl.innerHTML = "";
        }
        
        let currentStatus = 0;
        try {
            currentStatus = parseInt(d.Status || "0");
        } catch(e) {}
        
        // 1. Network indicators
        const udpOk = b.lastUdpTime > 0 && (now - b.lastUdpTime < 5000);
        const loraOk = b.lastLoraTime > 0 && (now - b.lastLoraTime < 5000);
        
        const udpInd = document.getElementById(`udp-indicator-${i}`);
        const udpRssiText = (udpOk && b.udpRssi !== null) ? ` (${b.udpRssi} dBm)` : "";
        udpInd.textContent = udpOk ? `UDP: OK${udpRssiText}` : "UDP: --";
        udpInd.className = "net-indicator" + (udpOk ? " ok" : "");
        
        const loraInd = document.getElementById(`lora-indicator-${i}`);
        const rssiText = (loraOk && b.loraRssi !== null) ? ` (${b.loraRssi} dBm)` : "";
        loraInd.textContent = loraOk ? `LoRa: OK${rssiText}` : "LoRa: --";
        loraInd.className = "net-indicator" + (loraOk ? " ok" : "");
        
        const syncInd = document.getElementById(`sync-indicator-${i}`);
        if (b.lastUdpContent && b.lastLoraContent) {
            const synced = b.lastUdpContent === b.lastLoraContent;
            syncInd.textContent = synced ? "Data: SYNC" : "Data: DIFF";
            syncInd.className = "net-indicator " + (synced ? "sync" : "diff");
        } else {
            syncInd.textContent = "Data: WAIT";
            syncInd.className = "net-indicator";
        }
        
        // 2. Status banners with override support
        const statusBanner = document.getElementById(`status-label-${i}`);
        if (now < b.statusLabelOverrideUntil) {
            statusBanner.textContent = b.statusLabelOverrideText;
        } else {
            if (!hasId) {
                statusBanner.textContent = "UNKNOWN";
            } else {
                statusBanner.textContent = currentStatus === MsgType.IDLE ? "IDLE" :
                                           [MsgType.LOCKING, MsgType.LOCKED].includes(currentStatus) ? "LOCKED" :
                                           [MsgType.DOCKING, MsgType.DOCKED].includes(currentStatus) ? "DOCKING" : `STATUS ${currentStatus}`;
            }
        }
        
        // 3. Enable / Disable button controls
        document.getElementById(`lock-btn-${i}`).disabled = !hasId;
        document.getElementById(`dock-btn-${i}`).disabled = !hasId;
        document.getElementById(`setup-btn-${i}`).disabled = !hasId;
        document.getElementById(`dirdist-send-${i}`).disabled = !hasId;
        document.getElementById(`mannav-btn-${i}`).disabled = !hasId;
        
        // Enable map only if valid Latitude/Longitude exists
        const lat = d["Latitude (Lat)"];
        const lon = d["Longitude (Lon)"];
        const hasGps = lat && lon && lat !== "N/A" && lat !== "nan" && lon !== "N/A" && lon !== "nan";
        document.getElementById(`map-btn-${i}`).disabled = !hasGps;
        
        // 4. Update Lock / Dock Button labels with overrides
        const lockBtn = document.getElementById(`lock-btn-${i}`);
        if (now < b.lockBtnOverrideUntil) {
            lockBtn.textContent = b.lockBtnOverrideText;
        } else {
            lockBtn.textContent = [MsgType.LOCKING, MsgType.LOCKED].includes(currentStatus) ? "IDLE" : "LOCK";
        }
        
        const dockBtn = document.getElementById(`dock-btn-${i}`);
        if (now < b.dockBtnOverrideUntil) {
            dockBtn.textContent = b.dockBtnOverrideText;
        } else {
            dockBtn.textContent = [MsgType.DOCKING, MsgType.DOCKED].includes(currentStatus) ? "IDLE" : "DOCK";
        }
        
        // 5. Compass windrose display values
        let mDir = d["Magnetic Dir"] || "N/A";
        let targetDistLabel = "-";
        let targetDirLabel = "-";
        let windDirLabel = "-";
        
        if (currentStatus !== MsgType.IDLE) {
            if ([MsgType.LOCKING, MsgType.LOCKED, MsgType.DOCKING, MsgType.DOCKED].includes(currentStatus)) {
                const rawDist = d["Target Dist"] || "0";
                targetDistLabel = `${parseFloat(rawDist).toFixed(2)}m`;
            }
            if ([MsgType.LOCKING, MsgType.LOCKED, MsgType.DOCKING, MsgType.DOCKED].includes(currentStatus)) {
                windDirLabel = d["Wind Dir"] || "0";
            }
            if ([MsgType.LOCKING, MsgType.LOCKED, MsgType.DOCKING, MsgType.DOCKED].includes(currentStatus)) {
                const rawTdir = d["Target Dir"] || "N/A";
                targetDirLabel = rawTdir !== "N/A" ? `Tg:${parseFloat(rawTdir).toFixed(0)}°` : "-";
            }
        }
        
        // Render gauges & canvas graphics
        const isIdle = currentStatus === MsgType.IDLE;
        const fDist = parseFloat(d["Target Dist"] || "0");
        
        const wAngle = (!isIdle && fDist <= 10.0) ? d["Wind Dir"] : "N/A";
        const tAngle = !isIdle ? d["Target Dir"] : "N/A";
        const gAngle = (!isIdle && fDist > 10.0) ? d["GPS Dir"] : "N/A";
        
        drawWindrose(
            document.getElementById(`windrose-${i}`),
            tAngle, mDir, wAngle, gAngle,
            targetDistLabel,
            windDirLabel, d["Wind StdDev"],
            mDir, targetDirLabel
        );
        
        // Update the new external HTML label rows (aligned outside the canvas with larger fonts)
        document.getElementById(`windrose-dist-${i}`).textContent = targetDistLabel;
        
        const windRowEl = document.getElementById(`windrose-wind-${i}`);
        if (windDirLabel !== "-" && windDirLabel !== "N/A" && windDirLabel !== undefined && !isNaN(parseFloat(windDirLabel))) {
            let windText = `Wind:${parseFloat(windDirLabel).toFixed(0)}°`;
            const windStd = d["Wind StdDev"];
            if (windStd !== undefined && windStd !== "" && !isNaN(parseFloat(windStd))) {
                windText += ` std:${parseFloat(windStd).toFixed(0)}°`;
            }
            windRowEl.textContent = windText;
        } else {
            windRowEl.textContent = "-";
        }
        
        document.getElementById(`windrose-tg-${i}`).textContent = targetDirLabel;
        
        const magRowEl = document.getElementById(`windrose-mag-${i}`);
        if (mDir !== "N/A" && mDir !== undefined && !isNaN(parseFloat(mDir))) {
            magRowEl.textContent = `Mag:${parseFloat(mDir).toFixed(0)}°`;
        } else {
            magRowEl.textContent = "Mag:-";
        }

        // The buoy detail view carries its own copy of the windrose and the figures around it.
        // Drawn from the same values as the card above rather than from a second reading of the
        // telemetry, so the two can never disagree. Skipped entirely unless that view is open -
        // it is one of three, and drawing a 200x200 canvas nobody is looking at, twice over, on
        // every 500 ms tick is wasted work.
        if (activeDetailBuoy === i) {
            const dRose = document.getElementById(`detail-windrose-${i}`);
            if (dRose) {
                drawWindrose(dRose, tAngle, mDir, wAngle, gAngle,
                             targetDistLabel, windDirLabel, d["Wind StdDev"], mDir, targetDirLabel);
            }
            const mirror = (id, text) => {
                const el = document.getElementById(id);
                if (el) el.textContent = text;
            };
            mirror(`detail-dist-${i}`, targetDistLabel);
            mirror(`detail-wind-${i}`, windRowEl.textContent);
            mirror(`detail-tg-${i}`, targetDirLabel);
            mirror(`detail-mag-${i}`, magRowEl.textContent);

            const srcStatus = document.getElementById(`status-label-${i}`);
            const dstStatus = document.getElementById(`detail-status-${i}`);
            if (srcStatus && dstStatus) {
                dstStatus.textContent = srcStatus.textContent;
                dstStatus.className = srcStatus.className;
            }
        }
        
        // 6. Bow & Stern Thruster Canvas Bars
        const bbThrust = isIdle ? 0 : parseFloat(d["Bow Thruster (BB)"] || "0");
        const sbThrust = isIdle ? 0 : parseFloat(d["Stern Thruster (SB)"] || "0");
        
        drawThrustBar(document.getElementById(`bb-bar-${i}`), bbThrust);
        drawThrustBar(document.getElementById(`sb-bar-${i}`), sbThrust);
        
        document.getElementById(`bb-val-${i}`).textContent = `${Math.round(bbThrust)}%`;
        document.getElementById(`sb-val-${i}`).textContent = `${Math.round(sbThrust)}%`;
        
        // PID terms
        document.getElementById(`is-val-${i}`).textContent = `Is: ${parseFloat(d["PID I-term"] || "0").toFixed(1)}`;
        document.getElementById(`ir-val-${i}`).textContent = `Ir: ${parseFloat(d["PID R-term"] || "0").toFixed(1)}`;
        
        // 7. Battery & Current bar animations
        const subBatV = parseFloat(d["Sub Battery V"] || "0.0");
        const vPct = Math.max(0, Math.min(100, ((subBatV - 17.0) / 8.2) * 100));
        document.getElementById(`volt-bar-${i}`).style.width = `${vPct}%`;
        document.getElementById(`volt-val-${i}`).textContent = `${subBatV.toFixed(1)}V`;
        
        const currentA = parseFloat(d["Current"] || "0.0");
        const cPct = Math.max(0, Math.min(100, ((currentA + 5.0) / 25.0) * 100));
        document.getElementById(`curr-bar-${i}`).style.width = `${cPct}%`;
        document.getElementById(`curr-val-${i}`).textContent = `${currentA.toFixed(1)}A`;
        
        // 8. Parameters panel tables updates
        const paramFields = ["Timestamp", "Wind Dir", "Wind StdDev", "PID I-term", "PID R-term", "Battery", "Current", "GPS Fix", "GPS Satellites", "Active Trim"];
        paramFields.forEach(field => {
            let value = "N/A";
            if (field === "Battery") value = `${subBatV.toFixed(1)}V`;
            else if (field === "Current") value = `${currentA.toFixed(1)}A`;
            else if (field === "Wind Dir") value = d["Wind Dir"] ? `${parseFloat(d["Wind Dir"]).toFixed(0)}°` : "N/A";
            else if (field === "Wind StdDev") value = d["Wind StdDev"] ? `${parseFloat(d["Wind StdDev"]).toFixed(0)}` : "N/A";
            else if (field === "Active Trim") {
                const en = d["compass_trim_enabled"] === "1" || d["compass_trim_enabled"] === 1 || d["compass_trim_enabled"] === true || d["compass_trim_enabled"] === "true";
                const trim = parseFloat(d["compass_trim"] || "0.0");
                value = en ? `ON (${trim >= 0 ? '+' : ''}${trim.toFixed(2)}°)` : `OFF (${trim >= 0 ? '+' : ''}${trim.toFixed(2)}°)`;
            }
            else value = d[field] !== undefined ? d[field] : "N/A";
            
            const paramEl = document.getElementById(`param-${field.replace(/\s+/g, '-')}-${i}`);
            if (paramEl) {
                paramEl.textContent = value;
                if (field === "Active Trim") {
                    const en = d["compass_trim_enabled"] === "1" || d["compass_trim_enabled"] === 1 || d["compass_trim_enabled"] === true || d["compass_trim_enabled"] === "true";
                    paramEl.style.color = en ? "#22c55e" : "#ef4444";
                    paramEl.style.fontWeight = "bold";
                }
            }
        });
        
        // 8.5 Full-screen Manual Calibration Real-time overlay updates!
        if (mancalWebActiveIndex === i) {
            // Update raw heading text
            const magValEl = document.getElementById("mancal-web-mag-val");
            if (magValEl) {
                if (mDir !== "N/A" && mDir !== undefined && !isNaN(parseFloat(mDir))) {
                    magValEl.textContent = `${parseFloat(mDir).toFixed(0)}°`;
                } else {
                    magValEl.textContent = "0°";
                }
            }

            // Imag: what a capture takes, and what N is aligned against. Stashed rather than
            // painted here - renderMancalWeb() owns that element, because the colour depends on
            // whether the hull is on the anchor and that rule lives in one place.
            const im = d["Imag"];
            const imNum = (im !== undefined && im !== "" && !isNaN(parseFloat(im)))
                        ? parseFloat(im) : null;
            if (imNum !== mancalWebImag) {
                mancalWebImag = imNum;
                renderMancalWeb();
            }
            
            // Update rotating visual compass arrow dynamically in real-time
            const arrowEl = document.getElementById("mancal-web-arrow");
            if (arrowEl) {
                if (mDir !== "N/A" && mDir !== undefined && !isNaN(parseFloat(mDir))) {
                    arrowEl.style.transform = `rotate(${parseFloat(mDir).toFixed(1)}deg)`;
                } else {
                    arrowEl.style.transform = `rotate(0deg)`;
                }
            }
            
            // Update Port and Starboard speedbars
            const bbPowerVal = isIdle ? 0 : parseFloat(d["Bow Thruster (BB)"] || "0");
            const sbPowerVal = isIdle ? 0 : parseFloat(d["Stern Thruster (SB)"] || "0");
            
            const bbBarEl = document.getElementById("mancal-bar-bb");
            const sbBarEl = document.getElementById("mancal-bar-sb");
            const bbValEl = document.getElementById("mancal-val-bb");
            const sbValEl = document.getElementById("mancal-val-sb");
            
            if (bbBarEl) {
                const bbPct = Math.max(-100, Math.min(100, bbPowerVal));
                if (bbPct >= 0) {
                    bbBarEl.style.bottom = "55px";
                    bbBarEl.style.height = `${bbPct * 0.55}px`; // max 55px
                    bbBarEl.style.backgroundColor = "#22c55e"; // green
                } else {
                    bbBarEl.style.bottom = `${55 + bbPct * 0.55}px`; // shift starting position down
                    bbBarEl.style.height = `${Math.abs(bbPct) * 0.55}px`;
                    bbBarEl.style.backgroundColor = "#ef4444"; // red
                }
            }
            if (sbBarEl) {
                const sbPct = Math.max(-100, Math.min(100, sbPowerVal));
                if (sbPct >= 0) {
                    sbBarEl.style.bottom = "55px";
                    sbBarEl.style.height = `${sbPct * 0.55}px`; // max 55px
                    sbBarEl.style.backgroundColor = "#22c55e"; // green
                } else {
                    sbBarEl.style.bottom = `${55 + sbPct * 0.55}px`;
                    sbBarEl.style.height = `${Math.abs(sbPct) * 0.55}px`;
                    sbBarEl.style.backgroundColor = "#ef4444"; // red
                }
            }
            
            if (bbValEl) bbValEl.textContent = `${Math.round(bbPowerVal)}%`;
            if (sbValEl) sbValEl.textContent = `${Math.round(sbPowerVal)}%`;
        }
    });
    
    // 9. Global controls states based on discovered active count
    document.getElementById("align-startline-btn").disabled = activeCount < 2;
    document.getElementById("align-track-btn").disabled = activeCount < 3;
    document.getElementById("dock-all-btn").disabled = activeCount < 1;
}

// Bind UI action event listeners (clicks, checkboxes, map opening, modal)
function initUIEventListeners() {
    buoys.forEach((b, i) => {
        // LOCK button trigger
        document.getElementById(`lock-btn-${i}`).addEventListener("click", () => {
            if (!b.id) return;
            const currentStatus = parseInt(b.data.Status || "0");
            const isLocked = [MsgType.LOCKING, MsgType.LOCKED, MsgType.LOCK_POS].includes(currentStatus);
            const cmd = isLocked ? MsgType.IDLING : MsgType.LOCKING;
            
            sendStatusCmd(b.id, cmd);
            
            // Apply immediate GUI overrides for responsiveness
            const targetTime = Date.now() + 2000;
            b.lockBtnOverrideUntil = targetTime;
            b.lockBtnOverrideText = isLocked ? "LOCK" : "IDLE";
            b.statusLabelOverrideUntil = targetTime;
            b.statusLabelOverrideText = isLocked ? "IDLE" : "LOCKED";
            
            document.getElementById(`lock-btn-${i}`).textContent = b.lockBtnOverrideText;
            document.getElementById(`status-label-${i}`).textContent = b.statusLabelOverrideText;
        });
        
        // DOCK button trigger
        document.getElementById(`dock-btn-${i}`).addEventListener("click", () => {
            if (!b.id) return;
            const currentStatus = parseInt(b.data.Status || "0");
            const isDocked = [MsgType.DOCKING, MsgType.DOCKED, MsgType.DOC].includes(currentStatus);
            const cmd = isDocked ? MsgType.IDLING : MsgType.DOCKING;
            
            sendStatusCmd(b.id, cmd);
            
            // Apply immediate GUI overrides for responsiveness
            const targetTime = Date.now() + 2000;
            b.dockBtnOverrideUntil = targetTime;
            b.dockBtnOverrideText = isDocked ? "DOCK" : "IDLE";
            b.statusLabelOverrideUntil = targetTime;
            b.statusLabelOverrideText = isDocked ? "IDLE" : "DOCKING";
            
            document.getElementById(`dock-btn-${i}`).textContent = b.dockBtnOverrideText;
            document.getElementById(`status-label-${i}`).textContent = b.statusLabelOverrideText;
        });
        
        // Target Dir/Dist custom coordinate send
        document.getElementById(`dirdist-send-${i}`).addEventListener("click", () => {
            if (!b.id) return;
            const dirVal = parseFloat(document.getElementById(`dir-input-${i}`).value);
            const distVal = parseFloat(document.getElementById(`dist-input-${i}`).value);
            
            if (isNaN(dirVal) || isNaN(distVal)) {
                alert("Please enter numeric values for both Dir and Dist!");
                return;
            }
            
            const payload = `${b.id},99,${MsgType.INF},${MsgType.DIRDIST},${MsgType.IDLE},${dirVal.toFixed(2)},${distVal.toFixed(2)}`;
            sendCommand(b.id, payload);
        });
        
        // Open Buoy in OpenStreetMap
        document.getElementById(`map-btn-${i}`).addEventListener("click", () => {
            const lat = b.data["Latitude (Lat)"];
            const lon = b.data["Longitude (Lon)"];
            if (lat && lon && lat !== "N/A" && lat !== "nan" && lon !== "N/A" && lon !== "nan") {
                const url = `https://www.openstreetmap.org/?mlat=${lat}&mlon=${lon}#map=18/${lat}/${lon}`;
                window.open(url, "_blank");
            }
        });
        
        // SETUP button trigger (modal flow)
        document.getElementById(`setup-btn-${i}`).addEventListener("click", () => {
            openSetupModal(i);
        });
        
        // UDP Enabled toggle
        document.getElementById(`udp-enabled-${i}`).addEventListener("change", (e) => {
            b.udpEnabled = e.target.checked;
        });
        
        // LoRa Enabled toggle
        document.getElementById(`lora-enabled-${i}`).addEventListener("change", (e) => {
            b.loraEnabled = e.target.checked;
        });
        
        // MANUAL sliders and indicators definitions
        const dirSlider = document.getElementById(`mannav-dir-${i}`);
        const dirVal = document.getElementById(`mannav-dir-val-${i}`);
        const speedSlider = document.getElementById(`mannav-speed-${i}`);
        const speedVal = document.getElementById(`mannav-speed-val-${i}`);
        
        // MANUAL button trigger: Toggles display of the manual steering panel
        document.getElementById(`mannav-btn-${i}`).addEventListener("click", () => {
            const panel = document.getElementById(`mannav-panel-${i}`);
            const btn = document.getElementById(`mannav-btn-${i}`);
            if (panel.style.display === "none") {
                panel.style.display = "block";
                btn.style.backgroundColor = "#0284c7"; // Highlight active manual button!
                
                // Initialize direction slider to match buoy's active magnetic heading on opening!
                const magHeadingStr = b.data["Magnetic Dir"] || b.data["Magnetic Dir (Mag)"] || "0";
                const magHeading = parseFloat(magHeadingStr);
                if (!isNaN(magHeading)) {
                    dirSlider.value = Math.round(magHeading);
                    dirVal.textContent = `${Math.round(magHeading)}°`;
                }
                
                // Initialize speed slider to 0% on opening!
                speedSlider.value = 0;
                speedVal.textContent = "0%";
            } else {
                panel.style.display = "none";
                btn.style.backgroundColor = "#334155"; // Reset button color
            }
        });
        
        // Helper to format and send REMOTE direct manual drive command (CMD 25)
        function sendWebRemoteCommand(dir, speed) {
            if (!b.id) return;
            // Standard SET command payload replicating C++ character-for-character!
            // Format: Target_Buoy_ID,98,6,25,25,tgDir,tgSpeed,,,,
            const payload = `${b.id},98,6,25,25,${parseFloat(dir).toFixed(1)},${parseFloat(speed).toFixed(1)},,,,`;
            sendCommand(b.id, payload); // Auto-routes to both Web Serial AND WebSocket!
        }
        
        let lastSendTime = 0;
        function throttledSendRemote(dir, speed) {
            const now = Date.now();
            if (now - lastSendTime > 150) {
                lastSendTime = now;
                sendWebRemoteCommand(dir, speed);
            }
        }
        
        // Slide Target Dir: Update indicator and send throttled packet
        dirSlider.addEventListener("input", (e) => {
            const val = e.target.value;
            dirVal.textContent = `${val}°`;
            throttledSendRemote(val, speedSlider.value);
        });
        
        dirSlider.addEventListener("change", (e) => {
            sendWebRemoteCommand(e.target.value, speedSlider.value); // Force final exact value
        });
        
        // Slide Target Speed: Update indicator and send throttled packet
        speedSlider.addEventListener("input", (e) => {
            const val = e.target.value;
            speedVal.textContent = `${val}%`;
            throttledSendRemote(dirSlider.value, val);
        });
        
        speedSlider.addEventListener("change", (e) => {
            sendWebRemoteCommand(dirSlider.value, e.target.value); // Force final exact value
        });
        
        // IDLE button inside MANUAL panel: stop motors and reset values
        document.getElementById(`mannav-idle-${i}`).addEventListener("click", () => {
            if (!b.id) return;
            
            // 1. Send immediate stop IDLE (CMD 8)
            sendStatusCmd(b.id, MsgType.IDLING);
            
            // 2. Reset speed slider and text values
            speedSlider.value = 0;
            speedVal.textContent = "0%";
            
            // 3. Clear target direction to match buoy's magnetic heading (if available) or default 0
            const magHeading = parseFloat(b.data["Magnetic Dir (Mag)"] || b.data["Magnetic Dir"] || "0");
            dirSlider.value = Math.round(magHeading);
            dirVal.textContent = `${Math.round(magHeading)}°`;
            
            // 4. Force synchronous REMOTE update at 0% speed
            setTimeout(() => {
                sendWebRemoteCommand(dirSlider.value, 0);
            }, 100);
        });
    });
    
    // --- Guided eight point calibration -------------------------------------------------------
    // The marks on the rose are the buttons. renderMancalWeb() decides which are live: the one
    // being asked for, and any already captured, which a tap replaces.
    document.querySelectorAll(".mancal-web-dot").forEach(btn => {
        btn.addEventListener("click", (e) => {
            const i = parseInt(e.currentTarget.getAttribute("data-leg"));
            flashButtonFeedback(e.currentTarget, "#22c55e", "black", 150);
            mancalWebCapture(i);
        });
    });

    const btnBegin = document.getElementById("mancal-web-begin");
    if (btnBegin) {
        btnBegin.addEventListener("click", (e) => {
            flashButtonFeedback(e.currentTarget, "#3b82f6", "white", 150);
            if (!mancalWebSession.known) {
                alert("The buoy has not said what its calibration is doing yet.\n\n"
                    + "Starting a run now could throw away captures made from another screen.");
                return;
            }
            mancalWebPendLeg = -1;
            mancalWebPress(Cal8.BEGIN);
        });
    }

    // The same capture as tapping the mark, for the direction currently being asked for.
    const btnSet = document.getElementById("mancal-web-set");
    if (btnSet) {
        btnSet.addEventListener("click", (e) => {
            flashButtonFeedback(e.currentTarget, "#22c55e", "black", 150);
            mancalWebCapture(mancalWebLeg());
        });
    }

    const btnStop = document.getElementById("mancal-web-stop");
    if (btnStop) {
        btnStop.addEventListener("click", (e) => {
            flashButtonFeedback(e.currentTarget, "#f97316", "black", 150);
            if (mancalWebActiveIndex === null) return;
            const b = buoys[mancalWebActiveIndex];
            if (b && b.id) sendStatusCmd(b.id, MsgType.IDLING);
        });
    }

    // Closes the panel and stops the hull. The run is deliberately left alone: it lives on the buoy
    // and has written nothing, so it can be picked up again here, on the touchscreen or on the
    // buoy's own web page.
    const mancalWebCloseOverlay = () => {
        if (mancalWebActiveIndex !== null) {
            const b = buoys[mancalWebActiveIndex];
            if (b && b.id) sendStatusCmd(b.id, MsgType.IDLING);
        }
        const overlay = document.getElementById("mancal-fullscreen-overlay");
        if (overlay) {
            overlay.classList.remove("active");
            setTimeout(() => { overlay.style.display = "none"; }, 200);
        }
        mancalWebActiveIndex = null;
        if (mancalWebQueryTimer) { clearTimeout(mancalWebQueryTimer); mancalWebQueryTimer = null; }
    };

    const btnClose = document.getElementById("mancal-web-close");
    if (btnClose) {
        btnClose.addEventListener("click", (e) => {
            flashButtonFeedback(e.currentTarget, "#475569", "white", 150);
            mancalWebCloseOverlay();
        });
    }

    // Discards the run. Only ever shown while one is going, and nothing has been written, so this
    // costs exactly the captures made so far and nothing on the buoy.
    const btnCancel = document.getElementById("mancal-web-cancel");
    if (btnCancel) {
        btnCancel.addEventListener("click", (e) => {
            flashButtonFeedback(e.currentTarget, "#ef4444", "white", 150);
            if (!mancalWebRunning()) return;
            if (!confirm("Discard this calibration run?\n\nNothing has been written to the buoy yet.")) return;
            mancalWebPendLeg = -1;
            mancalWebPress(Cal8.CANCEL);
            if (mancalWebActiveIndex !== null) {
                const b = buoys[mancalWebActiveIndex];
                if (b && b.id) sendStatusCmd(b.id, MsgType.IDLING);
            }
        });
    }

    // Commits. The buoy writes all eight entries or none, and beeps when it lands; the panel closes
    // on the buoy's own answer rather than on hope. It does NOT touch the compass offset - which
    // way the sensor is mounted is Set as North's job, and can be done at any time afterwards.
    const btnExit = document.getElementById("mancal-web-exit");
    if (btnExit) {
        btnExit.addEventListener("click", (e) => {
            flashButtonFeedback(e.currentTarget, "#22c55e", "black", 150);
            if (!mancalWebComplete()) return;
            const b = buoys[mancalWebActiveIndex];
            if (!b || !b.id) return;

            mancalWebPress(Cal8.SAVE);
            logMessage(`Buoy ${b.id.toUpperCase()}: saving the eight point calibration - `
                     + `waiting for the buoy to confirm.`, "UDP OUT");

            // Wait for the session to go back to idle, which is the buoy saying it committed.
            const deadline = Date.now() + 5000;
            const check = setInterval(() => {
                if (!mancalWebRunning()) {
                    clearInterval(check);
                    logMessage(`Buoy ${b.id.toUpperCase()}: calibration stored - it should have `
                             + `beeped.`, "UDP IN");
                    mancalWebCloseOverlay();
                } else if (Date.now() > deadline) {
                    clearInterval(check);
                    alert("No confirmation from the buoy.\n\nNothing was saved and the run is "
                        + "still open - press SAVE again, or finish it from another screen.");
                }
            }, 200);
        });
    }

    // Global Action Button listeners
    document.getElementById("align-startline-btn").addEventListener("click", () => {
        logMessage("Global: Align Startline Clicked", "UDP OUT");
        const mainBuoy = buoys.find(b => b.id);
        if (mainBuoy) {
            sendStatusCmd(mainBuoy.id, MsgType.COMPUTESTART);
        }
    });
    
    document.getElementById("align-track-btn").addEventListener("click", () => {
        logMessage("Global: Align Track Clicked", "UDP OUT");
        const mainBuoy = buoys.find(b => b.id);
        if (mainBuoy) {
            sendStatusCmd(mainBuoy.id, MsgType.COMPUTETRACK);
        }
    });
    
    document.getElementById("dock-all-btn").addEventListener("click", () => {
        logMessage("Global: Dock All Clicked", "UDP OUT");
        buoys.forEach((b, idx) => {
            if (b.id) {
                const currentStatus = parseInt(b.data.Status || "0");
                const isDocked = [MsgType.DOCKING, MsgType.DOCKED, MsgType.DOC].includes(currentStatus);
                if (!isDocked) {
                    sendStatusCmd(b.id, MsgType.DOCKING);
                }
            }
        });
    });
    
    // Modal window closes
    document.getElementById("modal-close-btn").addEventListener("click", closeSetupModal);

    // Global Map Button Listener (opens standalone map webpage in a new tab)
    const globalMapBtn = document.getElementById("global-map-btn");
    if (globalMapBtn) {
        globalMapBtn.addEventListener("click", () => {
            window.open("/?view=map", "_blank");
        });
    }
    document.getElementById("setup-modal").addEventListener("click", (e) => {
        if (e.target.id === "setup-modal") closeSetupModal();
    });
    
    // Set as North button click handler
    document.getElementById("setup-setNorth-btn").addEventListener("click", () => {
        if (activeSetupBuoyIndex === null) return;
        const b = buoys[activeSetupBuoyIndex];
        if (!b) return;
        
        const mDir = b.data["Magnetic Dir"];
        if (mDir === undefined || mDir === null || mDir === "N/A" || isNaN(parseFloat(mDir))) {
            alert("No valid magnetic heading data available from buoy yet.");
            return;
        }
        
        const currentOffset = parseInt(document.getElementById("setup-compassOffset").value) || 0;
        const currentHeading = parseFloat(mDir);
        
        let newOffset = Math.round(currentOffset - currentHeading);
        
        // Normalize newOffset to [-180, 180]
        while (newOffset < -180) newOffset += 360;
        while (newOffset > 180) newOffset -= 360;
        
        document.getElementById("setup-compassOffset").value = newOffset;
    });
    
    // In-Field Calibration buttons. These act at once and do not wait for Save, exactly as the
    // matching buttons in RoboTop's Setup dialog do. Command numbers are the msg_t values in
    // RoboCompute.h; sender 99 is the same ID the rest of this page already uses.
    const setupActionBuoy = () => {
        if (activeSetupBuoyIndex === null) return null;
        const b = buoys[activeSetupBuoyIndex];
        return (b && b.id) ? b : null;
    };
    const sendSetupAction = (cmd, payload) => {
        const b = setupActionBuoy();
        if (!b) return;
        // ack 6 = INF. NOT 3 (GETACK): RoboTop's LoRa sender puts a GETACK packet in its
        // retransmit table with retry = 5 and resends it until the target answers, and none
        // of these actions answers - REBOOT sent as GETACK reboots the buoy five times.
        sendCommand(b.id, `${b.id},99,6,${cmd},${cmd},${payload === undefined ? "" : payload}`);
    };

    document.getElementById("setup-deskCal-btn").addEventListener("click", () => {
        if (!confirm("Start the 60 second desk calibration?\n\nTurn the buoy in figures of eight "
                   + "for the whole minute. This replaces the stored hard/soft iron calibration.")) return;
        sendSetupAction(27); // CALIBRATE_MAGNETIC_COMPASS
    });

    document.getElementById("setup-manCal-btn").addEventListener("click", () => {
        // 1. Read active setup index FIRST before closing the modal clears it!
        const index = activeSetupBuoyIndex !== null ? activeSetupBuoyIndex : 0;
        
        // 2. Close Setup Modal
        closeSetupModal();
        
        // 3. Initialize active full-screen mancal indexes with the correct index
        const b = buoys[index];
        if (!b.id) return;
        
        mancalWebActiveIndex = index;
        mancalWebSession = { known: false, active: false, next: 0, mask: 0, seq: 0,
                             captured: Array(8).fill(0) };
        mancalWebOffsets.fill(0);
        mancalWebOffsetsLoaded = false;
        mancalWebImag = null;
        mancalWebPendLeg = -1;
        mancalWebQueryTries = 0;

        // Update header texts
        document.getElementById("mancal-web-buoy-title").textContent = `BUOY ${index + 1} (${b.id.toUpperCase()})`;
        document.getElementById("mancal-web-mag-val").textContent = "0°";

        // Reset speedbars
        document.getElementById("mancal-bar-bb").style.height = "0%";
        document.getElementById("mancal-bar-sb").style.height = "0%";
        document.getElementById("mancal-val-bb").textContent = "0%";
        document.getElementById("mancal-val-sb").textContent = "0%";

        renderMancalWeb();

        // Stop the buoy. Opening this panel commands no heading.
        sendStatusCmd(b.id, MsgType.IDLING);

        // Ask what the buoy's calibration is doing, and for the table it is running. Entry does NOT
        // start a run and does NOT touch the correction - it never has to, because captures take
        // Imag, which sits before the table. A run already going on the buoy, started here, on the
        // touchscreen or on its own web page, is picked up at the same step.
        setTimeout(mancalWebPollState, 400);
        // 5. Open and reveal full-screen overlay with aggressive cached-asset protection and CSS active class
        const overlay = document.getElementById("mancal-fullscreen-overlay");
        if (!overlay) {
            alert("Cache Detected: Your browser is still using an older cached version of the index.html page!\n\nPlease perform a Force Refresh (press Ctrl + F5 or Ctrl + Shift + R) or open this dashboard in an Incognito / Private window to load the new interface.");
            return;
        }
        overlay.style.display = "flex";
        // Force reflow before adding the class so the transition operates perfectly
        overlay.offsetWidth; 
        overlay.classList.add("active");
        
        logMessage(`Buoy ${b.id.toUpperCase()}: Dedicated Fullscreen Manual Compass Calibration started!`, "UDP OUT");
    });

    document.getElementById("setup-reboot-btn").addEventListener("click", () => {
        if (!confirm("Are you sure you want to reboot this buoy?")) return;
        sendSetupAction(85); // REBOOT
    });

    // Modal save submission
    document.getElementById("modal-form").addEventListener("submit", (e) => {
        e.preventDefault();
        saveSetupForm();
    });
}

// Open modal, start polling the buoy for setup keys
function openSetupModal(buoyIndex) {
    // Clear any active leftover polling timers to prevent modal-close conflicts
    clearTimeout(setupCheckTimer);
    
    // Order matters: bail out BEFORE claiming the slot. Setting activeSetupBuoyIndex first meant a
    // click on a buoy that has not reported yet left the dialog closed but the index armed, so the
    // next poll or a stray save would act on a buoy the operator never opened.
    const b = buoys[buoyIndex];
    if (!b || !b.id) return;
    activeSetupBuoyIndex = buoyIndex;
    
    // Reset any setup fields to force fetch reload
    const setupKeys = ["Kpr", "Kir", "Kdr", "Kps", "Kis", "Kds", "maxSpeed", "minSpeed", "pivotSpeed", "compassOffset", "holdRad", "revBB", "revSB", "swap_BB_SB", "compass_trim", "compass_trim_enabled"];
    setupKeys.forEach(key => delete b.data[key]);
    
    document.getElementById("modal-buoy-title").textContent = `Setup Buoy ${b.id.toUpperCase()}`;
    document.getElementById("modal-loading-view").classList.remove("hidden");
    document.getElementById("modal-form").classList.add("hidden");
    document.getElementById("setup-modal").classList.add("active");
    
    setupCheckRetries = 0;
    querySetupAndPoll();
}

function querySetupAndPoll() {
    if (activeSetupBuoyIndex === null) return;
    const b = buoys[activeSetupBuoyIndex];
    
    if (setupCheckRetries > 25) { // Timeout after 37.5 seconds (25 attempts * 1500ms)
        alert(`Timeout: Could not retrieve setup data for Buoy ${b.id}`);
        closeSetupModal();
        return;
    }
    
    // Check if we have received all required keys
    const hasKeys = ["Kpr", "maxSpeed", "pivotSpeed", "holdRad"].every(key => b.data[key] !== undefined);
    
    if (hasKeys) {
        // Data received! Hide loading, populate inputs, reveal form
        document.getElementById("modal-loading-view").classList.add("hidden");
        document.getElementById("modal-form").classList.remove("hidden");
        
        document.getElementById("setup-Kpr").value = parseFloat(b.data["Kpr"]) || 0;
        document.getElementById("setup-Kir").value = parseFloat(b.data["Kir"]) || 0;
        document.getElementById("setup-Kdr").value = parseFloat(b.data["Kdr"]) || 0;
        
        document.getElementById("setup-Kps").value = parseFloat(b.data["Kps"]) || 0;
        document.getElementById("setup-Kis").value = parseFloat(b.data["Kis"]) || 0;
        document.getElementById("setup-Kds").value = parseFloat(b.data["Kds"]) || 0;
        
        document.getElementById("setup-maxSpeed").value = parseFloat(b.data["maxSpeed"]) || 0;
        document.getElementById("setup-minSpeed").value = parseFloat(b.data["minSpeed"]) || 0;
        document.getElementById("setup-pivotSpeed").value = parseFloat(b.data["pivotSpeed"]) || 0.2;
        document.getElementById("setup-compassOffset").value = parseInt(b.data["compassOffset"]) || 0;
        document.getElementById("setup-holdRad").value = parseFloat(b.data["holdRad"]) || 2.0;
        
        const trimEn = b.data["compass_trim_enabled"] === "1" || b.data["compass_trim_enabled"] === 1 || b.data["compass_trim_enabled"] === true || b.data["compass_trim_enabled"] === "true";
        document.getElementById("setup-compassTrimEnabled").checked = trimEn;
        document.getElementById("setup-revBB").checked = b.data["revBB"] === "1";
        document.getElementById("setup-revSB").checked = b.data["revSB"] === "1";
        document.getElementById("setup-harmonic").checked = b.data["harmonic_enabled"] === "1";
        document.getElementById("setup-swap").checked = b.data["swap_BB_SB"] === "1";
        document.getElementById("setup-dockAppDist").value = parseInt(b.data["dockAppDist"]) || 20;
        document.getElementById("setup-dockAppDir").value = parseInt(b.data["dockAppDir"]) || 180;
        document.getElementById("setup-dockToWP").checked = b.data["dockToWP"] === "1" || b.data["dockToWP"] === 1 || b.data["dockToWP"] === true || b.data["dockToWP"] === "true";
    } else {
        // Send fetch query request ($buoyId,99,MsgType.GET,MsgType.SETUPDATA,,,,,,,)
        sendCommand(b.id, `${b.id},99,${MsgType.GET},${MsgType.SETUPDATA},,,,,,,`);
        
        // Also query ADAPTIVE_TRIM
        sendCommand(b.id, `${b.id},99,${MsgType.GET},${MsgType.ADAPTIVE_TRIM},,,,,,,`);
        
        // Wait 1500ms and retry to prevent LoRa channel congestion
        setupCheckRetries++;
        setupCheckTimer = setTimeout(querySetupAndPoll, 1500);
    }
}

function closeSetupModal() {
    activeSetupBuoyIndex = null;
    clearTimeout(setupCheckTimer);
    document.getElementById("setup-modal").classList.remove("active");
}

// Build and transmit set command on setup save
function saveSetupForm() {
    if (activeSetupBuoyIndex === null) return;
    const b = buoys[activeSetupBuoyIndex];
    if (!b.id) return;
    
    const currStatus = b.data.Status || "7";
    const trimEn = document.getElementById("setup-compassTrimEnabled").checked ? "1" : "0";
    
    const values = [
        document.getElementById("setup-Kpr").value,
        document.getElementById("setup-Kir").value,
        document.getElementById("setup-Kdr").value,
        document.getElementById("setup-Kps").value,
        document.getElementById("setup-Kis").value,
        document.getElementById("setup-Kds").value,
        document.getElementById("setup-maxSpeed").value,
        document.getElementById("setup-minSpeed").value,
        document.getElementById("setup-pivotSpeed").value,
        document.getElementById("setup-compassOffset").value,
        document.getElementById("setup-holdRad").value,
        document.getElementById("setup-revBB").checked ? "1" : "0",
        document.getElementById("setup-revSB").checked ? "1" : "0",
        document.getElementById("setup-swap").checked ? "1" : "0",
        trimEn,
        document.getElementById("setup-dockAppDist").value,
        document.getElementById("setup-dockAppDir").value,
        document.getElementById("setup-dockToWP").checked ? "1" : "0",
        // Tri-state, see the parser above: never a plain 0.
        document.getElementById("setup-harmonic").checked ? "2" : "1"
    ];
    
    // Construct message: Target,99,SET,SETUPDATA,Status,vals...
    const commandPayload = `${b.id},99,${MsgType.SET},${MsgType.SETUPDATA},${currStatus},${values.join(",")}`;
    sendCommand(b.id, commandPayload);
    
    // Also explicitly send an ADAPTIVE_TRIM (CMD 84) set command to sync right away
    const currentTrimVal = parseFloat(b.data["compass_trim"] || "0.0");
    const trimPayload = `${b.id},99,${MsgType.SET},${MsgType.ADAPTIVE_TRIM},${currStatus},${currentTrimVal.toFixed(4)},${trimEn}`;
    sendCommand(b.id, trimPayload);
    
    closeSetupModal();
}

// Render list of other discovered devices on the dashboard
function renderOtherDevices() {
    const container = document.getElementById("other-devices-container");
    const listEl = document.getElementById("other-devices-list");
    if (!container || !listEl) return;
    
    // Filter out devices that haven't been seen in the last 30 seconds to keep the list fresh
    const now = Date.now();
    for (const id in otherDevices) {
        if (now - otherDevices[id].lastSeen > 30000) {
            delete otherDevices[id];
        }
    }
    
    const ids = Object.keys(otherDevices);
    if (ids.length === 0) {
        container.classList.add("hidden");
        listEl.innerHTML = "";
        return;
    }
    
    container.classList.remove("hidden");
    
    // Construct clickable links
    const links = ids.map(id => {
        const dev = otherDevices[id];
        const label = id.toUpperCase();
        
        let ipLink = "";
        if (dev.ip) {
            ipLink = ` (<a href="http://${dev.ip}/" target="_blank" class="other-device-ip-link" title="Open device web page">${dev.ip}</a>)`;
        }
        
        const rssiText = (dev.loraRssi !== null && (now - dev.lastLoraTime < 5000)) ? ` [${dev.loraRssi} dBm]` : "";
        
        return `<span class="other-device-item">
            <a href="#" onclick="assignOtherDevice('${id}'); return false;" class="other-device-id-link" title="Click to assign to an active buoy slot">${label}</a>${rssiText}${ipLink}
        </span>`;
    });
    
    listEl.innerHTML = links.join(" ");
}

// Interactively assign/swap an other device to one of the 3 active buoy slots
function assignOtherDevice(deviceId) {
    const slotStr = prompt(`Enter buoy slot number (1, 2, or 3) to assign device ${deviceId.toUpperCase()}:`, "1");
    if (slotStr === null) return;
    const slotIdx = parseInt(slotStr, 10) - 1;
    if (slotIdx >= 0 && slotIdx < 3) {
        const b = buoys[slotIdx];
        const oldId = b.id;
        
        // Assign the new device ID
        b.id = deviceId.toLowerCase();
        b.title = `Buoy: ${deviceId.toUpperCase()}`;
        b.ip = otherDevices[deviceId]?.ip || null;
        b.lastLoraTime = otherDevices[deviceId]?.lastLoraTime || 0;
        b.lastUdpTime = otherDevices[deviceId]?.lastUdpTime || 0;
        b.loraRssi = otherDevices[deviceId]?.loraRssi || null;
        b.data = {}; // Reset data for a clean slate
        
        // Update the header title on the card
        document.getElementById(`buoy-title-${slotIdx}`).textContent = b.title;
        
        // Remove from other devices
        delete otherDevices[deviceId];
        
        // If there was an active device already in that slot, recycle it back to other devices
        if (oldId) {
            otherDevices[oldId] = {
                id: oldId,
                ip: b.ip,
                lastSeen: Date.now(),
                lastLoraTime: b.lastLoraTime,
                lastUdpTime: b.lastUdpTime,
                loraRssi: b.loraRssi
            };
        }
        
        updateGUI();
        renderOtherDevices();
    } else {
        alert("Invalid slot number! Please enter 1, 2, or 3.");
    }
}

// Bind handlers to global window scope so dynamic HTML onclick can trigger them
window.assignOtherDevice = assignOtherDevice;
window.renderOtherDevices = renderOtherDevices;


// Standalone Fullscreen Map View Initializer (Super Safe & Isolated)
const urlParams = new URLSearchParams(window.location.search);
const isMapView = urlParams.get('view') === 'map' || window.location.hash === '#map';

if (isMapView) {
    document.addEventListener("DOMContentLoaded", () => {
        // Hide standard dashboard layout elements
        const header = document.querySelector("header");
        if (header) header.style.display = "none";
        const main = document.querySelector("main");
        if (main) main.style.display = "none";
        
        // Show full map canvas container
        const fullMapView = document.getElementById("full-map-view");
        if (fullMapView) {
            fullMapView.style.display = "block";
            
            // Adjust canvas size to window size dynamically
            const canvas = document.getElementById("fieldCanvas");
            if (canvas) {
                canvas.width = window.innerWidth;
                canvas.height = window.innerHeight;
                window.addEventListener("resize", () => {
                    canvas.width = window.innerWidth;
                    canvas.height = window.innerHeight;
                    drawFieldMap();
                });
            }
        }
        
        // Point the gateway at the device this page was served from, exactly as the dashboard's
        // load handler does. That handler returns early in map view, so without this the map tab
        // connects to the placeholder baked into index.html and never receives any telemetry.
        const wsUrlInput = document.getElementById("ws-url");
        if (wsUrlInput && window.location.hostname) {
            wsUrlInput.value = `ws://${window.location.hostname}:81`;
        }

        // Auto connect WebSocket if not already connected
        if (typeof connectWebSocket === "function") {
            setTimeout(connectWebSocket, 100);
        }
        
        // Start line length panel - measured once on open, see initStartLinePanel().
        
        initStartLinePanel();

        
        // Start independent high-performance map redraw loop
        setInterval(drawFieldMap, 500);
    });
}

// Project GPS coordinates to a new GPS position given bearing (degrees) and distance (meters)
function projectCoords(lat, lng, bearing, distance) {
    const R = 6371000; // Earth's radius in meters
    const brng = bearing * Math.PI / 180;
    const lat1 = lat * Math.PI / 180;
    const lon1 = lng * Math.PI / 180;
    const dR = distance / R;

    const lat2 = Math.asin(Math.sin(lat1) * Math.cos(dR) +
                           Math.cos(lat1) * Math.sin(dR) * Math.cos(brng));
    const lon2 = lon1 + Math.atan2(Math.sin(brng) * Math.sin(dR) * Math.cos(lat1),
                                  Math.cos(dR) - Math.sin(lat1) * Math.sin(lat2));

    return {
        lat: lat2 * 180 / Math.PI,
        lng: lon2 * 180 / Math.PI
    };
}

// =================================================================================================
// Map page - the same page the CYD shows under TRACK SETTINGS, in the browser.
//
// North up, not wind up. The plot is centred on the fleet, the start line is drawn between the two
// buoys that are CLOSEST together (the rule RoboCompute::recalcStartLine() itself applies - the
// shortest leg is the line, the remaining buoy is the upwind HEAD mark), and the wind is an arrow
// parked on the upwind rim blowing inwards.
//
// The previous version rotated the whole plot to put the wind at the top and assigned the PORT and
// STBD roles by tab order, so the line was drawn between whichever two buoys happened to be first
// in the list rather than between the two that actually form it.
// =================================================================================================
const MAP_LINE_STEP_M = 5;
const MAP_WAYPOINT_MAX_M = 300;    // Past this a "target" is a fault, not a mark - see below
// Sailing convention, matching the CYD touchscreen exactly: starboard hand GREEN, port hand RED,
// the upwind HEAD mark BLUE, and plain green for any buoy that is present while the fleet is not
// yet fully locked. Roles only exist once EVERY present buoy is locked - before that no course has
// been laid out, so there are no sides to be on.
const MAP_COLOR_DEFAULT   = "#22c55e";  // locked, but no course laid out to have sides of
const MAP_COLOR_UNLOCKED  = "#eab308";  // present but not holding a station yet
const MAP_COLOR_STARBOARD = "#22c55e";
const MAP_COLOR_PORT      = "#ef4444";
const MAP_COLOR_HEAD      = "#3b82f6";
const MAP_COLOR_OFFLINE   = "#64748b";  // greyed, never red - red means port hand here

let mapLineCurM = -1;     // Live measurement, < 0 when there is no line to measure
let mapLineTgtM = -1;     // Dialled by - / +, < 0 when nothing is pending
let mapLineEnds = null;   // [{index, id, lat, lng}, ...] for the two ends of the line
let mapFleet = 0;         // Buoys with a usable fix
let mapLocked = 0;        // ...of which are holding a lock position

function mapMeters(lat1, lng1, lat2, lng2) {
    const R = 6371000, rad = Math.PI / 180;
    const dLat = (lat2 - lat1) * rad, dLng = (lng2 - lng1) * rad;
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2)
            + Math.cos(lat1 * rad) * Math.cos(lat2 * rad) * Math.sin(dLng / 2) * Math.sin(dLng / 2);
    return 2 * R * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

function mapBearing(lat1, lng1, lat2, lng2) {
    const rad = Math.PI / 180;
    const y = Math.sin((lng2 - lng1) * rad) * Math.cos(lat2 * rad);
    const x = Math.cos(lat1 * rad) * Math.sin(lat2 * rad)
            - Math.sin(lat1 * rad) * Math.cos(lat2 * rad) * Math.cos((lng2 - lng1) * rad);
    return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
}

// Metres east / north of a reference point. Flat earth, exact enough over a course.
function mapOffset(refLat, refLng, lat, lng) {
    const rad = Math.PI / 180;
    return {
        dx: (lng - refLng) * Math.cos(refLat * rad) * 111320,
        dy: (lat - refLat) * 111139
    };
}

// The buoys with a usable fix, and the waypoint each is steering at.
//
// A buoy only has a set point while it is actually holding one. When it is idle the reported
// Target Dir / Target Dist still describe the last leg it sailed, and projecting that is not
// merely cosmetic - the waypoint is included in the auto-scale bounds, so one stale target
// kilometres away shrinks the plot until the whole fleet collapses into a single dot.
function mapValidBuoys() {
    const out = [];
    buoys.forEach((b, i) => {
        if (!b.id) return;
        const lat = parseFloat(b.data["Latitude (Lat)"]);
        const lng = parseFloat(b.data["Longitude (Lon)"]);
        if (isNaN(lat) || isNaN(lng) || lat === 0 || lng === 0) return;

        const st = parseInt(b.data.Status || "0");
        const holdsTarget = (st === MsgType.LOCKING || st === MsgType.LOCKED
                          || st === MsgType.DOCKING || st === MsgType.DOCKED);
        const dist = parseFloat(b.data["Target Dist"]);
        const dir = parseFloat(b.data["Target Dir"]);

        let wp = null;
        if (holdsTarget && !isNaN(dist) && !isNaN(dir) && dist > 0) wp = projectCoords(lat, lng, dir, dist);

        // LOCKING counts as locked: the buoy owns a real lock position and is on its way to
        // it. DOCKING/DOCKED do not - a buoy going home is not part of a course.
        const locked = (st === MsgType.LOCKING || st === MsgType.LOCKED);

        out.push({
            index: i,
            id: b.id,
            locked: locked,
            lat: lat, lng: lng,
            wp: wp,
            wpDist: wp ? dist : 0,
            wDir: parseFloat(b.data["Wind Dir"]) || 0,
            wStd: parseFloat(b.data["Wind StdDev"]) || 0,
            heading: parseFloat(b.data["Magnetic Dir"]) || 0
        });
    });
    return out;
}

// Which buoy is HEAD / PORT / STARBOARD, or null for "no course laid out".
//
// Derived exactly the way RoboCompute::recalcStartLine() derives it, and the way the CYD firmware's
// compute_buoy_roles() does, so all three agree: the two buoys closest together are the start line,
// the odd one out is the upwind HEAD mark, and the starboard end is whichever end lies towards
// wDir + 90 from the middle of the line.
//
// Anything missing - fewer than two buoys, one of them not locked, or no wind reading anywhere -
// returns no roles at all. Green for "present, no course" is honest; guessing a side from
// incomplete data and painting a buoy red would not be.
function mapBuoyRoles(list) {
    const roles = {};
    if (list.length < 2 || list.filter(b => b.locked).length !== list.length) return roles;

    const wind = list.find(b => b.wDir !== 0 || b.wStd !== 0);
    if (!wind) return roles;

    let a = list[0], b = list[1];
    if (list.length === 3) {
        let best = -1;
        for (let i = 0; i < list.length; i++) {
            for (let j = i + 1; j < list.length; j++) {
                const pi = mapLinePoint(list[i]), pj = mapLinePoint(list[j]);
                const d = mapMeters(pi.lat, pi.lng, pj.lat, pj.lng);
                if (best < 0 || d < best) { best = d; a = list[i]; b = list[j]; }
            }
        }
        const head = list.find(x => x !== a && x !== b);
        if (head) roles[head.index] = MAP_COLOR_HEAD;
    }

    const pa = mapLinePoint(a), pb = mapLinePoint(b);
    const midLat = (pa.lat + pb.lat) / 2, midLng = (pa.lng + pb.lng) / 2;
    const sb = (wind.wDir + 90) % 360;
    const off = p => Math.abs(((mapBearing(midLat, midLng, p.lat, p.lng) - sb + 540) % 360) - 180);
    if (off(pa) <= off(pb)) {
        roles[a.index] = MAP_COLOR_STARBOARD;
        roles[b.index] = MAP_COLOR_PORT;
    } else {
        roles[a.index] = MAP_COLOR_PORT;
        roles[b.index] = MAP_COLOR_STARBOARD;
    }
    return roles;
}

// Where a buoy's end of the line is: its commanded waypoint if it is holding one, otherwise its own
// fix. Preferring the waypoint keeps the geometry clean - a buoy on station wanders inside its hold
// radius, and measuring off that wobble makes both the length and the bearing EXECUTE re-uses
// jitter between refreshes. The two coincide once the buoy has arrived.
function mapLinePoint(b) {
    return b.wp ? { lat: b.wp.lat, lng: b.wp.lng } : { lat: b.lat, lng: b.lng };
}

// The buoy whose wind reading should drive a course computation. COMPUTESTART/COMPUTETRACK are
// carried out by whichever buoy receives them, against ITS OWN wind, and the Top refuses outright
// when wDir and wStd are both 0 - a failed compass reports 0/0, which is indistinguishable from a
// real due-north calm, and computing anyway squares the line to a fake northerly.
function mapWindRefBuoy() {
    return mapValidBuoys().find(b => b.wDir !== 0 || b.wStd !== 0) || null;
}

function mapMessage(text, color) {
    const el = document.getElementById("map-msg");
    if (!el) return;
    el.textContent = text || "";
    el.style.color = color || "#94a3b8";
}

function mapNudge(delta) {
    if (mapFleet < 2 || mapLineCurM <= 0 || !mapLineEnds) return;
    const base = (mapLineTgtM > 0) ? mapLineTgtM : mapLineCurM;
    mapLineTgtM = Math.max(MAP_LINE_STEP_M, base + delta);
    mapMessage("");
    drawFieldMap();
}

// ALIGN STARTLINE squares the line to the wind, ALIGN TRACK lays out the whole course. Both need
// every deployed buoy to be holding a lock position - the Top bails out and beeps otherwise.
function mapAlign(cmdId) {
    const need = (cmdId === MsgType.COMPUTETRACK) ? 3 : 2;
    if (mapLocked < need) {
        mapMessage("Need " + need + " buoys LOCKED before the line can be squared - "
                 + mapLocked + " locked now.", "#f87171");
        return;
    }

    const ref = mapWindRefBuoy();
    if (!ref) {
        mapMessage("No buoy is reporting wind - nothing to square the line against.", "#f87171");
        return;
    }

    // The fleet is about to be given new targets, so nothing dialled before this still applies.
    mapLineTgtM = -1;
    sendStatusCmd(ref.id, cmdId);

    const what = (cmdId === MsgType.COMPUTETRACK) ? "ALIGN TRACK" : "ALIGN STARTLINE";
    logMessage(`Map: ${what} sent to ${ref.id.toUpperCase()}`, "UDP OUT");
    mapMessage(what + " sent to " + ref.id.toUpperCase()
             + ". It refuses unless every deployed buoy is locked.", "#94a3b8");
    drawFieldMap();
}

// Move both ends of the line symmetrically about its current midpoint, keeping its present bearing.
// Length only - squaring it to the wind is what ALIGN STARTLINE is for. SETLOCKPOS is the same
// command the Top uses to push computed line ends to the other buoy: store the point, sail, lock.
function mapExecute() {
    if (!mapLineEnds || mapLineCurM <= 0 || mapLineTgtM <= 0) return;
    if (Math.round(mapLineTgtM) === Math.round(mapLineCurM)) return;

    const a = mapLineEnds[0], b = mapLineEnds[1];
    const midLat = (a.lat + b.lat) / 2, midLng = (a.lng + b.lng) / 2;
    const brgA = mapBearing(midLat, midLng, a.lat, a.lng);
    const brgB = mapBearing(midLat, midLng, b.lat, b.lng);
    const half = mapLineTgtM / 2;

    if (!confirm("Re-align the start line to " + Math.round(mapLineTgtM) + " m?\n\n"
               + "Both end buoys will motor to their new ends of the line.")) return;

    [[a, projectCoords(midLat, midLng, brgA, half)],
     [b, projectCoords(midLat, midLng, brgB, half)]].forEach(function (pair) {
        const end = pair[0], pos = pair[1];
        const status = (buoys[end.index] && buoys[end.index].data.Status) || String(MsgType.IDLE);
        sendCommand(end.id, end.id + ",99," + MsgType.SET + "," + MsgType.SETLOCKPOS + ","
                          + status + "," + pos.lat.toFixed(10) + "," + pos.lng.toFixed(10));
        logMessage("Buoy " + end.id.toUpperCase() + ": start line end moved to "
                 + pos.lat.toFixed(6) + ", " + pos.lng.toFixed(6), "UDP OUT");
    });

    mapMessage("Start line set to " + Math.round(mapLineTgtM) + " m.", "#22c55e");

    // The dialled figure is the line now. Clearing the pending value hands the read-out back to the
    // live measurement, which walks to the new length as the two buoys motor out to it.
    mapLineTgtM = -1;
}

function mapSetControls(buoyList) {
    const canLen = mapFleet >= 2 && mapLineCurM > 0 && mapLineEnds !== null;
    const canExec = canLen && mapLineTgtM > 0
                    && Math.round(mapLineTgtM) !== Math.round(mapLineCurM);
    const set = (id, on, label) => {
        const el = document.getElementById(id);
        if (!el) return;
        el.disabled = !on;
        el.style.opacity = on ? "1" : "0.35";
        el.style.cursor = on ? "pointer" : "not-allowed";
        if (label !== undefined) el.textContent = label;
    };
    set("map-minus", canLen);
    set("map-plus", canLen);
    // ALIGN squares the line through the two ends' LOCK positions. With a buoy unlocked the
    // Top would compute against whatever stale target it still carries - a dock position
    // kilometres away - so the button stays dead until the fleet is actually locked. The Top
    // refuses the same request independently; this is so the button says so first.
    set("map-align-start", mapLocked >= 2);
    set("map-align-track", mapLocked >= 3);

    const warn = document.getElementById("map-lockwarn");
    if (warn) warn.style.display = (mapLocked < 2) ? "block" : "none";
    set("map-execute", canExec, canExec ? "EXECUTE " + Math.round(mapLineTgtM) + " m" : "EXECUTE");

    const lenEl = document.getElementById("map-line-len");
    if (lenEl) {
        if (mapLineTgtM > 0) {
            lenEl.textContent = Math.round(mapLineTgtM) + " m";
            lenEl.style.color = "#fb923c";           // Amber: dialled, not committed
        } else if (mapLineCurM > 0) {
            lenEl.textContent = Math.round(mapLineCurM) + " m";
            lenEl.style.color = "#e879f9";           // Magenta, same as the line on the plot
        } else {
            lenEl.textContent = "--";
            lenEl.style.color = "#64748b";
        }
    }

    const windEl = document.getElementById("map-wind");
    if (windEl) {
        const w = buoyList.find(b => b.wDir !== 0 || b.wStd !== 0);
        if (!w) {
            windEl.textContent = "--";
            windEl.style.color = "#64748b";
        } else {
            windEl.textContent = w.wStd !== 0
                ? Math.round(w.wDir) + "° ±" + Math.round(w.wStd)
                : Math.round(w.wDir) + "°";
            windEl.style.color = "#facc15";
        }
    }
}

// Draw the radar map of the buoys and the start line on the canvas
function drawFieldMap() {
    const canvas = document.getElementById("fieldCanvas");
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    const list = mapValidBuoys();
    mapFleet = list.length;
    mapLocked = list.filter(b => b.locked).length;

    const cx = canvas.width / 2, cy = canvas.height / 2;
    const rMax = Math.min(canvas.width, canvas.height) / 2 - 60;

    // --- Grid, north up ---
    ctx.strokeStyle = "rgba(71, 85, 105, 0.5)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(cx - rMax, cy); ctx.lineTo(cx + rMax, cy);
    ctx.moveTo(cx, cy - rMax); ctx.lineTo(cx, cy + rMax);
    ctx.stroke();

    ctx.fillStyle = "#22c55e";
    ctx.font = "bold 18px Arial";
    ctx.textAlign = "center";
    ctx.fillText("N", cx, cy - rMax - 10);
    ctx.fillText("S", cx, cy + rMax + 24);
    ctx.textAlign = "right";
    ctx.fillText("W", cx - rMax - 10, cy + 6);
    ctx.textAlign = "left";
    ctx.fillText("E", cx + rMax + 10, cy + 6);
    ctx.textAlign = "center";

    if (list.length === 0) {
        mapLineCurM = -1; mapLineTgtM = -1; mapLineEnds = null; mapLocked = 0;
        ctx.fillStyle = "#94a3b8";
        ctx.font = "18px Arial";
        ctx.fillText("No live GPS data available", cx, cy);
        mapSetControls(list);
        return;
    }

    // --- Centre on the fleet: the middle of the line with two, the centroid with three ---
    let refLat = 0, refLng = 0;
    list.forEach(b => { refLat += b.lat; refLng += b.lng; });
    refLat /= list.length; refLng /= list.length;

    const roles = mapBuoyRoles(list);
    // Role colour where there is one; otherwise yellow until the buoy is actually holding a
    // station. Matches the CYD touchscreen exactly - anything yellow is not on station yet.
    const colorOf = b => roles[b.index] || (b.locked ? MAP_COLOR_DEFAULT : MAP_COLOR_UNLOCKED);

    // --- Range. Waypoints widen it too, so a buoy well off station still has its mark on screen -
    //     but only out to MAP_WAYPOINT_MAX_M. A target further away than that is a fault rather
    //     than a course, and zooming out to fit it squashes the whole fleet into one dot; those
    //     are pegged to the rim by clampToPlot() below instead. ---
    let maxDist = 10;
    list.forEach(b => {
        const o = mapOffset(refLat, refLng, b.lat, b.lng);
        maxDist = Math.max(maxDist, Math.hypot(o.dx, o.dy));
        if (b.wp) {
            const w = mapOffset(refLat, refLng, b.wp.lat, b.wp.lng);
            const d = Math.hypot(w.dx, w.dy);
            if (d <= MAP_WAYPOINT_MAX_M) maxDist = Math.max(maxDist, d);
        }
    });

    let range = 25;
    for (const r of [25, 50, 100, 250, 500, 1000]) { range = r; if (maxDist <= r * 0.95) break; }
    const S = rMax / range;

    ctx.fillStyle = "#64748b";
    ctx.font = "14px Arial";
    ctx.fillText("EDGE = " + range + " m", cx, cy - rMax - 34);

    const toScreen = (lat, lng) => {
        const o = mapOffset(refLat, refLng, lat, lng);
        return { x: cx + o.dx * S, y: cy - o.dy * S };
    };
    const clampToPlot = p => {
        const x = Math.max(cx - rMax, Math.min(cx + rMax, p.x));
        const y = Math.max(cy - rMax, Math.min(cy + rMax, p.y));
        return { x: x, y: y, pegged: (x !== p.x || y !== p.y) };
    };

    // --- Wind arrow, first so the line and the dots end up on top of it. wDir follows the
    //     RoboCompute convention: the bearing the wind blows FROM, i.e. the upwind side. ---
    const wind = list.find(b => b.wDir !== 0 || b.wStd !== 0);
    if (wind) {
        const th = wind.wDir * Math.PI / 180;
        const s = Math.sin(th), c = Math.cos(th);
        const tail = { x: cx + (rMax - 4) * s, y: cy - (rMax - 4) * c };
        const tip = { x: cx + (rMax - 70) * s, y: cy - (rMax - 70) * c };
        ctx.strokeStyle = "rgba(250, 204, 21, 0.75)";
        ctx.lineWidth = 4;
        ctx.beginPath();
        ctx.moveTo(tail.x, tail.y); ctx.lineTo(tip.x, tip.y);
        ctx.stroke();
        const base = { x: cx + (rMax - 52) * s, y: cy - (rMax - 52) * c };
        ctx.fillStyle = "rgba(250, 204, 21, 0.75)";
        ctx.beginPath();
        ctx.moveTo(tip.x, tip.y);
        ctx.lineTo(base.x + 9 * c, base.y + 9 * s);
        ctx.lineTo(base.x - 9 * c, base.y - 9 * s);
        ctx.fill();
    }

    // --- Start line: between the two buoys CLOSEST together, the same rule the firmware uses.
    //     Measured off the two ends' line points rather than off the plotted dots, because that is
    //     the figure - and the bearing - EXECUTE has to be able to reproduce. ---
    let slA = null, slB = null, slDist = 0;
    for (let i = 0; i < list.length; i++) {
        for (let j = i + 1; j < list.length; j++) {
            const pi = mapLinePoint(list[i]), pj = mapLinePoint(list[j]);
            const d = mapMeters(pi.lat, pi.lng, pj.lat, pj.lng);
            if (slA === null || d < slDist) { slDist = d; slA = list[i]; slB = list[j]; }
        }
    }

    if (slA) {
        const pa = mapLinePoint(slA), pb = mapLinePoint(slB);
        mapLineCurM = slDist;
        mapLineEnds = [{ index: slA.index, id: slA.id, lat: pa.lat, lng: pa.lng },
                       { index: slB.index, id: slB.id, lat: pb.lat, lng: pb.lng }];

        const sa = clampToPlot(toScreen(pa.lat, pa.lng));
        const sb = clampToPlot(toScreen(pb.lat, pb.lng));
        ctx.strokeStyle = "#e879f9";
        ctx.lineWidth = 5;
        ctx.beginPath();
        ctx.moveTo(sa.x, sa.y); ctx.lineTo(sb.x, sb.y);
        ctx.stroke();

        ctx.fillStyle = "#e879f9";
        ctx.font = "italic bold 14px Arial";
        ctx.fillText(slDist.toFixed(1) + " m", (sa.x + sb.x) / 2, (sa.y + sb.y) / 2 - 10);
    } else {
        mapLineCurM = -1; mapLineEnds = null; mapLineTgtM = -1;
    }

    // --- Each buoy's waypoint and the gap to it: a hollow cross-ring against the filled dot, so
    //     "where it should be" and "where it is" never need labels to tell apart. ---
    list.forEach(b => {
        if (!b.wp) return;
        const col = colorOf(b);
        const p = clampToPlot(toScreen(b.lat, b.lng));
        const w = clampToPlot(toScreen(b.wp.lat, b.wp.lng));

        ctx.strokeStyle = col;
        ctx.lineWidth = 1.5;
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        ctx.moveTo(p.x, p.y); ctx.lineTo(w.x, w.y);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.beginPath();
        ctx.arc(w.x, w.y, 6, 0, Math.PI * 2);
        ctx.stroke();
        if (w.pegged) { ctx.beginPath(); ctx.arc(w.x, w.y, 10, 0, Math.PI * 2); ctx.stroke(); }
        ctx.beginPath();
        ctx.moveTo(w.x - 10, w.y); ctx.lineTo(w.x + 10, w.y);
        ctx.moveTo(w.x, w.y - 10); ctx.lineTo(w.x, w.y + 10);
        ctx.stroke();

        ctx.fillStyle = col;
        ctx.font = "12px Arial";
        ctx.textAlign = "left";
        ctx.fillText(b.wpDist.toFixed(1) + " m", w.x + 13, w.y + 16);
        ctx.textAlign = "center";
    });

    // --- The buoys themselves, on top of everything ---
    list.forEach(b => {
        const col = colorOf(b);
        const p = clampToPlot(toScreen(b.lat, b.lng));

        // Heading whisker, north up
        const hr = b.heading * Math.PI / 180;
        ctx.strokeStyle = col;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(p.x, p.y);
        ctx.lineTo(p.x + 26 * Math.sin(hr), p.y - 26 * Math.cos(hr));
        ctx.stroke();

        ctx.fillStyle = col;
        ctx.strokeStyle = "#0f172a";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(p.x, p.y, 9, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        ctx.fillStyle = "#fff";
        ctx.font = "bold 14px Arial";
        ctx.textAlign = "left";
        ctx.fillText("B" + (b.index + 1), p.x + 14, p.y - 12);
        ctx.textAlign = "center";
    });

    mapSetControls(list);
}

// =================================================================================================
// Simplified main page + per-buoy detail view
//
// The main page carries only what is worth reading at arm's length on the water: status banner,
// windrose, speed bars, and LOCK / DOCK. Everything else - setup, manual steering, the dir/dist
// entry, the parameter table, the network indicators and the voltage/current gauges - is moved
// into a per-buoy overlay behind the three "Buoy n ..." buttons.
//
// The blocks are MOVED at runtime rather than duplicated or re-authored in index.html. The
// dashboard addresses these elements by id in well over a hundred places, and appendChild() keeps
// every one of those references alive: the node is the same node, it just hangs somewhere else in
// the tree. Rewriting the markup would have meant re-pointing all of them.
// =================================================================================================

// Per-card selectors to relocate. Order here is the order they appear in the overlay.
// Which buoy's detail overlay is open, or -1. Declared here rather than next to
// openBuoyDetail() below because updateGUI() tests it, and updateGUI() runs first.
let activeDetailBuoy = -1;

const BUOY_DETAIL_BLOCKS = [
    ".network-indicators",
    ".coordinate-control",
    ".mannav-control",
    ".parameters-panel",
    ".udp-enable-wrapper",
    ".color-legend",
];

function buildBuoyDetailViews() {
    for (let i = 0; i < 3; i++) {
        const card = document.getElementById("buoy-card-" + i);
        const body = document.getElementById("buoy-detail-body-" + i);
        if (!card || !body) continue;

        // A windrose of its own, first in the view. Not the card's node moved across - that one is
        // the whole point of the card - but a second canvas fed by the same drawWindrose() call.
        // The status line and the six figures around it are the same ones the card shows, so the
        // detail view is readable on its own instead of being a page of leftovers.
        const rose = document.createElement("div");
        rose.className = "detail-rose";
        rose.innerHTML =
            '<div class="status-banner" id="detail-status-' + i + '">UNKNOWN</div>'
          + '<div class="windrose-top-row">'
          + '  <span class="windrose-label label-red"  id="detail-dist-' + i + '">-</span>'
          + '  <span class="windrose-label label-blue" id="detail-wind-' + i + '">-</span>'
          + '</div>'
          + '<canvas id="detail-windrose-' + i + '" width="200" height="200" class="windrose-canvas"></canvas>'
          + '<div class="windrose-bottom-info">'
          + '  <span class="windrose-sub-label text-red"   id="detail-tg-' + i + '">-</span>'
          + '  <span class="windrose-sub-label text-green" id="detail-mag-' + i + '">Mag:-</span>'
          + '</div>';
        body.appendChild(rose);

        // SETUP and MANUAL move; LOCK and DOCK stay on the card. Pull them out first so the row
        // of buttons that lands in the overlay keeps them side by side.
        const actionRow = card.querySelector(".action-buttons");
        if (actionRow) {
            const moved = document.createElement("div");
            moved.className = "action-buttons";
            ["setup-btn-", "mannav-btn-"].forEach(prefix => {
                const b = document.getElementById(prefix + i);
                if (b) moved.appendChild(b);
            });
            if (moved.childElementCount) body.appendChild(moved);
        }

        // The two gauge bars live inside .center-gauge next to the windrose, so they have to be
        // picked out individually rather than by moving their parent. Voltage STAYS on the card -
        // it is the one worth a glance from the water. Current moves into the detail view.
        // Identified through its own bar element rather than by position, so re-ordering the two
        // containers in index.html cannot silently swap which one goes.
        const currBar = document.getElementById("curr-bar-" + i);
        const currBox = currBar && currBar.closest(".gauge-bar-container");
        if (currBox) body.appendChild(currBox);

        BUOY_DETAIL_BLOCKS.forEach(sel => {
            card.querySelectorAll(sel).forEach(el => {
                el.style.marginTop = "10px";
                body.appendChild(el);
            });
        });
    }
}


function openBuoyDetail(i) {
    activeDetailBuoy = i;
    const b = buoys[i];
    document.getElementById("buoy-detail-title").textContent =
        (b && b.id) ? "Buoy " + (i + 1) + " (" + b.id.toUpperCase() + ")" : "Buoy " + (i + 1);
    for (let n = 0; n < 3; n++) {
        const body = document.getElementById("buoy-detail-body-" + n);
        if (body) body.style.display = (n === i) ? "block" : "none";
    }
    document.getElementById("buoy-detail-overlay").style.display = "block";
}

function closeBuoyDetail() {
    activeDetailBuoy = -1;
    document.getElementById("buoy-detail-overlay").style.display = "none";
}

function initBuoyDetailViews() {
    buildBuoyDetailViews();
    for (let i = 0; i < 3; i++) {
        const btn = document.getElementById("buoy-detail-btn-" + i);
        if (btn) btn.addEventListener("click", () => openBuoyDetail(i));
    }
    const close = document.getElementById("buoy-detail-close");
    if (close) close.addEventListener("click", closeBuoyDetail);
}

// =================================================================================================
// Map page control bar
//
// The same six controls the CYD carries under its plot, in the same order:
//
//     - 5 m   ALIGN STARTLINE   ALIGN TRACK   + 5 m        EXECUTE
//
// Nothing in the system stores a "line length": recalcStartLine() on the Top keeps whatever
// distance the two buoys already are apart and only swings the line square to the wind. So the
// length is MEASURED off the fleet by drawFieldMap(), and changing it means physically
// repositioning both end buoys - which is what EXECUTE does, by pushing each a new SETLOCKPOS.
// =================================================================================================
function initStartLinePanel() {
    const view = document.getElementById("full-map-view");
    if (!view) return;

    const readout = document.createElement("div");
    readout.style.cssText = "position:absolute; top:15px; right:15px; z-index:10000;"
        + "display:flex; gap:26px; align-items:baseline; background:#1e293b; border:1px solid #334155;"
        + "border-radius:8px; padding:10px 16px; box-shadow:0 10px 25px -5px rgba(0,0,0,0.5);"
        + "font-family:inherit; color:#e2e8f0;";
    readout.innerHTML =
        '<div><div style="font-size:0.65rem; color:#94a3b8; text-transform:uppercase; letter-spacing:0.5px;">Start line</div>'
      + '<div id="map-line-len" style="font-size:1.7rem; font-weight:bold; color:#64748b;">--</div></div>'
      + '<div style="text-align:right;"><div style="font-size:0.65rem; color:#94a3b8; text-transform:uppercase; letter-spacing:0.5px;">Wind</div>'
      + '<div id="map-wind" style="font-size:1.7rem; font-weight:bold; color:#64748b;">--</div></div>';
    view.appendChild(readout);

    const bar = document.createElement("div");
    bar.style.cssText = "position:absolute; bottom:18px; left:50%; transform:translateX(-50%);"
        + "z-index:10000; display:flex; flex-direction:column; align-items:center; gap:8px;"
        + "font-family:inherit;";

    const btn = "border:none; border-radius:6px; padding:12px 0; font-weight:bold; cursor:pointer;"
              + "font-size:0.95rem; letter-spacing:0.5px;";
    bar.innerHTML =
        '<div id="map-lockwarn" style="color:#facc15; font-weight:bold; font-size:1.15rem;'
      + ' letter-spacing:1px; text-shadow:0 2px 6px rgba(0,0,0,0.8); display:none;">LOCK 2 BUOYS FIRST</div>'
      + '<div style="display:flex; gap:10px;">'
      + '  <button id="map-minus"       style="' + btn + ' width:80px;  background:#8a4b16; color:#fff;">&minus;5 m</button>'
      + '  <button id="map-align-start" style="' + btn + ' width:190px; background:#7dd3fc; color:#0f172a;">ALIGN STARTLINE</button>'
      + '  <button id="map-align-track" style="' + btn + ' width:190px; background:#2563eb; color:#fff;">ALIGN TRACK</button>'
      + '  <button id="map-plus"        style="' + btn + ' width:80px;  background:#8a4b16; color:#fff;">+5 m</button>'
      + '</div>'
      + '<button id="map-execute" style="' + btn + ' width:100%; background:#16a34a; color:#fff;">EXECUTE</button>'
      + '<div id="map-msg" style="color:#94a3b8; font-size:0.8rem; min-height:1.2em; text-align:center;"></div>';
    view.appendChild(bar);

    document.getElementById("map-minus").addEventListener("click", () => mapNudge(-MAP_LINE_STEP_M));
    document.getElementById("map-plus").addEventListener("click", () => mapNudge(MAP_LINE_STEP_M));
    document.getElementById("map-align-start").addEventListener("click", () => mapAlign(MsgType.COMPUTESTART));
    document.getElementById("map-align-track").addEventListener("click", () => mapAlign(MsgType.COMPUTETRACK));
    document.getElementById("map-execute").addEventListener("click", mapExecute);
}
