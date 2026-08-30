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
    REBOOT: 85
};

// Global variables for full-screen Manual Fourier Calibration inside browser
let mancalWebActiveIndex = null;
let mancalWebActiveLeg = 0;
const mancalWebOffsets = Array(8).fill(0);
// The eight offsets are the ones the buoy is already running, not a blank slate. Until its answer
// to the STORE_INTERPOLATION_TABLE GET arrives they are all zero, and zero is a real calibration
// value - so "not answered yet" has to be distinguishable from "answered, and it is 0".
let mancalWebOffsetsLoaded = false;
// True between opening the screen and switching the buoy's harmonic correction off. The switch
// waits for the table: the Sub reports the table that is IN EFFECT, so asking after the switch
// returns the identity table and every correction reads 0.
let mancalWebHarmonicPending = false;
let mancalWebQueryTries = 0;
// Which of the eight directions have been steered to at least once this session. SAVE commits all
// eight entries in one frame, not just the one on screen, so a session that only looked at two of
// them would still overwrite the other six on the buoy with whatever is in the buffer. The button
// stays locked until every direction has been visited.
const mancalWebVisited = Array(8).fill(false);
// This screen switches the harmonic correction off so the dialing works against the raw compass, and
// it must be ON again by the time the screen is left - by EITHER exit. A buoy left with it off
// answers every later table request with the identity table, so the corrections still in NVS become
// invisible and the screen reads as "this buoy has no calibration".
let mancalWebQueryTimer = null;
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

/**
 * Paints the correction readouts of the full-screen manual calibration overlay: the big CORR value
 * for the direction being worked on, and the whole eight-point table underneath the compass. Only
 * the active direction used to be shown, so the seven corrections the buoy was already running were
 * invisible and the screen read as an uncalibrated buoy.
 */
function renderMancalWebOffsets() {
    const offValEl = document.getElementById("mancal-web-offset-val");
    if (offValEl) {
        if (!mancalWebOffsetsLoaded) {
            // Not "+0°" - zero is a calibration value in its own right, and showing it before the
            // buoy has answered claims a correction we have not been told about.
            offValEl.textContent = "?";
        } else if (mancalWebActiveLeg === -1) {
            offValEl.textContent = "-";
        } else {
            const off = mancalWebOffsets[mancalWebActiveLeg];
            offValEl.textContent = (off >= 0 ? "+" : "") + off + "°";
        }
    }

    for (let i = 0; i < 8; i++) {
        const cell = document.getElementById("mancal-web-tbl-" + i);
        if (!cell) continue;
        const off = mancalWebOffsets[i];
        cell.textContent = mancalWebOffsetsLoaded ? (off >= 0 ? "+" : "") + off + "°" : "?";
        cell.style.color = (i === mancalWebActiveLeg) ? "#22c55e" : "#e2e8f0";
    }

    const statusEl = document.getElementById("mancal-web-table-status");
    if (statusEl) {
        statusEl.textContent = mancalWebOffsetsLoaded
            ? "Corrections now stored on the buoy - adjust from here"
            : "Reading the stored table from the buoy...";
    }

    updateMancalWebSaveState();
    updateMancalWebDotLocks();
}

/**
 * North has to be done first. Everything downstream is measured against it: SET AS NORTH folds the
 * North error into compassOffset, and because the Sub applies that offset BEFORE the table lookup
 * (RoboSub/src/compass.cpp:936 then :944) it shifts every other sector by the same amount. Doing
 * North first means there is nothing else dialled in yet for it to invalidate. Once North has been
 * visited the operator is free - take SET AS NORTH, or just leave a Fourier offset on North.
 */
function mancalWebLegLocked(leg) {
    return leg !== 0 && !mancalWebVisited[0];
}

/**
 * Greys out the seven non-North dots until North has been visited, and rings North as the way in.
 */
function updateMancalWebDotLocks() {
    document.querySelectorAll(".mancal-web-dot").forEach(btn => {
        const leg = parseInt(btn.getAttribute("data-leg"));
        const locked = mancalWebLegLocked(leg);
        btn.disabled = locked;
        btn.style.opacity = locked ? "0.3" : "1";
        btn.style.cursor = locked ? "not-allowed" : "pointer";
        btn.title = locked ? "Set North first - it is the base every other direction is measured against" : "";
        // North, still waiting to be pressed - ring it so it reads as the way in.
        if (leg === 0 && !mancalWebVisited[0] && leg !== mancalWebActiveLeg) {
            btn.style.border = "2px solid #eab308";
        }
    });

    const statusEl = document.getElementById("mancal-web-table-status");
    if (statusEl && mancalWebOffsetsLoaded && !mancalWebVisited[0]) {
        statusEl.textContent = "Start with North (N) - it is the base every other direction is measured against";
    }
}

/**
 * Locks SAVE & EXIT until every direction has been visited. SAVE writes all eight table entries at
 * once, so committing after visiting only some of them overwrites the untouched sectors on the buoy
 * with whatever the buffer happened to hold. The button counts down rather than just refusing the
 * click, so it is obvious how many are left.
 */
function updateMancalWebSaveState() {
    const done = mancalWebVisited.filter(Boolean).length;
    const ready = done === 8;

    const btn = document.getElementById("mancal-web-exit");
    if (btn) {
        btn.textContent = ready ? "SAVE & EXIT" : `SAVE (${done}/8)`;
        btn.disabled = !ready;
        btn.style.opacity = ready ? "1" : "0.45";
        btn.style.cursor = ready ? "pointer" : "not-allowed";
        btn.title = ready ? "" : "Steer to all 8 directions before saving - SAVE writes the whole table at once";
    }

    // Dim the directions that are still carrying the value that came off the buoy.
    for (let i = 0; i < 8; i++) {
        const cell = document.getElementById("mancal-web-tbl-" + i);
        if (cell) cell.style.opacity = mancalWebVisited[i] ? "1" : "0.4";
    }
}

/**
 * Switches the buoy's harmonic (Fourier) correction on or off. Off is what makes the +/- dialing and
 * the table written on SAVE both work against the raw compass, but it is deliberately not sent on
 * entry: the Sub answers a table request with the table that is IN EFFECT, so asking after the
 * switch-off has landed returns the identity table and every stored correction comes back as 0.
 */
function mancalWebSetHarmonic(b, on) {
    b.data["harmonic_enabled"] = on ? "1" : "0";
    const currStatus = b.data.Status || "7";
    const trimEn = b.data["compass_trim_enabled"] === "1" ? "1" : "0";
    const values = [
        b.data["Kpr"] || "1.00", b.data["Kir"] || "0.01", b.data["Kdr"] || "0.05",
        b.data["Kps"] || "1.50", b.data["Kis"] || "0.05", b.data["Kds"] || "0.10",
        b.data["maxSpeed"] || "100", b.data["minSpeed"] || "10", b.data["pivotSpeed"] || "0.20",
        b.data["compassOffset"] || "0", b.data["holdRad"] || "2.0",
        b.data["revBB"] === "1" ? "1" : "0", b.data["revSB"] === "1" ? "1" : "0",
        b.data["swap_BB_SB"] === "1" ? "1" : "0", trimEn,
        b.data["dockAppDist"] || "20", b.data["dockAppDir"] || "180",
        b.data["dockToWP"] === "1" ? "1" : "0",
        on ? "2" : "1" // wire encoding: 2 = on, 1 = off
    ];
    sendCommand(b.id, `${b.id},99,${MsgType.SET},${MsgType.SETUPDATA},${currStatus},${values.join(",")}`);
}

/**
 * Asks the buoy for its 8-point table until it answers, then finishes the entry sequence. The GET
 * is fire-and-forget, and a lost one would leave the screen showing eight zeros - which reads as
 * "this buoy has no corrections" rather than "the answer never came". Gives up after
 * MANCAL_WEB_QUERY_MAX_TRIES so an unreachable buoy still leaves a usable screen.
 */
function mancalWebPollTable() {
    if (mancalWebQueryTimer) {
        clearTimeout(mancalWebQueryTimer);
        mancalWebQueryTimer = null;
    }
    if (!mancalWebHarmonicPending || mancalWebActiveIndex === null) return;

    const b = buoys[mancalWebActiveIndex];
    if (!b || !b.id) return;

    if (mancalWebOffsetsLoaded || mancalWebQueryTries >= MANCAL_WEB_QUERY_MAX_TRIES) {
        if (!mancalWebOffsetsLoaded) {
            logMessage(`Buoy ${b.id.toUpperCase()}: no answer to the Fourier table request - starting from zero.`, "UDP IN");
            // Zero is the baseline we are going to dial from, so stop calling it unknown.
            mancalWebOffsetsLoaded = true;
            renderMancalWebOffsets();
        }
        mancalWebHarmonicPending = false;
        mancalWebSetHarmonic(b, false);
        return;
    }

    mancalWebQueryTries++;
    sendCommand(b.id, `${b.id},99,${MsgType.GET},88,,,,,,,`);
    mancalWebQueryTimer = setTimeout(mancalWebPollTable, MANCAL_WEB_QUERY_INTERVAL_MS);
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
        
        // Save the IP address of the buoy if it was received via UDP
        if (source === "UDP" && senderIp) {
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
                "Current": fields[21] || "0"
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
                renderMancalWebOffsets();

                // The answer we were waiting for: finish opening the screen now instead of sitting
                // out the rest of the retry interval.
                if (mancalWebHarmonicPending) mancalWebPollTable();

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
    
    // --- Global Isolated Full-Screen Manual Calibration Event Listeners with Safe Null Checks ---
    const webLegBtns = document.querySelectorAll(".mancal-web-dot");
    if (webLegBtns && webLegBtns.length > 0) {
        webLegBtns.forEach(btn => {
            btn.addEventListener("click", () => {
                // Steering before the buoy has reported its table, and before its harmonic
                // correction is off, would be dialed against a compass that is about to shift.
                if (mancalWebHarmonicPending) return;

                const leg = parseInt(btn.getAttribute("data-leg"));
                // The button is disabled in this state, but guard anyway.
                if (mancalWebLegLocked(leg)) return;
                mancalWebActiveLeg = leg;
                mancalWebVisited[leg] = true;

                // Highlight active button and unhighlight others
                webLegBtns.forEach(bL => {
                    const bLeg = parseInt(bL.getAttribute("data-leg"));
                    if (bLeg === leg) {
                        bL.style.backgroundColor = "#3b82f6"; // Blue active
                        bL.style.color = "white";
                        bL.style.border = "2px solid white";
                    } else {
                        bL.style.backgroundColor = "#334155"; // Dark grey inactive
                        bL.style.color = "#94a3b8";
                        bL.style.border = "1px solid #475569";
                    }
                });
                
                // Update offset display - the correction this direction is already carrying
                renderMancalWebOffsets();

                // Show/Hide "Set as North" button based on whether North (leg 0) is selected
                const northBtn = document.getElementById("mancal-web-setNorth");
                if (northBtn) northBtn.style.display = (leg === 0) ? "block" : "none";
                
                // Send direct steer command to buoy (REMOTE 25). The correction this direction
                // already carries goes in straight away, the same way the +/- buttons apply it -
                // steering to the bare angle would make the first 1° tap swing the buoy by the
                // whole stored offset.
                if (mancalWebActiveIndex !== null) {
                    const b = buoys[mancalWebActiveIndex];
                    if (b.id) {
                        let steer_dir = leg * 45 - mancalWebOffsets[leg];
                        while (steer_dir < 0) steer_dir += 360;
                        while (steer_dir >= 360) steer_dir -= 360;
                        const payload = `${b.id},98,6,25,25,${steer_dir.toFixed(1)},0.0,,,,`;
                        sendCommand(b.id, payload);
                    }
                }
            });
        });
    }
    
    const adjustWebMancalOffset = (step) => {
        if (mancalWebActiveIndex === null || mancalWebActiveLeg === -1) return; // Guard against unselected directions!
        if (mancalWebHarmonicPending) return; // entry sequence still in flight
        const b = buoys[mancalWebActiveIndex];
        if (!b.id) return;

        mancalWebOffsets[mancalWebActiveLeg] += step;
        if (mancalWebOffsets[mancalWebActiveLeg] < -180) mancalWebOffsets[mancalWebActiveLeg] = -180;
        if (mancalWebOffsets[mancalWebActiveLeg] > 180) mancalWebOffsets[mancalWebActiveLeg] = 180;

        const off = mancalWebOffsets[mancalWebActiveLeg];
        renderMancalWebOffsets();

        // Send direct steering target shift to buoy in real-time
        const target_angle = mancalWebActiveLeg * 45;
        let steer_dir = target_angle - off;
        while (steer_dir < 0) steer_dir += 360;
        while (steer_dir >= 360) steer_dir -= 360;
        
        const payload = `${b.id},98,6,25,25,${parseFloat(steer_dir).toFixed(1)},0.0,,,,`;
        sendCommand(b.id, payload);
        
        // Send temporary offset to fourier table (Command 78)
        const currStatus = b.data.Status || "7";
        const calPayload = `${b.id},99,${MsgType.SET},78,${currStatus},${mancalWebActiveLeg},${off},,,,`;
        sendCommand(b.id, calPayload);
    };
    
    const btnSub10 = document.getElementById("mancal-web-sub10");
    if (btnSub10) {
        btnSub10.addEventListener("click", (e) => {
            flashButtonFeedback(e.currentTarget, "#ef4444", "white", 150); // Flash Red!
            adjustWebMancalOffset(-10);
        });
    }
    const btnSub1 = document.getElementById("mancal-web-sub1");
    if (btnSub1) {
        btnSub1.addEventListener("click", (e) => {
            flashButtonFeedback(e.currentTarget, "#ef4444", "white", 150); // Flash Red!
            adjustWebMancalOffset(-1);
        });
    }
    const btnAdd1 = document.getElementById("mancal-web-add1");
    if (btnAdd1) {
        btnAdd1.addEventListener("click", (e) => {
            flashButtonFeedback(e.currentTarget, "#22c55e", "black", 150); // Flash Green!
            adjustWebMancalOffset(1);
        });
    }
    const btnAdd10 = document.getElementById("mancal-web-add10");
    if (btnAdd10) {
        btnAdd10.addEventListener("click", (e) => {
            flashButtonFeedback(e.currentTarget, "#22c55e", "black", 150); // Flash Green!
            adjustWebMancalOffset(10);
        });
    }
    
    // "Set as North" button handler
    const btnSetNorth = document.getElementById("mancal-web-setNorth");
    if (btnSetNorth) {
        btnSetNorth.addEventListener("click", (e) => {
            flashButtonFeedback(e.currentTarget, "#f59e0b", "black", 150); // Flash Orange!
            if (mancalWebActiveIndex === null) return;
            const b = buoys[mancalWebActiveIndex];
            if (!b.id) return;
            
            const currentOffset = parseFloat(b.data["compassOffset"] || "0.0");
            const dialedOffset = mancalWebOffsets[0]; // North leg offset
            let newOffset = currentOffset + dialedOffset;
            while (newOffset < -180) newOffset += 360;
            while (newOffset > 180) newOffset -= 360;
            
            b.data["compassOffset"] = newOffset;
            mancalWebOffsets[0] = 0; // reset North fourier correction to 0
            renderMancalWebOffsets();
            
            // Save setup payload
            const currStatus = b.data.Status || "7";
            const trimEn = document.getElementById("setup-compassTrimEnabled")?.checked ? "1" : "0";
            const values = [
                b.data["Kpr"] || "1.00", b.data["Kir"] || "0.01", b.data["Kdr"] || "0.05",
                b.data["Kps"] || "1.50", b.data["Kis"] || "0.05", b.data["Kds"] || "0.10",
                b.data["maxSpeed"] || "100", b.data["minSpeed"] || "10", b.data["pivotSpeed"] || "0.20",
                newOffset, b.data["holdRad"] || "2.0",
                b.data["revBB"] === "1" ? "1" : "0", b.data["revSB"] === "1" ? "1" : "0",
                b.data["swap_BB_SB"] === "1" ? "1" : "0", trimEn,
                b.data["dockAppDist"] || "20", b.data["dockAppDir"] || "180",
                b.data["dockToWP"] === "1" ? "1" : "0",
                b.data["harmonic_enabled"] === "1" ? "2" : "1"
            ];
            const setupPayload = `${b.id},99,${MsgType.SET},${MsgType.SETUPDATA},${currStatus},${values.join(",")}`;
            sendCommand(b.id, setupPayload);
            
            // Refresh steering target to North (0)
            const payload = `${b.id},98,6,25,25,0.0,0.0,,,,`;
            sendCommand(b.id, payload);
            
            logMessage(`Buoy ${b.id.toUpperCase()}: Global North Offset saved and applied successfully!`, "UDP OUT");
        });
    }
    
    // "CANCEL" button handler
    const btnCancel = document.getElementById("mancal-web-cancel");
    if (btnCancel) {
        btnCancel.addEventListener("click", (e) => {
            flashButtonFeedback(e.currentTarget, "#ef4444", "white", 150); // Flash Red!
            if (mancalWebActiveIndex === null) return;
            const b = buoys[mancalWebActiveIndex];
            if (!b.id) return;

            // Leave with the harmonic correction ON. Entering this screen switches it off so the
            // dialing works on the raw compass; walking away and leaving it off makes every later
            // visit read the identity table, so the corrections still in NVS become invisible.
            mancalWebSetHarmonic(b, true);

            // Send IDLE command to shut down thrusters/motors
            sendStatusCmd(b.id, MsgType.IDLING);

            // Close overlay with smooth fade-out animation matching other modals
            const overlay = document.getElementById("mancal-fullscreen-overlay");
            if (overlay) {
                overlay.classList.remove("active");
                setTimeout(() => {
                    overlay.style.display = "none";
                }, 200); // 200ms matches style.css modal-overlay transition time
            }
            mancalWebActiveIndex = null;
            // Cancel any table query still in flight, or its follow-up would switch the harmonic
            // correction back off on a screen that has already been closed.
            mancalWebHarmonicPending = false;
            if (mancalWebQueryTimer) { clearTimeout(mancalWebQueryTimer); mancalWebQueryTimer = null; }
        });
    }
    
    // "SAVE & EXIT" button handler
    const btnExit = document.getElementById("mancal-web-exit");
    if (btnExit) {
        btnExit.addEventListener("click", (e) => {
            flashButtonFeedback(e.currentTarget, "#3b82f6", "white", 150); // Flash Blue!
            if (mancalWebActiveIndex === null) return;
            // The button is disabled in this state, but guard anyway - a stray programmatic click
            // must not commit a table with sectors that were never visited.
            if (mancalWebVisited.filter(Boolean).length !== 8) return;
            const b = buoys[mancalWebActiveIndex];
            if (!b.id) return;
            
            // 1. Construct the entire 8-point fourier table from the current dialed offsets
            const tableVals = [];
            for (let j = 0; j < 8; j++) {
                let expected = j * 45;
                let off = mancalWebOffsets[j];
                let val = expected - off;
                while (val < 0) val += 360;
                while (val >= 360) val -= 360;
                tableVals.push(val.toFixed(2));
            }
            
            // 2. Send the entire 8-point table as Command 88 SET to commit permanently to NVS!
            const currStatus = b.data.Status || "7";
            const calPayload = `${b.id},99,${MsgType.SET},88,${currStatus},${tableVals.join(",")}`;
            sendCommand(b.id, calPayload);
            
            // 3. Re-enable Fourier table interpolation (harmonic_enabled = true)
            b.data["harmonic_enabled"] = "1";
            const trimEn = document.getElementById("setup-compassTrimEnabled")?.checked ? "1" : "0";
            const values = [
                b.data["Kpr"] || "1.00", b.data["Kir"] || "0.01", b.data["Kdr"] || "0.05",
                b.data["Kps"] || "1.50", b.data["Kis"] || "0.05", b.data["Kds"] || "0.10",
                b.data["maxSpeed"] || "100", b.data["minSpeed"] || "10", b.data["pivotSpeed"] || "0.20",
                b.data["compassOffset"] || "0", b.data["holdRad"] || "2.0",
                b.data["revBB"] === "1" ? "1" : "0", b.data["revSB"] === "1" ? "1" : "0",
                b.data["swap_BB_SB"] === "1" ? "1" : "0", trimEn,
                b.data["dockAppDist"] || "20", b.data["dockAppDir"] || "180",
                b.data["dockToWP"] === "1" ? "1" : "0",
                "2" // harmonic_enabled = true (2)
            ];
            const setupPayload = `${b.id},99,${MsgType.SET},${MsgType.SETUPDATA},${currStatus},${values.join(",")}`;
            sendCommand(b.id, setupPayload);
            
            // Send IDLE command to shut down thrusters/motors
            sendStatusCmd(b.id, MsgType.IDLING);
            
            // Close overlay with smooth fade-out animation matching other modals
            const overlay = document.getElementById("mancal-fullscreen-overlay");
            if (overlay) {
                overlay.classList.remove("active");
                setTimeout(() => {
                    overlay.style.display = "none";
                }, 200);
            }
            mancalWebActiveIndex = null;
            // Cancel any table query still in flight, or its follow-up would switch the harmonic
            // correction back off on a screen that has already been closed.
            mancalWebHarmonicPending = false;
            if (mancalWebQueryTimer) { clearTimeout(mancalWebQueryTimer); mancalWebQueryTimer = null; }
            
            logMessage(`Buoy ${b.id.toUpperCase()}: Manual Calibration saved and committed permanently to Sub NVS! Buoy set to IDLE.`, "UDP OUT");
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

    // The two modes differ only in how the difference between opposite legs is read. In still
    // water that difference is residual hard iron and must be corrected; with a current running it
    // is the current and must be thrown away. Picking the wrong one is not cosmetic - still-water
    // mode on a tidal day writes the current straight into the compass table.
    const startGpsFourier = (stillWater) => {
        const detail = stillWater
            ? "STILL WATER mode: corrects each heading on its own measurement, including the "
              + "one-cycle hard-iron error.\n\nOnly choose this if there is genuinely no current "
              + "and little windage - otherwise the current gets written into the compass table."
            : "CURRENT RUNNING mode: opposite legs are averaged so a steady current cancels.\n\n"
              + "Safe in moving water, but it cannot correct one-cycle (hard-iron) error.";
        if (!confirm("Start the GPS Fourier compass calibration?\n\n" + detail
                   + "\n\nThe buoy will sail 8 legs of at least 100 m (about 30 minutes) and then "
                   + "return to where it is now. Make sure it has open water all around and a GPS fix."
                   + "\n\nPress IDLE at any time to stop.")) return;
        sendSetupAction(89, stillWater ? "1" : "0"); // GPS_FOURIER_CALIBRATE
    };
    document.getElementById("setup-gpsStill-btn").addEventListener("click", () => startGpsFourier(true));
    document.getElementById("setup-manCal-btn").addEventListener("click", () => {
        // 1. Read active setup index FIRST before closing the modal clears it!
        const index = activeSetupBuoyIndex !== null ? activeSetupBuoyIndex : 0;
        
        // 2. Close Setup Modal
        closeSetupModal();
        
        // 3. Initialize active full-screen mancal indexes with the correct index
        const b = buoys[index];
        if (!b.id) return;
        
        mancalWebActiveIndex = index;
        mancalWebActiveLeg = -1; // -1 means NO active steering yet!
        mancalWebOffsets.fill(0);
        mancalWebVisited.fill(false);
        mancalWebOffsetsLoaded = false;

        // Update header texts
        document.getElementById("mancal-web-buoy-title").textContent = `BUOY ${index + 1} (${b.id.toUpperCase()})`;
        document.getElementById("mancal-web-mag-val").textContent = "0°";
        renderMancalWebOffsets();
        
        // Unhighlight ALL sector buttons initially (all inactive)
        const webLegBtns = document.querySelectorAll(".mancal-web-dot");
        webLegBtns.forEach(btn => {
            btn.style.backgroundColor = "#334155";
            btn.style.color = "#94a3b8";
            btn.style.border = "1px solid #475569";
        });
        // After the reset above, not before it - that loop rewrites every border and would
        // otherwise wipe the ring that marks North as the one direction available to press.
        updateMancalWebDotLocks();

        // Hide "Set as North" button initially (leg 0 is NOT active yet)
        document.getElementById("mancal-web-setNorth").style.display = "none";
        
        // Reset speedbars
        document.getElementById("mancal-bar-bb").style.height = "0%";
        document.getElementById("mancal-bar-sb").style.height = "0%";
        document.getElementById("mancal-val-bb").textContent = "0%";
        document.getElementById("mancal-val-sb").textContent = "0%";
        
        // 3. Stop the buoy. This screen commands no heading until a direction is picked, so
        //    whatever it was doing has to end here rather than at the first tap.
        sendStatusCmd(b.id, MsgType.IDLING);

        // 4. Ask for the 8-point table (Command 88) WHILE the harmonic correction is on, and only
        //    switch it off once the answer is in - mancalWebPollTable() does that last step.
        //    Sending the switch first, as this used to, made the Sub answer with the identity
        //    table, which is why every stored correction showed up as 0.
        mancalWebHarmonicPending = true;
        mancalWebQueryTries = 0;

        // Switch the correction ON before reading. Unconditionally: a buoy whose correction is off
        // answers with the identity table, and b.data["harmonic_enabled"] is only a cached copy
        // that can be stale (anything changing the flag by another route does not reach this page).
        // Its table is still in NVS either way - only the reporting of it depends on the flag.
        mancalWebSetHarmonic(b, true);
        setTimeout(mancalWebPollTable, 400);

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
    
    activeSetupBuoyIndex = buoyIndex;
    const b = buoys[buoyIndex];
    if (!b.id) return;
    
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

// Draw the local radar map of the buoys and the computed regatta track on the canvas
function drawFieldMap() {
    const canvas = document.getElementById("fieldCanvas");
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    let validBuoys = [];
    buoys.forEach((b, i) => {
        const lat = parseFloat(b.data["Latitude (Lat)"]);
        const lng = parseFloat(b.data["Longitude (Lon)"]);
        if (b.id && !isNaN(lat) && !isNaN(lng) && lat !== 0 && lng !== 0) {
            const statusVal = parseInt(b.data.Status || "0");
            const holdsTarget = (statusVal === MsgType.LOCKING || statusVal === MsgType.LOCKED || statusVal === MsgType.DOCKING || statusVal === MsgType.DOCKED);
            const dist = parseFloat(b.data["Target Dist"]);
            const dir = parseFloat(b.data["Target Dir"]);
            
            let tgLat = null, tgLng = null;
            if (holdsTarget && !isNaN(dist) && !isNaN(dir) && dist > 0) {
                const tg = projectCoords(lat, lng, dir, dist);
                tgLat = tg.lat;
                tgLng = tg.lng;
            }

            validBuoys.push({
                id: b.id,
                index: i,
                lat: lat,
                lng: lng,
                tgLat: tgLat,
                tgLng: tgLng,
                wDir: parseFloat(b.data["Wind Dir"]) || 0,
                heading: parseFloat(b.data["Magnetic Dir"]) || 0,
                holdsTarget: holdsTarget,
                status: statusVal
            });
        }
    });

    if (validBuoys.length === 0) {
        ctx.fillStyle = "#94a3b8";
        ctx.font = "16px Arial";
        ctx.textAlign = "center";
        ctx.fillText("No Live GPS Data Available", canvas.width/2, canvas.height/2);
        return;
    }

    const latRef = validBuoys[0].lat;
    const lngRef = validBuoys[0].lng;
    const deg2rad = Math.PI / 180;

    let refWDir = 0;
    let refBuoy = validBuoys.find(b => b.wDir > 0);
    if (refBuoy) refWDir = refBuoy.wDir;

    let points = [];
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    let sumX = 0, sumY = 0;

    validBuoys.forEach(b => {
        let cx = 0, cy = 0;
        if (b.lat !== latRef || b.lng !== lngRef) {
            let dy = (b.lat - latRef) * 111132.954;
            let dx = (b.lng - lngRef) * 111132.954 * Math.cos(latRef * deg2rad);
            let dist = Math.sqrt(dx*dx + dy*dy);
            let bearing = Math.atan2(dx, dy);
            let relBearing = bearing - (refWDir * deg2rad);
            cx = dist * Math.sin(relBearing);
            cy = -dist * Math.cos(relBearing);
        }

        if (cx < minX) minX = cx;
        if (cx > maxX) maxX = cx;
        if (cy < minY) minY = cy;
        if (cy > maxY) maxY = cy;
        sumX += cx;
        sumY += cy;

        let tgCx = null, tgCy = null;
        if (b.tgLat !== null && b.tgLng !== null) {
            let dy = (b.tgLat - latRef) * 111132.954;
            let dx = (b.tgLng - lngRef) * 111132.954 * Math.cos(latRef * deg2rad);
            let dist = Math.sqrt(dx*dx + dy*dy);
            let bearing = Math.atan2(dx, dy);
            let relBearing = bearing - (refWDir * deg2rad);
            tgCx = dist * Math.sin(relBearing);
            tgCy = -dist * Math.cos(relBearing);

            if (tgCx < minX) minX = tgCx;
            if (tgCx > maxX) maxX = tgCx;
            if (tgCy < minY) minY = tgCy;
            if (tgCy > maxY) maxY = tgCy;
        }

        let computedTrackPos = 0;
        if (validBuoys.length === 2) {
            // If only 2 buoys are deployed, they form the startline: Buoy 1 is STBD (3), Buoy 2 is PORT (2)
            if (b.index === 0) computedTrackPos = 3;
            else if (b.index === 1) computedTrackPos = 2;
        } else {
            // If 3 buoys are deployed, they form the full track: B1 is HEAD (1), B2 is PORT (2), B3 is STBD (3)
            if (b.index === 0) computedTrackPos = 1;
            else if (b.index === 1) computedTrackPos = 2;
            else if (b.index === 2) computedTrackPos = 3;
        }

        points.push({
            id: b.id,
            cx: cx,
            cy: cy,
            tgCx: tgCx,
            tgCy: tgCy,
            heading: b.heading,
            holdsTarget: b.holdsTarget,
            status: b.status,
            trackPos: computedTrackPos
        });
    });

    let centerX = (minX + maxX) / 2;
    let centerY = (minY + maxY) / 2;

    let maxDim = Math.max(maxX - minX, maxY - minY);
    if (maxDim < 10) maxDim = 30; // default minimum span
    let scale = Math.min(canvas.width, canvas.height) * 0.7 / maxDim;

    // Draw grid
    ctx.strokeStyle = "rgba(71, 85, 105, 0.15)";
    ctx.lineWidth = 1;
    let step = 10;
    if (maxDim > 100) step = 50;
    if (maxDim > 500) step = 100;
    
    ctx.beginPath();
    for (let x = -1000; x <= 1000; x += step) {
        let sx = (x - centerX) * scale + canvas.width/2;
        ctx.moveTo(sx, 0);
        ctx.lineTo(sx, canvas.height);
    }
    for (let y = -1000; y <= 1000; y += step) {
        let sy = (y - centerY) * scale + canvas.height/2;
        ctx.moveTo(0, sy);
        ctx.lineTo(canvas.width, sy);
    }
    ctx.stroke();

    // Wind Indicator
    ctx.strokeStyle = "#38bdf8";
    ctx.fillStyle = "#38bdf8";
    ctx.font = "bold 12px Arial";
    ctx.textAlign = "left";
    ctx.fillText("Wind (" + refWDir.toFixed(0) + "°)", 15, 25);
    drawArrow(ctx, 35, 65, 35, 35, "#38bdf8", 2.5);

    // Distance lines
    ctx.strokeStyle = "rgba(255,255,255,0.12)";
    ctx.lineWidth = 1;
    ctx.setLineDash([4, 4]);
    for(let i=0; i<points.length; i++) {
        for(let j=i+1; j<points.length; j++) {
            let p1 = points[i];
            let p2 = points[j];
            let sx1 = (p1.cx - centerX) * scale + canvas.width/2;
            let sy1 = (p1.cy - centerY) * scale + canvas.height/2;
            let sx2 = (p2.cx - centerX) * scale + canvas.width/2;
            let sy2 = (p2.cy - centerY) * scale + canvas.height/2;

            ctx.beginPath();
            ctx.moveTo(sx1, sy1);
            ctx.lineTo(sx2, sy2);
            ctx.stroke();

            let dist = Math.sqrt(Math.pow(p1.cx - p2.cx, 2) + Math.pow(p1.cy - p2.cy, 2));
            let mx = (sx1 + sx2) / 2;
            let my = (sy1 + sy2) / 2;
            ctx.fillStyle = "#94a3b8";
            ctx.font = "10px Arial";
            ctx.textAlign = "center";
            ctx.fillText(dist.toFixed(0) + "m", mx, my - 4);
        }
    }
    ctx.setLineDash([]);

    // Draw Regatta Track Lines
    let headBuoy = points.find(p => p.trackPos === 1);
    let portBuoy = points.find(p => p.trackPos === 2);
    let starboardBuoy = points.find(p => p.trackPos === 3);

    function getCoords(p) {
        if (!p) return null;
        if (p.tgCx !== null && p.tgCy !== null) {
            return { x: p.tgCx, y: p.tgCy };
        }
        return { x: p.cx, y: p.cy };
    }

    let headCoords = getCoords(headBuoy);
    let portCoords = getCoords(portBuoy);
    let starboardCoords = getCoords(starboardBuoy);

    if (portCoords && starboardCoords) {
        let psx = (portCoords.x - centerX) * scale + canvas.width/2;
        let psy = (portCoords.y - centerY) * scale + canvas.height/2;
        let ssx = (starboardCoords.x - centerX) * scale + canvas.width/2;
        let ssy = (starboardCoords.y - centerY) * scale + canvas.height/2;

        ctx.strokeStyle = "rgba(250, 204, 21, 0.85)"; // Amber Yellow
        ctx.lineWidth = 3.5;
        ctx.beginPath();
        ctx.moveTo(psx, psy);
        ctx.lineTo(ssx, ssy);
        ctx.stroke();

        let midSx = (psx + ssx) / 2;
        let midSy = (psy + ssy) / 2;
        ctx.fillStyle = "#facc15";
        ctx.font = "italic bold 10px Arial";
        ctx.textAlign = "center";
        ctx.fillText("Start Line", midSx, midSy - 8);
    }

    if (headCoords && portCoords && starboardCoords) {
        let hsx = (headCoords.x - centerX) * scale + canvas.width/2;
        let hsy = (headCoords.y - centerY) * scale + canvas.height/2;
        let psx = (portCoords.x - centerX) * scale + canvas.width/2;
        let psy = (portCoords.y - centerY) * scale + canvas.height/2;
        let ssx = (starboardCoords.x - centerX) * scale + canvas.width/2;
        let ssy = (starboardCoords.y - centerY) * scale + canvas.height/2;

        ctx.strokeStyle = "rgba(56, 189, 248, 0.8)"; // Blue
        ctx.lineWidth = 2.5;
        ctx.setLineDash([5, 4]);
        ctx.beginPath();
        ctx.moveTo(psx, psy);
        ctx.lineTo(hsx, hsy);
        ctx.stroke();

        ctx.strokeStyle = "rgba(74, 222, 128, 0.8)"; // Green
        ctx.beginPath();
        ctx.moveTo(ssx, ssy);
        ctx.lineTo(hsx, hsy);
        ctx.stroke();
        ctx.setLineDash([]);
    }

    // Target Waypoint Lines and Waypoints
    points.forEach(p => {
        if (p.tgCx !== null && p.tgCy !== null) {
            let sx = (p.cx - centerX) * scale + canvas.width/2;
            let sy = (p.cy - centerY) * scale + canvas.height/2;
            let tgSx = (p.tgCx - centerX) * scale + canvas.width/2;
            let tgSy = (p.tgCy - centerY) * scale + canvas.height/2;

            ctx.strokeStyle = "rgba(250, 204, 21, 0.5)"; // Amber line from buoy to its target
            ctx.lineWidth = 1.5;
            ctx.setLineDash([3, 3]);
            ctx.beginPath();
            ctx.moveTo(sx, sy);
            ctx.lineTo(tgSx, tgSy);
            ctx.stroke();
            ctx.setLineDash([]);

            ctx.fillStyle = "rgba(250, 204, 21, 0.85)";
            ctx.strokeStyle = "#0f172a";
            ctx.lineWidth = 1;
            ctx.beginPath();
            ctx.arc(tgSx, tgSy, 4, 0, Math.PI*2);
            ctx.fill();
            ctx.stroke();
        }
    });

    // Buoys
    points.forEach(p => {
        let sx = (p.cx - centerX) * scale + canvas.width/2;
        let sy = (p.cy - centerY) * scale + canvas.height/2;

        ctx.fillStyle = "#ef4444"; // Red buoy
        ctx.strokeStyle = "#0f172a";
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(sx, sy, 7, 0, Math.PI*2);
        ctx.fill();
        ctx.stroke();

        let roleLabel = "";
        if (p.trackPos === 1) roleLabel = " (HEAD)";
        else if (p.trackPos === 2) roleLabel = " (PORT)";
        else if (p.trackPos === 3) roleLabel = " (STBD)";

        ctx.fillStyle = "#fff";
        ctx.font = "bold 11px Arial";
        ctx.textAlign = "left";
        ctx.fillText("B" + p.id + roleLabel, sx + 12, sy + 4);

        // Heading Arrow (Green)
        if (p.heading !== null && !isNaN(p.heading)) {
            let relHeading = p.heading - refWDir;
            let hRad = relHeading * deg2rad;
            let hx = sx + 15 * Math.sin(hRad);
            let hy = sy - 15 * Math.cos(hRad);

            ctx.strokeStyle = "#4ade80";
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(sx, sy);
            ctx.lineTo(hx, hy);
            ctx.stroke();
        }
    });
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

        // The legacy inline mancal panel, if this build still has it.
        const legacyMancal = document.getElementById("mancal-web-panel-" + i);
        if (legacyMancal) body.appendChild(legacyMancal);
    }
}

let activeDetailBuoy = -1;

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
// Map page: start line length
//
// There is no stored "line length" anywhere in the system - recalcStartLine() on the Top keeps
// whatever distance the two buoys already are apart and only swings the line square to the wind.
// So the length shown here is measured from the two buoys' own target positions when the map is
// opened, and changing it means physically repositioning both of them.
//
// Captured once on open and deliberately NOT live-updated: the buoys drift and creep toward their
// targets constantly, and a figure that ticks while you are trying to set it is unusable.
// =================================================================================================
const START_LINE_STEP_M = 5;
let startLineOriginalM = null;   // as measured when the map was opened
let startLineTargetM = null;     // what the +/- buttons have dialled it to
let startLinePair = null;        // the two buoys that form the line

function metersBetween(lat1, lng1, lat2, lng2) {
    const R = 6371000, rad = Math.PI / 180;
    const dLat = (lat2 - lat1) * rad, dLng = (lng2 - lng1) * rad;
    const a = Math.sin(dLat / 2) * Math.sin(dLat / 2)
            + Math.cos(lat1 * rad) * Math.cos(lat2 * rad) * Math.sin(dLng / 2) * Math.sin(dLng / 2);
    return 2 * R * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

function bearingBetween(lat1, lng1, lat2, lng2) {
    const rad = Math.PI / 180;
    const y = Math.sin((lng2 - lng1) * rad) * Math.cos(lat2 * rad);
    const x = Math.cos(lat1 * rad) * Math.sin(lat2 * rad)
            - Math.sin(lat1 * rad) * Math.cos(lat2 * rad) * Math.cos((lng2 - lng1) * rad);
    return (Math.atan2(y, x) * 180 / Math.PI + 360) % 360;
}

// Where a buoy's end of the line actually is. The dashboard is never sent absolute target
// coordinates - it derives them the same way drawFieldMap() does, by projecting the reported
// Target Dist / Target Dir from the buoy's own fix. A buoy sitting on its waypoint reports a
// distance of ~0, so this collapses to its current position, which is the same point.
function buoyLinePoint(b) {
    const lat = parseFloat(b.data["Latitude (Lat)"]);
    const lng = parseFloat(b.data["Longitude (Lon)"]);
    if (isNaN(lat) || isNaN(lng) || lat === 0 || lng === 0) return null;

    const statusVal = parseInt(b.data.Status || "0");
    const holdsTarget = (statusVal === MsgType.LOCKING || statusVal === MsgType.LOCKED
                      || statusVal === MsgType.DOCKING || statusVal === MsgType.DOCKED);
    const dist = parseFloat(b.data["Target Dist"]);
    const dir = parseFloat(b.data["Target Dir"]);
    if (holdsTarget && !isNaN(dist) && !isNaN(dir) && dist > 0) {
        return projectCoords(lat, lng, dir, dist);
    }
    return { lat: lat, lng: lng };
}

// The two buoys forming the line. A third buoy, if there is one, is a mark and is left alone.
function findStartLinePair() {
    const held = buoys.filter(b => b.id && buoyLinePoint(b));
    return held.length >= 2 ? [held[0], held[1]] : null;
}

function startLinePanelRefresh() {
    const cur = document.getElementById("sl-current");
    const tgt = document.getElementById("sl-target");
    const btn = document.getElementById("sl-apply");
    if (!cur) return;

    if (startLineOriginalM === null) {
        cur.textContent = "--";
        tgt.textContent = "--";
        btn.disabled = true;
        btn.textContent = "RE-ALIGN";
        return;
    }
    cur.textContent = Math.round(startLineOriginalM) + " m";
    tgt.textContent = Math.round(startLineTargetM) + " m";
    const changed = Math.round(startLineTargetM) !== Math.round(startLineOriginalM);
    btn.disabled = !changed;
    btn.style.opacity = changed ? "1" : "0.4";
    btn.textContent = changed ? "RE-ALIGN TO " + Math.round(startLineTargetM) + " m" : "RE-ALIGN";
}

function startLineCapture() {
    startLinePair = findStartLinePair();
    if (!startLinePair) { startLineOriginalM = null; startLinePanelRefresh(); return; }
    const pa = buoyLinePoint(startLinePair[0]);
    const pb = buoyLinePoint(startLinePair[1]);
    if (!pa || !pb) { startLineOriginalM = null; startLinePanelRefresh(); return; }
    startLineOriginalM = metersBetween(pa.lat, pa.lng, pb.lat, pb.lng);
    startLineTargetM = startLineOriginalM;
    startLinePanelRefresh();
}

function startLineNudge(delta) {
    if (startLineOriginalM === null) return;
    startLineTargetM = Math.max(START_LINE_STEP_M, startLineTargetM + delta);
    startLinePanelRefresh();
}

// Moves both ends symmetrically about the existing midpoint, keeping the line's current bearing -
// this changes the length only. Squaring it to the wind is what Align Startline is for.
function startLineApply() {
    if (!startLinePair || startLineOriginalM === null) return;
    const a = startLinePair[0], b = startLinePair[1];
    const ea = buoyLinePoint(a), eb = buoyLinePoint(b);
    if (!ea || !eb) return;
    const aLat = ea.lat, aLng = ea.lng;
    const bLat = eb.lat, bLng = eb.lng;

    const midLat = (aLat + bLat) / 2, midLng = (aLng + bLng) / 2;
    const brgA = bearingBetween(midLat, midLng, aLat, aLng);
    const brgB = bearingBetween(midLat, midLng, bLat, bLng);
    const half = startLineTargetM / 2;

    if (!confirm("Re-align the start line to " + Math.round(startLineTargetM) + " m?\n\n"
               + "Both buoys will motor to their new ends of the line.")) return;

    const pa = projectCoords(midLat, midLng, brgA, half);
    const pb = projectCoords(midLat, midLng, brgB, half);

    [[a, pa], [b, pb]].forEach(function (pair) {
        const buoy = pair[0], pos = pair[1];
        const status = buoy.data.Status || "7";
        // SETLOCKPOS: the Top stores the point, sails to it and locks - the same command it uses
        // to push computed start line ends to the other buoy.
        sendCommand(buoy.id, buoy.id + ",99," + MsgType.SET + "," + MsgType.SETLOCKPOS + ","
                           + status + "," + pos.lat.toFixed(10) + "," + pos.lng.toFixed(10));
        logMessage("Buoy " + buoy.id.toUpperCase() + ": start line end moved to "
                 + pos.lat.toFixed(6) + ", " + pos.lng.toFixed(6), "UDP OUT");
    });

    // The dialled figure is the line now, so the panel settles on it rather than showing a
    // permanent pending change.
    startLineOriginalM = startLineTargetM;
    startLinePanelRefresh();
}

function initStartLinePanel() {
    const view = document.getElementById("full-map-view");
    if (!view) return;
    const panel = document.createElement("div");
    panel.style.cssText = "position:absolute; top:15px; right:15px; z-index:10000; background:#1e293b;"
        + "border:1px solid #334155; border-radius:8px; padding:12px; min-width:220px;"
        + "box-shadow:0 10px 25px -5px rgba(0,0,0,0.5); font-family:inherit; color:#e2e8f0;";
    panel.innerHTML =
        '<div style="font-size:0.7rem; color:#94a3b8; text-transform:uppercase; letter-spacing:0.5px;">Start line</div>'
      + '<div style="display:flex; align-items:baseline; gap:8px; margin:4px 0 2px 0;">'
      + '  <span id="sl-target" style="font-size:1.6rem; font-weight:bold; color:#eab308;">--</span>'
      + '</div>'
      + '<div style="font-size:0.7rem; color:#64748b; margin-bottom:8px;">on opening: <span id="sl-current">--</span></div>'
      + '<div style="display:flex; gap:8px;">'
      + '  <button id="sl-minus" style="flex:1; background:#475569; color:#fff; border:none; border-radius:6px; padding:8px 0; font-weight:bold; cursor:pointer;">&minus;5 m</button>'
      + '  <button id="sl-plus"  style="flex:1; background:#475569; color:#fff; border:none; border-radius:6px; padding:8px 0; font-weight:bold; cursor:pointer;">+5 m</button>'
      + '</div>'
      + '<button id="sl-apply" style="width:100%; margin-top:8px; background:#3b82f6; color:#fff; border:none; border-radius:6px; padding:9px 0; font-weight:bold; cursor:pointer;" disabled>RE-ALIGN</button>';
    view.appendChild(panel);

    document.getElementById("sl-minus").addEventListener("click", function () { startLineNudge(-START_LINE_STEP_M); });
    document.getElementById("sl-plus").addEventListener("click", function () { startLineNudge(START_LINE_STEP_M); });
    document.getElementById("sl-apply").addEventListener("click", startLineApply);

    // The buoys are not on the wire yet when the page loads, so take the measurement on the first
    // tick that actually has two waypoints rather than on a fixed timer.
    const grab = setInterval(function () {
        if (startLineOriginalM !== null) { clearInterval(grab); return; }
        startLineCapture();
    }, 500);
    setTimeout(function () { clearInterval(grab); }, 30000);
}
