// MsgType definitions matching Python's MsgType
const MsgType = {
    GET: 1,
    SET: 2,
    GETACK: 3,
    ACK: 4,
    NAC: 5,
    INF: 6,
    IDLE: 7,
    IDELING: 8,
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
    REMOTEING: 26,
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
    ADAPTIVE_TRIM: 84
};

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
    initWebSockets();
    initUIEventListeners();
    
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
    
    // Send over WebSocket (acts as UDP bridge when ESP32 or server bridges it)
    const targetBuoy = buoys.find(b => b.id === targetId);
    const udpAllowed = targetBuoy ? targetBuoy.udpEnabled : true;
    
    if (useWs && udpAllowed && socket && socket.readyState === WebSocket.OPEN) {
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
                if (raw.startsWith("LORA:")) {
                    raw = raw.substring(5);
                    source = "LoRa";
                } else if (raw.startsWith("UDP:")) {
                    raw = raw.substring(4);
                    source = "UDP";
                    
                    // Check for optional IP prefix, e.g. "192.168.1.78:$..."
                    const colonIdx = raw.indexOf(":$");
                    if (colonIdx !== -1) {
                        senderIp = raw.substring(0, colonIdx);
                        raw = raw.substring(colonIdx + 1);
                    }
                }
                
                // Log message to the correct panel (LORA LOG or UDP LOG)
                logMessage(raw, source + " IN");
                
                // Handle raw NMEA formats
                if (raw.startsWith("$")) {
                    parseMessage(raw, source, senderIp);
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
function parseMessage(message, source, senderIp = null) {
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
        
        // Find existing match by either target or sender ID
        for (let b of buoys) {
            if (b.id !== null && (b.id === senderId || b.id === targetId)) {
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
        
        if (!targetBuoy) return;
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
        } else {
            targetBuoy.lastUdpTime = now;
            targetBuoy.lastUdpContent = content;
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
                "compass_trim_enabled": fields[16] || "0"
            });
        } else if (cmd === MsgType.ADAPTIVE_TRIM && fields.length >= 6) {
            Object.assign(parsedData, {
                "compass_trim": fields[5],
                "compass_trim_enabled": fields[6]
            });
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
        udpInd.textContent = udpOk ? "UDP: OK" : "UDP: --";
        udpInd.className = "net-indicator" + (udpOk ? " ok" : "");
        
        const loraInd = document.getElementById(`lora-indicator-${i}`);
        loraInd.textContent = loraOk ? "LoRa: OK" : "LoRa: --";
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
            const cmd = isLocked ? MsgType.IDELING : MsgType.LOCKING;
            
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
            const cmd = isDocked ? MsgType.IDELING : MsgType.DOCKING;
            
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
    });
    
    // Global Action Button listeners
    document.getElementById("align-startline-btn").addEventListener("click", () => {
        logMessage("Global: Align Startline Clicked", "UDP OUT");
    });
    
    document.getElementById("align-track-btn").addEventListener("click", () => {
        logMessage("Global: Align Track Clicked", "UDP OUT");
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
        document.getElementById("setup-revSB").checked = b.data["revSB"] === "1";
        document.getElementById("setup-swap").checked = b.data["swap_BB_SB"] === "1";
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
        trimEn, // Index 16 on the wire (corresponds to compass_trim_enabled in newer builds)
        document.getElementById("setup-revSB").checked ? "1" : "0",
        document.getElementById("setup-swap").checked ? "1" : "0"
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
