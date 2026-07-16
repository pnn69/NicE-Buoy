#ifndef COMPASSTEST_HTML_H
#define COMPASSTEST_HTML_H

#include <Arduino.h>

const char COMPASSTEST_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>CompassTest Dashboard</title>
    <!-- Three.js CDN for 3D graphics -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
    <style>
        :root {
            --bg-color: #0f172a;
            --card-bg: #1e293b;
            --text-color: #f8fafc;
            --text-muted: #94a3b8;
            --primary: #3b82f6;
            --primary-hover: #2563eb;
            --success: #10b981;
            --warning: #f59e0b;
            --danger: #ef4444;
            --border: #334155;
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
        }

        body {
            background-color: var(--bg-color);
            color: var(--text-color);
            padding: 20px;
            display: flex;
            flex-direction: column;
            align-items: center;
            min-height: 100vh;
        }

        header {
            width: 100%;
            max-width: 1200px;
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding-bottom: 20px;
            border-bottom: 1px solid var(--border);
            margin-bottom: 20px;
        }

        h1 {
            font-size: 1.5rem;
            font-weight: 700;
            display: flex;
            align-items: center;
            gap: 10px;
        }

        h1 span { color: var(--primary); }

        .status {
            display: flex;
            align-items: center;
            gap: 8px;
            font-size: 0.875rem;
            background: var(--card-bg);
            padding: 6px 12px;
            border-radius: 9999px;
            border: 1px solid var(--border);
        }

        .status-dot { width: 10px; height: 10px; border-radius: 50%; background-color: var(--danger); }
        .status-dot.connected { background-color: var(--success); box-shadow: 0 0 8px var(--success); }

        .tabs {
            display: flex;
            background: var(--card-bg);
            padding: 4px;
            border-radius: 8px;
            border: 1px solid var(--border);
            margin-bottom: 20px;
        }

        .tab-btn {
            background: transparent;
            border: none;
            color: var(--text-muted);
            padding: 8px 16px;
            font-size: 0.95rem;
            font-weight: 600;
            border-radius: 6px;
            cursor: pointer;
            transition: all 0.2s;
        }

        .tab-btn.active { background: var(--primary); color: white; }

        .container {
            width: 100%;
            max-width: 1200px;
            display: none;
            gap: 20px;
            grid-template-columns: 1fr;
        }

        .container.active { display: grid; }

        @media (min-width: 900px) {
            #dashboard-container { grid-template-columns: 1fr; }
            #calibration-container { grid-template-columns: 1fr 1fr; }
        }

        .card {
            background: var(--card-bg);
            border: 1px solid var(--border);
            border-radius: 12px;
            padding: 20px;
            display: flex;
            flex-direction: column;
        }

        .card h2 {
            font-size: 1.1rem;
            margin-bottom: 15px;
            color: var(--text-muted);
            border-bottom: 1px solid var(--border);
            padding-bottom: 8px;
        }

        .windrose-grid {
            display: grid;
            grid-template-columns: 1fr;
            gap: 20px;
        }

        @media (min-width: 600px) {
            .windrose-grid { grid-template-columns: 1fr 1fr; }
        }
        @media (min-width: 1024px) {
            .windrose-grid { grid-template-columns: 1fr 1fr 1fr 1fr; }
        }

        .windrose-card {
            display: flex;
            flex-direction: column;
            align-items: center;
            text-align: center;
        }

        .windrose-card h3 {
            font-size: 1rem;
            margin-bottom: 10px;
            color: var(--text-color);
        }

        .compass-wrapper {
            position: relative;
            width: 180px;
            height: 180px;
            margin-bottom: 10px;
        }

        .compass { width: 100%; height: 100%; }

        .needle {
            transform-origin: 100px 100px;
            transition: transform 0.15s cubic-bezier(0.25, 0.46, 0.45, 0.94);
        }

        .heading-val {
            font-size: 1.5rem;
            font-weight: bold;
            margin-top: 5px;
            font-family: monospace;
            color: var(--primary);
        }

        #visualizer3d {
            width: 100%;
            height: 300px;
            border-radius: 8px;
            overflow: hidden;
            background: #000;
            border: 1px solid var(--border);
            margin-top: 20px;
        }

        .telemetry-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 12px;
            margin-top: 15px;
        }

        .telemetry-item {
            background: rgba(15, 23, 42, 0.5);
            padding: 10px;
            border-radius: 6px;
            border: 1px solid var(--border);
        }

        .telemetry-label { font-size: 0.75rem; color: var(--text-muted); margin-bottom: 4px; }
        .telemetry-value { font-size: 1.1rem; font-weight: bold; font-family: monospace; }

        .btn {
            background: var(--primary);
            color: white;
            border: none;
            padding: 12px 20px;
            font-size: 1rem;
            font-weight: 600;
            border-radius: 8px;
            cursor: pointer;
            transition: background 0.2s;
            margin-bottom: 12px;
            width: 100%;
            text-align: center;
        }

        .btn:hover { background: var(--primary-hover); }
        .btn:disabled { background: var(--border); color: var(--text-muted); cursor: not-allowed; }
        .btn-success { background: var(--success); }
        .btn-success:hover { background: #059669; }
        .btn-danger { background: var(--danger); }
        .btn-danger:hover { background: #dc2626; }

        #scatter-canvas {
            width: 100%;
            height: 300px;
            background: #0f172a;
            border: 1px solid var(--border);
            border-radius: 8px;
        }

        .calibration-status {
            background: rgba(15, 23, 42, 0.4);
            padding: 15px;
            border-radius: 8px;
            border: 1px solid var(--border);
            margin-top: 15px;
            font-size: 0.9rem;
            line-height: 1.5;
        }
        .calibration-status h4 { color: var(--primary); margin-bottom: 8px; }
        .matrix-display {
            font-family: monospace;
            white-space: pre-wrap;
            background: #090d16;
            padding: 10px;
            border-radius: 4px;
            border: 1px solid var(--border);
            margin-top: 8px;
            font-size: 0.8rem;
        }
    </style>
</head>
<body>

    <header>
        <h1><span>CompassTest</span> Dashboard</h1>
        <div style="display:flex; gap:10px; align-items:center;">
            <button class="btn" style="background-color: var(--card-bg); border: 1px solid var(--border); color: var(--text-color); margin-bottom: 0px; padding: 6px 12px; font-size: 0.875rem; width: auto;" onclick="location.href='/'">➔ Return to Main Page</button>
            <div class="status">
                <div id="status-dot" class="status-dot"></div>
                <span id="status-text">Disconnected</span>
            </div>
        </div>
    </header>

    <div class="tabs">
        <button id="btn-dash" class="tab-btn active" onclick="switchTab('dashboard')">Dashboard</button>
        <button id="btn-cal" class="tab-btn" onclick="switchTab('calibration')">Calibration</button>
    </div>

    <!-- DASHBOARD CONTAINER -->
    <div id="dashboard-container" class="container active">
        <div class="card">
            <h2>Real-time Compasses</h2>
            <div class="windrose-grid">

                <!-- RAW -->
                <div class="windrose-card">
                    <h3>Raw Values</h3>
                    <div class="compass-wrapper">
                        <svg class="compass" viewBox="0 0 200 200">
                            <circle cx="100" cy="100" r="95" fill="#161616" stroke="#2c2c2c" stroke-width="5"/>
                            <circle cx="100" cy="100" r="82" fill="none" stroke="#00e6ff" stroke-width="1.5" stroke-dasharray="3, 5"/>
                            <line x1="100" y1="12" x2="100" y2="18" stroke="#ff3333" stroke-width="3"/>
                            <line x1="100" y1="182" x2="100" y2="188" stroke="#eee" stroke-width="2"/>
                            <line x1="12" y1="100" x2="18" y2="100" stroke="#eee" stroke-width="2"/>
                            <line x1="182" y1="100" x2="188" y2="100" stroke="#eee" stroke-width="2"/>
                            <text x="100" y="32" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#ff3333" text-anchor="middle">N</text>
                            <text x="100" y="174" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">S</text>
                            <text x="170" y="106" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">E</text>
                            <text x="30" y="106" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">W</text>
                            <g class="needle" id="rose-raw-needle">
                                <polygon points="100,20 108,100 100,108" fill="#ff3333"/>
                                <polygon points="100,20 92,100 100,108" fill="#cc0000"/>
                                <circle cx="100" cy="100" r="7" fill="#ffd700" stroke="#121212" stroke-width="2"/>
                            </g>
                        </svg>
                    </div>
                    <div id="val-rose-raw" class="heading-val">000&deg;</div>
                </div>

                <!-- HARD IRON -->
                <div class="windrose-card">
                    <h3>Hard Iron Comp</h3>
                    <div class="compass-wrapper">
                        <svg class="compass" viewBox="0 0 200 200">
                            <circle cx="100" cy="100" r="95" fill="#161616" stroke="#2c2c2c" stroke-width="5"/>
                            <circle cx="100" cy="100" r="82" fill="none" stroke="#00e6ff" stroke-width="1.5" stroke-dasharray="3, 5"/>
                            <line x1="100" y1="12" x2="100" y2="18" stroke="#ff3333" stroke-width="3"/>
                            <line x1="100" y1="182" x2="100" y2="188" stroke="#eee" stroke-width="2"/>
                            <line x1="12" y1="100" x2="18" y2="100" stroke="#eee" stroke-width="2"/>
                            <line x1="182" y1="100" x2="188" y2="100" stroke="#eee" stroke-width="2"/>
                            <text x="100" y="32" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#ff3333" text-anchor="middle">N</text>
                            <text x="100" y="174" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">S</text>
                            <text x="170" y="106" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">E</text>
                            <text x="30" y="106" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">W</text>
                            <g class="needle" id="rose-hard-needle">
                                <polygon points="100,20 108,100 100,108" fill="#ff3333"/>
                                <polygon points="100,20 92,100 100,108" fill="#cc0000"/>
                                <circle cx="100" cy="100" r="7" fill="#ffd700" stroke="#121212" stroke-width="2"/>
                            </g>
                        </svg>
                    </div>
                    <div id="val-rose-hard" class="heading-val">000&deg;</div>
                </div>

                <!-- HARD + SOFT IRON -->
                <div class="windrose-card">
                    <h3>Hard & Soft Comp</h3>
                    <div class="compass-wrapper">
                        <svg class="compass" viewBox="0 0 200 200">
                            <circle cx="100" cy="100" r="95" fill="#161616" stroke="#2c2c2c" stroke-width="5"/>
                            <circle cx="100" cy="100" r="82" fill="none" stroke="#00e6ff" stroke-width="1.5" stroke-dasharray="3, 5"/>
                            <line x1="100" y1="12" x2="100" y2="18" stroke="#ff3333" stroke-width="3"/>
                            <line x1="100" y1="182" x2="100" y2="188" stroke="#eee" stroke-width="2"/>
                            <line x1="12" y1="100" x2="18" y2="100" stroke="#eee" stroke-width="2"/>
                            <line x1="182" y1="100" x2="188" y2="100" stroke="#eee" stroke-width="2"/>
                            <text x="100" y="32" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#ff3333" text-anchor="middle">N</text>
                            <text x="100" y="174" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">S</text>
                            <text x="170" y="106" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">E</text>
                            <text x="30" y="106" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">W</text>
                            <g class="needle" id="rose-soft-needle">
                                <polygon points="100,20 108,100 100,108" fill="#ff3333"/>
                                <polygon points="100,20 92,100 100,108" fill="#cc0000"/>
                                <circle cx="100" cy="100" r="7" fill="#ffd700" stroke="#121212" stroke-width="2"/>
                            </g>
                        </svg>
                    </div>
                    <div id="val-rose-soft" class="heading-val">000&deg;</div>
                </div>

                <!-- FULL COMP (TILT) -->
                <div class="windrose-card">
                    <h3>Tilt Compensated</h3>
                    <div class="compass-wrapper">
                        <svg class="compass" viewBox="0 0 200 200">
                            <circle cx="100" cy="100" r="95" fill="#161616" stroke="#2c2c2c" stroke-width="5"/>
                            <circle cx="100" cy="100" r="82" fill="none" stroke="#00e6ff" stroke-width="1.5" stroke-dasharray="3, 5"/>
                            <line x1="100" y1="12" x2="100" y2="18" stroke="#ff3333" stroke-width="3"/>
                            <line x1="100" y1="182" x2="100" y2="188" stroke="#eee" stroke-width="2"/>
                            <line x1="12" y1="100" x2="18" y2="100" stroke="#eee" stroke-width="2"/>
                            <line x1="182" y1="100" x2="188" y2="100" stroke="#eee" stroke-width="2"/>
                            <text x="100" y="32" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#ff3333" text-anchor="middle">N</text>
                            <text x="100" y="174" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">S</text>
                            <text x="170" y="106" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">E</text>
                            <text x="30" y="106" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">W</text>
                            <g class="needle" id="rose-tilt-needle">
                                <polygon points="100,20 108,100 100,108" fill="#ff3333"/>
                                <polygon points="100,20 92,100 100,108" fill="#cc0000"/>
                                <circle cx="100" cy="100" r="7" fill="#ffd700" stroke="#121212" stroke-width="2"/>
                            </g>
                        </svg>
                    </div>
                    <div id="val-rose-tilt" class="heading-val">000&deg;</div>
                </div>

            </div>

            <div id="visualizer3d"></div>

            <div class="telemetry-grid">
                <div class="telemetry-item">
                    <div class="telemetry-label">Pitch</div>
                    <div id="val-pitch" class="telemetry-value">0.0&deg;</div>
                </div>
                <div class="telemetry-item">
                    <div class="telemetry-label">Roll</div>
                    <div id="val-roll" class="telemetry-value">0.0&deg;</div>
                </div>
                <div class="telemetry-item">
                    <div class="telemetry-label">Raw Mag (X, Y, Z)</div>
                    <div id="val-raw-mag" class="telemetry-value">0, 0, 0</div>
                </div>
                <div class="telemetry-item">
                    <div class="telemetry-label">Cal Mag (X, Y, Z)</div>
                    <div id="val-cal-mag" class="telemetry-value">0, 0, 0</div>
                </div>
            </div>
        </div>
    </div>

    <!-- CALIBRATION CONTAINER -->
    <div id="calibration-container" class="container">
        <div class="card">
            <h2>Magnetometer Calibration</h2>
            <button id="btn-start-cal" class="btn" onclick="toggleCalibration()">Start Calibration</button>
            <button id="btn-save-cal" class="btn btn-success" onclick="saveCalibration()" disabled>Store Calibration to NVM</button>

            <div class="calibration-status">
                <h4>Instructions</h4>
                <ol style="margin-left: 20px;">
                    <li>Click <strong>Start Calibration</strong>.</li>
                    <li>Rotate the device in a figure-eight pattern.</li>
                    <li>Map a sphere on the scatter plot.</li>
                    <li>Click <strong>Stop Calibration</strong>.</li>
                    <li>Click <strong>Store Calibration to NVM</strong>.</li>
                </ol>
                <div id="cal-results" style="display:none; margin-top: 12px;">
                    <div style="font-weight:bold;">Computed Coefficients:</div>
                    <div id="cal-coefficients" class="matrix-display">...</div>
                </div>
            </div>

            <hr style="margin: 20px 0; border: none; border-top: 1px solid var(--border);">
            <h2>Accelerometer Level Calibration</h2>
            <button id="btn-level-cal" class="btn" style="background-color: var(--secondary);" onclick="calibrateLevel()">Calibrate Flat Level</button>
            <div id="level-cal-status" style="margin-top: 8px; font-size: 0.85rem; color: var(--text-muted); line-height: 1.3;">
                Place the device housing on a perfectly flat horizontal surface and click the button above.
                This calculates any mechanical mounting misalignment of the accelerometer chip and mathematically offsets it,
                forcing pitch and roll to read exactly 0&deg; when horizontal.
            </div>
        </div>

        <div class="card">
            <h2>Scatter Plot</h2>
            <canvas id="scatter-canvas" width="400" height="300"></canvas>
            <div class="telemetry-grid">
                <div class="telemetry-item">
                    <div class="telemetry-label">Data Points</div>
                    <div id="cal-pts-count" class="telemetry-value">0</div>
                </div>
                <div class="telemetry-item">
                    <div class="telemetry-label">Status</div>
                    <div id="cal-state" class="telemetry-value" style="color: var(--warning);">Idle</div>
                </div>
            </div>
        </div>
    </div>

    <script>
        let ws;
        let wsConnected = false;
        let activeTab = 'dashboard';

        let headingRaw = 0, headingHard = 0, headingSoft = 0, headingTilt = 0;
        let rotRaw = 0, rotHard = 0, rotSoft = 0, rotTilt = 0;
        let currentRoll = 0, currentPitch = 0;

        let isCalibrating = false;
        let rawPoints = [];
        let maxPoints = 1500;
        let hardIronOffset = [0, 0, 0];
        let softIronScale = [1, 1, 1];

        let scene, camera, renderer, trackerMesh;
        const scatterCanvas = document.getElementById('scatter-canvas');

        function getShortestRotation(current, target) {
            target = (target % 360 + 360) % 360;
            let currentNorm = (current % 360 + 360) % 360;
            let diff = target - currentNorm;
            if (diff > 180) diff -= 360;
            if (diff < -180) diff += 360;
            return current + diff;
        }

        window.addEventListener('load', () => {
            if (typeof THREE !== 'undefined') {
                try {
                    initThreeJS();
                } catch (e) {
                    console.error("Error initializing ThreeJS:", e);
                    document.getElementById('visualizer3d').innerHTML = '<div style="color:var(--text-muted); text-align:center; padding-top:130px; font-weight:bold;">3D Visualizer Init Failed</div>';
                }
            } else {
                console.warn("Three.js not loaded. Skipping 3D visualization.");
                document.getElementById('visualizer3d').innerHTML = '<div style="color:var(--text-muted); text-align:center; padding-top:130px; font-weight:bold;">3D PCB Visualizer Offline (No Internet for CDN)</div>';
            }
            initWebSockets();
            drawScatter();
            setInterval(() => { if (isCalibrating) drawScatter(); }, 100);
        });

        function switchTab(tabId) {
            activeTab = tabId;
            document.querySelectorAll('.container').forEach(el => el.classList.remove('active'));
            document.querySelectorAll('.tab-btn').forEach(el => el.classList.remove('active'));

            if (tabId === 'dashboard') {
                document.getElementById('dashboard-container').classList.add('active');
                document.getElementById('btn-dash').classList.add('active');
                if (renderer) {
                    try {
                        const c = document.getElementById('visualizer3d');
                        if (c && c.clientWidth > 0 && c.clientHeight > 0) {
                            renderer.setSize(c.clientWidth, c.clientHeight);
                        }
                    } catch (e) {
                        console.warn("Could not resize WebGL renderer:", e);
                    }
                }
            } else {
                document.getElementById('calibration-container').classList.add('active');
                document.getElementById('btn-cal').classList.add('active');
                drawScatter();
            }
        }

        function initWebSockets() {
            const host = window.location.hostname || '192.168.4.1';
            ws = new WebSocket(`ws://${host}:81`);

            ws.onopen = () => {
                wsConnected = true;
                document.getElementById('status-dot').className = 'status-dot connected';
                document.getElementById('status-text').innerText = 'Connected';
            };

            ws.onclose = () => {
                wsConnected = false;
                document.getElementById('status-dot').className = 'status-dot';
                document.getElementById('status-text').innerText = 'Disconnected';
                setTimeout(initWebSockets, 2000);
            };

            ws.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);

                    headingRaw = data.raw;
                    headingHard = data.hard;
                    headingSoft = data.hardSoft;
                    headingTilt = data.tilt;
                    currentRoll = data.roll;
                    currentPitch = data.pitch;

                    document.getElementById('val-rose-raw').innerText = Math.round(headingRaw).toString().padStart(3, '0') + '°';
                    document.getElementById('val-rose-hard').innerText = Math.round(headingHard).toString().padStart(3, '0') + '°';
                    document.getElementById('val-rose-soft').innerText = Math.round(headingSoft).toString().padStart(3, '0') + '°';
                    document.getElementById('val-rose-tilt').innerText = Math.round(headingTilt).toString().padStart(3, '0') + '°';

                    document.getElementById('val-pitch').innerText = currentPitch.toFixed(1) + '°';
                    document.getElementById('val-roll').innerText = currentRoll.toFixed(1) + '°';
                    document.getElementById('val-raw-mag').innerText = `${Math.round(data.mx_raw)}, ${Math.round(data.my_raw)}, ${Math.round(data.mz_raw)}`;
                    document.getElementById('val-cal-mag').innerText = `${Math.round(data.mx_cal)}, ${Math.round(data.my_cal)}, ${Math.round(data.mz_cal)}`;

                    rotRaw = getShortestRotation(rotRaw, headingRaw);
                    document.getElementById('rose-raw-needle').style.transform = `rotate(${rotRaw}deg)`;
                    rotHard = getShortestRotation(rotHard, headingHard);
                    document.getElementById('rose-hard-needle').style.transform = `rotate(${rotHard}deg)`;
                    rotSoft = getShortestRotation(rotSoft, headingSoft);
                    document.getElementById('rose-soft-needle').style.transform = `rotate(${rotSoft}deg)`;
                    rotTilt = getShortestRotation(rotTilt, headingTilt);
                    document.getElementById('rose-tilt-needle').style.transform = `rotate(${rotTilt}deg)`;

                    if (isCalibrating) {
                        rawPoints.push([data.mx_raw, data.my_raw, data.mz_raw]);
                        if (rawPoints.length > maxPoints) rawPoints.shift();
                        document.getElementById('cal-pts-count').innerText = rawPoints.length;
                    }

                    if (trackerMesh) {
                        const yawRad = -headingTilt * Math.PI / 180;
                        const pitchRad = currentPitch * Math.PI / 180;
                        const rollRad = -currentRoll * Math.PI / 180;
                        trackerMesh.rotation.order = 'YXZ';
                        trackerMesh.rotation.set(pitchRad, yawRad, rollRad);
                    }
                } catch (e) {}
            };
        }

        function initThreeJS() {
            const container = document.getElementById('visualizer3d');
            scene = new THREE.Scene();
            scene.background = new THREE.Color('#0b0f19');
            camera = new THREE.PerspectiveCamera(45, container.clientWidth / container.clientHeight, 0.1, 100);
            camera.position.set(3, 2, 3);
            camera.lookAt(0, 0, 0);

            renderer = new THREE.WebGLRenderer({ antialias: true });
            renderer.setSize(container.clientWidth, container.clientHeight);
            container.appendChild(renderer.domElement);

            scene.add(new THREE.AmbientLight(0xffffff, 0.6));
            const dl = new THREE.DirectionalLight(0xffffff, 0.8);
            dl.position.set(5, 8, 5);
            scene.add(dl);

            const grid = new THREE.GridHelper(4, 10, 0x3b82f6, 0x1e293b);
            grid.position.y = -1;
            scene.add(grid);

            trackerMesh = new THREE.Group();
            const pcb = new THREE.Mesh(new THREE.BoxGeometry(1.6, 0.1, 1.0), new THREE.MeshLambertMaterial({ color: 0x1e293b }));
            trackerMesh.add(pcb);
            trackerMesh.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(0, 0.1, 0), 1.4, 0xef4444));
            scene.add(trackerMesh);

            window.addEventListener('resize', () => {
                if (activeTab === 'dashboard') {
                    camera.aspect = container.clientWidth / container.clientHeight;
                    camera.updateProjectionMatrix();
                    renderer.setSize(container.clientWidth, container.clientHeight);
                }
            });

            function animate() {
                requestAnimationFrame(animate);
                renderer.render(scene, camera);
            }
            animate();
        }

        function drawScatter() {
            const ctx = scatterCanvas.getContext('2d');
            const w = scatterCanvas.width, h = scatterCanvas.height;

            ctx.fillStyle = '#0f172a';
            ctx.fillRect(0, 0, w, h);
            ctx.strokeStyle = '#334155';
            ctx.beginPath(); ctx.moveTo(w/2, 0); ctx.lineTo(w/2, h); ctx.stroke();

            if (rawPoints.length === 0) return;

            let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity, minZ = Infinity, maxZ = -Infinity;
            rawPoints.forEach(p => {
                if(p[0]<minX)minX=p[0]; if(p[0]>maxX)maxX=p[0];
                if(p[1]<minY)minY=p[1]; if(p[1]>maxY)maxY=p[1];
                if(p[2]<minZ)minZ=p[2]; if(p[2]>maxZ)maxZ=p[2];
            });
            const pad = 30;
            minX-=pad; maxX+=pad; minY-=pad; maxY+=pad; minZ-=pad; maxZ+=pad;

            const map = (v, min, max, outMin, outMax) => (v - min) * (outMax - outMin) / (max - min) + outMin;

            ctx.fillStyle = 'rgba(59, 130, 246, 0.8)';
            rawPoints.forEach(p => ctx.fillRect(map(p[0], minX, maxX, 15, w/2-15), map(p[1], minY, maxY, h-15, 25), 2, 2));

            ctx.fillStyle = 'rgba(16, 185, 129, 0.8)';
            rawPoints.forEach(p => ctx.fillRect(map(p[1], minY, maxY, w/2+15, w-15), map(p[2], minZ, maxZ, h-15, 25), 2, 2));
        }

        function toggleCalibration() {
            const btn = document.getElementById('btn-start-cal');
            const state = document.getElementById('cal-state');

            if (!isCalibrating) {
                isCalibrating = true;
                rawPoints = [];
                btn.innerText = 'Stop & Calculate Calibration';
                btn.className = 'btn btn-danger';
                state.innerText = 'Calibrating...';
                state.style.color = 'var(--danger)';
                document.getElementById('btn-save-cal').disabled = true;
            } else {
                isCalibrating = false;
                btn.innerText = 'Start Calibration';
                btn.className = 'btn';
                state.innerText = 'Complete';
                state.style.color = 'var(--success)';
                calculateCalibrationCoefficients();
            }
        }

        function calculateCalibrationCoefficients() {
            if (rawPoints.length < 50) return alert("Need more points!");

            let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity, minZ = Infinity, maxZ = -Infinity;
            rawPoints.forEach(p => {
                if(p[0]<minX)minX=p[0]; if(p[0]>maxX)maxX=p[0];
                if(p[1]<minY)minY=p[1]; if(p[1]>maxY)maxY=p[1];
                if(p[2]<minZ)minZ=p[2]; if(p[2]>maxZ)maxZ=p[2];
            });

            hardIronOffset = [(maxX+minX)/2, (maxY+minY)/2, (maxZ+minZ)/2];
            const rx = (maxX-minX)/2, ry = (maxY-minY)/2, rz = (maxZ-minZ)/2;
            const avg = (rx+ry+rz)/3;
            softIronScale = [avg/rx, avg/ry, avg/rz];

            document.getElementById('cal-results').style.display = 'block';
            document.getElementById('cal-coefficients').innerText = `Hard: ${hardIronOffset.map(v=>v.toFixed(2)).join(', ')}\nSoft: ${softIronScale.map(v=>v.toFixed(4)).join(', ')}`;
            document.getElementById('btn-save-cal').disabled = false;
        }

        function saveCalibration() {
            if (!wsConnected) return;
            ws.send(JSON.stringify({
                command: 'save_cal',
                hi_x: hardIronOffset[0], hi_y: hardIronOffset[1], hi_z: hardIronOffset[2],
                si_x: softIronScale[0], si_y: softIronScale[1], si_z: softIronScale[2]
            }));
            alert("Saved!");
        }

        function calibrateLevel() {
            if (!wsConnected) return;
            ws.send(JSON.stringify({
                command: 'calibrate_level'
            }));
            alert("Level calibration saved!");
        }
    </script>
</body>
</html>
)rawliteral";

#endif /* COMPASSTEST_HTML_H */
