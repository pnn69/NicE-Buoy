#ifndef SHOWACTUALDATA_HTML_H
#define SHOWACTUALDATA_HTML_H

#include <Arduino.h>

const char SHOWACTUALDATA_HTML[] PROGMEM = R"rawliteral(
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
            .windrose-grid { grid-template-columns: repeat(2, 1fr); }
        }
        @media (min-width: 900px) {
            .windrose-grid { grid-template-columns: repeat(3, 1fr); }
        }
        @media (min-width: 1200px) {
            .windrose-grid { grid-template-columns: repeat(5, 1fr); }
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

        .visualizer-row {
            display: flex;
            flex-direction: column;
            gap: 20px;
            margin-top: 20px;
            width: 100%;
        }

        #visualizer3d, #visualizer3d-pitchroll {
            width: 100%;
            height: 300px;
            border-radius: 8px;
            overflow: hidden;
            background: #000;
            border: 1px solid var(--border);
            margin-top: 0;
        }

        @media (min-width: 1024px) {
            .visualizer-row {
                flex-direction: row;
            }
            #visualizer3d {
                flex: 1.5;
            }
            #visualizer3d-pitchroll {
                flex: 1.5;
            }
            .pitch-roll-card {
                flex: 1;
            }
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
        <div class="status">
            <div id="status-dot" class="status-dot"></div>
            <span id="status-text">Disconnected</span>
        </div>
    </header>

    <!-- DASHBOARD CONTAINER -->
    <div id="dashboard-container" class="container" style="display: grid;">
        <div class="card">
            <h2 style="display: flex; align-items: center; flex-wrap: wrap; justify-content: space-between; width: 100%;">
                Real-time Compasses
                <span id="cal-load-text" style="font-size: 0.8rem; font-family: monospace; color: var(--text-muted); font-weight: normal; margin-left: auto; padding-left: 10px;">HI: [0.0, 0.0, 0.0] SI: [1.00, 1.00, 1.00]</span>
            </h2>
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
                            <text x="100" y="174" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">S</text>
                            <text x="168" y="106" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">E</text>
                            <text x="32" y="106" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">W</text>
                            <g class="needle" id="rose-raw-needle">
                                <polygon points="100,20 108,100 100,108" fill="#ff3333"/>
                                <polygon points="100,20 92,100 100,108" fill="#cc0000"/>
                                <circle cx="100" cy="100" r="7" fill="#ffd700" stroke="#121212" stroke-width="2"/>
                            </g>
                        </svg>
                    </div>
                    <div id="val-rose-raw" class="heading-val">000°</div>
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
                            <text x="100" y="174" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">S</text>
                            <text x="168" y="106" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">E</text>
                            <text x="32" y="106" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">W</text>
                            <g class="needle" id="rose-hard-needle">
                                <polygon points="100,20 108,100 100,108" fill="#ff3333"/>
                                <polygon points="100,20 92,100 100,108" fill="#cc0000"/>
                                <circle cx="100" cy="100" r="7" fill="#ffd700" stroke="#121212" stroke-width="2"/>
                            </g>
                        </svg>
                    </div>
                    <div id="val-rose-hard" class="heading-val">000°</div>
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
                            <text x="100" y="174" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">S</text>
                            <text x="168" y="106" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">E</text>
                            <text x="32" y="106" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">W</text>
                            <g class="needle" id="rose-soft-needle">
                                <polygon points="100,20 108,100 100,108" fill="#ff3333"/>
                                <polygon points="100,20 92,100 100,108" fill="#cc0000"/>
                                <circle cx="100" cy="100" r="7" fill="#ffd700" stroke="#121212" stroke-width="2"/>
                            </g>
                        </svg>
                    </div>
                    <div id="val-rose-soft" class="heading-val">000°</div>
                </div>

                <!-- TILT COMPENSATED OPTION 3 -->
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
                            <text x="100" y="174" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">S</text>
                            <text x="168" y="106" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">E</text>
                            <text x="32" y="106" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">W</text>
                            <g class="needle" id="rose-opt3-needle">
                                <polygon points="100,20 108,100 100,108" fill="#ff3333"/>
                                <polygon points="100,20 92,100 100,108" fill="#cc0000"/>
                                <circle cx="100" cy="100" r="7" fill="#ffd700" stroke="#121212" stroke-width="2"/>
                            </g>
                        </svg>
                    </div>
                    <div id="val-rose-opt3" class="heading-val">000°</div>
                </div>

                <!-- 9-DOF FUSION OPTION 4 -->
                <div class="windrose-card">
                    <h3>9-DOF Fusion Comp</h3>
                    <div class="compass-wrapper">
                        <svg class="compass" viewBox="0 0 200 200">
                            <circle cx="100" cy="100" r="95" fill="#161616" stroke="#2c2c2c" stroke-width="5"/>
                            <circle cx="100" cy="100" r="82" fill="none" stroke="#00e6ff" stroke-width="1.5" stroke-dasharray="3, 5"/>
                            <line x1="100" y1="12" x2="100" y2="18" stroke="#ff3333" stroke-width="3"/>
                            <line x1="100" y1="182" x2="100" y2="188" stroke="#eee" stroke-width="2"/>
                            <line x1="12" y1="100" x2="18" y2="100" stroke="#eee" stroke-width="2"/>
                            <line x1="182" y1="100" x2="188" y2="100" stroke="#eee" stroke-width="2"/>
                            <text x="100" y="32" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#ff3333" text-anchor="middle">N</text>
                            <text x="100" y="174" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">S</text>
                            <text x="168" y="106" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">E</text>
                            <text x="32" y="106" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#eee" text-anchor="middle">W</text>
                            <g class="needle" id="rose-tilt-needle">
                                <polygon points="100,20 108,100 100,108" fill="#ff3333"/>
                                <polygon points="100,20 92,100 100,108" fill="#cc0000"/>
                                <circle cx="100" cy="100" r="7" fill="#ffd700" stroke="#121212" stroke-width="2"/>
                            </g>
                        </svg>
                    </div>
                    <div id="val-rose-tilt" class="heading-val">000°</div>
                </div>

            </div>
            
            <div class="visualizer-row">
                <!-- 1. FULL 3D MODEL (YAW, PITCH, ROLL) -->
                <div id="visualizer3d"></div>
                
                <!-- 2. PITCH & ROLL ONLY 3D CUBE (NO YAW) -->
                <div id="visualizer3d-pitchroll"></div>
                
                <!-- 3. ATTITUDE / BUBBLE LEVEL GAUGE -->
                <div class="windrose-card pitch-roll-card" style="min-width: 250px; background: var(--card-bg); border: 1px solid var(--border); border-radius: 12px; padding: 20px; display: flex; flex-direction: column; align-items: center; justify-content: center;">
                    <h3 style="font-size: 1rem; margin-bottom: 10px; color: var(--text-color);">Attitude / Bubble Level</h3>
                    <div style="position: relative; width: 100%; height: 180px; display: flex; align-items: center; justify-content: center;">
                        <svg id="attitude-gauge" width="160" height="180" viewBox="0 0 200 200">
                            <!-- Background circle -->
                            <circle cx="100" cy="100" r="90" fill="#111827" stroke="#2d3748" stroke-width="4"/>
                            <!-- Ring markers for 10, 20, 30 deg -->
                            <circle cx="100" cy="100" r="30" fill="none" stroke="#4a5568" stroke-dasharray="2, 4" stroke-width="1"/>
                            <circle cx="100" cy="100" r="60" fill="none" stroke="#4a5568" stroke-dasharray="2, 4" stroke-width="1"/>
                            <!-- Crosshairs -->
                            <line x1="100" y1="10" x2="100" y2="190" stroke="#4a5568" stroke-width="1.5"/>
                            <line x1="10" y1="100" x2="190" y2="100" stroke="#4a5568" stroke-width="1.5"/>
                            <!-- Degree labels -->
                            <text x="100" y="65" font-size="10" fill="#718096" text-anchor="middle">10°</text>
                            <text x="100" y="35" font-size="10" fill="#718096" text-anchor="middle">20°</text>
                            <!-- Level Indicator Bubble (Moving dot) -->
                            <circle id="attitude-bubble" cx="100" cy="100" r="12" fill="#38bdf8" opacity="0.8" stroke="#0284c7" stroke-width="2" style="transition: cx 0.1s ease, cy 0.1s ease;"/>
                            <circle cx="100" cy="100" r="4" fill="#ff3333"/>
                        </svg>
                    </div>
                    <div style="display: flex; justify-content: space-around; width: 100%; text-align: center; margin-top: 10px;">
                        <div>
                            <div style="font-size:0.75rem; color:var(--text-muted); font-weight:600;">Pitch</div>
                            <div id="gauge-pitch-val" style="font-size:1.4rem; font-weight:bold; color:#ef4444;">0.0°</div>
                        </div>
                        <div>
                            <div style="font-size:0.75rem; color:var(--text-muted); font-weight:600;">Roll</div>
                            <div id="gauge-roll-val" style="font-size:1.4rem; font-weight:bold; color:#38bdf8;">0.0°</div>
                        </div>
                    </div>
                </div>
            </div>
            
            <div class="telemetry-grid">
                <div class="telemetry-item">
                    <div class="telemetry-label">Pitch</div>
                    <div id="val-pitch" class="telemetry-value">0.0°</div>
                </div>
                <div class="telemetry-item">
                    <div class="telemetry-label">Roll</div>
                    <div id="val-roll" class="telemetry-value">0.0°</div>
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

    <div style="max-width: 600px; margin: 20px auto 40px auto; padding: 0 10px;">
        <button onclick="location.href='/'" class="btn" style="background-color: var(--card-bg); border: 1px solid var(--border); color: var(--text-main); margin-bottom: 0;">➔ Back to Main Dashboard</button>
    </div>

    <script>
        let ws;
        let wsConnected = false;
        let activeTab = 'dashboard';

        let headingRaw = 0, headingHard = 0, headingSoft = 0, headingOpt3 = 0, headingTilt = 0;
        let rotRaw = 0, rotHard = 0, rotSoft = 0, rotOpt3 = 0, rotTilt = 0;
        let currentRoll = 0, currentPitch = 0;

        let scene, camera, renderer, trackerMesh;
        let scene2, camera2, renderer2, trackerMesh2;

        function getShortestRotation(current, target) {
            target = (target % 360 + 360) % 360;
            let currentNorm = (current % 360 + 360) % 360;
            let diff = target - currentNorm;
            if (diff > 180) diff -= 360;
            if (diff < -180) diff += 360;
            return current + diff;
        }

        window.addEventListener('load', () => {
            initThreeJS();
            initWebSockets();
        });

        function initWebSockets() {
            document.getElementById('status-dot').className = 'status-dot connected';
            document.getElementById('status-text').innerText = 'Polling Active';

            function poll() {
                fetch('/data')
                    .then(response => response.json())
                    .then(data => {
                        headingRaw = data.raw;
                        headingHard = data.hard;
                        headingSoft = data.hardSoft;
                        headingOpt3 = data.opt3;
                        // Use direct clockwise heading from ESP32 (offset-free)
                        headingTilt = data.icm_no_offset;
                        let rollVal = Number(data.roll !== undefined ? data.roll : (data.ir || 0));
                        let pitchVal = Number(data.pitch !== undefined ? data.pitch : (data.ip || 0));
                        if (isNaN(rollVal)) rollVal = 0;
                        if (isNaN(pitchVal)) pitchVal = 0;
                        currentRoll = rollVal;
                        currentPitch = pitchVal;

                        if (data.cal_load !== undefined) {
                            document.getElementById('cal-load-text').innerText = data.cal_load;
                        }

                        document.getElementById('val-rose-raw').innerText = Math.round(headingRaw).toString().padStart(3, '0') + '°';
                        document.getElementById('val-rose-hard').innerText = Math.round(headingHard).toString().padStart(3, '0') + '°';
                        document.getElementById('val-rose-soft').innerText = Math.round(headingSoft).toString().padStart(3, '0') + '°';
                        document.getElementById('val-rose-opt3').innerText = Math.round(headingOpt3).toString().padStart(3, '0') + '°';
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
                        rotOpt3 = getShortestRotation(rotOpt3, headingOpt3);
                        document.getElementById('rose-opt3-needle').style.transform = `rotate(${rotOpt3}deg)`;
                        rotTilt = getShortestRotation(rotTilt, headingTilt);
                        document.getElementById('rose-tilt-needle').style.transform = `rotate(${rotTilt}deg)`;

                        if (trackerMesh) {
                            const yawRad = -headingTilt * Math.PI / 180;
                            const pitchRad = currentPitch * Math.PI / 180;
                            const rollRad = -currentRoll * Math.PI / 180;
                            trackerMesh.rotation.order = 'YXZ';
                            trackerMesh.rotation.set(pitchRad, yawRad, rollRad);
                        }

                        // Update Attitude Bubble Level indicator
                        const bubble = document.getElementById('attitude-bubble');
                        if (bubble) {
                            const scale = 3.0; // 3 pixels per degree of tilt
                            let dx = currentRoll * scale;
                            let dy = -currentPitch * scale; // Invert pitch to make dot move UP when nose is pitched UP
                            
                            const dist = Math.sqrt(dx*dx + dy*dy);
                            if (dist > 80) { // Keep dot within safety circle boundary
                                dx = (dx / dist) * 80;
                                dy = (dy / dist) * 80;
                            }
                            bubble.setAttribute('cx', (100 + dx).toFixed(1));
                            bubble.setAttribute('cy', (100 + dy).toFixed(1));
                        }
                        const gpVal = document.getElementById('gauge-pitch-val');
                        if (gpVal) gpVal.innerText = currentPitch.toFixed(1) + '°';
                        const grVal = document.getElementById('gauge-roll-val');
                        if (grVal) grVal.innerText = currentRoll.toFixed(1) + '°';

                        if (trackerMesh2) {
                            const pitchRad = currentPitch * Math.PI / 180;
                            const rollRad = -currentRoll * Math.PI / 180;
                            const yawRad = 0; // Lock Yaw to 0!
                            trackerMesh2.rotation.order = 'YXZ';
                            trackerMesh2.rotation.set(pitchRad, yawRad, rollRad);
                        }

                        // Schedule next poll - constant 100ms when idle/telemetry viewing
                        setTimeout(poll, 100);
                    })
                    .catch(e => {
                        console.error("Poll error on dashboard:", e);
                        document.getElementById('status-dot').className = 'status-dot';
                        document.getElementById('status-text').innerText = 'Polling Error';
                        setTimeout(poll, 2000);
                    });
            }
            poll();
        }

        function initThreeJS() {
            // --- First visualizer (Full 3D) ---
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

            // --- Second visualizer (Pitch & Roll Only Cube) ---
            const container2 = document.getElementById('visualizer3d-pitchroll');
            if (container2) {
                scene2 = new THREE.Scene();
                scene2.background = new THREE.Color('#0b0f19');
                camera2 = new THREE.PerspectiveCamera(45, container2.clientWidth / container2.clientHeight, 0.1, 100);
                camera2.position.set(3, 2, 3);
                camera2.lookAt(0, 0, 0);

                renderer2 = new THREE.WebGLRenderer({ antialias: true });
                renderer2.setSize(container2.clientWidth, container2.clientHeight);
                container2.appendChild(renderer2.domElement);

                scene2.add(new THREE.AmbientLight(0xffffff, 0.6));
                const dl2 = new THREE.DirectionalLight(0xffffff, 0.8);
                dl2.position.set(5, 8, 5);
                scene2.add(dl2);

                const grid2 = new THREE.GridHelper(4, 10, 0xa855f7, 0x1e293b); // Purple grid for distinction!
                grid2.position.y = -1;
                scene2.add(grid2);

                trackerMesh2 = new THREE.Group();
                const materials = [
                    new THREE.MeshLambertMaterial({ color: 0xef4444 }), // Front (Red)
                    new THREE.MeshLambertMaterial({ color: 0x3b82f6 }), // Back (Blue)
                    new THREE.MeshLambertMaterial({ color: 0x10b981 }), // Top (Green)
                    new THREE.MeshLambertMaterial({ color: 0xf59e0b }), // Bottom (Orange)
                    new THREE.MeshLambertMaterial({ color: 0xbc8cff }), // Left (Purple)
                    new THREE.MeshLambertMaterial({ color: 0x06b6d4 })  // Right (Cyan)
                ];
                const cube = new THREE.Mesh(new THREE.BoxGeometry(1.2, 1.2, 1.2), materials);
                trackerMesh2.add(cube);
                trackerMesh2.add(new THREE.ArrowHelper(new THREE.Vector3(0, 0, -1), new THREE.Vector3(0, 0.61, 0), 1.0, 0xffffff));
                scene2.add(trackerMesh2);
            }

            window.addEventListener('resize', () => {
                camera.aspect = container.clientWidth / container.clientHeight;
                camera.updateProjectionMatrix();
                renderer.setSize(container.clientWidth, container.clientHeight);

                if (container2) {
                    camera2.aspect = container2.clientWidth / container2.clientHeight;
                    camera2.updateProjectionMatrix();
                    renderer2.setSize(container2.clientWidth, container2.clientHeight);
                }
            });

            function animate() {
                requestAnimationFrame(animate);
                renderer.render(scene, camera);
                if (renderer2) {
                    renderer2.render(scene2, camera2);
                }
            }
            animate();
        }
    </script>
</body>
</html>
)rawliteral";

#endif
