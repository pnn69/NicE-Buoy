#ifndef CALIBRATION_HTML_H
#define CALIBRATION_HTML_H

const char CALIBRATION_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Magnetometer Calibration - ESP32 Compass</title>
    <style>
        :root {
            --bg-color: #0d1117;
            --card-bg: #161b22;
            --border-color: #30363d;
            --text-main: #c9d1d9;
            --text-muted: #8b949e;
            --primary: #58a6ff;
            --success: #3fb950;
            --warning: #d29922;
            --danger: #f85149;
            --accent: #bc8cff;
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Helvetica, Arial, sans-serif;
            background-color: var(--bg-color);
            color: var(--text-main);
            padding: 20px;
            min-height: 100vh;
        }

        header {
            text-align: center;
            margin-bottom: 30px;
            padding-bottom: 15px;
            border-bottom: 1px solid var(--border-color);
        }

        header h1 {
            font-size: 24px;
            color: var(--warning);
            font-weight: 600;
            letter-spacing: 0.5px;
        }

        header p {
            color: var(--text-muted);
            font-size: 14px;
            margin-top: 5px;
        }

        .status-badge {
            display: inline-block;
            padding: 4px 10px;
            border-radius: 12px;
            font-size: 12px;
            font-weight: bold;
            margin-top: 10px;
            background-color: #21262d;
            border: 1px solid var(--border-color);
        }

        .status-badge.connected {
            color: var(--success);
            border-color: rgba(63, 185, 80, 0.3);
            background-color: rgba(63, 185, 80, 0.1);
        }

        .status-badge.disconnected {
            color: var(--danger);
            border-color: rgba(248, 81, 73, 0.3);
            background-color: rgba(248, 81, 73, 0.1);
        }

        .container {
            max-width: 600px;
            margin: 0 auto;
        }

        .card {
            background-color: var(--card-bg);
            border: 1px solid var(--border-color);
            border-radius: 8px;
            padding: 20px;
            box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: flex-start;
            margin-bottom: 20px;
        }

        .card h2 {
            font-size: 18px;
            color: var(--text-main);
            margin-bottom: 20px;
            align-self: flex-start;
            border-left: 3px solid var(--warning);
            padding-left: 10px;
            width: 100%;
        }

        /* Buttons & Controls */
        .btn {
            display: inline-flex;
            align-items: center;
            justify-content: center;
            padding: 12px 18px;
            font-size: 14px;
            font-weight: 600;
            border-radius: 6px;
            cursor: pointer;
            transition: background-color 0.2s, border-color 0.2s;
            border: 1px solid transparent;
            width: 100%;
            margin-bottom: 12px;
        }

        .btn-primary {
            background-color: #238636;
            color: white;
        }
        .btn-primary:hover { background-color: #2ea043; }

        .btn-danger {
            background-color: #da3633;
            color: white;
        }
        .btn-danger:hover { background-color: #f85149; }

        .btn-secondary {
            background-color: #21262d;
            color: var(--text-main);
            border-color: var(--border-color);
        }
        .btn-secondary:hover {
            background-color: #30363d;
            border-color: #8b949e;
        }

        .btn:disabled {
            opacity: 0.5;
            cursor: not-allowed;
        }

        /* Calibration Panel Specifics */
        .cal-points-visual {
            width: 100%;
            height: 240px;
            background-color: #0d1117;
            border-radius: 6px;
            border: 1px solid var(--border-color);
            margin-bottom: 15px;
            position: relative;
            overflow: hidden;
        }

        .cal-point-count {
            position: absolute;
            top: 10px;
            right: 15px;
            font-size: 12px;
            color: var(--text-muted);
            font-weight: bold;
        }

        .cal-stats-grid {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 10px;
            width: 100%;
            margin-bottom: 15px;
        }

        .cal-stat-card {
            background-color: rgba(255,255,255,0.02);
            border: 1px solid var(--border-color);
            border-radius: 6px;
            padding: 10px;
            font-size: 12px;
            text-align: center;
        }

        .cal-stat-title {
            color: var(--text-muted);
            margin-bottom: 4px;
            font-size: 11px;
            text-transform: uppercase;
        }

        .cal-stat-val {
            font-family: monospace;
            font-weight: bold;
            color: var(--warning);
        }

        /* Range Bars */
        .range-container {
            width: 100%;
            margin-bottom: 20px;
        }
        .range-bar-row {
            display: flex;
            align-items: center;
            font-size: 12px;
            margin-bottom: 8px;
        }
        .range-axis {
            width: 15px;
            font-weight: bold;
            color: var(--warning);
        }
        .range-track {
            flex-grow: 1;
            height: 10px;
            background-color: #21262d;
            border-radius: 5px;
            margin: 0 10px;
            position: relative;
            overflow: hidden;
        }
        .range-fill {
            position: absolute;
            height: 100%;
            background-color: var(--success);
            border-radius: 5px;
            left: 0%;
            width: 0%;
        }
        .range-vals {
            width: 85px;
            text-align: right;
            font-family: monospace;
            color: var(--text-muted);
        }

        .btn-nav {
            display: inline-flex;
            align-items: center;
            justify-content: center;
            padding: 12px 24px;
            font-size: 15px;
            font-weight: bold;
            border-radius: 6px;
            cursor: pointer;
            text-decoration: none;
            transition: background-color 0.2s, border-color 0.2s, transform 0.1s;
            border: 1px solid var(--border-color);
            background-color: #21262d;
            color: var(--text-main);
            width: 100%;
            text-align: center;
        }

        .btn-nav:hover {
            background-color: #30363d;
            border-color: #8b949e;
            transform: translateY(-1px);
        }

        .planar-warning {
            width: 100%;
            background-color: rgba(248, 81, 73, 0.15);
            border: 1px dashed var(--danger);
            border-radius: 6px;
            padding: 10px;
            margin-bottom: 15px;
            text-align: center;
            color: var(--danger);
            font-weight: bold;
            display: none;
            animation: pulse-warn 1.5s infinite;
        }

        @keyframes pulse-warn {
            0% { opacity: 0.8; }
            50% { opacity: 1; }
            100% { opacity: 0.8; }
        }

        .cal-hud {
            width: 100%;
            display: grid;
            grid-template-columns: 1fr 1fr 1fr 1fr;
            gap: 10px;
            margin-bottom: 15px;
        }

        .hud-card {
            background-color: rgba(255,255,255,0.02);
            border: 1px solid var(--border-color);
            border-radius: 6px;
            padding: 10px;
            text-align: center;
            font-size: 11px;
            color: var(--text-muted);
            text-transform: uppercase;
        }

        .hud-val {
            font-size: 16px;
            font-weight: bold;
            font-family: monospace;
            margin-top: 4px;
        }

        .hud-good { color: var(--success); }
        .hud-warn { color: var(--warning); }
        .hud-bad { color: var(--danger); }

        /* Responsive Mobile Layout (Requirement 2 refined) */
        @media (max-width: 600px) {
            .cal-stats-grid {
                grid-template-columns: 1fr;
            }
            .cal-hud {
                grid-template-columns: 1fr 1fr;
            }
        }

        /* Report Modal */
        .modal {
            position: fixed;
            top: 0; left: 0; width: 100%; height: 100%;
            background-color: rgba(0,0,0,0.8);
            display: none;
            align-items: center;
            justify-content: center;
            z-index: 10000;
            padding: 20px;
        }

        .modal-content {
            background-color: var(--card-bg);
            border: 1px solid var(--border-color);
            border-radius: 8px;
            padding: 24px;
            max-width: 450px;
            width: 100%;
            box-shadow: 0 10px 30px rgba(0,0,0,0.5);
            text-align: center;
        }

        .score-large {
            font-size: 72px;
            font-weight: 900;
            color: var(--warning);
            margin: 15px 0;
            line-height: 1.1;
        }
    </style>
</head>
<body>


    <!-- Co-Pilot Sticky Bottom Status and Hint Bar -->
    <div id="calibrationCoPilot" style="position: fixed; bottom: 0; left: 0; width: 100%; background: #161b22; border-top: 4px solid var(--warning); padding: 20px; z-index: 9999; display: none; flex-direction: column; align-items: center; justify-content: center; box-shadow: 0 -8px 20px rgba(0,0,0,0.6);">
        <!-- Large High-Visibility Percentages inside structured blocks -->
        <div id="coPilotStats" style="font-size: 24px; font-weight: 900; color: var(--text-main); font-family: monospace; margin-bottom: 12px; letter-spacing: 1px; text-align: center; width: 100%; display: flex; justify-content: space-around; flex-wrap: wrap; gap: 10px;">
            <div style="background:#0d1117; padding: 6px 12px; border-radius: 8px; border: 1px solid var(--border-color); min-width: 110px;">XY Cov: <span id="cpXY" style="color:var(--primary);">0%</span></div>
            <div style="background:#0d1117; padding: 6px 12px; border-radius: 8px; border: 1px solid var(--border-color); min-width: 110px; border-color: var(--accent);">XZ Cov: <span id="cpXZ" style="color:var(--accent);">0%</span></div>
            <div style="background:#0d1117; padding: 6px 12px; border-radius: 8px; border: 1px solid var(--border-color); min-width: 110px; border-color: var(--accent);">YZ Cov: <span id="cpYZ" style="color:var(--accent);">0%</span></div>
            <div style="background:#0d1117; padding: 6px 12px; border-radius: 8px; border: 1px solid var(--border-color); min-width: 110px; border-color: var(--success);">3D Cov: <span id="cp3D" style="color:var(--success);">0%</span></div>
        </div>
        <div id="coPilotHint" style="font-size: 20px; font-weight: bold; color: var(--warning); font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif; text-align: center; line-height: 1.4;">
            ➔ Hint: Waiting for sensor data stream...
        </div>
    </div>

    <div class="container" style="padding-bottom: 180px;"> <!-- Padding bottom to prevent co-pilot bar overlapping back button -->
        <header>
            <h1>Magnetometer Calibration</h1>
            <p>ESP32 ICM-20948 Hardware Calibration Tool</p>
            <div id="status" class="status-badge disconnected">Disconnected</div>
        </header>

        <div class="card">
            <h2 style="margin-bottom: 5px;">Interactive Scatter Plot Projections</h2>
            <div style="display: flex; justify-content: space-around; width: 100%; margin-bottom: 8px; font-size: 0.85rem; font-weight: bold; text-align: center;">
                <div style="flex: 1; color: var(--primary, #58a6ff);">XY (Horizontal)</div>
                <div style="flex: 1; color: var(--accent, #bc8cff);">XZ (Vertical Lateral)</div>
                <div style="flex: 1; color: var(--success, #56d364);">YZ (Vertical Frontal)</div>
            </div>

            <div class="planar-warning" id="planarWarning">⚠ Calibration is currently 2D only. Tilt the buoy up/down.</div>

            <div class="cal-hud" id="calHud">
                <div class="hud-card">
                    3D Coverage
                    <div class="hud-val" id="hudCov">0%</div>
                </div>
                <div class="hud-card">
                    Fit Quality (RMS)
                    <div class="hud-val" id="hudRms">--</div>
                </div>
                <div class="hud-card">
                    Matrix Condition
                    <div class="hud-val" id="hudCond">--</div>
                </div>
                <div class="hud-card">
                    Overall Score
                    <div class="hud-val" id="hudScore">0/100</div>
                </div>
            </div>

            <div class="cal-points-visual" id="calCanvas" style="padding: 0; background: none; border: none; height: 300px; position: relative;">
                <canvas id="scatterCanvas" style="width: 100%; height: 100%; border-radius: 6px; border: 1px solid var(--border-color); background: #0d1117; display: block;"></canvas>
                <div class="cal-point-count" id="pointCount" style="position: absolute; bottom: 10px; right: 15px; background: rgba(0,0,0,0.5); padding: 2px 6px; border-radius: 4px;">Points: 0</div>
            </div>

            <div class="cal-stats-grid">
                <div class="cal-stat-card">
                    <div class="cal-stat-title">Hard Iron Offset (HI)</div>
                    <div class="cal-stat-val" id="calHiVal">X: 0.0, Y: 0.0, Z: 0.0</div>
                </div>
                <div class="cal-stat-card">
                    <div class="cal-stat-title">Soft Iron Scaling (SI)</div>
                    <div class="cal-stat-val" id="calSiVal">X: 1.0, Y: 1.0, Z: 1.0</div>
                </div>
                <div class="cal-stat-card">
                    <div class="cal-stat-title">Soft Iron Matrix (W)</div>
                    <div class="cal-stat-val" id="calSiMatrix" style="font-size: 10px; line-height: 1.3; font-family: monospace; white-space: pre; margin-top: 3px;">[1.00, 0.00, 0.00]
[0.00, 1.00, 0.00]
[0.00, 0.00, 1.00]</div>
                </div>
            </div>

            <div class="range-container">
                <div class="range-bar-row">
                    <div class="range-axis">X</div>
                    <div class="range-track"><div id="rangeBarX" class="range-fill"></div></div>
                    <div class="range-vals" id="rangeValsX">[-0.0, 0.0]</div>
                </div>
                <div class="range-bar-row">
                    <div class="range-axis">Y</div>
                    <div class="range-track"><div id="rangeBarY" class="range-fill"></div></div>
                    <div class="range-vals" id="rangeValsY">[-0.0, 0.0]</div>
                </div>
                <div class="range-bar-row">
                    <div class="range-axis">Z</div>
                    <div class="range-track"><div id="rangeBarZ" class="range-fill"></div></div>
                    <div class="range-vals" id="rangeValsZ">[-0.0, 0.0]</div>
                </div>
            </div>

            <button class="btn btn-primary" id="btnCal">Start Interactive Calibration</button>
            <button class="btn btn-secondary" id="btnSave" disabled>Save &amp; Upload Calibration</button>
        </div>

        <div class="card">
            <h2>ICM Profile &amp; Persistence</h2>
            <div style="width:100%;margin-bottom:15px;text-align:left;">
                <div style="font-size:12px;font-family:monospace;word-break:break-all;color:var(--text-muted);background:#0d1117;padding:12px;border-radius:6px;border:1px solid var(--border-color);line-height:1.6;margin-bottom:12px;">
                    <strong>Loaded Profile (NVS):</strong> <span id="cal_load" style="color:var(--warning)">Loading...</span><br>
                    <strong>Calibration Status:</strong> <span id="cal_ver" style="color:var(--accent)">Loading...</span>
                </div>
                <div style="display:flex;gap:10px;width:100%;">
                    <button id="calButton" onclick="startCalib()" class="btn btn-secondary" style="margin-bottom:0;flex:1;">ICM Status</button>
                    <button onclick="saveCalib()" class="btn btn-primary" style="margin-bottom:0;flex:1;background-color:#2a6a2a;">Save Calib</button>
                </div>
            </div>
        </div>

        <div class="card">
            <h2>ICM Calibration Mode</h2>
            <div style="width:100%;text-align:left;font-size:13px;line-height:1.8;padding:5px;">
                <label style="display:flex;align-items:center;margin-bottom:12px;cursor:pointer;color:var(--text-main);font-weight:600;">
                    <input type="radio" name="icmMode" value="1" onclick="setIcmMode(1)" style="margin-right:12px;width:18px;height:18px;cursor:pointer;">
                    1. Only Hard Iron Correction (No Soft Iron, No Tilt)
                </label>
                <label style="display:flex;align-items:center;margin-bottom:12px;cursor:pointer;color:var(--text-main);font-weight:600;">
                    <input type="radio" name="icmMode" value="2" onclick="setIcmMode(2)" style="margin-right:12px;width:18px;height:18px;cursor:pointer;">
                    2. Hard and Soft Iron Correction (No Tilt)
                </label>
                <label style="display:flex;align-items:center;margin-bottom:12px;cursor:pointer;color:var(--text-main);font-weight:600;">
                    <input type="radio" name="icmMode" value="3" onclick="setIcmMode(3)" style="margin-right:12px;width:18px;height:18px;cursor:pointer;">
                    3. Hard Iron Correction &amp; Pitch &amp; Roll Tilt-Comp
                </label>
                <label style="display:flex;align-items:center;margin-bottom:12px;cursor:pointer;color:var(--text-main);font-weight:600;">
                    <input type="radio" name="icmMode" value="4" onclick="setIcmMode(4)" style="margin-right:12px;width:18px;height:18px;cursor:pointer;">
                    4. Hard &amp; Soft Iron &amp; Pitch &amp; Roll Tilt-Comp
                </label>
            </div>
        </div>

        <div style="margin-bottom: 40px;">
            <a href="/" class="btn-nav">➔ Back to Compass Dashboard</a>
        </div>
    </div>

    <!-- Calibration Grade Report Modal -->
    <div class="modal" id="reportModal">
        <div class="modal-content">
            <h2 style="color:var(--text-main); border:none; padding:0; text-align:center;">Calibration Quality Report</h2>
            <div class="score-large" id="reportGrade">A+</div>
            <div style="text-align:left; background:#0d1117; padding:15px; border-radius:6px; border:1px solid var(--border-color); font-family:monospace; font-size:13px; line-height:1.8; margin-bottom:20px; color:var(--text-main);">
                3D Coverage  : <span id="repCov" style="color:var(--success); font-weight:bold;">0%</span><br>
                Observability: <span id="repObs" style="color:var(--success); font-weight:bold;">0%</span><br>
                RMS Fit Error: <span id="repRms" style="color:var(--success); font-weight:bold;">0%</span><br>
                Condition No : <span id="repCond" style="color:var(--success); font-weight:bold;">1.0</span>
            </div>
            <button class="btn btn-primary" id="btnModalSave" style="margin-bottom:0;">Upload &amp; Save Calibration</button>
        </div>
    </div>

    <script>
        // Mathematical Observability Threshold Constants
        const PLANAR_RATIO = 0.15;
        const GOOD_RATIO = 0.20;

        let isCalibrating = false;
        let calPoints = [];
        let maxPoints = 1500; // Optimal performance cache buffer
        let pollTimer = null;
        let icmModeLoaded = false;
        let enoughPointsMet = false;
        let calTimeoutTimer = null;
        let lastCoverageIncreaseTime = 0;

        // Throttling drawPoints() via requestAnimationFrame
        let drawPending = false;

        // Throttling DOM reads/writes to reduce paint and layout reflow overhead
        let lastHudUpdate = 0;

        // 3D Spherical Coverage Grid (16 Azimuth bins x 8 Elevation bins)
        const AZ_BINS = 16;
        const EL_BINS = 8;
        let globalSphereGrid = Array(AZ_BINS).fill(0).map(() => Array(EL_BINS).fill(false));
        let binCounts = Array(AZ_BINS).fill(0).map(() => Array(EL_BINS).fill(0)); // 20 points/bin deterministic tracking (Requirement 2)

        // 2D Angular Occupancy Arrays (12 sectors of 30 degrees each)
        let globalCovXY = Array(12).fill(false);
        let globalCovXZ = Array(12).fill(false);
        let globalCovYZ = Array(12).fill(false);

        // Bounding limits for calibration solver
        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;
        let minZ = Infinity, maxZ = -Infinity;

        // Stable EWMA magnitude tracking for disturbance rejection (Requirement 5)
        let magAvg = null;

        // Keep track of the last 5 fits for adaptive completion
        let lastFiveRms = [];

        // Cached observability metrics to prevent continuous O(N) calculations on every point
        let currentObs = { ok: false, ratio: 0.0, cond: 15.0 };

        // Physical display radius dynamically mapped to fitted sphere field strength (Requirement 3 refined)
        let displayRadius = 50.0;

        // Final calibration parameters
        let hix = 0.0, hiy = 0.0, hiz = 0.0;
        let six = 1.0, siy = 1.0, siz = 1.0;
        let siMatrix = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]
        ];

        const statusBadge = document.getElementById('status');
        const btnCal = document.getElementById('btnCal');
        const btnSave = document.getElementById('btnSave');
        const pointCount = document.getElementById('pointCount');
        const calHiVal = document.getElementById('calHiVal');
        const calSiVal = document.getElementById('calSiVal');
        const calSiMatrix = document.getElementById('calSiMatrix');

        const rangeBarX = document.getElementById('rangeBarX');
        const rangeBarY = document.getElementById('rangeBarY');
        const rangeBarZ = document.getElementById('rangeBarZ');
        const rangeValsX = document.getElementById('rangeValsX');
        const rangeValsY = document.getElementById('rangeValsY');
        const rangeValsZ = document.getElementById('rangeValsZ');

        const planarWarning = document.getElementById('planarWarning');
        const hudCov = document.getElementById('hudCov');
        const hudRms = document.getElementById('hudRms');
        const hudCond = document.getElementById('hudCond');
        const hudScore = document.getElementById('hudScore');

        function connect() {
            statusBadge.textContent = 'Connected';
            statusBadge.className = 'status-badge connected';
            pollData();
        }

        function pollData() {
            fetch('/data')
                .then(r => r.json())
                .then(data => {
                    if (isCalibrating) {
                        if (data.points !== undefined && Array.isArray(data.points)) {
                            data.points.forEach(p => processRawPoint(p[0], p[1], p[2]));
                        } else if (data.mx_raw !== undefined && data.my_raw !== undefined && data.mz_raw !== undefined) {
                            processRawPoint(data.mx_raw, data.my_raw, data.mz_raw);
                        }
                    }
                    
                    if (data.cal_load !== undefined) {
                        document.getElementById('cal_load').textContent = data.cal_load;
                    }
                    if (data.cal_ver !== undefined) {
                        document.getElementById('cal_ver').textContent = data.cal_ver;
                    }

                    if (data.icm_mode !== undefined && !icmModeLoaded) {
                        const radios = document.getElementsByName('icmMode');
                        for (let i = 0; i < radios.length; i++) {
                            if (parseInt(radios[i].value) === data.icm_mode) {
                                radios[i].checked = true;
                                icmModeLoaded = true;
                                break;
                            }
                        }
                    }

                    const b = document.getElementById('calButton');
                    if (b) {
                        b.style.backgroundColor = '#2a6a2a';
                        b.style.color = '#fff';
                        b.innerText = 'ICM Active';
                    }

                    // Relaxed polling interval to reduce ESP32 serving stress
                    const interval = isCalibrating ? 150 : 500;
                    pollTimer = setTimeout(pollData, interval);
                })
                .catch(err => {
                    console.error("Poll error", err);
                    const interval = isCalibrating ? 1000 : 2000;
                    pollTimer = setTimeout(pollData, interval);
                });
        }

        function processRawPoint(x, y, z) {
            if (x === 0 && y === 0 && z === 0) return;
            if (Math.abs(x) > 1000 || Math.abs(y) > 1000 || Math.abs(z) > 1000) return;

            // Stable EWMA magnitude disturbance rejection (Requirement 5)
            let mag = Math.sqrt(x*x + y*y + z*z);
            if (magAvg === null) {
                magAvg = mag;
            } else {
                if (Math.abs(mag - magAvg) / magAvg > 0.25) {
                    // Slowly leak the EWMA towards the current mag to allow eventual recovery without locks (Requirement 2 refined)
                    magAvg = 0.999 * magAvg + 0.001 * mag;
                    return; // Reject points that spike away from EWMA
                }
                magAvg = 0.98 * magAvg + 0.02 * mag; // Dynamically track slow changes
            }

            // Update bounding limits first to fix midpoint calculation ordering (Requirement 4)
            minX = Math.min(minX, x); maxX = Math.max(maxX, x);
            minY = Math.min(minY, y); maxY = Math.max(maxY, y);
            minZ = Math.min(minZ, z); maxZ = Math.max(maxZ, z);

            // Center Validation for Coverage Metrics (Requirement 3):
            // Use midpoint center during first 200 points, then use ellipsoid center
            let cx_offset = hix;
            let cy_offset = hiy;
            let cz_offset = hiz;
            if (calPoints.length < 200) {
                cx_offset = (maxX + minX) / 2.0;
                cy_offset = (maxY + minY) / 2.0;
                cz_offset = (maxZ + minZ) / 2.0;
                if (!isFinite(cx_offset)) {
                    cx_offset = 0; cy_offset = 0; cz_offset = 0;
                }
            }

            let dx = x - cx_offset;
            let dy = y - cy_offset;
            let dz = z - cz_offset;
            let magNorm = Math.sqrt(dx*dx + dy*dy + dz*dz) || 1.0;

            let azimuth = Math.atan2(dy, dx) * 180 / Math.PI;
            if (azimuth < 0) azimuth += 360;

            // Equal-Area Spherical Latitude Bands mapping to prevent pole bias (Requirement 1)
            let u = (dz / magNorm + 1.0) / 2.0; // Uniform 0.0 to 1.0 mapping
            let azBin = Math.floor(azimuth / (360 / AZ_BINS)) % AZ_BINS;
            let elBin = Math.floor(u * EL_BINS);
            elBin = Math.max(0, Math.min(EL_BINS - 1, elBin));

            // Deterministic Point Thinning (Requirement 2):
            // Accept exactly the first 20 samples per 3D bin, reject everything after.
            let isNewBin = (binCounts[azBin][elBin] === 0);
            if (binCounts[azBin][elBin] >= 20) {
                return; // Unconditionally reject to maintain perfect determinism
            }
            binCounts[azBin][elBin]++;
            globalSphereGrid[azBin][elBin] = true;

            // Map to 2D occupancy arrays
            let xyAngle = Math.atan2(dy, dx) * 180 / Math.PI; if (xyAngle < 0) xyAngle += 360;
            let xzAngle = Math.atan2(dz, dx) * 180 / Math.PI; if (xzAngle < 0) xzAngle += 360;
            let yzAngle = Math.atan2(dz, dy) * 180 / Math.PI; if (yzAngle < 0) yzAngle += 360;

            globalCovXY[Math.floor(xyAngle / 30) % 12] = true;
            globalCovXZ[Math.floor(xzAngle / 30) % 12] = true;
            globalCovYZ[Math.floor(yzAngle / 30) % 12] = true;

            calPoints.push({x, y, z});

            // Accurate No-Improvement Timeout (Requirement 6):
            // Only reset timeout when we discover a brand new, previously unoccupied bin
            if (isNewBin) {
                lastCoverageIncreaseTime = Date.now();
            }

            if (calPoints.length > maxPoints) {
                calPoints.shift();
            }

            // Dynamic Fitting Update Rate (relaxed fitting update rate)
            let updateInterval = 50;
            if (calPoints.length < 500) updateInterval = 50;
            else if (calPoints.length < 1500) updateInterval = 100;
            else updateInterval = 200;

            if (calPoints.length >= 50 && (calPoints.length % updateInterval) === 0) {
                runEllipsoidFitting();
            }

            // Throttled DOM HUD writes to twice a second
            const nowTime = Date.now();
            if (nowTime - lastHudUpdate > 250) {
                lastHudUpdate = nowTime;

                // Robust Planar Detection (Requirement 4): Use cached mathematical observability
                if (calPoints.length > 100 && currentObs.ratio < PLANAR_RATIO) {
                    planarWarning.style.display = 'block';
                } else {
                    planarWarning.style.display = 'none';
                }

                // Compute Coverage percentages
                let filledXY = globalCovXY.filter(Boolean).length;
                let filledXZ = globalCovXZ.filter(Boolean).length;
                let filledYZ = globalCovYZ.filter(Boolean).length;
                let filled3D = 0;
                for (let a = 0; a < AZ_BINS; a++) {
                    for (let e = 0; e < EL_BINS; e++) {
                        if (globalSphereGrid[a][e]) filled3D++;
                    }
                }

                const xyPct = Math.round((filledXY / 12) * 100);
                const xzPct = Math.round((filledXZ / 12) * 100);
                const yzPct = Math.round((filledYZ / 12) * 100);
                const cov3DPct = Math.round((filled3D / (AZ_BINS * EL_BINS)) * 100);

                // Format real-time 3D coverage telemetry
                const statText = `Points: ${calPoints.length} | XY: ${xyPct}% | XZ: ${xzPct}% | YZ: ${yzPct}% | 3D Cov: ${cov3DPct}%`;
                pointCount.textContent = statText; // Use textContent instead of innerHTML

                // Update HUD values
                hudCov.textContent = `${cov3DPct}%`;
                document.getElementById('cpXY').textContent = xyPct + '%';
                document.getElementById('cpXZ').textContent = xzPct + '%';
                document.getElementById('cpYZ').textContent = yzPct + '%';
                document.getElementById('cp3D').textContent = cov3DPct + '%';

                // Select Actionable Hints based on missing data sectors (Requirement 5)
                let hint = "➔ Hint: Rotating buoy...";
                if (xyPct < 100) {
                    hint = "➔ Rotate the buoy clockwise in a full flat circle.";
                } else if (currentObs.ratio < PLANAR_RATIO) {
                    // Intuitively handle 2D flat runs without confusion! (Requirement 12)
                    hint = "➔ Flat 2D complete! Tap 'Stop & Save' below, or tilt/roll in 3D to expand advanced vertical calibration.";
                } else if (xzPct < 100) {
                    hint = "➔ Tilt the buoy's nose slowly upward and downward.";
                } else if (yzPct < 100) {
                    hint = "➔ Roll the buoy's left and right sides toward the sky.";
                } else if (cov3DPct < 85) {
                    hint = "➔ Perform slow figure-8 roll and pitch maneuvers to fill the 3D spherical gaps.";
                } else {
                    hint = "➔ Calibration complete! Tap Stop above to view the final report card.";
                }
                document.getElementById('coPilotHint').textContent = hint;

                // Check flat change over last 5 fits for convergence
                let isConverged = false;
                if (lastFiveRms.length >= 5) {
                    let maxRms = Math.max(...lastFiveRms);
                    let minRms = Math.min(...lastFiveRms);
                    if (maxRms - minRms < 0.2 && cov3DPct >= 80) {
                        isConverged = true;
                    }
                }

                // Live Quality Score calculation with rebalanced weightings & planar capping (Requirement 9 refined)
                let rms_val = lastFiveRms.length > 0 ? lastFiveRms[lastFiveRms.length - 1] : 99.0;
                let cond_val = currentObs.cond;
                if (!isFinite(cond_val)) cond_val = 15.0;

                let covScore = cov3DPct * 0.4; // Up to 40 pts (40%)
                
                // Piece-wise professional marine grading curve for RMS score (Requirement 8 refined)
                let rmsScore = 40.0; // Max 40% (40 pts)
                if (rms_val > 4.0) {
                    rmsScore = Math.max(0, 100 - (rms_val - 4.0) * 4.0) * 0.4;
                }
                
                let condScore = Math.max(0, 100 - (cond_val - 1.0) * 10) * 0.2; // Up to 20 pts (20% - 0 at Condition 11)
                let overallScore = Math.round(covScore + rmsScore + condScore);
                
                // Forcefully cap score at 60 if planar (Requirement 3 refined)
                if (calPoints.length > 100 && currentObs.ratio < PLANAR_RATIO) {
                    overallScore = Math.min(60, overallScore);
                }
                overallScore = Math.max(0, Math.min(100, overallScore));

                // Format HUD indicators
                if (rms_val > 50.0 || isNaN(rms_val)) {
                    hudRms.textContent = "PLANAR";
                    hudRms.className = "hud-val hud-bad";
                } else {
                    hudRms.textContent = `${rms_val.toFixed(2)}%`;
                    hudRms.className = "hud-val " + (rms_val < 4.0 ? "hud-good" : (rms_val < 8.0 ? "hud-warn" : "hud-bad"));
                }

                hudCond.textContent = isFinite(currentObs.cond) ? currentObs.cond.toFixed(1) : "PLANAR";
                hudCond.className = "hud-val " + (currentObs.cond <= 7.0 ? "hud-good" : (currentObs.cond <= 15.0 ? "hud-warn" : "hud-bad"));

                hudScore.textContent = `${overallScore}/100`;
                hudScore.className = "hud-val " + (overallScore >= 80 ? "hud-good" : (overallScore >= 60 ? "hud-warn" : "hud-bad"));

                // Calibration stops automatically when optimal parameters are locked
                // Dynamically allow flat 2D runs to successfully auto-complete so users don't get stuck! (Requirement 12)
                const isPlanarTargetMet = (currentObs.ratio < PLANAR_RATIO && xyPct >= 95 && calPoints.length >= 150);
                const isTargetMet = (calPoints.length >= 500 && cov3DPct >= 85 && currentObs.ratio >= GOOD_RATIO && isConverged) || isPlanarTargetMet;
                const isNoImprovementTimeout = (calPoints.length >= 400 && cov3DPct >= 80 && (Date.now() - lastCoverageIncreaseTime > 45000));

                if ((isTargetMet || isNoImprovementTimeout) && !enoughPointsMet) {
                    enoughPointsMet = true;
                    btnCal.textContent = '➔ Stop & Save Calibration';
                    btnCal.style.backgroundColor = 'var(--success)';
                    btnCal.style.borderColor = 'var(--success)';
                    btnCal.style.color = '#fff';
                }

                // Render stats
                calHiVal.textContent = `X: ${hix.toFixed(1)}, Y: ${hiy.toFixed(1)}, Z: ${hiz.toFixed(1)}`;

                // Update Range Bars
                updateRangeBar(rangeBarX, rangeValsX, minX, maxX, x);
                updateRangeBar(rangeBarY, rangeValsY, minY, maxY, y);
                updateRangeBar(rangeBarZ, rangeValsZ, minZ, maxZ, z);
            }

            // Sync redraw to browser refresh rate instead of every point
            requestDraw();
        }

        function updateRangeBar(barElement, valElement, min, max, cur) {
            valElement.textContent = `[${min.toFixed(0)}, ${max.toFixed(0)}]`;
            const range = max - min || 1;
            const pct = ((cur - min) / range) * 100;
            barElement.style.width = `${Math.max(2, Math.min(100, pct))}%`;
            barElement.style.left = `0%`;
        }

        function requestDraw() {
            if (drawPending) return;
            drawPending = true;
            requestAnimationFrame(() => {
                drawPoints();
                drawPending = false;
            });
        }

        function drawPoints() {
            const canvas = document.getElementById('scatterCanvas');
            if (!canvas) return;
            const ctx = canvas.getContext('2d');
            
            if (canvas.width !== canvas.clientWidth || canvas.height !== canvas.clientHeight) {
                canvas.width = canvas.clientWidth;
                canvas.height = canvas.clientHeight;
            }
            
            const w = canvas.width, h = canvas.height;

            ctx.fillStyle = '#0d1117';
            ctx.fillRect(0, 0, w, h);
            
            // Draw three dividers for XY, XZ, YZ plots (Requirement 10)
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
            ctx.lineWidth = 1.5;
            ctx.beginPath(); 
            ctx.moveTo(w/3, 0); ctx.lineTo(w/3, h);
            ctx.moveTo(2*w/3, 0); ctx.lineTo(2*w/3, h);
            ctx.stroke();

            if (calPoints.length === 0) return;

            const pad = 20;
            const dMinX = minX - pad, dMaxX = maxX + pad;
            const dMinY = minY - pad, dMaxY = maxY + pad;
            const dMinZ = minZ - pad, dMaxZ = maxZ + pad;

            const map = (v, min, max, outMin, outMax) => (v - min) * (outMax - outMin) / (max - min) + outMin;

            // 100x Speedup: Single Path Drawing per plane (Copilot suggestion 3/masterclass)
            // Grouping arcs by plane and filling ONLY once per color group drops drawing stress to near 0ms!
            
            // 1. Draw Left Plot: X vs Y Projection (Horizontal Plane)
            ctx.fillStyle = 'rgba(88, 166, 255, 0.75)'; // Theme Blue
            ctx.beginPath();
            calPoints.forEach(pt => {
                const xPos = map(pt.x, dMinX, dMaxX, 15, w/3 - 15);
                const yPos = map(pt.y, dMinY, dMaxY, h - 15, 15);
                ctx.moveTo(xPos + 2.5, yPos);
                ctx.arc(xPos, yPos, 2.5, 0, 2 * Math.PI);
            });
            ctx.fill();

            // 2. Draw Middle Plot: X vs Z Projection (Vertical Lateral Plane)
            ctx.fillStyle = 'rgba(188, 140, 255, 0.75)'; // Theme Purple
            ctx.beginPath();
            calPoints.forEach(pt => {
                const xPos = map(pt.x, dMinX, dMaxX, w/3 + 15, 2*w/3 - 15);
                const yPos = map(pt.z, dMinZ, dMaxZ, h - 15, 15);
                ctx.moveTo(xPos + 2.5, yPos);
                ctx.arc(xPos, yPos, 2.5, 0, 2 * Math.PI);
            });
            ctx.fill();

            // 3. Draw Right Plot: Y vs Z Projection (Vertical Frontal Plane)
            ctx.fillStyle = 'rgba(86, 211, 100, 0.75)'; // Theme Green
            ctx.beginPath();
            calPoints.forEach(pt => {
                const xPos = map(pt.y, dMinY, dMaxY, 2*w/3 + 15, w - 15);
                const yPos = map(pt.z, dMinZ, dMaxZ, h - 15, 15);
                ctx.moveTo(xPos + 2.5, yPos);
                ctx.arc(xPos, yPos, 2.5, 0, 2 * Math.PI);
            });
            ctx.fill();

            // Draw Mathematically Exact Rotated 3D Ellipsoid Projections (Requirement 5 refined)
            let invW = invert3x3(siMatrix);
            if (!invW) {
                invW = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]];
            }

            ctx.lineWidth = 1.0;
            ctx.setLineDash([4, 4]);
            ctx.strokeStyle = 'rgba(212, 153, 34, 0.45)'; // Amber reference
            const R = displayRadius; // Use dynamically fitted physical field strength! (Requirement 3 refined)

            // XY Projection Ellipse Drawing
            ctx.beginPath();
            for (let i = 0; i <= 72; i++) {
                const theta = (i * 5) * Math.PI / 180;
                const px = R * Math.cos(theta);
                const py = R * Math.sin(theta);
                const pz = 0;
                
                // Project via inverse Soft-Iron shape matrix (W^-1) and add offset
                const ex = (invW[0][0] * px + invW[0][1] * py + invW[0][2] * pz) + hix;
                const ey = (invW[1][0] * px + invW[1][1] * py + invW[1][2] * pz) + hiy;
                
                const canvasX = map(ex, dMinX, dMaxX, 15, w/3 - 15);
                const canvasY = map(ey, dMinY, dMaxY, h - 15, 15);
                if (i === 0) ctx.moveTo(canvasX, canvasY);
                else ctx.lineTo(canvasX, canvasY);
            }
            ctx.stroke();

            // XZ Projection Ellipse Drawing
            ctx.beginPath();
            for (let i = 0; i <= 72; i++) {
                const theta = (i * 5) * Math.PI / 180;
                const px = R * Math.cos(theta);
                const py = 0;
                const pz = R * Math.sin(theta);
                
                const ex = (invW[0][0] * px + invW[0][1] * py + invW[0][2] * pz) + hix;
                const ez = (invW[2][0] * px + invW[2][1] * py + invW[2][2] * pz) + hiz;
                
                const canvasX = map(ex, dMinX, dMaxX, w/3 + 15, 2*w/3 - 15);
                const canvasY = map(ez, dMinZ, dMaxZ, h - 15, 15);
                if (i === 0) ctx.moveTo(canvasX, canvasY);
                else ctx.lineTo(canvasX, canvasY);
            }
            ctx.stroke();

            // YZ Projection Ellipse Drawing
            ctx.beginPath();
            for (let i = 0; i <= 72; i++) {
                const theta = (i * 5) * Math.PI / 180;
                const px = 0;
                const py = R * Math.cos(theta);
                const pz = R * Math.sin(theta);
                
                const ey = (invW[1][0] * px + invW[1][1] * py + invW[1][2] * pz) + hiy;
                const ez = (invW[2][0] * px + invW[2][1] * py + invW[2][2] * pz) + hiz;
                
                const canvasX = map(ey, dMinY, dMaxY, 2*w/3 + 15, w - 15);
                const canvasY = map(ez, dMinZ, dMaxZ, h - 15, 15);
                if (i === 0) ctx.moveTo(canvasX, canvasY);
                else ctx.lineTo(canvasX, canvasY);
            }
            ctx.stroke();

            ctx.setLineDash([]); // Reset line dash

            // Draw center reticles
            ctx.strokeStyle = '#f85149'; // Theme Danger Red
            ctx.lineWidth = 1.5;
            
            // Reticle 1 (XY Center)
            const cenLeftX = map(hix, dMinX, dMaxX, 15, w/3 - 15);
            const cenLeftY = map(hiy, dMinY, dMaxY, h - 15, 15);
            ctx.beginPath(); ctx.arc(cenLeftX, cenLeftY, 5, 0, 2*Math.PI); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(cenLeftX - 8, cenLeftY); ctx.lineTo(cenLeftX + 8, cenLeftY); ctx.moveTo(cenLeftX, cenLeftY - 8); ctx.lineTo(cenLeftX, cenLeftY + 8); ctx.stroke();

            // Reticle 2 (XZ Center)
            const cenMidX = map(hix, dMinX, dMaxX, w/3 + 15, 2*w/3 - 15);
            const cenMidY = map(hiz, dMinZ, dMaxZ, h - 15, 15);
            ctx.beginPath(); ctx.arc(cenMidX, cenMidY, 5, 0, 2*Math.PI); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(cenMidX - 8, cenMidY); ctx.lineTo(cenMidX + 8, cenMidY); ctx.moveTo(cenMidX, cenMidY - 8); ctx.lineTo(cenMidX, cenMidY + 8); ctx.stroke();

            // Reticle 3 (YZ Center)
            const cenRightX = map(hiy, dMinY, dMaxY, 2*w/3 + 15, w - 15);
            const cenRightY = map(hiz, dMinZ, dMaxZ, h - 15, 15);
            ctx.beginPath(); ctx.arc(cenRightX, cenRightY, 5, 0, 2*Math.PI); ctx.stroke();
            ctx.beginPath(); ctx.moveTo(cenRightX - 8, cenRightY); ctx.lineTo(cenRightX + 8, cenRightY); ctx.moveTo(cenRightX, cenRightY - 8); ctx.lineTo(cenRightX, cenRightY + 8); ctx.stroke();
        }

        btnCal.onclick = () => {
            if (!isCalibrating) {
                isCalibrating = true;
                calPoints = [];
                minX = Infinity; maxX = -Infinity;
                minY = Infinity; maxY = -Infinity;
                minZ = Infinity; maxZ = -Infinity;
                enoughPointsMet = false;
                lastFiveRms = [];
                magAvg = null; // EWMA reset (Requirement 7 refined)
                currentObs = { ok: false, ratio: 0.0, cond: 15.0 }; // Reset observability
                displayRadius = 50.0; // Reset display radius
                lastHudUpdate = 0; // Reset DOM throttle

                // Holistic State Reset (Requirement 7): Reset all fit variables on start
                hix = hiy = hiz = 0.0;
                six = siy = siz = 1.0;
                siMatrix = [
                    [1.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0],
                    [0.0, 0.0, 1.0]
                ];

                globalCovXY.fill(false);
                globalCovXZ.fill(false);
                globalCovYZ.fill(false);
                for (let a = 0; a < AZ_BINS; a++) {
                    globalSphereGrid[a].fill(false);
                    binCounts[a].fill(0); // Reset deterministic thinning counts
                }
                lastCoverageIncreaseTime = Date.now();
                
                btnCal.textContent = 'Calibrating... (Keep Rotating)';
                btnCal.style.backgroundColor = '#6c757d';
                btnCal.style.borderColor = '#6c757d';
                btnCal.style.color = '#fff';
                btnSave.disabled = true;

                document.getElementById('calibrationCoPilot').style.display = 'flex';

                fetch('/start_cal').catch(err => console.warn("Failed to trigger start beep", err));

                if (calTimeoutTimer) clearTimeout(calTimeoutTimer);
                calTimeoutTimer = setTimeout(() => {
                    if (isCalibrating) {
                        console.log("Calibration safety timeout reached. Auto-saving...");
                        stopAndSaveCalibration();
                    }
                }, 180000); // 3 minutes safety timeout

                pollData();
            } else {
                stopAndSaveCalibration();
            }
        };

        function stopAndSaveCalibration() {
            isCalibrating = false;
            enoughPointsMet = false;
            if (calTimeoutTimer) { clearTimeout(calTimeoutTimer); calTimeoutTimer = null; }
            if (pollTimer) { clearTimeout(pollTimer); pollTimer = null; }
            document.getElementById('calibrationCoPilot').style.display = 'none';
            btnCal.textContent = 'Start Interactive Calibration';
            btnCal.style.backgroundColor = ''; btnCal.style.borderColor = ''; btnCal.style.color = '';
            
            if (calPoints.length >= 50) {
                runEllipsoidFitting();
                
                // Show final Quality scorecard grading modal (Requirement 8 refined)
                let rms = lastFiveRms.length > 0 ? lastFiveRms[lastFiveRms.length - 1] : 99.0;
                let cond = currentObs.cond;
                if (!isFinite(cond)) cond = 15.0;
                
                let filled3D = 0;
                for (let a = 0; a < AZ_BINS; a++) {
                    for (let e = 0; e < EL_BINS; e++) {
                        if (globalSphereGrid[a][e]) filled3D++;
                    }
                }
                const cov3DPct = Math.round((filled3D / (AZ_BINS * EL_BINS)) * 100);

                // Re-calculate the Overall Score for grading
                let covScore = cov3DPct * 0.4;
                
                // Piece-wise professional marine grading curve for RMS score (Requirement 8 refined)
                let rmsScore = 40.0;
                if (rms > 4.0) {
                    rmsScore = Math.max(0, 100 - (rms - 4.0) * 4.0) * 0.4;
                }
                
                let condScore = Math.max(0, 100 - (cond - 1.0) * 10) * 0.2;
                let overallScore = Math.round(covScore + rmsScore + condScore);
                
                // Forcefully cap score at 60 if planar (Requirement 3 refined)
                if (calPoints.length > 100 && currentObs.ratio < PLANAR_RATIO) {
                    overallScore = Math.min(60, overallScore);
                }
                overallScore = Math.max(0, Math.min(100, overallScore));

                // Relaxed report card grading based on realistic marine environments (Requirement 8)
                let grade = "F";
                let color = "var(--danger)";
                if (overallScore >= 90) { grade = "A+"; color = "var(--success)"; }
                else if (overallScore >= 80) { grade = "A"; color = "var(--success)"; }
                else if (overallScore >= 70) { grade = "B"; color = "var(--warning)"; }
                else if (overallScore >= 55) { grade = "C"; color = "var(--warning)"; }
                else if (overallScore >= 40) { grade = "D"; color = "var(--warning)"; }

                document.getElementById('reportGrade').textContent = grade + ` (${overallScore}/100)`;
                document.getElementById('reportGrade').style.color = color;
                document.getElementById('repCov').textContent = cov3DPct + "%";
                document.getElementById('repObs').textContent = (currentObs.ratio * 100).toFixed(1) + "%";
                document.getElementById('repRms').textContent = rms > 50.0 || isNaN(rms) ? "PLANAR (99%)" : rms.toFixed(2) + "%";
                document.getElementById('repCond').textContent = isFinite(cond) ? cond.toFixed(1) : "PLANAR";

                document.getElementById('reportModal').style.display = 'flex';
                pointCount.innerHTML = `<span style="color:var(--success); font-weight:bold;">Reviewing Report Card...</span>`;
            }
        }

        document.getElementById('btnModalSave').onclick = () => {
            document.getElementById('reportModal').style.display = 'none';
            btnSave.disabled = false;
            btnSave.click();
        };

        btnSave.onclick = () => {
            btnSave.textContent = "Saving...";
            btnSave.disabled = true;
            const url = `/save_cal?hx=${hix}&hy=${hiy}&hz=${hiz}` +
                        `&sx=${six}&sy=${siy}&sz=${siz}` +
                        `&sxx=${siMatrix[0][0]}&sxy=${siMatrix[0][1]}&sxz=${siMatrix[0][2]}` +
                        `&syx=${siMatrix[1][0]}&syy=${siMatrix[1][1]}&syz=${siMatrix[1][2]}` +
                        `&szx=${siMatrix[2][0]}&szy=${siMatrix[2][1]}&szz=${siMatrix[2][2]}`;
            fetch(url)
                .then(r => r.text())
                .then(text => {
                    if (text === "OK") {
                        btnSave.textContent = "Success!";
                        console.log("Calibration saved successfully!");
                        
                        document.getElementById('calibrationCoPilot').style.display = 'flex';
                        let redirectTime = 5;
                        const hintElem = document.getElementById('coPilotHint');
                        hintElem.innerHTML = `➔ Success! Calibration saved. Redirecting in <span id="redirCount" style="color:var(--warning); font-weight:bold;">${redirectTime}</span> seconds, or click "Back to Compass Dashboard" below.`;
                        hintElem.style.color = "var(--success)";

                        const redirInterval = setInterval(() => {
                            redirectTime--;
                            const redirCountSpan = document.getElementById('redirCount');
                            if (redirCountSpan) redirCountSpan.textContent = redirectTime;
                            if (redirectTime <= 0) {
                                clearInterval(redirInterval);
                                window.location.href = "/";
                            }
                        }, 1000);
                    } else {
                        btnSave.textContent = "Save Calibration";
                        btnSave.disabled = false;
                        console.error("Error saving calibration: " + text);
                    }
                })
                .catch(err => {
                    btnSave.textContent = "Save Calibration";
                    btnSave.disabled = false;
                    console.error("Network error: " + err);
                });
        };

        function setIcmMode(m) {
            fetch(`/set_icm_mode?mode=${m}`)
                .then(r => r.text())
                .then(txt => {
                    if (txt === "OK") {
                        console.log(`ICM Mode changed to: ${m}`);
                    } else {
                        alert("Error changing ICM mode: " + txt);
                    }
                })
                .catch(err => alert("Network error: " + err));
        }

        function startCalib() {
            alert('ICM-20948 operates automatically with high-stability Madgwick fusion.');
        }

        function saveCalib() {
            if (confirm('Save current ICM calibration profile to NVS?')) {
                fetch('/savecal')
                    .then(r => r.text())
                    .then(() => alert('NVS Save Command Sent!'));
            }
        }

        function runEllipsoidFitting() {
            if (calPoints.length < 50) return;
            
            const spanX = maxX - minX;
            const spanY = maxY - minY;
            if (calPoints.length < 150 || spanX < 30.0 || spanY < 30.0) { // Lower initial stable fitting required (Requirement 1)
                let minX_val = Infinity, maxX_val = -Infinity;
                let minY_val = Infinity, maxY_val = -Infinity;
                let minZ_val = Infinity, maxZ_val = -Infinity;
                calPoints.forEach(p => {
                    if (p.x < minX_val) minX_val = p.x;
                    if (p.x > maxX_val) maxX_val = p.x;
                    if (p.y < minY_val) minY_val = p.y;
                    if (p.y > maxY_val) maxY_val = p.y;
                    if (p.z < minZ_val) minZ_val = p.z;
                    if (p.z > maxZ_val) maxZ_val = p.z;
                });
                hix = (maxX_val + minX_val) / 2.0;
                hiy = (maxY_val + minY_val) / 2.0;
                hiz = (maxZ_val + minZ_val) / 2.0;

                six = 1.0; siy = 1.0; siz = 1.0;
                siMatrix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
                displayRadius = 50.0;
                
                calHiVal.textContent = `X: ${hix.toFixed(1)}, Y: ${hiy.toFixed(1)}, Z: ${hiz.toFixed(1)}`;
                calSiVal.textContent = `X: ${six.toFixed(2)}, Y: ${siy.toFixed(2)}, Z: ${siz.toFixed(2)} (Midpoint Fallback)`;
                hudRms.textContent = "Calculating...";
                hudCond.textContent = "Calculating...";
                drawPoints();
                return;
            }

            const pts = calPoints.map(p => [p.x, p.y, p.z]);
            
            // Re-compute and cache observability ONLY on the fitting cadence (Requirement 2 refined)
            // This drops CPU burden by >95% while keeping metrics accurate
            currentObs = evaluatePointsObservability(pts);

            const fit = fitEllipsoid(pts);
            if (fit) {
                hix = fit.offset[0];
                hiy = fit.offset[1];
                hiz = fit.offset[2];
                
                // Keep the exact, unaltered soft-iron matrix from solver! (Requirement 1 refined)
                siMatrix = fit.softIronMatrix;
                
                // Extract absolute diagonal elements for display only
                six = Math.abs(siMatrix[0][0]);
                siy = Math.abs(siMatrix[1][1]);
                siz = Math.abs(siMatrix[2][2]);
                
                // Use dynamically fitted field strength for ellipsoid projections! (Requirement 3 refined)
                displayRadius = fit.meanR || 50.0;
                
                calHiVal.textContent = `X: ${hix.toFixed(1)}, Y: ${hiy.toFixed(1)}, Z: ${hiz.toFixed(1)}`;
                calSiVal.textContent = `X: ${six.toFixed(2)}, Y: ${siy.toFixed(2)}, Z: ${siz.toFixed(2)}`;

                // Update full 3x3 soft iron matrix text element
                document.getElementById('calSiMatrix').textContent = 
                    `[${siMatrix[0][0].toFixed(2)}, ${siMatrix[0][1].toFixed(2)}, ${siMatrix[0][2].toFixed(2)}]\n` +
                    `[${siMatrix[1][0].toFixed(2)}, ${siMatrix[1][1].toFixed(2)}, ${siMatrix[1][2].toFixed(2)}]\n` +
                    `[${siMatrix[2][0].toFixed(2)}, ${siMatrix[2][1].toFixed(2)}, ${siMatrix[2][2].toFixed(2)}]`;

                // Update Quality scorecard in HUD (Requirement 6)
                hudRms.textContent = fit.rmsError > 50.0 || isNaN(fit.rmsError) ? "PLANAR" : `${fit.rmsError.toFixed(2)}%`;
                hudCond.textContent = isFinite(currentObs.cond) ? currentObs.cond.toFixed(1) : "PLANAR";

                // Style RMS error HUD based on fit quality
                hudRms.className = "hud-val " + (fit.rmsError < 4.0 ? "hud-good" : (fit.rmsError < 8.0 ? "hud-warn" : "hud-bad"));
                hudCond.className = "hud-val " + (currentObs.cond <= 7.0 ? "hud-good" : (currentObs.cond <= 15.0 ? "hud-warn" : "hud-bad"));
                
                // Save fit RMS for convergence checking
                lastFiveRms.push(fit.rmsError);
                if (lastFiveRms.length > 5) lastFiveRms.shift();

                drawPoints();
            }
        }

        function fitEllipsoid(points) {
            const N = points.length;
            if (N < 9) return null;

            let minZ_val = Infinity, maxZ_val = -Infinity;
            let minX_val = Infinity, maxX_val = -Infinity;
            let minY_val = Infinity, maxY_val = -Infinity;
            points.forEach(p => {
                if (p[2] < minZ_val) minZ_val = p[2];
                if (p[2] > maxZ_val) maxZ_val = p[2];
                if (p[0] < minX_val) minX_val = p[0];
                if (p[0] > maxX_val) maxX_val = p[0];
                if (p[1] < minY_val) minY_val = p[1];
                if (p[1] > maxY_val) maxY_val = p[1];
            });

            // Use sole covariance observability ratio for planar fallback (Requirement 4)
            if (currentObs.ratio < PLANAR_RATIO) {
                const hix_val = (maxX_val + minX_val) / 2.0;
                const hiy_val = (maxY_val + minY_val) / 2.0;
                const hiz_val = (maxZ_val + minZ_val) / 2.0;

                return {
                    offset: [hix_val, hiy_val, hiz_val],
                    softIronMatrix: [
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0]
                    ],
                    rmsError: 99.0, // Planar fallback reports high RMS to prevent score boosting (Requirement 3 refined)
                    meanR: 50.0
                };
            }

            let meanX = 0, meanY = 0, meanZ = 0;
            points.forEach(p => {
                meanX += p[0];
                meanY += p[1];
                meanZ += p[2];
            });
            meanX /= N; meanY /= N; meanZ /= N;

            const S = Array(10).fill(0).map(() => Array(10).fill(0));
            points.forEach(p => {
                const x = p[0] - meanX;
                const y = p[1] - meanY;
                const z = p[2] - meanZ;
                const r = [
                    x * x,
                    y * y,
                    z * z,
                    2.0 * x * y,
                    2.0 * x * z,
                    2.0 * y * z,
                    2.0 * x,
                    2.0 * y,
                    2.0 * z,
                    1.0
                ];
                for (let i = 0; i < 10; i++) {
                    for (let j = 0; j < 10; j++) {
                        S[i][j] += r[i] * r[j];
                    }
                }
            });

            const S11 = Array(6).fill(0).map((_, i) => S[i].slice(0, 6));
            const S12 = Array(6).fill(0).map((_, i) => S[i].slice(6, 10));
            const S22 = Array(4).fill(0).map((_, i) => S[i + 6].slice(6, 10));

            const X = Array(4).fill(0).map(() => Array(6).fill(0));
            for (let j = 0; j < 6; j++) {
                const b_col = [S12[j][0], S12[j][1], S12[j][2], S12[j][3]];
                const sol = solveLinearSystem(S22, b_col, 4);
                if (!sol) return null;
                for (let i = 0; i < 4; i++) {
                    X[i][j] = sol[i];
                }
            }

            const S_prime = Array(6).fill(0).map(() => Array(6).fill(0));
            for (let i = 0; i < 6; i++) {
                for (let j = 0; j < 6; j++) {
                    let sum = 0;
                    for (let k = 0; k < 4; k++) sum += S12[i][k] * X[k][j];
                    S_prime[i][j] = S11[i][j] - sum;
                }
            }

            for (let i = 0; i < 6; i++) {
                S_prime[i][i] += 1e-9;
            }

            const C11 = [
                [-1.0,  1.0,  1.0,  0.0,  0.0,  0.0],
                [ 1.0, -1.0,  1.0,  0.0,  0.0,  0.0],
                [ 1.0,  1.0, -1.0,  0.0,  0.0,  0.0],
                [ 0.0,  0.0,  0.0, -4.0,  0.0,  0.0],
                [ 0.0,  0.0,  0.0,  0.0, -4.0,  0.0],
                [ 0.0,  0.0,  0.0,  0.0,  0.0, -4.0]
            ];

            const L_tri = cholesky6x6(S_prime);
            if (!L_tri) return null;

            const invL = invertLowerTriangular6x6(L_tri);
            if (!invL) return null;

            const B = computeBMatrix6x6(invL, C11);

            const decomp = jacobiEigen6x6(B);
            const L_vals = decomp.eigenvalues;
            const V_vectors = decomp.eigenvectors;

            let posIdx = -1;
            let maxPositiveVal = -Infinity;
            for (let i = 0; i < 6; i++) {
                if (L_vals[i] > 0 && L_vals[i] > maxPositiveVal) {
                    maxPositiveVal = L_vals[i];
                    posIdx = i;
                }
            }
            if (posIdx === -1) return null;

            const y = [];
            for (let i = 0; i < 6; i++) y.push(V_vectors[i][posIdx]);

            const v1 = Array(6).fill(0);
            for (let i = 0; i < 6; i++) {
                let sum = 0;
                for (let k = 0; k < 6; k++) sum += invL[k][i] * y[k];
                v1[i] = sum;
            }

            let v1Cv1 = v1[0]*(-v1[0] + v1[1] + v1[2]) +
                        v1[1]*(v1[0] - v1[1] + v1[2]) +
                        v1[2]*(v1[0] + v1[1] - v1[2]) -
                        4.0 * (v1[3]*v1[3] + v1[4]*v1[4] + v1[5]*v1[5]);
            if (v1Cv1 > 0) {
                const s = 1.0 / Math.sqrt(v1Cv1);
                for (let i = 0; i < 6; i++) v1[i] *= s;
            }

            const S12Tv1 = Array(4).fill(0);
            for (let i = 0; i < 4; i++) {
                let sum = 0;
                for (let k = 0; k < 6; k++) sum += S12[k][i] * v1[k];
                S12Tv1[i] = sum;
            }
            const a2 = solveLinearSystem(S22, S12Tv1.map(val => -val), 4);
            if (!a2) return null;

            const coeffs = [...v1, ...a2];
            let A_mat = [
                [coeffs[0], coeffs[5], coeffs[4]],
                [coeffs[5], coeffs[1], coeffs[3]],
                [coeffs[4], coeffs[3], coeffs[2]]
            ];
            let b_vec = [coeffs[6], coeffs[7], coeffs[8]];
            let j_val = coeffs[9];

            let decomp_A = jacobiEigen3x3(A_mat);
            let eigenvalues_A = decomp_A.eigenvalues;
            let negCount = 0;
            eigenvalues_A.forEach(val => { if (val < 0) negCount++; });
            if (negCount >= 2) {
                for (let i = 0; i < 10; i++) coeffs[i] = -coeffs[i];
                A_mat = [
                    [coeffs[0], coeffs[5], coeffs[4]],
                    [coeffs[5], coeffs[1], coeffs[3]],
                    [coeffs[4], coeffs[3], coeffs[2]]
                ];
                b_vec = [coeffs[6], coeffs[7], coeffs[8]];
                j_val = coeffs[9];
                decomp_A = jacobiEigen3x3(A_mat);
                eigenvalues_A = decomp_A.eigenvalues;
            }

            if (Math.min(...eigenvalues_A) <= 0.0) return null;

            const invA = invert3x3(A_mat);
            if (!invA) return null;

            const offset_zc = [
                -(invA[0][0] * b_vec[0] + invA[0][1] * b_vec[1] + invA[0][2] * b_vec[2]),
                -(invA[1][0] * b_vec[0] + invA[1][1] * b_vec[1] + invA[1][2] * b_vec[2]),
                -(invA[2][0] * b_vec[0] + invA[2][1] * b_vec[1] + invA[2][2] * b_vec[2])
            ];

            const offset = [
                offset_zc[0] + meanX,
                offset_zc[1] + meanY,
                offset_zc[2] + meanZ
            ];

            // Estimate physical magnetic field strength in uT before determinant normalization (Requirement 3 refined)
            let sumMag = 0;
            points.forEach(p => {
                const dx = p[0] - offset[0];
                const dy = p[1] - offset[1];
                const dz = p[2] - offset[2];
                sumMag += Math.sqrt(dx*dx + dy*dy + dz*dz);
            });
            const rawMagnitude = sumMag / N;

            const vtAv = b_vec[0] * offset_zc[0] + b_vec[1] * offset_zc[1] + b_vec[2] * offset_zc[2];
            const k_val = vtAv - j_val;
            if (k_val <= 0) return null;

            const M = [
                [A_mat[0][0]/k_val, A_mat[0][1]/k_val, A_mat[0][2]/k_val],
                [A_mat[1][0]/k_val, A_mat[1][1]/k_val, A_mat[1][2]/k_val],
                [A_mat[2][0]/k_val, A_mat[2][1]/k_val, A_mat[2][2]/k_val]
            ];

            const decomp_M = jacobiEigen3x3(M);
            if (Math.min(...decomp_M.eigenvalues) <= 1e-8) return null;

            let W = matrixSquareRoot3x3(M);
            if (!W) return null;

            const detW = determinant3x3(W);
            if (detW > 0) {
                const scale = 1.0 / Math.cbrt(detW);
                for (let i = 0; i < 3; i++) {
                    for (let j = 0; j < 3; j++) {
                        W[i][j] *= scale;
                    }
                }
            }

            let radii = [];
            points.forEach(p => {
                const dx = p[0] - offset[0];
                const dy = p[1] - offset[1];
                const dz = p[2] - offset[2];
                const cx = W[0][0] * dx + W[0][1] * dy + W[0][2] * dz;
                const cy = W[1][0] * dx + W[1][1] * dy + W[1][2] * dz;
                const cz = W[2][0] * dx + W[2][1] * dy + W[2][2] * dz;
                radii.push(Math.sqrt(cx*cx + cy*cy + cz*cz));
            });
            const meanR = radii.reduce((sum, r) => sum + r, 0) / N;
            let sumSqError = 0;
            radii.forEach(r => {
                const err = Math.abs(r - meanR) / meanR;
                sumSqError += err * err;
            });
            const rmsError = Math.sqrt(sumSqError / N) * 100;

            return {
                offset: offset,
                softIronMatrix: W,
                rmsError: rmsError,
                meanR: rawMagnitude // Return the physically meaningful estimated field magnitude!
            };
        }

        function cholesky6x6(A) {
            const n = 6;
            const L = Array(n).fill(0).map(() => Array(n).fill(0));
            for (let i = 0; i < n; i++) {
                for (let j = 0; j <= i; j++) {
                    let sum = 0;
                    for (let k = 0; k < j; k++) sum += L[i][k] * L[j][k];
                    if (i === j) {
                        const val = A[i][i] - sum;
                        if (val <= 0) return null;
                        L[i][j] = Math.sqrt(val);
                    } else {
                        L[i][j] = (A[i][j] - sum) / L[j][j];
                    }
                }
            }
            return L;
        }

        function invertLowerTriangular6x6(L) {
            const n = 6;
            const inv = Array(n).fill(0).map(() => Array(n).fill(0));
            for (let i = 0; i < n; i++) {
                inv[i][i] = 1.0 / L[i][i];
                for (let j = 0; j < i; j++) {
                    let sum = 0;
                    for (let k = j; k < i; k++) sum += L[i][k] * inv[k][j];
                    inv[i][j] = -sum / L[i][i];
                }
            }
            return inv;
        }

        function computeBMatrix6x6(invL, C11) {
            const n = 6;
            const Temp = Array(n).fill(0).map(() => Array(n).fill(0));
            for (let i = 0; i < n; i++) {
                for (let j = 0; j < n; j++) {
                    let sum = 0;
                    for (let k = 0; k < n; k++) sum += invL[i][k] * C11[k][j];
                    Temp[i][j] = sum;
                }
            }
            const B = Array(n).fill(0).map(() => Array(n).fill(0));
            for (let i = 0; i < n; i++) {
                for (let j = 0; j < n; j++) {
                    let sum = 0;
                    for (let k = 0; k < n; k++) sum += Temp[i][k] * invL[j][k];
                    B[i][j] = sum;
                }
            }
            return B;
        }

        function jacobiEigen6x6(B) {
            const n = 6;
            let A = Array(n).fill(0).map((_, i) => [...B[i]]);
            let V = Array(n).fill(0).map((_, i) => {
                const r = Array(n).fill(0);
                r[i] = 1.0;
                return r;
            });

            const maxIterations = 100;
            const eps = 1e-15;

            for (let iter = 0; iter < maxIterations; iter++) {
                let p = 0, q = 1;
                let maxVal = Math.abs(A[0][1]);
                for (let i = 0; i < n; i++) {
                    for (let j = i + 1; j < n; j++) {
                        if (Math.abs(A[i][j]) > maxVal) {
                            maxVal = Math.abs(A[i][j]);
                            p = i; q = j;
                        }
                    }
                }

                if (maxVal < eps) break;

                let phi = 0.5 * Math.atan2(2.0 * A[p][q], A[q][q] - A[p][p]);
                let c = Math.cos(phi);
                let s = Math.sin(phi);

                let app = c * c * A[p][p] - 2.0 * s * c * A[p][q] + s * s * A[q][q];
                let aqq = s * s * A[p][p] + 2.0 * s * c * A[p][q] + c * c * A[q][q];
                A[p][p] = app;
                A[q][q] = aqq;
                A[p][q] = 0.0;
                A[q][p] = 0.0;

                for (let i = 0; i < n; i++) {
                    if (i !== p && i !== q) {
                        let a_ip = c * A[i][p] - s * A[i][q];
                        let a_iq = s * A[i][p] + c * A[i][q];
                        A[i][p] = a_ip; A[p][i] = a_ip;
                        A[i][q] = a_iq; A[q][i] = a_iq;
                    }
                }

                for (let i = 0; i < n; i++) {
                    let v_ip = c * V[i][p] - s * V[i][q];
                    let v_iq = s * V[i][p] + c * V[i][q];
                    V[i][p] = v_ip;
                    V[i][q] = v_iq;
                }
            }

            const eigenvalues = [];
            for (let i = 0; i < n; i++) eigenvalues.push(A[i][i]);

            return { eigenvalues, eigenvectors: V };
        }

        function evaluatePointsObservability(points) {
            const N = points.length;
            if (N < 20) return { ok: false, ratio: 0, cond: Infinity };

            let mean = [0, 0, 0];
            points.forEach(p => { mean[0] += p[0]; mean[1] += p[1]; mean[2] += p[2]; });
            mean = mean.map(val => val / N);

            const Cov = [
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0]
            ];
            points.forEach(p => {
                const dx = p[0] - mean[0];
                const dy = p[1] - mean[1];
                const dz = p[2] - mean[2];
                Cov[0][0] += dx * dx; Cov[0][1] += dx * dy; Cov[0][2] += dx * dz;
                Cov[1][0] += dy * dx; Cov[1][1] += dy * dy; Cov[1][2] += dy * dz;
                Cov[2][0] += dz * dx; Cov[2][1] += dz * dy; Cov[2][2] += dz * dz;
            });

            for (let i = 0; i < 3; i++) {
                for (let j = 0; j < 3; j++) {
                    Cov[i][j] /= (N - 1);
                }
            }

            const decomp = jacobiEigen3x3(Cov);
            const L = decomp.eigenvalues.map(val => Math.abs(val)).sort((a, b) => b - a);

            const ratio = L[0] > 0 ? L[2] / L[0] : 0;
            const cond = L[2] > 0 ? L[0] / L[2] : Infinity;
            return {
                ok: ratio >= PLANAR_RATIO,
                ratio: ratio,
                cond: cond
            };
        }

        function determinant3x3(W) {
            return W[0][0] * (W[1][1] * W[2][2] - W[2][1] * W[1][2]) -
                   W[0][1] * (W[1][0] * W[2][2] - W[1][2] * W[2][0]) +
                   W[0][2] * (W[1][0] * W[2][1] - W[1][1] * W[2][0]);
        }

        function jacobiEigen3x3(M) {
            let A = [
                [M[0][0], M[0][1], M[0][2]],
                [M[1][0], M[1][1], M[1][2]],
                [M[2][0], M[2][1], M[2][2]]
            ];
            let V = [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]
            ];

            const maxIterations = 50;
            const eps = 1e-15;

            for (let iter = 0; iter < maxIterations; iter++) {
                let p = 0, q = 1;
                let maxVal = Math.abs(A[0][1]);
                if (Math.abs(A[0][2]) > maxVal) { p = 0; q = 2; maxVal = Math.abs(A[0][2]); }
                if (Math.abs(A[1][2]) > maxVal) { p = 1; q = 2; maxVal = Math.abs(A[1][2]); }

                if (maxVal < eps) break;

                let phi = 0.5 * Math.atan2(2.0 * A[p][q], A[q][q] - A[p][p]);
                let c = Math.cos(phi);
                let s = Math.sin(phi);

                let app = c * c * A[p][p] - 2.0 * s * c * A[p][q] + s * s * A[q][q];
                let aqq = s * s * A[p][p] + 2.0 * s * c * A[p][q] + c * c * A[q][q];
                A[p][p] = app;
                A[q][q] = aqq;
                A[p][q] = 0.0;
                A[q][p] = 0.0;

                for (let i = 0; i < 3; i++) {
                    if (i !== p && i !== q) {
                        let a_ip = c * A[i][p] - s * A[i][q];
                        let a_iq = s * A[i][p] + c * A[i][q];
                        A[i][p] = a_ip; A[p][i] = a_ip;
                        A[i][q] = a_iq; A[q][i] = a_iq;
                    }
                }

                for (let i = 0; i < 3; i++) {
                    let v_ip = c * V[i][p] - s * V[i][q];
                    let v_iq = s * V[i][p] + c * V[i][q];
                    V[i][p] = v_ip;
                    V[i][q] = v_iq;
                }
            }

            return {
                eigenvalues: [A[0][0], A[1][1], A[2][2]],
                eigenvectors: V
            };
        }

        function matrixSquareRoot3x3(M) {
            const decomp = jacobiEigen3x3(M);
            const V = decomp.eigenvectors;
            const L = decomp.eigenvalues;

            const sqrtL = L.map(val => val > 0 ? Math.sqrt(val) : 0);

            const W = [
                [0, 0, 0],
                [0, 0, 0],
                [0, 0, 0]
            ];

            for (let i = 0; i < 3; i++) {
                for (let j = 0; j < 3; j++) {
                    W[i][j] = V[i][0] * sqrtL[0] * V[j][0] +
                              V[i][1] * sqrtL[1] * V[j][1] +
                              V[i][2] * sqrtL[2] * V[j][2];
                }
            }
            return W;
        }

        function solveLinearSystem(A, B, n = 9) {
            const M = Array(n).fill(0).map((_, i) => {
                const row = [...A[i]];
                row.push(B[i]);
                return row;
            });
            for (let i = 0; i < n; i++) {
                let maxRow = i;
                for (let k = i + 1; k < n; k++) {
                    if (Math.abs(M[k][i]) > Math.abs(M[maxRow][i])) {
                        maxRow = k;
                    }
                }
                const temp = M[i];
                M[i] = M[maxRow];
                M[maxRow] = temp;
                if (Math.abs(M[i][i]) < 1e-12) return null;
                for (let k = i + 1; k < n; k++) {
                    const factor = M[k][i] / M[i][i];
                    for (let j = i; j <= n; j++) {
                        M[k][j] -= factor * M[i][j];
                    }
                }
            }
            const x = Array(n).fill(0);
            for (let i = n - 1; i >= 0; i--) {
                let sum = M[i][n];
                for (let j = i + 1; j < n; j++) {
                    sum -= M[i][j] * x[j];
                }
                x[i] = sum / M[i][i];
            }
            return x;
        }

        function solve3x3System(A, B) {
            const det = A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]) -
                        A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                        A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
            if (Math.abs(det) < 1e-12) return null;

            const det0 = B[0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]) -
                         A[0][1] * (B[1] * A[2][2] - A[1][2] * B[2]) +
                         A[0][2] * (B[1] * A[2][1] - A[1][1] * B[2]);

            const det1 = A[0][0] * (B[1] * A[2][2] - B[2] * A[1][2]) -
                         B[0] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                         A[0][2] * (A[1][0] * B[2] - B[1] * A[2][0]);

            const det2 = A[0][0] * (A[1][1] * B[2] - A[2][1] * B[1]) -
                         A[0][1] * (A[1][0] * B[2] - B[1] * A[2][0]) +
                         B[0] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

            return [det0 / det, det1 / det, det2 / det];
        }

        function invert3x3(A) {
            const det = A[0][0] * (A[1][1] * A[2][2] - A[2][1] * A[1][2]) -
                        A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                        A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
            if (Math.abs(det) < 1e-12) return null;
            const invdet = 1.0 / det;
            return [
                [
                    (A[1][1] * A[2][2] - A[2][1] * A[1][2]) * invdet,
                    (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * invdet,
                    (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invdet
                ],
                [
                    (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * invdet,
                    (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invdet,
                    (A[1][0] * A[0][2] - A[0][0] * A[1][2]) * invdet
                ],
                [
                    (A[1][0] * A[2][1] - A[2][0] * A[1][1]) * invdet,
                    (A[2][0] * A[0][1] - A[0][0] * A[2][1]) * invdet,
                    (A[0][0] * A[2][1] - A[1][0] * A[0][1]) * invdet
                ]
            ];
        }

        connect();
    </script>
</body>
</html>
)rawliteral";

#endif /* CALIBRATION_HTML_H */