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
            grid-template-columns: 1fr 1fr;
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
    </style>
</head>
<body>
    <!-- Instructions Overlay Modal -->
    <div id="instructionModal" style="display:none; position:fixed; top:0; left:0; width:100%; height:100%; background:rgba(13,17,23,0.95); z-index:9999; justify-content:center; align-items:center; flex-direction:column; padding:30px; text-align:center;">
        <h1 style="color:var(--warning); font-size:32px; margin-bottom:20px; font-weight:bold; font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;">Calibration Instructions</h1>
        <p style="color:var(--text-main); font-size:22px; max-width:550px; line-height:1.6; margin-bottom:30px; font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif; font-weight:500;">
            Rotate the buoy slowly 360&deg; in all directions.<br>
            Tilt it up and down in a slow figure-8 pattern.<br>
            Keep rotating until points form a dense circle.
        </p>
        <div id="countdownTimer" style="font-size:54px; font-weight:bold; color:var(--primary); font-family:monospace; background:#161b22; padding:10px 30px; border-radius:12px; border:1px solid var(--border-color);">10</div>
    </div>

    <!-- Co-Pilot Sticky Bottom Status and Hint Bar -->
    <div id="calibrationCoPilot" style="position: fixed; bottom: 0; left: 0; width: 100%; background: #161b22; border-top: 4px solid var(--warning); padding: 20px; z-index: 9999; display: none; flex-direction: column; align-items: center; justify-content: center; box-shadow: 0 -8px 20px rgba(0,0,0,0.6);">
        <!-- Large High-Visibility Percentages inside structured blocks -->
        <div id="coPilotStats" style="font-size: 32px; font-weight: 900; color: var(--text-main); font-family: monospace; margin-bottom: 12px; letter-spacing: 1px; text-align: center; width: 100%; display: flex; justify-content: space-around; flex-wrap: wrap; gap: 15px;">
            <div style="background:#0d1117; padding: 10px 20px; border-radius: 8px; border: 1px solid var(--border-color); min-width: 110px;">Q1: <span id="cpQ1" style="color:var(--primary);">0%</span></div>
            <div style="background:#0d1117; padding: 10px 20px; border-radius: 8px; border: 1px solid var(--border-color); min-width: 110px;">Q2: <span id="cpQ2" style="color:var(--primary);">0%</span></div>
            <div style="background:#0d1117; padding: 10px 20px; border-radius: 8px; border: 1px solid var(--border-color); min-width: 110px;">Q3: <span id="cpQ3" style="color:var(--primary);">0%</span></div>
            <div style="background:#0d1117; padding: 10px 20px; border-radius: 8px; border: 1px solid var(--border-color); min-width: 110px;">Q4: <span id="cpQ4" style="color:var(--primary);">0%</span></div>
            <div style="background:#0d1117; padding: 10px 20px; border-radius: 8px; border: 1px solid var(--border-color); min-width: 110px; border-color: var(--accent);">Tilt: <span id="cpTilt" style="color:var(--accent);">0%</span></div>
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
            <h2>Interactive 3D Sphere Plotter (Proj: X/Y)</h2>
            
            <div class="cal-points-visual" id="calCanvas">
                <svg width="100%" height="100%" id="calSvg">
                    <line x1="0" y1="50%" x2="100%" y2="50%" stroke="rgba(255,255,255,0.05)" />
                    <line x1="50%" y1="0" x2="50%" y2="100%" stroke="rgba(255,255,255,0.05)" />
                    <g id="pointsGroup"></g>
                </svg>
                <div class="cal-point-count" id="pointCount">Points: 0</div>
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

    <script>
        let isCalibrating = false;
        let calPoints = [];
        let maxPoints = 1200; // Expanded memory cache to safely buffer full multi-quadrant tilt maneuvers
        let pollTimer = null;
        let icmModeLoaded = false;

        // Bounding limits for calibration solver
        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;
        let minZ = Infinity, maxZ = -Infinity;

        // Final calibration parameters
        let hix = 0.0, hiy = 0.0, hiz = 0.0;
        let six = 1.0, siy = 1.0, siz = 1.0;

        const statusBadge = document.getElementById('status');
        const btnCal = document.getElementById('btnCal');
        const btnSave = document.getElementById('btnSave');
        const pointCount = document.getElementById('pointCount');
        const pointsGroup = document.getElementById('pointsGroup');
        const calHiVal = document.getElementById('calHiVal');
        const calSiVal = document.getElementById('calSiVal');

        const rangeBarX = document.getElementById('rangeBarX');
        const rangeBarY = document.getElementById('rangeBarY');
        const rangeBarZ = document.getElementById('rangeBarZ');
        const rangeValsX = document.getElementById('rangeValsX');
        const rangeValsY = document.getElementById('rangeValsY');
        const rangeValsZ = document.getElementById('rangeValsZ');

        function connect() {
            // Since we are running over standard HTTP polling, establish connected status immediately
            statusBadge.textContent = 'Connected';
            statusBadge.className = 'status-badge connected';
            // Start continuous high-speed background polling immediately on connection (100ms interval matching ShowActualData)
            setInterval(pollData, 100);
        }

        function pollData() {
            fetch('/data')
                .then(r => r.json())
                .then(data => {
                    // Update raw points if calibrating
                    if (isCalibrating && data.mx_raw !== undefined && data.my_raw !== undefined && data.mz_raw !== undefined) {
                        processRawPoint(data.mx_raw, data.my_raw, data.mz_raw);
                    }
                    
                    // Update persistence fields (Always update so user sees NVS state!)
                    if (data.cal_load !== undefined) {
                        document.getElementById('cal_load').textContent = data.cal_load;
                    }
                    if (data.cal_ver !== undefined) {
                        document.getElementById('cal_ver').textContent = data.cal_ver;
                    }

                    // Pre-select correct radio button based on ESP state once on startup
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

                    // Update ICM Active style
                    const b = document.getElementById('calButton');
                    if (b) {
                        b.style.backgroundColor = '#2a6a2a';
                        b.style.color = '#fff';
                        b.innerText = 'ICM Active';
                    }
                })
                .catch(err => {
                    console.error("Poll error", err);
                });
        }

        function processRawPoint(x, y, z) {
            // Immediate outlier filter: Discard absolute sensor glitches (zeros or extreme spikes)
            if (x === 0 && y === 0 && z === 0) return;
            if (Math.abs(x) > 1000 || Math.abs(y) > 1000 || Math.abs(z) > 1000) return;

            calPoints.push({x, y, z});

            if (calPoints.length > maxPoints) {
                calPoints.shift();
            }

            // Update bounding limits
            minX = Math.min(minX, x); maxX = Math.max(maxX, x);
            minY = Math.min(minY, y); maxY = Math.max(maxY, y);
            minZ = Math.min(minZ, z); maxZ = Math.max(maxZ, z);

            // Calculate auto calibration parameters
            hix = (maxX + minX) / 2;
            hiy = (maxY + minY) / 2;
            hiz = (maxZ + minZ) / 2;

            const rx = maxX - minX || 1.0;
            const ry = maxY - minY || 1.0;
            const rz = maxZ - minZ || 1.0;
            const rAvg = (rx + ry + rz) / 3;

            six = rAvg / rx;
            siy = rAvg / ry;
            siz = rAvg / rz;

            // Compute coverage in all four 2D quadrants relative to the current solved center (hix, hiy)
            let qCounts = [0, 0, 0, 0];
            calPoints.forEach(pt => {
                if (pt.x > hix && pt.y > hiy) qCounts[0]++;       // Quadrant 1 (Top-Right)
                else if (pt.x <= hix && pt.y > hiy) qCounts[1]++;  // Quadrant 2 (Top-Left)
                else if (pt.x <= hix && pt.y <= hiy) qCounts[2]++; // Quadrant 3 (Bottom-Left)
                else if (pt.x > hix && pt.y <= hiy) qCounts[3]++;  // Quadrant 4 (Bottom-Right)
            });

            // Calculate active Z-axis variation (figure-8 / pitch & roll check)
            const targetZVar = 30.0; // 30 uT target span forces deep figure-8 pitch & roll tilt movements (at least 35-45 degrees)
            const zVar = (maxZ !== -Infinity && minZ !== Infinity) ? (maxZ - minZ) : 0.0;
            const zPct = Math.min(100, Math.round((zVar / targetZVar) * 100));

            // Target 150 points per quadrant to guarantee comprehensive, dense flat data collection
            const targetQ = 150;
            const q1Pct = Math.min(100, Math.round((qCounts[0] / targetQ) * 100));
            const q2Pct = Math.min(100, Math.round((qCounts[1] / targetQ) * 100));
            const q3Pct = Math.min(100, Math.round((qCounts[2] / targetQ) * 100));
            const q4Pct = Math.min(100, Math.round((qCounts[3] / targetQ) * 100));

            // Format real-time 3D coverage telemetry (both top and sticky co-pilot progress bar)
            const statText = `Points: ${calPoints.length} | Q1: ${q1Pct}% Q2: ${q2Pct}% Q3: ${q3Pct}% Q4: ${q4Pct}% | Tilt: ${zPct}%`;
            pointCount.innerHTML = statText;

            // Update large-font Co-Pilot progress bar
            document.getElementById('cpQ1').textContent = q1Pct + '%';
            document.getElementById('cpQ1').style.color = q1Pct === 100 ? 'var(--success)' : 'var(--primary)';
            document.getElementById('cpQ2').textContent = q2Pct + '%';
            document.getElementById('cpQ2').style.color = q2Pct === 100 ? 'var(--success)' : 'var(--primary)';
            document.getElementById('cpQ3').textContent = q3Pct + '%';
            document.getElementById('cpQ3').style.color = q3Pct === 100 ? 'var(--success)' : 'var(--primary)';
            document.getElementById('cpQ4').textContent = q4Pct + '%';
            document.getElementById('cpQ4').style.color = q4Pct === 100 ? 'var(--success)' : 'var(--primary)';
            document.getElementById('cpTilt').textContent = zPct + '%';
            document.getElementById('cpTilt').style.color = zPct === 100 ? 'var(--success)' : 'var(--accent)';

            // Select dynamic Co-Pilot Hint based on what is missing
            let hint = "➔ Hint: Rotating buoy...";
            if (q1Pct < 100) {
                hint = "➔ Hint: Face the buoy North-East (Top-Right of graph)";
            } else if (q2Pct < 100) {
                hint = "➔ Hint: Face the buoy North-West (Top-Left of graph)";
            } else if (q3Pct < 100) {
                hint = "➔ Hint: Face the buoy South-West (Bottom-Left of graph)";
            } else if (q4Pct < 100) {
                hint = "➔ Hint: Face the buoy South-East (Bottom-Right of graph)";
            } else if (zPct < 100) {
                hint = "➔ Hint: Perform figure-8 pitch & roll tilts (up/down) to calibrate 3D Tilt!";
            } else {
                hint = "➔ Calibration complete! Storing data...";
            }
            document.getElementById('coPilotHint').textContent = hint;

            // Intelligent 3D Stop Condition: Auto-save ONLY when flat circle (Q1-Q4) AND figure-8 tilts (Z-span) are robustly covered AND at least 500 total points!
            if (calPoints.length >= 500 && qCounts[0] >= targetQ && qCounts[1] >= targetQ && qCounts[2] >= targetQ && qCounts[3] >= targetQ && zVar >= targetZVar) {
                isCalibrating = false;
                if (pollTimer) {
                    clearTimeout(pollTimer);
                    pollTimer = null;
                }
                btnCal.textContent = 'Start Interactive Calibration';
                btnCal.className = 'btn btn-primary';

                pointCount.innerHTML = `<span style="color:var(--success); font-weight:bold;">Calibration Successful! Saving &amp; Redirecting...</span>`;
                document.getElementById('coPilotHint').textContent = "➔ Success! Calibration saved. Redirecting to main dashboard...";
                document.getElementById('coPilotHint').style.color = "var(--success)";

                // Automatically click save which sends parameter payload to NVS, plays success tune, and redirects
                setTimeout(() => {
                    btnSave.disabled = false;
                    btnSave.click();
                }, 100);
                return;
            }

            // Render stats
            calHiVal.textContent = `X: ${hix.toFixed(1)}, Y: ${hiy.toFixed(1)}, Z: ${hiz.toFixed(1)}`;
            calSiVal.textContent = `X: ${six.toFixed(2)}, Y: ${siy.toFixed(2)}, Z: ${siz.toFixed(2)}`;

            // Update Range Bars
            updateRangeBar(rangeBarX, rangeValsX, minX, maxX, x);
            updateRangeBar(rangeBarY, rangeValsY, minY, maxY, y);
            updateRangeBar(rangeBarZ, rangeValsZ, minZ, maxZ, z);

            // Draw 2D scatter cloud
            drawPoints();
        }

        function updateRangeBar(barElement, valElement, min, max, cur) {
            valElement.textContent = `[${min.toFixed(0)}, ${max.toFixed(0)}]`;
            const range = max - min || 1;
            const pct = ((cur - min) / range) * 100;
            barElement.style.width = `${Math.max(2, Math.min(100, pct))}%`;
            barElement.style.left = `0%`;
        }

        function drawPoints() {
            const rX = maxX - minX || 1;
            const rY = maxY - minY || 1;

            let pointsHtml = '';
            calPoints.forEach((pt, idx) => {
                const alpha = idx / calPoints.length;
                const posX = ((pt.x - minX) / rX) * 90 + 5;
                const posY = ((pt.y - minY) / rY) * 90 + 5;
                pointsHtml += `<circle cx="${posX}%" cy="${posY}%" r="2.5" fill="var(--warning)" opacity="${alpha * 0.7}" />`;
            });

            // Draw offset reticle
            const offsetPctX = ((hix - minX) / rX) * 90 + 5;
            const offsetPctY = ((hiy - minY) / rY) * 90 + 5;
            pointsHtml += `<circle cx="${offsetPctX}%" cy="${offsetPctY}%" r="6" fill="none" stroke="var(--danger)" stroke-width="1.5" />`;
            pointsHtml += `<line x1="${offsetPctX - 4}%" y1="${offsetPctY}%" x2="${offsetPctX + 4}%" y2="${offsetPctY}%" stroke="var(--danger)" stroke-width="1" />`;
            pointsHtml += `<line x1="${offsetPctX}%" y1="${offsetPctY - 4}%" x2="${offsetPctX}%" y2="${offsetPctY + 4}%" stroke="var(--danger)" stroke-width="1" />`;

            pointsGroup.innerHTML = pointsHtml;
        }

        btnCal.onclick = () => {
            if (!isCalibrating) {
                isCalibrating = true;
                calPoints = [];
                minX = Infinity; maxX = -Infinity;
                minY = Infinity; maxY = -Infinity;
                minZ = Infinity; maxZ = -Infinity;
                
                btnCal.textContent = 'Stop Calibration & Lock Parameters';
                btnCal.className = 'btn btn-danger';
                btnSave.disabled = true;

                // Show instruction modal overlay with countdown for 10 seconds
                const modal = document.getElementById('instructionModal');
                const timer = document.getElementById('countdownTimer');
                modal.style.display = 'flex';
                let timeLeft = 10;
                timer.textContent = timeLeft;
                
                // Show bottom Co-Pilot bar INSTANTLY so they see it from a distance at the start!
                document.getElementById('calibrationCoPilot').style.display = 'flex';
                
                const interval = setInterval(() => {
                    timeLeft--;
                    timer.textContent = timeLeft;
                    if (timeLeft <= 0) {
                        clearInterval(interval);
                        modal.style.display = 'none';
                    }
                }, 1000);

                // Signal start of calibration to ESP32 to trigger buzzer beeps!
                fetch('/start_cal')
                    .catch(err => console.warn("Failed to trigger start beep", err));

                pollData();
            } else {
                isCalibrating = false;
                document.getElementById('calibrationCoPilot').style.display = 'none';
                btnCal.textContent = 'Start Interactive Calibration';
                btnCal.className = 'btn btn-primary';
                
                if (calPoints.length > 10) {
                    btnSave.disabled = false;
                }
            }
        };

        btnSave.onclick = () => {
            btnSave.textContent = "Saving...";
            btnSave.disabled = true;
            const url = `/save_cal?hx=${hix}&hy=${hiy}&hz=${hiz}&sx=${six}&sy=${siy}&sz=${siz}`;
            fetch(url)
                .then(r => r.text())
                .then(text => {
                    if (text === "OK") {
                        btnSave.textContent = "Success!";
                        console.log("Calibration saved and uploaded successfully!");
                        
                        // Keep bottom co-pilot bar visible and show 30-second live countdown hint
                        document.getElementById('calibrationCoPilot').style.display = 'flex';
                        let redirectTime = 30;
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

        connect();
    </script>
</body>
</html>
)rawliteral";

#endif /* CALIBRATION_HTML_H */