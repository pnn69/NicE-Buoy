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


    <!-- Co-Pilot Sticky Bottom Status and Hint Bar -->
    <div id="calibrationCoPilot" style="position: fixed; bottom: 0; left: 0; width: 100%; background: #161b22; border-top: 4px solid var(--warning); padding: 20px; z-index: 9999; display: none; flex-direction: column; align-items: center; justify-content: center; box-shadow: 0 -8px 20px rgba(0,0,0,0.6);">
        <!-- Large High-Visibility Percentages inside structured blocks -->
        <div id="coPilotStats" style="font-size: 24px; font-weight: 900; color: var(--text-main); font-family: monospace; margin-bottom: 12px; letter-spacing: 1px; text-align: center; width: 100%; display: flex; justify-content: space-around; flex-wrap: wrap; gap: 10px;">
            <div style="background:#0d1117; padding: 6px 12px; border-radius: 8px; border: 1px solid var(--border-color); min-width: 80px;">Q1: <span id="cpQ1" style="color:var(--primary);">0%</span></div>
            <div style="background:#0d1117; padding: 6px 12px; border-radius: 8px; border: 1px solid var(--border-color); min-width: 80px;">Q2: <span id="cpQ2" style="color:var(--primary);">0%</span></div>
            <div style="background:#0d1117; padding: 6px 12px; border-radius: 8px; border: 1px solid var(--border-color); min-width: 80px;">Q3: <span id="cpQ3" style="color:var(--primary);">0%</span></div>
            <div style="background:#0d1117; padding: 6px 12px; border-radius: 8px; border: 1px solid var(--border-color); min-width: 80px;">Q4: <span id="cpQ4" style="color:var(--primary);">0%</span></div>
            <div style="background:#0d1117; padding: 6px 12px; border-radius: 8px; border: 1px solid var(--border-color); min-width: 80px; border-color: var(--accent);">XZ: <span id="cpXZ" style="color:var(--accent);">0%</span></div>
            <div style="background:#0d1117; padding: 6px 12px; border-radius: 8px; border: 1px solid var(--border-color); min-width: 80px; border-color: var(--accent);">YZ: <span id="cpYZ" style="color:var(--success);">0%</span></div>
            <div style="background:#0d1117; padding: 6px 12px; border-radius: 8px; border: 1px solid var(--border-color); min-width: 80px; border-color: var(--accent);">Tilt: <span id="cpTilt" style="color:var(--accent);">0%</span></div>
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
                <div style="flex: 1; color: var(--primary, #58a6ff);">◀ X vs Y (Horizontal Plane)</div>
                <div style="flex: 1; color: var(--success, #56d364);">Y vs Z (Vertical Plane) ▶</div>
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
        let maxPoints = 2000; // Expanded memory cache to safely buffer full multi-quadrant tilt maneuvers
        let pollTimer = null;
        let icmModeLoaded = false;
        let enoughPointsMet = false;
        let calTimeoutTimer = null;
        let globalCovXY = Array(12).fill(false);
        let globalCovXZ = Array(12).fill(false);
        let globalCovYZ = Array(12).fill(false);

        // Bounding limits for calibration solver
        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;
        let minZ = Infinity, maxZ = -Infinity;

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
            // Start continuous background polling immediately on connection
            pollData();
        }

        function pollData() {
            fetch('/data')
                .then(r => r.json())
                .then(data => {
                    // Update raw points if calibrating (handles both batched array of points and single-point fallback)
                    if (isCalibrating) {
                        if (data.points !== undefined && Array.isArray(data.points)) {
                            data.points.forEach(p => processRawPoint(p[0], p[1], p[2]));
                        } else if (data.mx_raw !== undefined && data.my_raw !== undefined && data.mz_raw !== undefined) {
                            processRawPoint(data.mx_raw, data.my_raw, data.mz_raw);
                        }
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

                    // Schedule next poll - relaxed to 100ms for perfect HTTP stability while capturing 100% of points via the ESP32 ring buffer!
                    const interval = isCalibrating ? 100 : 500;
                    pollTimer = setTimeout(pollData, interval);
                })
                .catch(err => {
                    console.error("Poll error", err);
                    const interval = isCalibrating ? 1000 : 2000;
                    pollTimer = setTimeout(pollData, interval);
                });
        }

        function processRawPoint(x, y, z) {
            // Immediate outlier filter: Discard absolute sensor glitches (zeros or extreme spikes)
            if (x === 0 && y === 0 && z === 0) return;
            if (Math.abs(x) > 1000 || Math.abs(y) > 1000 || Math.abs(z) > 1000) return;

            // Distance-based jump filter to discard brief I2C read glitches or electromagnetic spikes
            if (calPoints.length > 10) {
                let avgX = 0, avgY = 0, avgZ = 0;
                const lastN = calPoints.slice(-10);
                lastN.forEach(p => { avgX += p.x; avgY += p.y; avgZ += p.z; });
                avgX /= lastN.length;
                avgY /= lastN.length;
                avgZ /= lastN.length;

                const dist = Math.sqrt((x - avgX)**2 + (y - avgY)**2 + (z - avgZ)**2);
                if (dist > 120.0) {
                    console.warn(`Outlier discarded: Dist=${dist.toFixed(1)} uT [X:${x}, Y:${y}, Z:${z}]`);
                    return; // Ignore this outlier completely!
                }
            }

            // Spatial density filter: Discard points that are too close to the last accepted point
            // to ensure an even distribution around the sphere and prevent clustered ill-conditioning when stationary.
            if (calPoints.length > 0) {
                const lastPt = calPoints[calPoints.length - 1];
                const distToLast = Math.sqrt((x - lastPt.x)**2 + (y - lastPt.y)**2 + (z - lastPt.z)**2);
                if (distToLast < 1.0) {
                    return; // Ignore redundant point when stationary or moving extremely slowly
                }
            }

            calPoints.push({x, y, z});

            if (calPoints.length > maxPoints) {
                calPoints.shift();
            }

            // Update bounding limits
            minX = Math.min(minX, x); maxX = Math.max(maxX, x);
            minY = Math.min(minY, y); maxY = Math.max(maxY, y);
            minZ = Math.min(minZ, z); maxZ = Math.max(maxZ, z);

            // Run high-precision 3D Least-Squares Ellipsoid Fitting every 25 points (fully real-time but lightweight!)
            if (calPoints.length >= 50 && (calPoints.length % 25) === 0) {
                runEllipsoidFitting();
            }

            // Compute coverage in all three 2D planes (XY, XZ, YZ) relative to the current solved center
            let qCounts = [0, 0, 0, 0];
            let qxz = [0, 0, 0, 0];
            let qyz = [0, 0, 0, 0];

            calPoints.forEach(pt => {
                let dx = pt.x - hix;
                let dy = pt.y - hiy;
                let dz = pt.z - hiz;

                if (dx > 0 && dy > 0) qCounts[0]++;       // Quadrant 1 (Top-Right)
                else if (dx <= 0 && dy > 0) qCounts[1]++;  // Quadrant 2 (Top-Left)
                else if (dx <= 0 && dy <= 0) qCounts[2]++; // Quadrant 3 (Bottom-Left)
                else if (dx > 0 && dy <= 0) qCounts[3]++;  // Quadrant 4 (Bottom-Right)

                if (dx > 0 && dz > 0) qxz[0]++;
                else if (dx <= 0 && dz > 0) qxz[1]++;
                else if (dx <= 0 && dz <= 0) qxz[2]++;
                else qxz[3]++;

                if (dy > 0 && dz > 0) qyz[0]++;
                else if (dy <= 0 && dz > 0) qyz[1]++;
                else if (dy <= 0 && dz <= 0) qyz[2]++;
                else qyz[3]++;
            });

            // Calculate active Z-axis span percentage (geographically-independent 3D tilt-maneuver check)
            const maxSpan = Math.max(maxX - minX, maxY - minY, maxZ - minZ) || 1.0;
            const zSpanPct = (maxZ - minZ) / maxSpan;
            const zPct = Math.min(100, Math.round((zSpanPct / 0.60) * 100));

            // Target 350 points per quadrant for XY plane, and 150 points for XZ and YZ planes
            const targetQ = 350;
            const target3D = 150;

            const q1Pct = Math.min(100, Math.round((qCounts[0] / targetQ) * 100));
            const q2Pct = Math.min(100, Math.round((qCounts[1] / targetQ) * 100));
            const q3Pct = Math.min(100, Math.round((qCounts[2] / targetQ) * 100));
            const q4Pct = Math.min(100, Math.round((qCounts[3] / targetQ) * 100));

            const minQXZ = Math.min(...qxz);
            const minQYZ = Math.min(...qyz);
            const xzPct = Math.min(100, Math.round((minQXZ / target3D) * 100));
            const yzPct = Math.min(100, Math.round((minQYZ / target3D) * 100));

            // Format real-time 3D coverage telemetry
            const statText = `Points: ${calPoints.length} | XY Q1: ${q1Pct}% Q2: ${q2Pct}% Q3: ${q3Pct}% Q4: ${q4Pct}% | XZ: ${xzPct}% | YZ: ${yzPct}% | Tilt: ${zPct}%`;
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
            document.getElementById('cpXZ').textContent = xzPct + '%';
            document.getElementById('cpXZ').style.color = xzPct === 100 ? 'var(--success)' : 'var(--accent)';
            document.getElementById('cpYZ').textContent = yzPct + '%';
            document.getElementById('cpYZ').style.color = yzPct === 100 ? 'var(--success)' : 'var(--success)';
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
            } else if (xzPct < 100) {
                hint = "➔ Hint: Perform slow XZ-plane pitch tilts (up/down) to calibrate XZ plane!";
            } else if (yzPct < 100) {
                hint = "➔ Hint: Perform slow YZ-plane roll tilts (left/right) to calibrate YZ plane!";
            } else if (zPct < 100) {
                hint = "➔ Hint: Continue figure-8 pitch & roll tilts to maximize 3D Tilt scale!";
            } else {
                hint = "➔ Calibration complete! Click the green button above to save.";
            }
            document.getElementById('coPilotHint').textContent = hint;

            // Check if 3D coverage targets are satisfied
            const isTargetMet = (calPoints.length >= 1500 && 
                                 qCounts[0] >= targetQ && qCounts[1] >= targetQ && qCounts[2] >= targetQ && qCounts[3] >= targetQ && 
                                 minQXZ >= target3D && minQYZ >= target3D && zSpanPct >= 0.60);

            if (isTargetMet && !enoughPointsMet) {
                enoughPointsMet = true;
                
                // Style button as green (ready to stop & save)
                btnCal.textContent = '➔ Stop & Save Calibration';
                btnCal.style.backgroundColor = 'var(--success, #56d364)'; // Green
                btnCal.style.borderColor = 'var(--success, #56d364)';
                btnCal.style.color = '#fff';
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
            const canvas = document.getElementById('scatterCanvas');
            if (!canvas) return;
            const ctx = canvas.getContext('2d');
            
            // Set internal resolution matching element size for crystal-clear high-DPI rendering
            if (canvas.width !== canvas.clientWidth || canvas.height !== canvas.clientHeight) {
                canvas.width = canvas.clientWidth;
                canvas.height = canvas.clientHeight;
            }
            
            const w = canvas.width, h = canvas.height;

            // Clear with dark slate background
            ctx.fillStyle = '#0d1117';
            ctx.fillRect(0, 0, w, h);
            
            // Draw center divider
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.08)';
            ctx.lineWidth = 1.5;
            ctx.beginPath(); 
            ctx.moveTo(w/2, 0); 
            ctx.lineTo(w/2, h); 
            ctx.stroke();

            if (calPoints.length === 0) return;

            const pad = 20;
            // Add padding around margins to keep points inside bounds
            const dMinX = minX - pad, dMaxX = maxX + pad;
            const dMinY = minY - pad, dMaxY = maxY + pad;
            const dMinZ = minZ - pad, dMaxZ = maxZ + pad;

            const map = (v, min, max, outMin, outMax) => (v - min) * (outMax - outMin) / (max - min) + outMin;

            // 1. Draw Left Half: X vs Y Projection (Horizontal Plane)
            calPoints.forEach((pt, idx) => {
                const alpha = idx / calPoints.length;
                ctx.fillStyle = `rgba(88, 166, 255, ${alpha * 0.85})`; // Theme Blue
                const xPos = map(pt.x, dMinX, dMaxX, 15, w/2 - 15);
                const yPos = map(pt.y, dMinY, dMaxY, h - 15, 15);
                ctx.beginPath();
                ctx.arc(xPos, yPos, 2.5, 0, 2 * Math.PI);
                ctx.fill();
            });

            // 2. Draw Right Half: Y vs Z Projection (Vertical Plane)
            calPoints.forEach((pt, idx) => {
                const alpha = idx / calPoints.length;
                ctx.fillStyle = `rgba(86, 211, 100, ${alpha * 0.85})`; // Theme Green
                const xPos = map(pt.y, dMinY, dMaxY, w/2 + 15, w - 15);
                const yPos = map(pt.z, dMinZ, dMaxZ, h - 15, 15);
                ctx.beginPath();
                ctx.arc(xPos, yPos, 2.5, 0, 2 * Math.PI);
                ctx.fill();
            });

            // 3. Draw Left Reticle (Offset center)
            const reticleLeftX = map(hix, dMinX, dMaxX, 15, w/2 - 15);
            const reticleLeftY = map(hiy, dMinY, dMaxY, h - 15, 15);
            ctx.strokeStyle = '#f85149'; // Theme Danger Red
            ctx.lineWidth = 1.5;
            ctx.beginPath();
            ctx.arc(reticleLeftX, reticleLeftY, 6, 0, 2 * Math.PI);
            ctx.stroke();
            ctx.beginPath();
            ctx.moveTo(reticleLeftX - 10, reticleLeftY);
            ctx.lineTo(reticleLeftX + 10, reticleLeftY);
            ctx.moveTo(reticleLeftX, reticleLeftY - 10);
            ctx.lineTo(reticleLeftX, reticleLeftY + 10);
            ctx.stroke();

            // 4. Draw Right Reticle (Offset center)
            const reticleRightX = map(hiy, dMinY, dMaxY, w/2 + 15, w - 15);
            const reticleRightY = map(hiz, dMinZ, dMaxZ, h - 15, 15);
            ctx.beginPath();
            ctx.arc(reticleRightX, reticleRightY, 6, 0, 2 * Math.PI);
            ctx.stroke();
            ctx.beginPath();
            ctx.moveTo(reticleRightX - 10, reticleRightY);
            ctx.lineTo(reticleRightX + 10, reticleRightY);
            ctx.moveTo(reticleRightX, reticleRightY - 10);
            ctx.lineTo(reticleRightX, reticleRightY + 10);
            ctx.stroke();
        }

        btnCal.onclick = () => {
            if (!isCalibrating) {
                isCalibrating = true;
                calPoints = [];
                minX = Infinity; maxX = -Infinity;
                minY = Infinity; maxY = -Infinity;
                minZ = Infinity; maxZ = -Infinity;
                enoughPointsMet = false;
                
                // Style button as gray (inactive calibrating look)
                btnCal.textContent = 'Calibrating... (Keep Rotating)';
                btnCal.style.backgroundColor = '#6c757d'; // Gray
                btnCal.style.borderColor = '#6c757d';
                btnCal.style.color = '#fff';
                btnSave.disabled = true;

                // Show bottom Co-Pilot bar INSTANTLY so they see it from a distance at the start!
                document.getElementById('calibrationCoPilot').style.display = 'flex';

                // Signal start of calibration to ESP32 to trigger buzzer beeps!
                fetch('/start_cal')
                    .catch(err => console.warn("Failed to trigger start beep", err));

                // 3-minute safety timeout (180 seconds)
                if (calTimeoutTimer) clearTimeout(calTimeoutTimer);
                calTimeoutTimer = setTimeout(() => {
                    if (isCalibrating) {
                        console.log("Calibration safety timeout reached (3 minutes). Auto-saving...");
                        stopAndSaveCalibration();
                    }
                }, 180000); // 3 minutes in ms

                pollData();
            } else {
                stopAndSaveCalibration();
            }
        };

        function stopAndSaveCalibration() {
            isCalibrating = false;
            enoughPointsMet = false;
            if (calTimeoutTimer) {
                clearTimeout(calTimeoutTimer);
                calTimeoutTimer = null;
            }
            if (pollTimer) {
                clearTimeout(pollTimer);
                pollTimer = null;
            }
            document.getElementById('calibrationCoPilot').style.display = 'none';
            btnCal.textContent = 'Start Interactive Calibration';
            btnCal.style.backgroundColor = ''; // Restore default styles
            btnCal.style.borderColor = '';
            btnCal.style.color = '';
            
            if (calPoints.length >= 50) {
                runEllipsoidFitting();
                pointCount.innerHTML = `<span style="color:var(--success); font-weight:bold;">Calibration Successful! Saving &amp; Redirecting...</span>`;
                
                // Automatically click save which sends parameter payload to NVS, plays success tune, and redirects
                setTimeout(() => {
                    btnSave.disabled = false;
                    btnSave.click();
                }, 100);
            }
        }

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
                        console.log("Calibration saved and uploaded successfully!");
                        
                        // Keep bottom co-pilot bar visible and show 5-second live countdown hint
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
            
            // For the first 350 points, or if the bounding box spans are too small,
            // the ellipsoid fit is highly ill-conditioned, so we use a stable centroid fallback to prevent wild jumps!
            const spanX = maxX - minX;
            const spanY = maxY - minY;
            if (calPoints.length < 350 || spanX < 30.0 || spanY < 30.0) {
                let sumX = 0, sumY = 0, sumZ = 0;
                calPoints.forEach(p => { sumX += p.x; sumY += p.y; sumZ += p.z; });
                hix = sumX / calPoints.length;
                hiy = sumY / calPoints.length;
                hiz = sumZ / calPoints.length;

                // Muted diagonal scale factors (unity fallback)
                six = 1.0; siy = 1.0; siz = 1.0;
                siMatrix = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
                
                calHiVal.textContent = `X: ${hix.toFixed(1)}, Y: ${hiy.toFixed(1)}, Z: ${hiz.toFixed(1)}`;
                calSiVal.textContent = `X: ${six.toFixed(2)}, Y: ${siy.toFixed(2)}, Z: ${siz.toFixed(2)}`;
                drawPoints();
                return;
            }

            const pts = calPoints.map(p => [p.x, p.y, p.z]);
            const fit = fitEllipsoid(pts);
            if (fit) {
                hix = fit.offset[0];
                hiy = fit.offset[1];
                hiz = fit.offset[2];
                siMatrix = fit.softIronMatrix;
                six = siMatrix[0][0];
                siy = siMatrix[1][1];
                siz = siMatrix[2][2];
                calHiVal.textContent = `X: ${hix.toFixed(1)}, Y: ${hiy.toFixed(1)}, Z: ${hiz.toFixed(1)}`;
                calSiVal.textContent = `X: ${six.toFixed(2)}, Y: ${siy.toFixed(2)}, Z: ${siz.toFixed(2)}`;
                drawPoints();
            }
        }

        function fitEllipsoid(points) {
            const N = points.length;
            if (N < 9) return null;

            // Check if Z-span is too small relative to horizontal spans, which indicates flat 2D rotation.
            // If flat, we run a highly stable 2D Circle Fit instead of collapsing a 3D Ellipsoid.
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
            const spanZ = maxZ_val - minZ_val;
            const maxSpan = Math.max(maxX_val - minX_val, maxY_val - minY_val, spanZ) || 1.0;
            const zSpanPct = spanZ / maxSpan;

            if (spanZ < 15.0 || zSpanPct < 0.35) {
                // Perform highly stable 2D circle fit on X and Y, keeping Z offset as mean and soft-iron as identity
                let sumZ = 0;
                points.forEach(p => { sumZ += p[2]; });
                const meanZ = sumZ / N;

                // 2D Circle Least Squares: solve M * [a; b; c] = V
                // where row i of design matrix is [x, y, 1] and B is -(x^2 + y^2)
                let Sxx = 0, Syy = 0, Sxy = 0, Sx = 0, Sy = 0;
                let Sxxx = 0, Sxyy = 0, Syyy = 0, Sxxy = 0;
                points.forEach(p => {
                    const x = p[0];
                    const y = p[1];
                    Sxx += x * x;
                    Syy += y * y;
                    Sxy += x * y;
                    Sx += x;
                    Sy += y;
                    Sxxx += x * x * x;
                    Sxyy += x * y * y;
                    Syyy += y * y * y;
                    Sxxy += x * x * y;
                });

                const M = [
                    [Sxx, Sxy, Sx],
                    [Sxy, Syy, Sy],
                    [Sx,  Sy,  N ]
                ];
                const V = [
                    -(Sxxx + Sxyy),
                    -(Sxxy + Syyy),
                    -(Sxx + Syy)
                ];

                const sol = solve3x3System(M, V);
                if (sol) {
                    const hix_val = -sol[0] / 2.0;
                    const hiy_val = -sol[1] / 2.0;
                    const hiz_val = meanZ;

                    return {
                        offset: [hix_val, hiy_val, hiz_val],
                        softIronMatrix: [
                            [1.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0],
                            [0.0, 0.0, 1.0]
                        ]
                    };
                }
            }

            // 1. Calculate centroid of points for zero-centering (pre-conditioning)
            let meanX = 0, meanY = 0, meanZ = 0;
            points.forEach(p => {
                meanX += p[0];
                meanY += p[1];
                meanZ += p[2];
            });
            meanX /= N; meanY /= N; meanZ /= N;

            // 2. Accumulate Design Matrix using zero-centered points to ensure excellent numerical conditioning
            const ATA = Array(9).fill(0).map(() => Array(9).fill(0));
            const ATB = Array(9).fill(0);
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
                    2.0 * z
                ];
                for (let i = 0; i < 9; i++) {
                    for (let j = 0; j < 9; j++) {
                        ATA[i][j] += r[i] * r[j];
                    }
                    ATB[i] += r[i];
                }
            });
            const coeff = solveLinearSystem(ATA, ATB);
            if (!coeff) return null;
            const A = [
                [coeff[0], coeff[3], coeff[4]],
                [coeff[3], coeff[1], coeff[5]],
                [coeff[4], coeff[5], coeff[2]]
            ];
            const b = [coeff[6], coeff[7], coeff[8]];
            const invA = invert3x3(A);
            if (!invA) return null;

            // Zero-centered offset
            const offset_zc = [
                -(invA[0][0] * b[0] + invA[0][1] * b[1] + invA[0][2] * b[2]),
                -(invA[1][0] * b[0] + invA[1][1] * b[1] + invA[1][2] * b[2]),
                -(invA[2][0] * b[0] + invA[2][1] * b[1] + invA[2][2] * b[2])
            ];

            // 3. Shift the offset back by adding the mean coordinates (un-centering)
            const offset = [
                offset_zc[0] + meanX,
                offset_zc[1] + meanY,
                offset_zc[2] + meanZ
            ];

            const vtAv = offset_zc[0] * (A[0][0] * offset_zc[0] + A[0][1] * offset_zc[1] + A[0][2] * offset_zc[2]) +
                         offset_zc[1] * (A[1][0] * offset_zc[0] + A[1][1] * offset_zc[1] + A[1][2] * offset_zc[2]) +
                         offset_zc[2] * (A[2][0] * offset_zc[0] + A[2][1] * offset_zc[1] + A[2][2] * offset_zc[2]);
            const d = 1.0 + vtAv;
            if (d <= 0) return null;
            const M = [
                [A[0][0]/d, A[0][1]/d, A[0][2]/d],
                [A[1][0]/d, A[1][1]/d, A[1][2]/d],
                [A[2][0]/d, A[2][1]/d, A[2][2]/d]
            ];

            // Calculate the exact Symmetric Matrix Square Root of M using Denman-Beavers quadratic convergence iteration
            let W = matrixSquareRoot3x3(M);
            if (!W) return null;

            // Calculate determinant of the symmetric matrix W
            const detW = W[0][0] * (W[1][1] * W[2][2] - W[2][1] * W[1][2]) -
                         W[0][1] * (W[1][0] * W[2][2] - W[1][2] * W[2][0]) +
                         W[0][2] * (W[1][0] * W[2][1] - W[1][1] * W[2][0]);

            if (Math.abs(detW) < 1e-6) return null; // Prevent singular matrix divide-by-zero or NaNs
            const scale = 1.0 / Math.cbrt(detW);
            for (let i = 0; i < 3; i++) {
                for (let j = 0; j < 3; j++) {
                    W[i][j] *= scale;
                }
            }
            return {
                offset: offset,
                softIronMatrix: W
            };
        }

        function matrixSquareRoot3x3(M) {
            let Y = [
                [M[0][0], M[0][1], M[0][2]],
                [M[1][0], M[1][1], M[1][2]],
                [M[2][0], M[2][1], M[2][2]]
            ];
            let Z = [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0]
            ];

            for (let k = 0; k < 5; k++) {
                const invY = invert3x3(Y);
                const invZ = invert3x3(Z);
                if (!invY || !invZ) return null;

                const nextY = [
                    [0.5 * (Y[0][0] + invZ[0][0]), 0.5 * (Y[0][1] + invZ[0][1]), 0.5 * (Y[0][2] + invZ[0][2])],
                    [0.5 * (Y[1][0] + invZ[1][0]), 0.5 * (Y[1][1] + invZ[1][1]), 0.5 * (Y[1][2] + invZ[1][2])],
                    [0.5 * (Y[2][0] + invZ[2][0]), 0.5 * (Y[2][1] + invZ[2][1]), 0.5 * (Y[2][2] + invZ[2][2])]
                ];

                const nextZ = [
                    [0.5 * (Z[0][0] + invY[0][0]), 0.5 * (Z[0][1] + invY[0][1]), 0.5 * (Z[0][2] + invY[0][2])],
                    [0.5 * (Z[1][0] + invY[1][0]), 0.5 * (Z[1][1] + invY[1][1]), 0.5 * (Z[1][2] + invY[1][2])],
                    [0.5 * (Z[2][0] + invY[2][0]), 0.5 * (Z[2][1] + invY[2][1]), 0.5 * (Z[2][2] + invY[2][2])]
                ];

                Y = nextY;
                Z = nextZ;
            }
            return Y;
        }

        function solveLinearSystem(A, B) {
            const n = 9;
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