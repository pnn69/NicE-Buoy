#ifndef CALIBRATION_HTML_H
#define CALIBRATION_HTML_H

const char CALIBRATION_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Magnetometer Calibration - ICM-20948</title>
    <style>
        :root {
            --bg-color: #0b0f19;
            --card-bg: rgba(17, 24, 39, 0.7);
            --card-border: rgba(255, 255, 255, 0.08);
            --text-color: #f3f4f6;
            --text-muted: #9ca3af;
            --cyan: #06b6d4;
            --green: #10b981;
            --purple: #8b5cf6;
            --orange: #f97316;
            --red: #ef4444;
        }

        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
            background-color: var(--bg-color);
            background-image: 
                radial-gradient(at 0% 0%, rgba(139, 92, 246, 0.1) 0px, transparent 50%),
                radial-gradient(at 100% 100%, rgba(6, 182, 212, 0.05) 0px, transparent 50%);
            background-attachment: fixed;
            color: var(--text-color);
            min-height: 100vh;
            padding: 1.5rem;
            line-height: 1.5;
        }

        .container {
            max-width: 900px;
            margin: 0 auto;
        }

        header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 2rem;
            border-bottom: 1px solid var(--card-border);
            padding-bottom: 1.5rem;
        }

        h1 {
            font-size: 1.75rem;
            font-weight: 800;
            letter-spacing: -0.025em;
            background: linear-gradient(to right, var(--purple), var(--cyan));
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }

        .card {
            background-color: var(--card-bg);
            backdrop-filter: blur(12px);
            -webkit-backdrop-filter: blur(12px);
            border: 1px solid var(--card-border);
            border-radius: 1rem;
            padding: 1.5rem;
            display: flex;
            flex-direction: column;
            gap: 1.25rem;
            box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.3);
            margin-bottom: 1.5rem;
        }

        .card-header {
            border-bottom: 1px solid rgba(255, 255, 255, 0.05);
            padding-bottom: 0.75rem;
        }

        .card-title {
            font-size: 1.25rem;
            font-weight: 700;
            display: flex;
            align-items: center;
            gap: 0.5rem;
            color: var(--purple);
        }

        .btn {
            display: inline-flex;
            align-items: center;
            justify-content: center;
            gap: 0.5rem;
            padding: 0.75rem 1.5rem;
            border-radius: 0.5rem;
            font-weight: 700;
            font-size: 0.95rem;
            cursor: pointer;
            transition: all 0.2s ease;
            border: none;
            outline: none;
            text-decoration: none;
        }

        .btn-purple {
            background: linear-gradient(to right, var(--purple), #7c3aed);
            color: white;
            box-shadow: 0 4px 14px rgba(139, 92, 246, 0.3);
        }
        .btn-purple:hover {
            transform: translateY(-1px);
            box-shadow: 0 6px 20px rgba(139, 92, 246, 0.4);
        }

        .btn-red {
            background: linear-gradient(to right, var(--red), #dc2626);
            color: white;
            box-shadow: 0 4px 14px rgba(239, 68, 68, 0.3);
        }
        .btn-red:hover {
            transform: translateY(-1px);
            box-shadow: 0 6px 20px rgba(239, 68, 68, 0.4);
        }

        .btn-outline {
            background-color: transparent;
            border: 1px solid var(--card-border);
            color: var(--text-color);
        }
        .btn-outline:hover {
            background-color: rgba(255, 255, 255, 0.03);
            border-color: rgba(255, 255, 255, 0.15);
        }

        .grid-3 {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 1rem;
        }

        .axis-box {
            background-color: rgba(0, 0, 0, 0.2);
            border-radius: 0.5rem;
            border: 1px solid rgba(255, 255, 255, 0.03);
            padding: 0.75rem;
            text-align: center;
        }

        .axis-title {
            font-weight: 800;
            font-size: 0.8rem;
            text-transform: uppercase;
            letter-spacing: 0.05em;
            margin-bottom: 0.5rem;
        }
        .axis-x { color: var(--cyan); }
        .axis-y { color: var(--green); }
        .axis-z { color: var(--purple); }

        .data-row {
            display: flex;
            justify-content: space-between;
            font-size: 0.85rem;
            color: var(--text-muted);
            margin-top: 0.25rem;
        }
        .data-val {
            font-family: monospace;
            font-weight: bold;
            color: var(--text-color);
        }

        .instructions {
            background-color: rgba(139, 92, 246, 0.04);
            border: 1px solid rgba(139, 92, 246, 0.15);
            border-radius: 0.75rem;
            padding: 1rem;
            display: flex;
            flex-direction: column;
            gap: 0.5rem;
        }

        .status-pill {
            display: inline-flex;
            align-items: center;
            gap: 0.375rem;
            padding: 0.25rem 0.625rem;
            border-radius: 9999px;
            font-size: 0.8rem;
            font-weight: bold;
            background-color: rgba(239, 68, 68, 0.1);
            color: var(--red);
            border: 1px solid rgba(239, 68, 68, 0.15);
        }
        .status-pill.active {
            background-color: rgba(16, 185, 129, 0.1);
            color: var(--green);
            border: 1px solid rgba(16, 185, 129, 0.15);
            animation: pulse-glow 2s infinite;
        }

        @keyframes pulse-glow {
            0% { box-shadow: 0 0 0 0 rgba(16, 185, 129, 0.2); }
            70% { box-shadow: 0 0 0 6px rgba(16, 185, 129, 0); }
            100% { box-shadow: 0 0 0 0 rgba(16, 185, 129, 0); }
        }
    </style>
</head>
<body>
    <div class="container">
        <header>
            <div>
                <h1>Magnetometer Calibration</h1>
                <p style="color: var(--text-muted); font-size: 0.9rem;">Hard-Iron & Soft-Iron Compensation Matrix Generator</p>
            </div>
            <a href="/" class="btn btn-outline">&larr; Back to Dashboard</a>
        </header>

        <!-- Main Calibration Action Card -->
        <div class="card">
            <div class="card-header" style="display: flex; justify-content: space-between; align-items: center;">
                <h2 class="card-title">
                    <svg width="20" height="20" fill="none" stroke="var(--purple)" stroke-width="2" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" d="M10.325 4.317c.426-1.756 2.924-1.756 3.35 0a1.724 1.724 0 002.573 1.066c1.543-.94 3.31.826 2.37 2.37a1.724 1.724 0 001.065 2.572c1.756.426 1.756 2.924 0 3.35a1.724 1.724 0 00-1.066 2.573c.94 1.543-.826 3.31-2.37 2.37a1.724 1.724 0 00-2.572 1.065c-.426 1.756-2.924 1.756-3.35 0a1.724 1.724 0 00-2.573-1.066c-1.543.94-3.31-.826-2.37-2.37a1.724 1.724 0 00-1.065-2.572c-1.756-.426-1.756-2.924 0-3.35a1.724 1.724 0 001.066-2.573c-.94-1.543.826-3.31 2.37-2.37.996.608 2.296.07 2.572-1.065z"/><path stroke-linecap="round" stroke-linejoin="round" d="M15 12a3 3 0 11-6 0 3 3 0 016 0z"/></svg>
                    Live Calibration Phase
                </h2>
                <div id="cal-status-pill" class="status-pill">
                    <span style="width:6px; height:6px; background-color:currentColor; border-radius:50%;"></span>
                    <span id="cal-status-text">Inactive</span>
                </div>
            </div>

            <div class="instructions">
                <div style="font-weight: bold; color: var(--purple);">Instructions for Calibration:</div>
                <div style="font-size: 0.9rem; color: var(--text-muted); display: flex; flex-direction: column; gap: 0.25rem;">
                    <div>1. Click <strong>Start Calibration</strong> below to begin collecting magnetic vectors.</div>
                    <div>2. Slowly rotate the sensor in all possible 3D directions (e.g., drawing horizontal/vertical figure-eights).</div>
                    <div>3. Ensure the sensor covers extreme tilts (pitch & roll) during rotation to capture 3D sphere boundaries.</div>
                    <div>4. Once at least 150+ samples are collected and offsets stabilize, click <strong>Stop & Save</strong>.</div>
                </div>
            </div>

            <!-- Controls -->
            <div style="display: flex; gap: 1rem; margin-top: 0.5rem;">
                <button id="btn-start" class="btn btn-purple" style="flex: 1;">
                    <svg width="20" height="20" fill="none" stroke="currentColor" stroke-width="2.5" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" d="M14.752 11.168l-3.197-2.132A1 1 0 0010 9.87v4.263a1 1 0 001.555.832l3.197-2.132a1 1 0 000-1.664z"/><path stroke-linecap="round" stroke-linejoin="round" d="M21 12a9 9 0 11-18 0 9 9 0 0118 0z"/></svg>
                    Start Calibration
                </button>
                <button id="btn-reset" class="btn btn-outline">Reset Buffer</button>
            </div>
        </div>

        <!-- Buffer Details Card -->
        <div class="card">
            <div class="card-header">
                <h2 class="card-title" style="color: var(--cyan);">Collected Buffer Statistics</h2>
            </div>
            
            <div style="display: flex; justify-content: space-between; align-items: center; background-color: rgba(0,0,0,0.15); padding: 0.75rem 1rem; border-radius: 0.5rem; border: 1px solid rgba(255,255,255,0.03);">
                <span style="font-weight: bold; font-size: 0.95rem;">Total Points In Buffer:</span>
                <span style="font-family: monospace; font-size: 1.5rem; font-weight: bold; color: var(--cyan);" id="point-counter">0</span>
            </div>

            <div class="grid-3">
                <!-- X Axis -->
                <div class="axis-box" style="border-top: 2px solid var(--cyan);">
                    <div class="axis-title axis-x">X Axis</div>
                    <div class="data-list" style="gap:0.25rem; margin-top:0.5rem;">
                        <div class="data-row"><span>Min:</span><span class="data-val" id="min-x">- µT</span></div>
                        <div class="data-row"><span>Max:</span><span class="data-val" id="max-x">- µT</span></div>
                    </div>
                </div>
                <!-- Y Axis -->
                <div class="axis-box" style="border-top: 2px solid var(--green);">
                    <div class="axis-title axis-y">Y Axis</div>
                    <div class="data-list" style="gap:0.25rem; margin-top:0.5rem;">
                        <div class="data-row"><span>Min:</span><span class="data-val" id="min-y">- µT</span></div>
                        <div class="data-row"><span>Max:</span><span class="data-val" id="max-y">- µT</span></div>
                    </div>
                </div>
                <!-- Z Axis -->
                <div class="axis-box" style="border-top: 2px solid var(--purple);">
                    <div class="axis-title axis-z">Z Axis</div>
                    <div class="data-list" style="gap:0.25rem; margin-top:0.5rem;">
                        <div class="data-row"><span>Min:</span><span class="data-val" id="min-z">- µT</span></div>
                        <div class="data-row"><span>Max:</span><span class="data-val" id="max-z">- µT</span></div>
                    </div>
                </div>
            </div>
        </div>

        <!-- Computed Parameters Card -->
        <div class="card">
            <div class="card-header">
                <h2 class="card-title" style="color: var(--green);">Estimated Compensation Parameters</h2>
            </div>
            
            <div style="display: flex; flex-direction: column; gap: 1rem;">
                <!-- Hard Iron Vector -->
                <div style="background-color: rgba(0,0,0,0.15); padding: 1rem; border-radius: 0.5rem; border: 1px solid rgba(255,255,255,0.03);">
                    <div style="font-weight: bold; color: var(--cyan); margin-bottom: 0.5rem;">Hard Iron Offset Vector (V_x, V_y, V_z)</div>
                    <div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 1rem; text-align: center; font-family: monospace; font-size: 1.15rem; font-weight: bold;">
                        <div style="background: rgba(0,0,0,0.25); padding: 0.5rem; border-radius: 0.25rem;" id="calc-vx">Vx: 0.00</div>
                        <div style="background: rgba(0,0,0,0.25); padding: 0.5rem; border-radius: 0.25rem;" id="calc-vy">Vy: 0.00</div>
                        <div style="background: rgba(0,0,0,0.25); padding: 0.5rem; border-radius: 0.25rem;" id="calc-vz">Vz: 0.00</div>
                    </div>
                </div>

                <!-- Soft Iron Matrix -->
                <div style="background-color: rgba(0,0,0,0.15); padding: 1rem; border-radius: 0.5rem; border: 1px solid rgba(255,255,255,0.03);">
                    <div style="font-weight: bold; color: var(--green); margin-bottom: 0.5rem;">Soft Iron Scaling Vector (S_x, S_y, S_z)</div>
                    <div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 1rem; text-align: center; font-family: monospace; font-size: 1.15rem; font-weight: bold;">
                        <div style="background: rgba(0,0,0,0.25); padding: 0.5rem; border-radius: 0.25rem;" id="calc-sx">Sx: 1.000</div>
                        <div style="background: rgba(0,0,0,0.25); padding: 0.5rem; border-radius: 0.25rem;" id="calc-sy">Sy: 1.000</div>
                        <div style="background: rgba(0,0,0,0.25); padding: 0.5rem; border-radius: 0.25rem;" id="calc-sz">Sz: 1.000</div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script>
        let isCalibrating = false;
        let points = []; // buffer to store collected {x, y, z}
        let pollInterval = null;

        // Min/Max trackers
        let minMax = {
            minX: Infinity, maxX: -Infinity,
            minY: Infinity, maxY: -Infinity,
            minZ: Infinity, maxZ: -Infinity
        };

        // UI DOM references
        const btnStart = document.getElementById('btn-start');
        const btnReset = document.getElementById('btn-reset');
        const calStatusPill = document.getElementById('cal-status-pill');
        const calStatusText = document.getElementById('cal-status-text');
        const pointCounter = document.getElementById('point-counter');

        const labelMinX = document.getElementById('min-x');
        const labelMaxX = document.getElementById('max-x');
        const labelMinY = document.getElementById('min-y');
        const labelMaxY = document.getElementById('max-y');
        const labelMinZ = document.getElementById('min-z');
        const labelMaxZ = document.getElementById('max-z');

        const labelVx = document.getElementById('calc-vx');
        const labelVy = document.getElementById('calc-vy');
        const labelVz = document.getElementById('calc-vz');
        const labelSx = document.getElementById('calc-sx');
        const labelSy = document.getElementById('calc-sy');
        const labelSz = document.getElementById('calc-sz');

        // Load current parameters from localStorage on startup
        window.onload = function() {
            const storedHard = localStorage.getItem('calibration_hard_iron');
            const storedSoft = localStorage.getItem('calibration_soft_iron');
            if (storedHard) {
                const hard = JSON.parse(storedHard);
                labelVx.innerText = `Vx: ${hard.vx.toFixed(2)}`;
                labelVy.innerText = `Vy: ${hard.vy.toFixed(2)}`;
                labelVz.innerText = `Vz: ${hard.vz.toFixed(2)}`;
            }
            if (storedSoft) {
                const soft = JSON.parse(storedSoft);
                labelSx.innerText = `Sx: ${soft.sx.toFixed(3)}`;
                labelSy.innerText = `Sy: ${soft.sy.toFixed(3)}`;
                labelSz.innerText = `Sz: ${soft.sz.toFixed(3)}`;
            }
        };

        btnStart.onclick = function() {
            if (!isCalibrating) {
                startCalibration();
            } else {
                stopAndSaveCalibration();
            }
        };

        btnReset.onclick = function() {
            resetCalibrationBuffer();
        };

        function startCalibration() {
            isCalibrating = true;
            btnStart.innerHTML = `
                <svg width="20" height="20" fill="none" stroke="currentColor" stroke-width="2.5" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" d="M21 12a9 9 0 11-18 0 9 9 0 0118 0z"/><path stroke-linecap="round" stroke-linejoin="round" d="M9 10a1 1 0 011-1h4a1 1 0 011 1v4a1 1 0 01-1 1h-4a1 1 0 01-1-1v-4z"/></svg>
                Stop & Save Calibration
            `;
            btnStart.className = "btn btn-red";
            calStatusPill.className = "status-pill active";
            calStatusText.innerText = "Calibrating";

            // Open high-speed real-time polling to capture points
            fetch('/start_cal').catch(err => console.error("Failed to start calibration:", err));

            pollInterval = setInterval(function() {
                fetch('/data')
                    .then(response => response.json())
                    .then(data => {
                        // Extract mx_raw, my_raw, mz_raw representing the raw sensor values polled at startup
                        if (data.mx_raw !== undefined && data.my_raw !== undefined && data.mz_raw !== undefined) {
                            recordPoint(data.mx_raw, data.my_raw, data.mz_raw);
                        }
                    })
                    .catch(err => console.error("Error polling data", err));
            }, 100);
        }

        function recordPoint(mx, my, mz) {
            points.push({ x: mx, y: my, z: mz });
            pointCounter.innerText = points.length;

            // Update limits on-the-fly
            if (mx < minMax.minX) minMax.minX = mx;
            if (mx > minMax.maxX) minMax.maxX = mx;
            if (my < minMax.minY) minMax.minY = my;
            if (my > minMax.maxY) minMax.maxY = my;
            if (mz < minMax.minZ) minMax.minZ = mz;
            if (mz > minMax.maxZ) minMax.maxZ = mz;

            // Render stats
            labelMinX.innerText = `${minMax.minX.toFixed(1)} µT`;
            labelMaxX.innerText = `${minMax.maxX.toFixed(1)} µT`;
            labelMinY.innerText = `${minMax.minY.toFixed(1)} µT`;
            labelMaxY.innerText = `${minMax.maxY.toFixed(1)} µT`;
            labelMinZ.innerText = `${minMax.minZ.toFixed(1)} µT`;
            labelMaxZ.innerText = `${minMax.maxZ.toFixed(1)} µT`;

            // Calculate estimated hard-iron on-the-fly
            const vx = (minMax.maxX + minMax.minX) / 2;
            const vy = (minMax.maxY + minMax.minY) / 2;
            const vz = (minMax.maxZ + minMax.minZ) / 2;

            labelVx.innerText = `Vx: ${vx.toFixed(2)}`;
            labelVy.innerText = `Vy: ${vy.toFixed(2)}`;
            labelVz.innerText = `Vz: ${vz.toFixed(2)}`;

            // Calculate estimated soft-iron scaling on-the-fly
            const avg_range_x = (minMax.maxX - minMax.minX) / 2;
            const avg_range_y = (minMax.maxY - minMax.minY) / 2;
            const avg_range_z = (minMax.maxZ - minMax.minZ) / 2;

            if (avg_range_x > 0 && avg_range_y > 0 && avg_range_z > 0) {
                const average_range = (avg_range_x + avg_range_y + avg_range_z) / 3;
                const sx = average_range / avg_range_x;
                const sy = average_range / avg_range_y;
                const sz = average_range / avg_range_z;

                labelSx.innerText = `Sx: ${sx.toFixed(3)}`;
                labelSy.innerText = `Sy: ${sy.toFixed(3)}`;
                labelSz.innerText = `Sz: ${sz.toFixed(3)}`;
            }
        }

        function stopAndSaveCalibration() {
            isCalibrating = false;
            btnStart.innerHTML = `
                <svg width="20" height="20" fill="none" stroke="currentColor" stroke-width="2.5" viewBox="0 0 24 24"><path stroke-linecap="round" stroke-linejoin="round" d="M14.752 11.168l-3.197-2.132A1 1 0 0010 9.87v4.263a1 1 0 001.555.832l3.197-2.132a1 1 0 000-1.664z"/><path stroke-linecap="round" stroke-linejoin="round" d="M21 12a9 9 0 11-18 0 9 9 0 0118 0z"/></svg>
                Start Calibration
            `;
            btnStart.className = "btn btn-purple";
            calStatusPill.className = "status-pill";
            calStatusText.innerText = "Inactive";

            if (pollInterval) {
                clearInterval(pollInterval);
                pollInterval = null;
            }

            // Save calculated values to localStorage and ESP32 NVS
            if (points.length >= 10) {
                const vx = (minMax.maxX + minMax.minX) / 2;
                const vy = (minMax.maxY + minMax.minY) / 2;
                const vz = (minMax.maxZ + minMax.minZ) / 2;

                const avg_range_x = (minMax.maxX - minMax.minX) / 2;
                const avg_range_y = (minMax.maxY - minMax.minY) / 2;
                const avg_range_z = (minMax.maxZ - minMax.minZ) / 2;

                let sx = 1.0, sy = 1.0, sz = 1.0;
                if (avg_range_x > 0 && avg_range_y > 0 && avg_range_z > 0) {
                    const average_range = (avg_range_x + avg_range_y + avg_range_z) / 3;
                    sx = average_range / avg_range_x;
                    sy = average_range / avg_range_y;
                    sz = average_range / avg_range_z;
                }

                localStorage.setItem('calibration_hard_iron', JSON.stringify({ vx, vy, vz }));
                localStorage.setItem('calibration_soft_iron', JSON.stringify({ sx, sy, sz }));

                // Save parameters to ESP32 server NVS via /save_cal endpoint
                const url = `/save_cal?hx=${vx.toFixed(2)}&hy=${vy.toFixed(2)}&hz=${vz.toFixed(2)}&sx=${sx.toFixed(3)}&sy=${sy.toFixed(3)}&sz=${sz.toFixed(3)}`;
                fetch(url)
                    .then(response => response.text())
                    .then(text => {
                        if (text === "OK") {
                            alert(`Calibration Saved successfully!\nSamples collected: ${points.length}\n\nVx: ${vx.toFixed(2)}, Vy: ${vy.toFixed(2)}, Vz: ${vz.toFixed(2)}\nSx: ${sx.toFixed(3)}, Sy: ${sy.toFixed(3)}, Sz: ${sz.toFixed(3)}`);
                        } else {
                            alert(`Calibration saved locally in browser, but failed to save to ESP32 NVS! Response: ${text}`);
                        }
                    })
                    .catch(err => {
                        alert(`Calibration saved locally in browser, but failed to connect to ESP32 to save NVS! Error: ${err.message}`);
                    });
            } else {
                alert("Calibration stopped, but not saved (insufficient points collected). Please collect at least 10+ samples.");
            }
        }

        function resetCalibrationBuffer() {
            points = [];
            pointCounter.innerText = "0";
            minMax = {
                minX: Infinity, maxX: -Infinity,
                minY: Infinity, maxY: -Infinity,
                minZ: Infinity, maxZ: -Infinity
            };
            labelMinX.innerText = "- µT";
            labelMaxX.innerText = "- µT";
            labelMinY.innerText = "- µT";
            labelMaxY.innerText = "- µT";
            labelMinZ.innerText = "- µT";
            labelMaxZ.innerText = "- µT";

            labelVx.innerText = "Vx: 0.00";
            labelVy.innerText = "Vy: 0.00";
            labelVz.innerText = "Vz: 0.00";
            labelSx.innerText = "Sx: 1.000";
            labelSy.innerText = "Sy: 1.000";
            labelSz.innerText = "Sz: 1.000";
        }
    </script>
</body>
</html>
)rawliteral";

#endif /* CALIBRATION_HTML_H */
