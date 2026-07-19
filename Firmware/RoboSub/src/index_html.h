#ifndef INDEX_HTML_H
#define INDEX_HTML_H

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>NicE-Buoy Sub</title><style>
body{font-family:Arial,sans-serif;background:#1a1a1a;color:#fff;text-align:center;margin:0;padding:10px}
.compass-wrapper{position:relative;width:250px;height:250px;margin:10px auto;display:block;}
.compass{width:100%;height:100%;}
.needle{transform-origin:100px 100px;transition:transform 0.15s cubic-bezier(0.25, 0.46, 0.45, 0.94);}
h2{margin:5px 0;color:#00d1ff}
.info{font-size:1.6em;margin-bottom:10px;display:flex;justify-content:center;gap:20px}
.icm{color:#00d1ff;font-weight:bold}
.raw-container{display:flex;justify-content:center;gap:10px;flex-wrap:wrap}
.raw-box{text-align:left;border:1px solid #333;padding:10px;border-radius:8px;background:#252525;min-width:280px}
.axis-row{display:flex;align-items:center;margin:8px 0;font-size:0.9em}
input,select{background:#333;color:#fff;border:1px solid #555;padding:4px;border-radius:3px;width:70px;margin-left:5px}
button{padding:6px 12px;background:#00d1ff;color:#1a1a1a;border:none;cursor:pointer;border-radius:4px;font-weight:bold;margin-left:5px}
.main-row{display:flex;justify-content:center;align-items:center;gap:10px;margin:10px auto;max-width:600px}
.side-panel{width:60px;font-size:0.9em;font-weight:bold}
.side-bar{width:30px;height:250px;background:#333;border:1px solid #555;border-radius:4px;position:relative;margin:5px auto;overflow:hidden}
.thruster-bar{width:100%;position:absolute;left:0;transition:height 0.1s,top 0.1s,bottom 0.1s}
.zero-line{position:absolute;width:100%;height:2px;background:#888;top:50%;z-index:1}
.cal-msg{color:#ffcc00;font-size:0.9em;margin:5px 0;min-height:1.2em;font-weight:bold}
</style></head><body>
<h2 id="mainTitle">NicE-Buoy Sub</h2>
<div class="info">
    <span>Heading: <span id="icmVal" class="icm">0</span>&deg;</span>
    <span style="font-size:0.85em; margin-left:15px; color:#aaa;">P: <span id="pitchVal" style="color:#00e6ff;font-weight:bold;">0.0</span>&deg;</span>
    <span style="font-size:0.85em; margin-left:15px; color:#aaa;">R: <span id="rollVal" style="color:#00e6ff;font-weight:bold;">0.0</span>&deg;</span>
</div>
<div style="display:flex;justify-content:center;gap:15px;align-items:center;margin:5px 0;min-height:1.2em;">
<div id="subStatus" style="color:#00d1ff;font-size:0.9em;font-weight:bold;">STATE: UNKNOWN</div>
<div id="calMsg" class="cal-msg" style="margin:0;">Initializing...</div>
<div style="font-size:0.9em;font-weight:bold;color:#aaa;">Ping: <span id="pingVal" style="color:#ffcc00;">0</span>ms</div>
</div>
<div class="main-row">
<div class="side-panel"><div>BB</div><div class="side-bar"><div class="zero-line"></div><div id="bb_bar" class="thruster-bar"></div></div><div><span id="bb_val">0</span>%</div><div style="font-size:0.85em;margin-top:5px;color:#00d1ff">Is:<span id="is_val">0.00</span></div></div>
<div style="display:flex;flex-direction:column;align-items:center;justify-content:center;">
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
<div style="width:400px;margin-top:10px;text-align:left;">
<div style="width:100%;height:30px;background:#333;border:1px solid #555;border-radius:4px;position:relative;overflow:hidden;margin-bottom:8px;">
<div id="vattBar" style="position:absolute;left:0;top:0;height:100%;width:0%;transition:width 0.2s,background-color 0.2s;"></div>
<div id="vattVal" style="position:absolute;width:100%;text-align:center;line-height:30px;font-weight:bold;font-size:1.1em;color:#fff;z-index:2;">0.0V</div>
</div>
<div style="width:100%;height:30px;background:#333;border:1px solid #555;border-radius:4px;position:relative;overflow:hidden;">
<div id="currBar" style="position:absolute;left:0;top:0;height:100%;width:0%;transition:width 0.2s;background-color:#5a32a8;"></div>
<div id="currVal" style="position:absolute;width:100%;text-align:center;line-height:30px;font-weight:bold;font-size:1.1em;color:#fff;z-index:2;">0.00A</div>
</div>
</div>
</div>
<div class="side-panel"><div>SB</div><div class="side-bar"><div class="zero-line"></div><div id="sb_bar" class="thruster-bar"></div></div><div><span id="sb_val">0</span>%</div><div style="font-size:0.85em;margin-top:5px;color:#ffcc00">Ir:<span id="ir_val">0.00</span></div></div>
</div>
<div class="raw-container">
<div class="raw-box"><b>Rudder PID</b>
<div class="axis-row">P:<input type="number" step="0.1" id="kpr_in"><button onclick="setParam('kpr')">Set</button></div>
<div class="axis-row">I:<input type="number" step="0.01" id="kir_in"><button onclick="setParam('kir')">Set</button></div>
<div class="axis-row">D:<input type="number" step="0.01" id="kdr_in"><button onclick="setParam('kdr')">Set</button></div>
</div>
<div class="raw-box"><b>Speed PID</b>
<div class="axis-row">P:<input type="number" step="0.1" id="kps_in"><button onclick="setParam('kps')">Set</button></div>
<div class="axis-row">I:<input type="number" step="0.01" id="kis_in"><button onclick="setParam('kis')">Set</button></div>
<div class="axis-row">D:<input type="number" step="0.01" id="kds_in"><button onclick="setParam('kds')">Set</button></div>
</div>
<div class="raw-box"><b>Compass</b>
<div class="axis-row">Off:<input type="number" id="coff_in"><button onclick="setParam('coff')">Set</button></div>
<div class="axis-row">Rad:<input type="number" step="0.1" id="holdrad_in"><button onclick="setParam('holdrad')">Set</button></div>
<div class="axis-row">Avg:<input type="number" id="cavg_in" min="1" max="200"><button onclick="setParam('cavg')">Set</button></div>
<button onclick="setAsNorth()" style="background:#ffcc00;color:#1a1a1a;width:100%;margin-top:8px;font-weight:bold;height:35px;border-radius:4px;border:none;cursor:pointer;">Set Current as North</button>
<div style="font-size:0.95em;font-family:monospace;color:#aaa;margin-top:8px;text-align:center;line-height:1.4;font-weight:600;">Loaded Profile (NVS):<br><span id="main_cal_load" style="color:#ffcc00">Loading...</span><br>Selected Mode:<br><span id="main_icm_mode" style="color:#58a6ff">Loading...</span></div>
</div>
<div class="raw-box"><b>Adaptive Trim</b>
<div class="axis-row" style="margin-bottom:0;">Trim:<span id="val_ctrim" style="font-weight:bold;color:#ffcc00;font-size:1.15em;">0.00°</span></div>
<div style="display:flex;gap:6px;margin-top:8px;width:100%;">
<button id="btn_trim_toggle" onclick="toggleTrim()" style="flex:1.2;font-weight:bold;height:32px;border-radius:4px;border:none;cursor:pointer;background:#58a6ff;color:#0d1117;font-size:0.9em;transition:all 0.15s;">Enable</button>
<button onclick="clearTrim()" style="flex:0.8;font-weight:bold;height:32px;border-radius:4px;border:none;cursor:pointer;background:#f85149;color:white;font-size:0.9em;">Clear</button>
</div>
</div>
<div class="raw-box"><b>Speed Limits</b>
<div class="axis-row">Min:<input type="number" id="minspd_in"><button onclick="setParam('minspd')">Set</button></div>
<div class="axis-row">Max:<input type="number" id="maxspd_in"><button onclick="setParam('maxspd')">Set</button></div>
<div class="axis-row">Piv:<input type="number" step="0.01" id="pvspd_in"><button onclick="setParam('pvspd')">Set</button></div>
</div>
<div class="raw-box"><b>Thrusters</b>
<div class="axis-row">BB Inv:<select id="revbb_in" onchange="setParam('revbb')"><option value="0">Normal</option><option value="1">Inverted</option></select></div>
<div class="axis-row">SB Inv:<select id="revsb_in" onchange="setParam('revsb')"><option value="0">Normal</option><option value="1">Inverted</option></select></div>
<div class="axis-row">Swap:<select id="tswap_in" onchange="setParam('tswap')"><option value="0">Normal</option><option value="1">Swapped</option></select></div>
</div>
<div class="raw-box" style="display:flex;flex-direction:column;justify-content:center;"><b>Compass Configuration</b>
<button onclick="location.href='/calibration'" style="background:#58a6ff;color:#0d1117;width:100%;height:50px;margin-top:10px;font-weight:bold;font-size:1em;border-radius:4px;border:none;cursor:pointer;">➔ Interactive Calibration</button>
<button onclick="location.href='/ShowActualData'" style="background:#10b981;color:white;width:100%;height:50px;margin-top:8px;font-weight:bold;font-size:1em;border-radius:4px;border:none;cursor:pointer;">➔ View 3D & Analytical Data</button>
</div>
</div>
<script>
let rotTilt = 0, lastParamRev = -1;

function getShortestRotation(current, target) {
    target = (target % 360 + 360) % 360;
    let currentNorm = (current % 360 + 360) % 360;
    let diff = target - currentNorm;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    return current + diff;
}

function updateThruster(id,v){const b=document.getElementById(id+'_bar'),l=document.getElementById(id+'_val');l.innerText=v;let h=Math.min(Math.abs(v),100)/2;b.style.height=h+'%';if(v<0){b.style.top='50%';b.style.bottom='auto';b.style.backgroundColor='red'}else{b.style.top='auto';b.style.bottom='50%';b.style.backgroundColor='green'}}
function fetchData(){
    const startTime = Date.now();

    fetch('/data')
    .then(r=>r.json())
    .then(d=>{
        const pingElem = document.getElementById('pingVal');
        if (pingElem) pingElem.innerText = Date.now() - startTime;
        document.getElementById('icmVal').innerText=Math.round(d.icm);
        document.getElementById('calMsg').innerText=d.cal_msg;
        if(d.cal_load !== undefined) {
            document.getElementById('main_cal_load').innerText = d.cal_load;
        }
        if(d.icm_mode !== undefined) {
            const modes = {
                1: "1. Only Hard Iron (No Soft, No Tilt)",
                2: "2. Hard & Soft Iron (No Tilt)",
                3: "3. Hard Iron & Pitch + Roll",
                4: "4. Hard & Soft Iron & Pitch + Roll"
            };
            document.getElementById('main_icm_mode').innerText = modes[d.icm_mode] || "Unknown";
        }
        const statusElem = document.getElementById('subStatus');
        if (statusElem && d.status_str) {
            statusElem.innerText = 'STATE: ' + d.status_str;
        }
        if(d.mac)document.getElementById('mainTitle').innerText='NicE-Buoy Sub '+d.mac;
        updateThruster('bb',d.speed_bb);
        updateThruster('sb',d.speed_sb);
        
        const isElem = document.getElementById('is_val');
        if (isElem && d.ip !== undefined) {
            isElem.innerText = d.ip.toFixed(2);
        }
        const irElem = document.getElementById('ir_val');
        if (irElem && d.ir !== undefined) {
            irElem.innerText = d.ir.toFixed(2);
        }
        
        if (d.pitch !== undefined) {
            document.getElementById('pitchVal').innerText = d.pitch.toFixed(1);
        }
        if (d.roll !== undefined) {
            document.getElementById('rollVal').innerText = d.roll.toFixed(1);
        }
        
        if (d.rev !== undefined) {
            if (lastParamRev !== -1 && d.rev > lastParamRev) {
                fetchParams();
            }
            lastParamRev = d.rev;
        }

        if (d.vatt !== undefined) {
            const v = d.vatt;
            const bar = document.getElementById('vattBar');
            const val = document.getElementById('vattVal');
            val.innerText = v.toFixed(1) + 'V';
            
            const pct = Math.max(0, Math.min(100, ((v - 17) / 9) * 100));
            bar.style.width = pct + '%';
            
            if (v >= 22) {
                bar.style.backgroundColor = '#2a6a2a'; // Green from 22 til 26
            } else if (v >= 19) {
                bar.style.backgroundColor = '#ffcc00'; // Yellow from 19 till 22
            } else {
                bar.style.backgroundColor = '#ff3333'; // Red from 17 till 19
            }
        }

        if (d.curr !== undefined) {
            const c = d.curr;
            const bar = document.getElementById('currBar');
            const val = document.getElementById('currVal');
            val.innerText = c.toFixed(2) + 'A';
            const pct = Math.max(0, Math.min(100, ((c + 20) / 40) * 100));
            bar.style.width = pct + '%';
        }

        if (d.ctrim !== undefined && d.ctrim_en !== undefined) {
            const t = d.ctrim;
            const en = d.ctrim_en == 1;
            console.log("Telemetry received -> ctrim:", t, "ctrim_en:", en);
            currentTrimEnabled = en;
            const valElem = document.getElementById('val_ctrim');
            if (valElem) {
                valElem.innerText = (t >= 0 ? '+' : '') + t.toFixed(2) + '°' + (en ? ' (Enabled)' : ' (Disabled)');
                valElem.style.color = en ? '#56d364' : '#ffcc00'; // Green if enabled, yellow if disabled
            }
            const btnElem = document.getElementById('btn_trim_toggle');
            if (btnElem) {
                if (en) {
                    btnElem.innerText = 'Disable';
                    btnElem.style.backgroundColor = '#f85149';
                    btnElem.style.color = 'white';
                } else {
                    btnElem.innerText = 'Enable';
                    btnElem.style.backgroundColor = '#58a6ff';
                    btnElem.style.color = '#0d1117';
                }
            }
        }
        
        rotTilt = getShortestRotation(rotTilt, d.icm);
        document.getElementById('rose-tilt-needle').style.transform = `rotate(${rotTilt}deg)`;
    })
    .catch(e=>{
        console.error("fetchData failed:", e);
    });
}
let currentTrimEnabled = false;
function toggleTrim() {
    const nextState = currentTrimEnabled ? 0 : 1;
    console.log("toggleTrim clicked. currentTrimEnabled:", currentTrimEnabled, "nextState:", nextState);
    fetch(`/setparam?p=ctrim_en&v=${nextState}`).then(() => setTimeout(fetchParams, 100));
}
function clearTrim() {
    console.log("clearTrim clicked. Clearing trim instantly.");
    fetch(`/setparam?p=ctrim_clr&v=1`).then(() => setTimeout(fetchParams, 100));
}
function setParam(p){const e=document.getElementById(p+'_in');if(!e)return;fetch('/setparam?p='+p+'&v='+e.value).then(()=>setTimeout(fetchParams,300))}
function setAsNorth(){
    fetch('/set_north')
        .then(r => r.text())
        .then(txt => {
            if(txt === "OK") {
                console.log('Compass calibrated to North successfully!');
            } else {
                console.error('Error calibrating North: ' + txt);
            }
        })
        .catch(e => console.error('Network error: ' + e));
}
function fetchParams(){fetch('/params?t='+Date.now()).then(r=>r.json()).then(d=>{let missing=false;['kpr','kir','kdr','kps','kis','kds','coff','pvspd','revbb','revsb','tswap','minspd','maxspd','holdrad','cavg'].forEach(p=>{const e=document.getElementById(p+'_in');if(e){if(d[p]!==undefined){if(document.activeElement!==e)e.value=d[p]}else missing=true}});if(missing||Object.keys(d).length<5)setTimeout(fetchParams,1000)}).catch(e=>{console.error(e);setTimeout(fetchParams,1000)})}
fetchParams();

setInterval(fetchData, 100);
</script></body></html>
)rawliteral";

#endif /* INDEX_HTML_H */