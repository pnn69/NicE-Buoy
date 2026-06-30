#ifndef INDEX_HTML_H
#define INDEX_HTML_H

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>NicE-Buoy Sub</title><style>
body{font-family:Arial,sans-serif;background:#1a1a1a;color:#fff;text-align:center;margin:0;padding:10px}
canvas{background:#2a2a2a;border-radius:50%;margin:10px auto;border:2px solid #444;max-width:90%;height:auto;display:block}
h2{margin:5px 0;color:#00d1ff}
.info{font-size:1.1em;margin-bottom:10px;display:flex;justify-content:center;gap:20px}
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
<div class="info"><span>Heading: <span id="icmVal" class="icm">0.0</span>&deg;</span><span>Ping: <span id="pingVal" style="color:#ffcc00;font-weight:bold">0</span>ms</span></div>
<div style="display:flex;justify-content:center;gap:15px;align-items:center;margin:5px 0;min-height:1.2em;">
<div id="subStatus" style="color:#00d1ff;font-size:0.9em;font-weight:bold;">STATE: UNKNOWN</div>
<div id="calMsg" class="cal-msg" style="margin:0;">Initializing...</div>
</div>
<div class="main-row">
<div class="side-panel"><div>BB</div><div class="side-bar"><div class="zero-line"></div><div id="bb_bar" class="thruster-bar"></div></div><div><span id="bb_val">0</span>%</div><div style="font-size:0.85em;margin-top:5px;color:#00d1ff">Is:<span id="is_val">0.00</span></div></div>
<div style="display:flex;flex-direction:column;align-items:center;justify-content:center;">
<canvas id="compassCanvas" width="400" height="400" style="margin:0 auto;display:block;"></canvas>
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
</div>
<div class="raw-box"><b>Speed Limits</b>
<div class="axis-row">Min:<input type="number" id="minspd_in"><button onclick="setParam('minspd')">Set</button></div>
<div class="axis-row">Max:<input type="number" id="maxspd_in"><button onclick="setParam('maxspd')">Set</button></div>
<div class="axis-row">Piv:<input type="number" step="0.01" id="pvspd_in"><button onclick="setParam('pvspd')">Set</button></div>
<div class="axis-row">Rad:<input type="number" step="0.1" id="holdrad_in"><button onclick="setParam('holdrad')">Set</button></div>
</div>
<div class="raw-box"><b>Thrusters</b>
<div class="axis-row">BB Inv:<select id="revbb_in" onchange="setParam('revbb')"><option value="0">Normal</option><option value="1">Inverted</option></select></div>
<div class="axis-row">SB Inv:<select id="revsb_in" onchange="setParam('revsb')"><option value="0">Normal</option><option value="1">Inverted</option></select></div>
<div class="axis-row">Swap:<select id="tswap_in" onchange="setParam('tswap')"><option value="0">Normal</option><option value="1">Swapped</option></select></div>
</div>
<div class="raw-box"><b>BNO Persistence</b>
<div style="font-size:0.7em;font-family:monospace;word-break:break-all;color:#aaa">L:<span id="cal_load">None</span><br>V:<span id="cal_ver">None</span></div>
<button id="calButton" onclick="startCalib()" style="background:#5a32a8;color:#fff;width:100%;margin-top:5px">BNO Status</button>
<button onclick="saveCalib()" style="background:#2a6a2a;color:#fff;width:100%;margin-top:5px">Save Calib</button>
</div>
</div>
<script>
const ctx=document.getElementById('compassCanvas').getContext('2d'),cx=200,cy=200,r=180;
let currentHeading = 0, targetHeading = 0, headingInitialized = false, lastParamRev = -1;

function smoothDraw() {
    let diff = targetHeading - currentHeading;
    while (diff < -180) diff += 360;
    while (diff > 180) diff -= 360;
    currentHeading += diff * 0.15;
    while (currentHeading < 0) currentHeading += 360;
    while (currentHeading >= 360) currentHeading -= 360;
    drawRose(currentHeading);
    requestAnimationFrame(smoothDraw);
}

function updateThruster(id,v){const b=document.getElementById(id+'_bar'),l=document.getElementById(id+'_val');l.innerText=v;let h=Math.min(Math.abs(v),100)/2;b.style.height=h+'%';if(v<0){b.style.top='50%';b.style.bottom='auto';b.style.backgroundColor='red'}else{b.style.top='auto';b.style.bottom='50%';b.style.backgroundColor='green'}}
function drawRose(h){ctx.clearRect(0,0,400,400);ctx.beginPath();ctx.arc(cx,cy,r,0,2*Math.PI);ctx.strokeStyle='#555';ctx.lineWidth=2;ctx.stroke();ctx.fillStyle='#888';ctx.font='bold 20px Arial';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText('N',cx,cy-r+20);ctx.fillText('S',cx,cy+r-20);ctx.fillText('E',cx+r-20,cy);ctx.fillText('W',cx-r+20,cy);const a=(h-90)*Math.PI/180;ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(cx+(r-40)*Math.cos(a),cy+(r-40)*Math.sin(a));ctx.strokeStyle='#00d1ff';ctx.lineWidth=4;ctx.stroke()}
function fetchData(){
    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), 1000);
    const startTime = Date.now();

    fetch('/data', { signal: controller.signal })
    .then(r=>r.json())
    .then(d=>{
        clearTimeout(timeoutId);
        const pingElem = document.getElementById('pingVal');
        if (pingElem) pingElem.innerText = Date.now() - startTime;
        document.getElementById('icmVal').innerText=d.icm.toFixed(1);
        document.getElementById('calMsg').innerText=d.cal_msg;
        const statusElem = document.getElementById('subStatus');
        if (statusElem && d.status_str) {
            statusElem.innerText = 'STATE: ' + d.status_str;
        }
        document.getElementById('cal_load').innerText=d.cal_load;
        document.getElementById('cal_ver').innerText=d.cal_ver;
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
        
        if (!headingInitialized) {
            currentHeading = d.icm;
            targetHeading = d.icm;
            headingInitialized = true;
            requestAnimationFrame(smoothDraw);
        } else {
            targetHeading = d.icm;
        }

        const c=d.cal_levels,b=document.getElementById('calButton');
        if(c&&c[3]===3){b.style.background='#2a6a2a';b.innerText='BNO Calibrated (3)'}
        else{b.style.background='#5a32a8';b.innerText='BNO Status'}
    })
    .catch(e=>{
        if (e.name !== 'AbortError') {
            console.error("fetchData failed:", e);
        }
    })
    .finally(()=>{
        clearTimeout(timeoutId);
        setTimeout(fetchData, 250);
    });
}
function startCalib(){alert('BNO055 calibrates automatically while moving. Rotate the buoy until Mag (M) shows 3.')}
function saveCalib(){if(confirm('Save current BNO calibration?')){fetch('/savecal').then(r=>alert('Save Sent!'))}}
function setParam(p){const e=document.getElementById(p+'_in');if(!e)return;fetch('/setparam?p='+p+'&v='+e.value).then(()=>setTimeout(fetchParams,300))}
function fetchParams(){fetch('/params?t='+Date.now()).then(r=>r.json()).then(d=>{let missing=false;['kpr','kir','kdr','kps','kis','kds','coff','pvspd','revbb','revsb','tswap','minspd','maxspd','holdrad'].forEach(p=>{const e=document.getElementById(p+'_in');if(e){if(d[p]!==undefined){if(document.activeElement!==e)e.value=d[p]}else missing=true}});if(missing||Object.keys(d).length<5)setTimeout(fetchParams,1000)}).catch(e=>{console.error(e);setTimeout(fetchParams,1000)})}
fetchParams();

fetchData();
</script></body></html>
)rawliteral";

#endif
