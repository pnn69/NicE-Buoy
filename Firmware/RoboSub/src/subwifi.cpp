/**
 * @file subwifi.cpp
 * @brief Dashboard and WiFi Management for NicE-Buoy Sub.
 * 
 * This module handles:
 * 1. WiFi connectivity via WiFiManager.
 * 2. WebServer (Port 80) providing the real-time Windrose and PID dashboard.
 * 3. JSON API (/data, /params) for live telemetry and parameter updates.
 * 4. AsyncUDP broadcasting for fleet coordination.
 * 
 * Task Strategy: Runs on Core 0 to leave Core 1 free for navigation and sensors.
 */

#include <WiFi.h>
#include <WiFiManager.h> 
#include <AsyncUDP.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Adafruit_Sensor.h>
#include <RoboCompute.h>
#include "main.h"
#include "io_sub.h"
#include "compass.h"
#include "datastorage.h"
#include "pidrudspeed.h"

// Global instances
WebServer subServer(80);
AsyncUDP udp;
extern float global_speed_bb;
extern float global_speed_sb;
extern bool icm_ready;
extern float global_icmHdg;
extern RoboStruct mainData;
extern SemaphoreHandle_t mainDataMutex;
extern QueueHandle_t compassIn;
extern float last_raw_x, last_raw_y, last_raw_z;
extern uint8_t bno_cal_sys, bno_cal_gyro, bno_cal_accel, bno_cal_mag;
extern String global_cal_msg;
extern String global_cal_load, global_cal_ver;
extern uint32_t global_loop_cnt;


static RoboStruct subWifiOut;
static RoboStruct subWifiIn;
QueueHandle_t udpOut = NULL;
QueueHandle_t udpIn = NULL;
String global_mac_str = "";

/**
 * @brief Dashboard HTML/CSS/JS Source.
 * Features a dynamic Windrose (Canvas), real-time PID tuning, and BNO persistence hex-view.
 */
const char* compass_html = R"rawliteral(
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
<div class="info"><span>Heading: <span id="icmVal" class="icm">0.0</span>&deg;</span></div>
<div id="calMsg" class="cal-msg">Initializing...</div>
<div class="main-row">
<div class="side-panel"><div>BB</div><div class="side-bar"><div class="zero-line"></div><div id="bb_bar" class="thruster-bar"></div></div><div><span id="bb_val">0</span>%</div></div>
<canvas id="compassCanvas" width="400" height="400"></canvas>
<div class="side-panel"><div>SB</div><div class="side-bar"><div class="zero-line"></div><div id="sb_bar" class="thruster-bar"></div></div><div><span id="sb_val">0</span>%</div></div>
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
function updateThruster(id,v){const b=document.getElementById(id+'_bar'),l=document.getElementById(id+'_val');l.innerText=v;let h=Math.min(Math.abs(v),100)/2;b.style.height=h+'%';if(v<0){b.style.top='50%';b.style.bottom='auto';b.style.backgroundColor='red'}else{b.style.top='auto';b.style.bottom='50%';b.style.backgroundColor='green'}}
function drawRose(h){ctx.clearRect(0,0,400,400);ctx.beginPath();ctx.arc(cx,cy,r,0,2*Math.PI);ctx.strokeStyle='#555';ctx.lineWidth=2;ctx.stroke();ctx.fillStyle='#888';ctx.font='bold 20px Arial';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText('N',cx,cy-r+20);ctx.fillText('S',cx,cy+r-20);ctx.fillText('E',cx+r-20,cy);ctx.fillText('W',cx-r+20,cy);const a=(h-90)*Math.PI/180;ctx.beginPath();ctx.moveTo(cx,cy);ctx.lineTo(cx+(r-40)*Math.cos(a),cy+(r-40)*Math.sin(a));ctx.strokeStyle='#00d1ff';ctx.lineWidth=4;ctx.stroke()}
function fetchData(){fetch('/data?t='+Date.now()).then(r=>r.json()).then(d=>{document.getElementById('icmVal').innerText=d.icm.toFixed(1);document.getElementById('calMsg').innerText=d.cal_msg;document.getElementById('cal_load').innerText=d.cal_load;document.getElementById('cal_ver').innerText=d.cal_ver;if(d.mac)document.getElementById('mainTitle').innerText='NicE-Buoy Sub '+d.mac;updateThruster('bb',d.speed_bb);updateThruster('sb',d.speed_sb);drawRose(d.icm);const c=d.cal_levels,b=document.getElementById('calButton');if(c&&c[3]===3){b.style.background='#2a6a2a';b.innerText='BNO Calibrated (3)'}else{b.style.background='#5a32a8';b.innerText='BNO Status'}}).catch(e=>{})}
function startCalib(){alert('BNO055 calibrates automatically while moving. Rotate the buoy until Mag (M) shows 3.')}
function saveCalib(){if(confirm('Save current BNO calibration?')){fetch('/savecal').then(r=>alert('Save Sent!'))}}
function setParam(p){const e=document.getElementById(p+'_in');if(!e)return;fetch('/setparam?p='+p+'&v='+e.value)}
fetch('/params').then(r=>r.json()).then(d=>{['kpr','kir','kdr','kps','kis','kds','coff','pvspd','revbb','revsb','tswap','minspd','maxspd','holdrad'].forEach(p=>{const e=document.getElementById(p+'_in');if(e)e.value=d[p]})});
setInterval(fetchData,250);
</script></body></html>
)rawliteral";

/**
 * @brief Retrieves local MAC address segment for unique device identification.
 */
unsigned long espMac(void) {
    byte m[6]; WiFi.macAddress(m);
    unsigned long r=0; for(int i=2;i<6;i++) r=(r<<8)|m[i];
    return r;
}

/**
 * @brief Initializes WiFi telemetry structures and Queues.
 */
void initwifi(void) {
    byte m[6]; WiFi.macAddress(m);
    char ms[25]; sprintf(ms, "%02X%02X%02X%02X%02X%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
    global_mac_str = String(ms);
    udpOut = xQueueCreate(10, sizeof(RoboStruct));
    udpIn = xQueueCreate(10, sizeof(RoboStruct));
}

/**
 * @brief Core 0 Task: Manages WiFi, WebServer, and UDP communications.
 */
void WiFiTask(void *arg) {
    WiFiManager wm; 
    // wm.resetSettings(); // Debug: Uncomment to clear credentials
    wm.autoConnect("NicE-Buoy-Sub");
    
    // Broadcast/Listen for other buoys on port 1001
    if (udp.listen(1001)) {
        udp.onPacket([](AsyncUDPPacket p) {
            RoboStruct d; 
            String s=String((const char*)p.data(),p.length()); 
            rfDeCode(s,&d);
            if(d.IDs!=-1 && d.IDs!=espMac()){
                //xQueueSend(udpIn,(void*)&d,10);
            }
        });
    }

    // Dashboard Endpoints
    subServer.on("/", [](){ subServer.send(200,"text/html",compass_html); });
    subServer.on("/savecal", [](){ int c=34; xQueueSend(compassIn,(void*)&c,10); subServer.send(200,"text/plain","OK"); });
    
    // Parameter Update API
    subServer.on("/setparam", [](){
        if(!subServer.hasArg("p")||!subServer.hasArg("v")){subServer.send(400,"text/plain","Err");return;}
        String p=subServer.arg("p"); float v=subServer.arg("v").toFloat();
        if(mainDataMutex && xSemaphoreTake(mainDataMutex, pdMS_TO_TICKS(500))){
            if(p=="kpr"){mainData.Kpr=v;pidRudderParameters(&mainData,SET);initRudPid(&mainData);}
            else if(p=="kir"){mainData.Kir=v;pidRudderParameters(&mainData,SET);initRudPid(&mainData);}
            else if(p=="kdr"){mainData.Kdr=v;pidRudderParameters(&mainData,SET);initRudPid(&mainData);}
            else if(p=="kps"){mainData.Kps=v;pidSpeedParameters(&mainData,SET);initSpeedPid(&mainData);}
            else if(p=="kis"){mainData.Kis=v;pidSpeedParameters(&mainData,SET);initSpeedPid(&mainData);}
            else if(p=="kds"){mainData.Kds=v;pidSpeedParameters(&mainData,SET);initSpeedPid(&mainData);}
            else if(p=="coff"){mainData.compassOffset=v;CompasOffset(&mainData,SET);}
            else if(p=="pvspd"){mainData.pivotSpeed=v;speedMaxMin(&mainData,SET);initSpeedPid(&mainData);initRudPid(&mainData);}
            else if(p=="holdrad"){mainData.minOfsetDist=v;computeParameters(&mainData,SET);initSpeedPid(&mainData);initRudPid(&mainData);}
            else if(p=="minspd"){mainData.minSpeed=(int)v;speedMaxMin(&mainData,SET);initSpeedPid(&mainData);initRudPid(&mainData);}
            else if(p=="maxspd"){mainData.maxSpeed=(int)v;speedMaxMin(&mainData,SET);initSpeedPid(&mainData);initRudPid(&mainData);}
            else if(p=="revbb"){mainData.revBB=(v>0.5);thrusterInversion(&mainData,SET);}
            else if(p=="revsb"){mainData.revSB=(v>0.5);thrusterInversion(&mainData,SET);}
            else if(p=="tswap"){mainData.swap_BB_SB=(v>0.5);thrusterSwap(&mainData,SET);}
            xSemaphoreGive(mainDataMutex);
        }
        subServer.send(200,"text/plain","OK");
    });

    // Telemetry and Parameter Read API
    subServer.on("/params", [](){
        static String last_params = "{\"kpr\":1.0,\"kir\":0.0,\"kdr\":0.0,\"kps\":1.0,\"kis\":0.0,\"kds\":0.0,\"coff\":0.0,\"pvspd\":0.5,\"minspd\":0,\"maxspd\":100,\"holdrad\":2.0,\"revbb\":0,\"revsb\":0,\"tswap\":0}";
        if(mainDataMutex && xSemaphoreTake(mainDataMutex, pdMS_TO_TICKS(100))){
            char buf[500];
            snprintf(buf, sizeof(buf), 
                "{\"kpr\":%.3f,\"kir\":%.3f,\"kdr\":%.3f,\"kps\":%.3f,\"kis\":%.3f,\"kds\":%.3f,\"coff\":%.1f,\"revbb\":%d,\"revsb\":%d,\"tswap\":%d,\"pvspd\":%.2f,\"minspd\":%d,\"maxspd\":%d,\"holdrad\":%.1f}",
                mainData.Kpr, mainData.Kir, mainData.Kdr, mainData.Kps, mainData.Kis, mainData.Kds, 
                (float)mainData.compassOffset, mainData.revBB?1:0, mainData.revSB?1:0, mainData.swap_BB_SB?1:0,
                (float)mainData.pivotSpeed, mainData.minSpeed, mainData.maxSpeed, (float)mainData.minOfsetDist
            );
            last_params = String(buf);
            xSemaphoreGive(mainDataMutex);
        }
        subServer.send(200,"application/json", last_params);
    });

    subServer.on("/data", [](){
        static int last_sbb = 0, last_ssb = 0;
        float icm = global_icmHdg;
        int sbb = last_sbb, ssb = last_ssb;
        
        if (mainDataMutex && xSemaphoreTake(mainDataMutex, pdMS_TO_TICKS(50))) {
            sbb = (int)mainData.speedBb; ssb = (int)mainData.speedSb;
            last_sbb = sbb; last_ssb = ssb;
            xSemaphoreGive(mainDataMutex);
        }

        String j="{"; 
        j+="\"icm\":"+String(icm,2)+",";
        j+="\"speed_bb\":"+String(sbb)+",";
        j+="\"speed_sb\":"+String(ssb)+",";
        j+="\"cal_load\":\""+global_cal_load+"\",";
        j+="\"cal_ver\":\""+global_cal_ver+"\",";
        j+="\"mac\":\""+global_mac_str+"\",";
        j+="\"cal_levels\":["+String(bno_cal_sys)+","+String(bno_cal_gyro)+","+String(bno_cal_accel)+","+String(bno_cal_mag)+"],";
        j+="\"cal_msg\":\""+global_cal_msg+"\"";
        j+="}";
        subServer.send(200,"application/json",j);
    });

    subServer.begin(); 
    
    // OTA Listener Setup
    ArduinoOTA.setHostname("RoboBuoySub");
    ArduinoOTA.onStart([]() { Serial.println("\nOTA Start"); });
    ArduinoOTA.onEnd([]() { Serial.println("\nOTA End"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();

    // Main Server Loop
    while(1){ 
        subServer.handleClient(); 
        ArduinoOTA.handle();
        if (udpOut && xQueueReceive(udpOut, (void *)&subWifiOut, 0) == pdTRUE) { 
            subWifiOut.IDs = espMac(); 
            String out = rfCode(&subWifiOut); udp.broadcast(out.c_str()); 
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}
