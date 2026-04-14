#include <WiFi.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <AsyncUDP.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <Adafruit_Sensor.h>
#include <RoboCompute.h>
#include "main.h"
#include "io_sub.h"
#include "compass.h"

// Define the global web server on port 80
WebServer subServer(80);

extern float global_speed_bb;
extern float global_speed_sb;
extern bool icm_ready;
extern float global_icmHdg;
extern float global_lsmHdg;

// Raw axis data for dashboard
extern sensors_event_t m_lsm_last, m_icm_last;

const char* compass_html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>Compass Debug</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { font-family: Arial; text-align: center; background: #222; color: #fff; margin: 0; padding: 20px; }
canvas { background: #333; border-radius: 50%; margin-top: 20px; border: 2px solid #555; max-width: 100%; height: auto; }      
h2 { margin-bottom: 5px; }
.info { font-size: 1.2em; margin-top: 10px; display: flex; justify-content: center; gap: 20px; }
.lsm { color: #f0ad4e; font-weight: bold; }
.icm { color: #5bc0de; font-weight: bold; }
.raw { font-size: 0.8em; color: #aaa; }
.raw-container { display: flex; justify-content: center; gap: 20px; margin-top: 15px; flex-wrap: wrap; }
.raw-box { text-align: left; border: 1px solid #444; padding: 10px; border-radius: 5px; background: #2a2a2a; min-width: 300px; }
.axis-row { display: flex; align-items: center; margin: 15px 0; font-size: 0.9em; }
.axis-label { width: 15px; font-weight: bold; }
.axis-bar-container { flex-grow: 1; background: #444; height: 16px; border-radius: 8px; position: relative; margin: 0 10px; border: 1px solid #666; }
.axis-dot { position: absolute; top: -4px; width: 10px; height: 24px; background: white; border-radius: 3px; transform: translateX(-5px); transition: left 0.1s; }
.axis-vals { width: 50px; text-align: right; font-family: monospace; }
.axis-minmax { font-size: 0.7em; color: #888; display: flex; justify-content: space-between; margin: -10px 60px 10px 25px; }
.min-col { color: #ff8888; }
.max-col { color: #88ff88; }
button { margin-top: 15px; padding: 8px 16px; background: #444; color: white; border: 1px solid #666; cursor: pointer; border-radius: 4px; }
button:hover { background: #555; }
.thrusters { margin-top: 20px; display: flex; flex-direction: column; align-items: center; gap: 10px; width: 100%; max-width: 400px; margin-left: auto; margin-right: auto; }
.thruster-bar-container { width: 100%; background: #444; border-radius: 5px; height: 25px; position: relative; display: flex; align-items: center; border: 1px solid #666; overflow: hidden; }
.thruster-bar { height: 100%; width: 0%; background: transparent; position: absolute; left: 50%; transition: width 0.1s, left 0.1s; }
.thruster-label { position: absolute; width: 100%; text-align: center; font-size: 0.9em; font-weight: bold; z-index: 1; pointer-events: none; text-shadow: 1px 1px 2px black; }
.center-line { position: absolute; left: 50%; width: 2px; height: 100%; background: #888; z-index: 0; }
</style>
</head>
<body>
<h2>Dual Compass Debug</h2>
<div class="info">
    <span class="lsm">LSM303: <span id="lsmVal">0.0</span>&deg;</span>
    <span class="icm">ICM20948: <span id="icmVal">0.0</span>&deg;</span>
</div>

<div class="thrusters">
    <div class="thruster-bar-container">
        <div class="center-line"></div>
        <div id="bb_bar" class="thruster-bar"></div>
        <div class="thruster-label">Port / BB (<span id="bb_val">0</span>%)</div>
    </div>
    <div class="thruster-bar-container">
        <div class="center-line"></div>
        <div id="sb_bar" class="thruster-bar"></div>
        <div class="thruster-label">Starboard / SB (<span id="sb_val">0</span>%)</div>
    </div>
</div>

<button onclick="resetMinMax()">Reset Min/Max</button>
<button onclick="startCalib()" style="background: #5a32a8;">Start Desk Calibration</button>

<div class="raw-container">
    <div class="raw-box lsm">
        <div style="font-weight:bold; margin-bottom:5px;">LSM Raw (Mag)</div>
        <div class="axis-row"><div class="axis-label">X</div><div class="axis-bar-container"><div id="lsm_x_dot" class="axis-dot" style="left:50%; background:#f0ad4e;"></div></div><div id="lsm_x_val" class="axis-vals">0.0</div></div>
        <div class="axis-minmax"><span id="lsm_x_min" class="min-col">0.0</span><span id="lsm_x_max" class="max-col">0.0</span></div>
        <div class="axis-row"><div class="axis-label">Y</div><div class="axis-bar-container"><div id="lsm_y_dot" class="axis-dot" style="left:50%; background:#f0ad4e;"></div></div><div id="lsm_y_val" class="axis-vals">0.0</div></div>
        <div class="axis-minmax"><span id="lsm_y_min" class="min-col">0.0</span><span id="lsm_y_max" class="max-col">0.0</span></div>
        <div class="axis-row"><div class="axis-label">Z</div><div class="axis-bar-container"><div id="lsm_z_dot" class="axis-dot" style="left:50%; background:#f0ad4e;"></div></div><div id="lsm_z_val" class="axis-vals">0.0</div></div>
        <div class="axis-minmax"><span id="lsm_z_min" class="min-col">0.0</span><span id="lsm_z_max" class="max-col">0.0</span></div>
    </div>
    <div class="raw-box icm">
        <div style="font-weight:bold; margin-bottom:5px;">ICM Raw (Mag)</div>
        <div class="axis-row"><div class="axis-label">X</div><div class="axis-bar-container"><div id="icm_x_dot" class="axis-dot" style="left:50%; background:#5bc0de;"></div></div><div id="icm_x_val" class="axis-vals">0.0</div></div>
        <div class="axis-minmax"><span id="icm_x_min" class="min-col">0.0</span><span id="icm_x_max" class="max-col">0.0</span></div>
        <div class="axis-row"><div class="axis-label">Y</div><div class="axis-bar-container"><div id="icm_y_dot" class="axis-dot" style="left:50%; background:#5bc0de;"></div></div><div id="icm_y_val" class="axis-vals">0.0</div></div>
        <div class="axis-minmax"><span id="icm_y_min" class="min-col">0.0</span><span id="icm_y_max" class="max-col">0.0</span></div>
        <div class="axis-row"><div class="axis-label">Z</div><div class="axis-bar-container"><div id="icm_z_dot" class="axis-dot" style="left:50%; background:#5bc0de;"></div></div><div id="icm_z_val" class="axis-vals">0.0</div></div>
        <div class="axis-minmax"><span id="icm_z_min" class="min-col">0.0</span><span id="icm_z_max" class="max-col">0.0</span></div>
    </div>
</div>
<canvas id="compassCanvas" width="400" height="400"></canvas>
<script>
const ctx = document.getElementById('compassCanvas').getContext('2d');
const cx = 200, cy = 200, r = 180;

let session = {
    lsm: { x: {min: null, max: null}, y: {min: null, max: null}, z: {min: null, max: null} },
    icm: { x: {min: null, max: null}, y: {min: null, max: null}, z: {min: null, max: null} }
};

function updateBar(sensor, axis, val) {
    if (session[sensor][axis].min === null || val < session[sensor][axis].min) session[sensor][axis].min = val;
    if (session[sensor][axis].max === null || val > session[sensor][axis].max) session[sensor][axis].max = val;
    
    let min = session[sensor][axis].min;
    let max = session[sensor][axis].max;
    
    document.getElementById(sensor + '_' + axis + '_val').innerText = val.toFixed(1);
    document.getElementById(sensor + '_' + axis + '_min').innerText = min.toFixed(1);
    document.getElementById(sensor + '_' + axis + '_max').innerText = max.toFixed(1);
    
    let range = max - min;
    let percent = 50;
    if (range > 0) {
        percent = ((val - min) / range) * 100;
    }
    document.getElementById(sensor + '_' + axis + '_dot').style.left = percent + '%';
}

function updateThruster(id, val) {
    const bar = document.getElementById(id + '_bar');
    const label = document.getElementById(id + '_val');
    label.innerText = val;

    // Width is percentage of half the bar
    let width = Math.abs(val) / 2; 
    bar.style.width = width + '%';

    if (val < 0) {
        bar.style.left = (50 - width) + '%';
        bar.style.backgroundColor = '#5bc0de'; // Blue for forward/reverse
    } else {
        bar.style.left = '50%';
        bar.style.backgroundColor = '#f0ad4e'; // Orange for opposite
    }
}

function drawRose(lsm, icm) {
    ctx.clearRect(0, 0, 400, 400);

    // Draw background
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, 2*Math.PI); ctx.strokeStyle = '#555'; ctx.lineWidth = 2; ctx.stroke();

    // Draw N, E, S, W
    ctx.fillStyle = '#aaa'; ctx.font = 'bold 20px Arial'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillText('N', cx, cy - r + 20); ctx.fillText('S', cx, cy + r - 20);
    ctx.fillText('E', cx + r - 20, cy); ctx.fillText('W', cx - r + 20, cy);

    // Draw LSM Vector (Orange)
    drawVector(lsm, '#f0ad4e', r - 30, 4);

    // Draw ICM Vector (Blue)
    drawVector(icm, '#5bc0de', r - 40, 4);
}

function drawVector(angle, color, length, width) {
    const rad = (angle - 90) * Math.PI / 180;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + length * Math.cos(rad), cy + length * Math.sin(rad));
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.stroke();
}

function fetchData() {
    fetch('/data')
    .then(r => r.json())
    .then(data => {
        document.getElementById('lsmVal').innerText = data.lsm.toFixed(1);
        document.getElementById('icmVal').innerText = data.icm.toFixed(1);

        updateBar('lsm', 'x', data.lsm_x);
        updateBar('lsm', 'y', data.lsm_y);
        updateBar('lsm', 'z', data.lsm_z);

        updateBar('icm', 'x', data.icm_x);
        updateBar('icm', 'y', data.icm_y);
        updateBar('icm', 'z', data.icm_z);

        updateThruster('bb', data.speed_bb);
        updateThruster('sb', data.speed_sb);

        drawRose(data.lsm, data.icm);
    })
    .catch(e => console.error(e));
}

function resetMinMax() {
    session = {
        lsm: { x: {min: null, max: null}, y: {min: null, max: null}, z: {min: null, max: null} },
        icm: { x: {min: null, max: null}, y: {min: null, max: null}, z: {min: null, max: null} }
    };
    ['lsm', 'icm'].forEach(sensor => {
        ['x', 'y', 'z'].forEach(axis => {
            document.getElementById(sensor + '_' + axis + '_min').innerText = '0.00';
            document.getElementById(sensor + '_' + axis + '_max').innerText = '0.00';
        });
    });
}

function startCalib() {
    if(confirm('Start Desk Compass Calibration? The system will beep and middle LED will blink purple for 60 seconds.')) {
        fetch('/calibrate').then(r => alert('Calibration Started! Observe the Sub LEDs.'));
    }
}

setInterval(fetchData, 200);
</script>
</body>
</html>
)rawliteral";

#include "datastorage.h"
#include "subwifi.h"

RoboStruct subwifiData;
// UdpMsg udpData;
static RoboStruct udpData;
static RoboStruct subWifiIn;
static RoboStruct subWifiOut;
AsyncUDP udp;
// static UdpData udpBuffer;
QueueHandle_t udpIn;
QueueHandle_t udpOut;

/**
 * @brief Sets up Over-The-Air (OTA) firmware update capability.
 * 
 * Configures the OTA hostname based on the device's MAC address and sets up 
 * several callback functions for different stages of the OTA process:
 * - onStart: Triggered when the update process begins.
 * - onEnd: Triggered when the update process finishes successfully.
 * - onProgress: Periodically triggered during the update to report progress (0-100%).
 * - onError: Triggered if an error occurs during the update process.
 * 
 * The OTA ID is printed to the Serial monitor upon successful initialization.
 * 
 * @return bool Returns true if the OTA setup and initialization were successful.
 */
bool setup_OTA()
{
    char ssidl[20];
    char buf[30];
    byte mac[6];
    WiFi.macAddress(mac);
    Serial.print("SETUP OTA...");
    sprintf(buf, "Sub_%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ArduinoOTA.setHostname(buf);
    ArduinoOTA.onStart([]()
                       {
    /* switch off all processes here!!!!! */
    Serial.println();
    Serial.println("Recieving new firmware now!"); });
    ArduinoOTA.onEnd([]()
                     {
    /* do stuff after update here!! */
    Serial.println("\nRecieving done!");
    Serial.println("Storing in memory and reboot!");
    Serial.println(); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          {
                              Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
                              digitalWrite(PWRENABLE, 1); // enable powersupply
                          });
    ArduinoOTA.onError([](ota_error_t error)
                       { ESP.restart(); });
    /* setup the OTA server */
    ArduinoOTA.begin();
    Serial.println("...done!");
    Serial.print("OTA ID: ");
    Serial.println(buf);
    return true;
}

/**
 * @brief Scans for a specific WiFi Access Point and attempts to establish a connection.
 * 
 * This function performs a network scan to locate an access point with an SSID matching 
 * the provided `ssidap`. If found, it attempts to connect using the provided password `ww`.
 * 
 * Special Handling:
 * If the connected SSID contains the prefix "PAIR_ME_", the function will replace this 
 * prefix with "BUOY_", save the updated credentials to persistent storage, and restart 
 * the device. This mechanism is primarily used for the initial pairing process.
 * 
 * @param ssidap Pointer to a String containing the target SSID or a substring to search for.
 * @param ww The WiFi password as a String.
 * @return bool Returns true if a connection is successfully established; otherwise, returns false.
 */
bool scan_for_wifi_ap(String *ssidap, String ww)
{
    unsigned long timeout = millis();
    Serial.print("scan for for ap:");
    Serial.println(*ssidap);
    int n = WiFi.scanNetworks();
    Serial.println("scan done");
    Serial.print(n);
    Serial.println(" networks found");
    if (n == 0)
    {
        return false;
    }
    else
    {
        for (int i = 0; i < n; ++i)
        {
            String ssid = WiFi.SSID(i);
            if (ssid.indexOf(*ssidap) != -1)
            {
                Serial.println("Access point found, attempting to connect...");
                WiFi.begin(ssid.c_str(), ww.c_str());
                while (WiFi.status() != WL_CONNECTED)
                {
                    delay(50);
                    Serial.print(".");
                    if (millis() - timeout > 120000) // 2 minute timeout
                    {
                        Serial.println("\nConnection timed out. Restarting...");
                        delay(100);
                        esp_restart();
                    }
                }
                Serial.println("\nConnected.");
                Serial.print("Logged in to SSID: ");
                Serial.println(ssid);
                Serial.print("IP address: ");
                Serial.println(WiFi.localIP());
                if (ssid.indexOf("PAIR_ME_") != -1)
                {
                    ssid.replace("PAIR_ME_", "BUOY_");
                    apParameters(&ssid, &ww, false); // store parameters
                    Serial.println("Network parameters stored:" + ssid);
                    Serial.println("Rebooting in 5 seconds...");
                    delay(5000);
                    esp_restart();
                }
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief Initializes the AsyncUDP listener for handling incoming network messages.
 * 
 * This function configures the device to listen for incoming UDP packets on port 1001.
 * It defines a callback mechanism that converts incoming packet data into a string, 
 * decodes it into a `RoboStruct` object using `rfDeCode`, and verifies its validity. 
 * If the decoded message is valid and does not originate from the device itself 
 * (to prevent self-looping), it pushes the data onto the `udpIn` FreeRTOS queue 
 * for the main task to process.
 * 
 * @return bool Returns true if the UDP listener is successfully started on port 1001, 
 *              false otherwise.
 */
bool setupudp(void)
{
    if (udp.listen(1001))
    {
        Serial.print("UDP server at IP: ");
        Serial.print(WiFi.localIP());
        Serial.println(" port: 1001");
        udp.onPacket([](AsyncUDPPacket packet)
                     {
                        RoboStruct udpDataIn;
                         String stringUdpIn = (const char *)packet.data();  // Convert incoming data to String
                         rfDeCode(stringUdpIn,&udpDataIn);     // Decode string to RoboStruct
                        if (udpDataIn.IDs != -1 && udpDataIn.IDs != subwifiData.mac) // ignore own messages
                        {
                            xQueueSend(udpIn, (void *)&udpDataIn, 10); // notify main there is new data
                        }
                        else
                        {
                            Serial.printf("CRC error from %s:%u >%s<\n",
                            packet.remoteIP().toString().c_str(),
                            packet.remotePort(),
                            stringUdpIn.c_str());
                        } });
        return true;
    }
    else
    {
        Serial.println("Failed to start UDP listener on port 1001");
    }
    return false;
}

/**
 * @brief Initializes core WiFi-related data structures and communication queues.
 * 
 * Sets the device's numeric MAC ID within the global `subwifiData` structure.
 * Additionally, it creates two FreeRTOS queues:
 * - `udpOut`: A queue for outgoing UDP messages to be broadcasted to the network.
 * - `udpIn`: A queue for incoming UDP messages received from the network to be processed.
 * 
 * This function should be called early in the initialization sequence before WiFi tasks begin.
 */
void initwifi(void)
{
    subwifiData.mac = espMac();
    udpOut = xQueueCreate(10, sizeof(RoboStruct));
    udpIn = xQueueCreate(10, sizeof(RoboStruct));
}

/**
 * @brief Performs a passive scan to check if a specific WiFi Access Point is visible.
 * 
 * This function executes a WiFi network scan to determine whether the target SSID 
 * is currently available in the surrounding environment. It does not attempt to connect.
 * 
 * @param ssipap The exact SSID to search for as a String.
 * @return bool Returns true if the specified SSID is found among the scanned networks; 
 *              otherwise, returns false.
 */
bool scan_for_wifi_ap(String ssipap)
{
    unsigned long timeout = millis();
    Serial.print("scan for for ap \"");
    Serial.print(ssipap);
    Serial.println("\"");
    
    // Try to find the AP for up to 120 seconds before giving up and setting up local AP
    while (millis() - timeout < 120000) 
    {
        int n = WiFi.scanNetworks();
        for (int i = 0; i < n; ++i)
        {
            if (WiFi.SSID(i) == ssipap.c_str())
            {
                Serial.println("Access point found!");
                return true;
            }
        }
        Serial.println("Access point not found, retrying...");
        vTaskDelay(pdMS_TO_TICKS(1000)); // wait 1s between scans
    }
    
    Serial.println("Access point not found after 2 minutes.");
    return false;
}

/**
 * @brief Retrieves the raw device MAC address.
 * 
 * Uses the Arduino WiFi API to fetch the 6-byte hardware MAC address of the ESP32
 * and stores it into the provided byte array.
 * 
 * @param mac Pointer to a pre-allocated 6-byte array where the MAC address will be stored.
 * @return uint8_t* Pointer to the same byte array that was passed as an argument.
 */
uint8_t *getMacId(uint8_t *mac)
{
    WiFi.macAddress(mac);
    return mac;
}

/**
 * @brief Computes a numeric ID from the device's MAC address.
 * Uses the last 4 bytes of the MAC address to create a unique unsigned long ID.
 * @return The numeric ID.
 */
unsigned long espMac(void)
{
    byte macarr[6];
    WiFi.macAddress(macarr);
    unsigned long mac = 0;
    for (int i = 2; i < 6; i++)
    {
        mac = (mac << 8) | macarr[i];
    }
    return mac;
}

/**
 * @brief Main WiFi management task.
 * Handles WiFi connection using WiFiManager, sets up OTA and UDP, 
 * and enters a loop to handle OTA updates and broadcast outgoing UDP packets.
 * @param arg Task arguments (not used).
 */
void WiFiTask(void *arg)
{
    int wifiConfig = *((int *)arg);
    WiFiManager wm;
    String ssid = "";
    String ww = "";
    WiFi.mode(WIFI_STA);
    byte mac[6];
    char macStr[25];
    WiFi.macAddress(mac);
    sprintf(macStr, "RoboSub_%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    // Force Setup AP mode if button was pressed during startup
    if (wifiConfig == 1) {
        Serial.println("Force starting AP setup portal due to button press");
        wm.resetSettings(); // clear any saved network so it is forced to AP mode
        wm.setConnectTimeout(120);
        wm.startConfigPortal(macStr); // block and start portal
        ssid = wm.getWiFiSSID();
        ww = wm.getWiFiPass();
        Serial.println("New credentials saved!");
        Serial.println("SSID:" + ssid + " WW:" + ww);
        apParameters(&ssid, &ww, SET);
        delay(500);
        esp_restart(); // reboot with new settings
    }
    
    apParameters(&ssid, &ww, GET);
    bool ap = false; // indicator accespoint found
    if (ssid != "")
    {
        ap = scan_for_wifi_ap(ssid);
    }
    if (ap == false)
    {
        wm.setConnectTimeout(120);
        wm.autoConnect(macStr);
        ssid = wm.getWiFiSSID();
        ww = wm.getWiFiPass();
        Serial.println("SSID:" + ssid + " WW:" + ww);
        apParameters(&ssid, &ww, SET);
        delay(100);
    }
    else
    {
        Serial.print("Connecting directly to ");
        Serial.println(ssid);
        WiFi.begin(ssid.c_str(), ww.c_str());
        unsigned long startAttemptTime = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 120000) {
            delay(500);
            Serial.print(".");
        }
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("\nFailed to connect. Restarting...");
            esp_restart();
        }
        Serial.println("\nConnected!");
    }
    Serial.print("Logged in to AP \"");
    Serial.print(ssid);
    Serial.println("\"");
    setup_OTA();
    setupudp();
    
    // Set up Web Server
    subServer.on("/", []() {
        subServer.send(200, "text/html", compass_html);
    });
    subServer.on("/data", []() {
        String json = "{";
        json += "\"lsm\":" + String(global_lsmHdg, 2) + ", ";
        json += "\"icm\":" + String(global_icmHdg, 2) + ", ";
        json += "\"lsm_x\":" + String(m_lsm_last.magnetic.x, 2) + ", ";
        json += "\"lsm_y\":" + String(m_lsm_last.magnetic.y, 2) + ", ";
        json += "\"lsm_z\":" + String(m_lsm_last.magnetic.z, 2) + ", ";
        json += "\"icm_x\":" + String(m_icm_last.magnetic.x, 2) + ", ";
        json += "\"icm_y\":" + String(m_icm_last.magnetic.y, 2) + ", ";
        json += "\"icm_z\":" + String(m_icm_last.magnetic.z, 2) + ", ";
        json += "\"speed_bb\":" + String(global_speed_bb, 0) + ", ";
        json += "\"speed_sb\":" + String(global_speed_sb, 0);
        json += "}";
        subServer.send(200, "application/json", json);
    });
    subServer.on("/calibrate", []() {
        int cmd = CALIBRATE_MAGNETIC_COMPASS;
        xQueueSend(compassIn, (void *)&cmd, 10);
        subServer.send(200, "text/plain", "OK");
    });
    subServer.begin();
    
    Serial.print("WiFI task running!\r\n");
    //***************************************************************************************************
    //  WiFi main loop
    //***************************************************************************************************
    for (;;)
    {
        subServer.handleClient();
        ArduinoOTA.handle();
        if (xQueueReceive(udpOut, (void *)&subWifiOut, 5) == pdTRUE)
        {
            subWifiOut.IDr = espMac();
            subWifiOut.IDs = espMac();
            String out = rfCode(&subWifiOut);
            udp.broadcast(out.c_str());
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 10 ms delay
    }
}
