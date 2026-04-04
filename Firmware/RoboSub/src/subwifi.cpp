#include <WiFi.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <AsyncUDP.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include <RoboCompute.h>
#include "main.h"
#include "io_sub.h"
#include "compass.h"

// Define the global web server on port 80
WebServer subServer(80);

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
</style>
</head>
<body>
<h2>Dual Compass Debug</h2>
<div class="info">
    <span class="lsm">LSM303: <span id="lsmVal">0.0</span>&deg;</span>
    <span class="icm">ICM20948: <span id="icmVal">0.0</span>&deg;</span>
</div>
<canvas id="compassCanvas" width="400" height="400"></canvas>
<script>
const ctx = document.getElementById('compassCanvas').getContext('2d');
const cx = 200, cy = 200, r = 180;

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
        drawRose(data.lsm, data.icm);
    })
    .catch(e => console.error(e));
}
setInterval(fetchData, 200);
drawRose(0, 0);
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
        json += "\"lsmHardX\":" + String(compassCalc.magHard[0], 2) + ", ";
        json += "\"lsmHardY\":" + String(compassCalc.magHard[1], 2) + ", ";
        json += "\"lsmHardZ\":" + String(compassCalc.magHard[2], 2) + ", ";
        json += "\"icmHardX\":" + String(compassCalc.icmMagHard[0], 2) + ", ";
        json += "\"icmHardY\":" + String(compassCalc.icmMagHard[1], 2) + ", ";
        json += "\"icmHardZ\":" + String(compassCalc.icmMagHard[2], 2);
        json += "}";
        subServer.send(200, "application/json", json);
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