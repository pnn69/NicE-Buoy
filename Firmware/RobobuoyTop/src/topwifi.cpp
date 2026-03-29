#include <WiFi.h>

#include <WebServer.h>
#include <ArduinoOTA.h>
#include <AsyncUDP.h>
#include "main.h"
#include "topwifi.h"
#include "leds.h"
#include "datastorage.h"
#include "buzzer.h"

static int statik = IDLE;
static RoboStruct msgIdOut;
static RoboStruct topWifiIn;
static RoboStruct udpBuffer;
static RoboStruct udpBufferRecieved;
static LedData wifiCollorUtil;
static bool ota = false;
static int8_t id = 0;
static char udpDataIn[MAXSTRINGLENG];
static IPAddress ipTop;
static unsigned long mac = 0;
AsyncUDP udp;
QueueHandle_t udpOut;
QueueHandle_t udpIn;
// static unsigned long tstart, tstop;
static unsigned long lastUpdMsg = 0;

WebServer server(80);

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>Robobuoy Dashboard</title>
<style>
body {font-family: Arial, sans-serif; background-color: #f4f4f4;}
.tab {overflow: hidden; border: 1px solid #ccc; background-color: #e0e0e0;}
.tab button {background-color: inherit; float: left; border: none; outline: none; cursor: pointer; padding: 14px 16px; transition: 0.3s; font-size: 17px;}
.tab button:hover {background-color: #ccc;}
.tab button.active {background-color: #fff;}
.tabcontent {display: none; padding: 20px; border: 1px solid #ccc; border-top: none; background-color: #fff;}
.card {padding: 15px; margin: 10px 0; background: white; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1);}
</style>
<script>
function openTab(evt, tabName) {
  var i, tabcontent, tablinks;
  tabcontent = document.getElementsByClassName("tabcontent");
  for (i = 0; i < tabcontent.length; i++) tabcontent[i].style.display = "none";
  tablinks = document.getElementsByClassName("tablinks");
  for (i = 0; i < tablinks.length; i++) tablinks[i].className = tablinks[i].className.replace(" active", "");
  document.getElementById(tabName).style.display = "block";
  if(evt) evt.currentTarget.className += " active";
}
function fetchUpdate() {
  fetch('/data').then(response => response.json()).then(data => {
    for (let i = 0; i < 3; i++) {
        let buoy = data.buoys[i];
        let bid = i + 1;
        if(buoy.ID && buoy.ID !== "0") {
            document.getElementById('b'+bid+'_id').innerText = buoy.ID;
            document.getElementById('b'+bid+'_status').innerText = buoy.Status;
            document.getElementById('b'+bid+'_speed').innerText = buoy.Speed;
            document.getElementById('b'+bid+'_bb').innerText = buoy.BB;
            document.getElementById('b'+bid+'_sb').innerText = buoy.SB;
            document.getElementById('b'+bid+'_magdir').innerText = buoy.MagDir;
            document.getElementById('b'+bid+'_tgdir').innerText = buoy.TgDir;
            document.getElementById('b'+bid+'_tgdist').innerText = buoy.TgDist;
            document.getElementById('b'+bid+'_gpsdir').innerText = buoy.GpsDir;
            document.getElementById('b'+bid+'_wdir').innerText = buoy.WDir;
            document.getElementById('b'+bid+'_wstd').innerText = buoy.WStd;
        }
    }
  });
}
setInterval(fetchUpdate, 2000);
window.onload = () => { fetchUpdate(); };
</script>
</head>
<body>
<h2>Robobuoy Dashboard</h2>
<div class="tab">
  <button class="tablinks active" onclick="openTab(event, 'Buoy1')">Top Buoy</button>
  <button class="tablinks" onclick="openTab(event, 'Buoy2')">Sub Buoy 1</button>
  <button class="tablinks" onclick="openTab(event, 'Buoy3')">Sub Buoy 2</button>
</div>

<div id="Buoy1" class="tabcontent" style="display:block;">
  <div class="card">
    <h3>Buoy ID: <span id="b1_id">Waiting...</span></h3>
    <p>Status: <span id="b1_status"></span></p>
    <p>Speed Set: <span id="b1_speed"></span></p>
    <p>BB Thruster: <span id="b1_bb"></span>%</p>
    <p>SB Thruster: <span id="b1_sb"></span>%</p>
    <p>Magnetic Dir: <span id="b1_magdir"></span>&deg;</p>
    <p>Target Dir: <span id="b1_tgdir"></span>&deg;</p>
    <p>Target Dist: <span id="b1_tgdist"></span>m</p>
    <p>GPS Dir: <span id="b1_gpsdir"></span>&deg;</p>
    <p>Wind Dir: <span id="b1_wdir"></span>&deg;</p>
    <p>Wind StdDev: <span id="b1_wstd"></span></p>
  </div>
</div>
<div id="Buoy2" class="tabcontent">
  <div class="card">
    <h3>Buoy ID: <span id="b2_id">Waiting...</span></h3>
    <p>Status: <span id="b2_status"></span></p>
    <p>Speed Set: <span id="b2_speed"></span></p>
    <p>BB Thruster: <span id="b2_bb"></span>%</p>
    <p>SB Thruster: <span id="b2_sb"></span>%</p>
    <p>Magnetic Dir: <span id="b2_magdir"></span>&deg;</p>
    <p>Target Dir: <span id="b2_tgdir"></span>&deg;</p>
    <p>Target Dist: <span id="b2_tgdist"></span>m</p>
    <p>GPS Dir: <span id="b2_gpsdir"></span>&deg;</p>
    <p>Wind Dir: <span id="b2_wdir"></span>&deg;</p>
    <p>Wind StdDev: <span id="b2_wstd"></span></p>
  </div>
</div>
<div id="Buoy3" class="tabcontent">
  <div class="card">
    <h3>Buoy ID: <span id="b3_id">Waiting...</span></h3>
    <p>Status: <span id="b3_status"></span></p>
    <p>Speed Set: <span id="b3_speed"></span></p>
    <p>BB Thruster: <span id="b3_bb"></span>%</p>
    <p>SB Thruster: <span id="b3_sb"></span>%</p>
    <p>Magnetic Dir: <span id="b3_magdir"></span>&deg;</p>
    <p>Target Dir: <span id="b3_tgdir"></span>&deg;</p>
    <p>Target Dist: <span id="b3_tgdist"></span>m</p>
    <p>GPS Dir: <span id="b3_gpsdir"></span>&deg;</p>
    <p>Wind Dir: <span id="b3_wdir"></span>&deg;</p>
    <p>Wind StdDev: <span id="b3_wstd"></span></p>
  </div>
</div>
</body>
</html>
)rawliteral";

/*
    Setup OTA
*/
bool setup_OTA()
{
    char ssidl[20];
    char buf[30];
    byte mac[6];
    WiFi.macAddress(mac);
    Serial.print("SETUP OTA...");
    sprintf(buf, "Top_%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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
                          { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
    ArduinoOTA.onError([](ota_error_t error)
                       { ESP.restart(); });
    /* setup the OTA server */
    ArduinoOTA.begin();
    Serial.println("...done!");
    Serial.print("OTA ID: ");
    Serial.println(buf);
    return true;
}

/*
    Scan for networks
*/
bool scan_for_wifi_ap(String ssipap, String ww, IPAddress *tmp)
{
    unsigned long timeout = millis();
    Serial.print("scan for for ap:");
    Serial.println(ssipap);
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
            if (WiFi.SSID(i) == ssipap.c_str())
            {
                Serial.print("Acces point foud, logging in...");
                WiFi.begin(ssipap.c_str(), ww.c_str());
                while (WiFi.status() != WL_CONNECTED)
                {
                    delay(50);
                    Serial.print(".");
                    if (timeout + 60 * 1000 < millis())
                    {
                        esp_restart();
                    }
                }
                Serial.print(".\r\n");
                Serial.print("Loggend in to SSIS: ");
                Serial.println(ssipap);
                *tmp = (WiFi.localIP());
                return true;
            }
        }
    }
    return false;
}

/*
    Setup accecpoint
*/
void setup_wifi_ap(String ap, String ww, IPAddress *tmp)
{
    WiFi.mode(WIFI_AP); // Set Wi-Fi mode to Access Point
    Serial.println("Setting up access point now");

    // Configure static IP
    IPAddress local_IP(192, 168, 1, 84); // Desired static IP address
    IPAddress subnet(255, 255, 255, 0);  // Subnet mask
    IPAddress gateway(192, 168, 1, 5);   // Gateway address
    IPAddress primaryDNS(0, 0, 0, 0);    // Primary DNS (optional)
    IPAddress secondaryDNS(0, 0, 0, 0);  // Secondary DNS (optional)

    // Set the static IP address if possible
    if (!WiFi.softAPConfig(local_IP, gateway, subnet))
    {
        Serial.println("Failed to configure static IP for AP");
    }

    // Start the access point with the given SSID and password
    if (WiFi.softAP(ap.c_str(), ww.c_str()))
    {
        Serial.print("AP SSID: ");
        Serial.println(ap);

        // Get the AP's IP address and store it in the pointer tmp
        *tmp = WiFi.softAPIP();
        Serial.print("AP IP address: ");
        Serial.println(*tmp);
    }
    else
    {
        Serial.println("Failed to start access point");
    }
}
/*
    Setup Async UDP
*/
bool udp_setup(int poort)
{
    if (udp.listen(poort))
    {
        Serial.print("Udp port: ");
        Serial.println(poort);
        udp.onPacket([](AsyncUDPPacket packet)
                     {
                        String stringUdpIn((const char *)packet.data(), packet.length());
                        RoboStruct udpDataIn;
                        rfDeCode(stringUdpIn,&udpDataIn);
                        if (udpDataIn.IDs != -1 && udpDataIn.IDs != mac) // ignore own messages
                        {
                            xQueueSend(udpIn, (void *)&udpDataIn, 10); // notify main there is new data
                            if (wifiCollorUtil.color != CRGB::DarkBlue)
                            {
                                wifiCollorUtil.blink = BLINK_SLOW;
                                wifiCollorUtil.color = CRGB::DarkBlue;
                                xQueueSend(ledUtil, (void *)&wifiCollorUtil, 0); // update GPS led
                            }
                            lastUpdMsg = millis();
                        }
                        else
                        {
                            Serial.println("crc error: " + stringUdpIn);
                        } });
        return true;
    }
    return false;
}

void udpSend(String data)
{
    udp.broadcast(data.c_str());
}

/*
    init WiFi que
*/
unsigned long espMac(void)
{
    byte macarr[6];
    if (mac == 0)
    {
        WiFi.macAddress(macarr);
        for (int i = 2; i < 6; i++)
        {
            mac = (mac << 8) | macarr[i];
        }
    }
    return mac;
}

/*
    init WiFi que
*/
unsigned long initwifiqueue(void)
{
    udpOut = xQueueCreate(10, sizeof(RoboStruct));
    udpIn = xQueueCreate(10, sizeof(RoboStruct));
    return espMac();
}

/*
    WiFi task
*/
void WiFiTask(void *arg)
{
    byte mac[6];
    char macStr[20];
    WiFi.macAddress(mac);
    sprintf(macStr, "%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    int wifiConfig = *((int *)arg);
    unsigned long nwUpdate = millis();
    unsigned char numClients = 0;
    String ap = "";
    String apww = "";
    unsigned long nextSamp = millis();
    if (wifiConfig == 1)
    {
        ap = "PAIR_ME_";
        ap += macStr;
        setup_wifi_ap(ap, apww, &ipTop);
    }
    else // try to find accespoint NicE_WiFi. If no succes make a accecpoint BUOY_[MAC]
    {
        wifiCollorUtil.color = CRGB::LightBlue;
        wifiCollorUtil.blink = FADE_ON;
        wifiCollorUtil.fadeAmount = 5;
        xQueueSend(ledStatus, (void *)&wifiCollorUtil, 10); // update util led
        ap = "NicE_WiFi";
        apww = "!Ni1001100110";
        if (scan_for_wifi_ap(ap, apww, &ipTop) == false)
        {
            ap = "BUOY_";
            ap += macStr;
            apww = "";
            setup_wifi_ap(ap, apww, &ipTop);
        }
    }
    Serial.print("IP address: ");
    Serial.println(ipTop);
    ota = setup_OTA();
    udp_setup(1001);
    wifiCollorUtil.color = CRGB::Black;
    wifiCollorUtil.blink = BLINK_OFF;
    xQueueSend(ledStatus, (void *)&wifiCollorUtil, 10); // update util led

    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", INDEX_HTML);
    });

    server.on("/data", HTTP_GET, []() {
        String json = "{\"buoys\":[";
        
        // Buoy 1 (Top Buoy local data)
        json += "{";
        json += "\"ID\":\"" + String(espMac(), HEX) + "\",";
        json += "\"Status\":" + String(mainData.status) + ",";
        json += "\"Speed\":\"" + String(mainData.speedSet, 2) + "\",";
        json += "\"BB\":\"" + String(mainData.speedBb) + "\",";
        json += "\"SB\":\"" + String(mainData.speedSb) + "\",";
        json += "\"MagDir\":\"" + String(mainData.dirMag, 2) + "\",";
        json += "\"TgDir\":\"" + String(mainData.tgDir, 2) + "\",";
        json += "\"TgDist\":\"" + String(mainData.tgDist, 2) + "\",";
        json += "\"GpsDir\":\"" + String(mainData.gpsDir) + "\",";
        json += "\"WDir\":\"" + String(mainData.wDir, 2) + "\",";
        json += "\"WStd\":\"" + String(mainData.wStd, 2) + "\"";
        json += "},";

        // Buoy 2 and 3 (from buoyPara placeholders)
        for (int i = 1; i < 3; i++) {
            json += "{";
            json += "\"ID\":\"" + String(buoyPara[i].IDs, HEX) + "\",";
            json += "\"Status\":" + String(buoyPara[i].status) + ",";
            json += "\"Speed\":\"" + String(buoyPara[i].speedSet, 2) + "\",";
            json += "\"BB\":\"" + String(buoyPara[i].speedBb) + "\",";
            json += "\"SB\":\"" + String(buoyPara[i].speedSb) + "\",";
            json += "\"MagDir\":\"" + String(buoyPara[i].dirMag, 2) + "\",";
            json += "\"TgDir\":\"" + String(buoyPara[i].tgDir, 2) + "\",";
            json += "\"TgDist\":\"" + String(buoyPara[i].tgDist, 2) + "\",";
            json += "\"GpsDir\":\"" + String(buoyPara[i].gpsDir) + "\",";
            json += "\"WDir\":\"" + String(buoyPara[i].wDir, 2) + "\",";
            json += "\"WStd\":\"" + String(buoyPara[i].wStd, 2) + "\"";
            json += "}";
            if (i < 2) json += ",";
        }
        json += "]}";
        server.send(200, "application/json", json);
    });

    server.begin();
    printf("WiFI task running!\r\n");
    /*
        WiFI loop
    */
    for (;;)
    {
        server.handleClient();
        if (ota == true)
        {
            ArduinoOTA.handle();
        }

        if (WiFi.softAPgetStationNum() != numClients)
        {
            numClients = WiFi.softAPgetStationNum();
            Serial.print("You found me!\r\n");
            if (ap.indexOf("PAIR_ME_") != -1) // reboot if in pairing mode only
            {
                Serial.println("Rebooting in 5 seconds");
                delay(5000);
                esp_restart();
            }
        }

        if (xQueueReceive(udpOut, (void *)&msgIdOut, 1) == pdTRUE)
        {
            if (msgIdOut.IDs == 0) msgIdOut.IDs = espMac();
            if (msgIdOut.IDr == 0) msgIdOut.IDr = ROBOBASE; 
            String out = rfCode(&msgIdOut);
            udp.broadcast(out.c_str());
        }
        delay(1);
    }
}