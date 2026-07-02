#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>

// Sensor Libraries
#include "ICM_20948.h"
#include <LSM303AGR_ACC_Sensor.h>
#include <LSM303AGR_MAG_Sensor.h>

// -----------------------------------------------------------------------------
// Preferences & Calibration NVS Storage
// -----------------------------------------------------------------------------
Preferences preferences;

bool isCalibrating = false;

// ICM-20948 calibration variables (stored in NVS)
float icmMinX = 10000.0f, icmMinY = 10000.0f, icmMinZ = 10000.0f;
float icmMaxX = -10000.0f, icmMaxY = -10000.0f, icmMaxZ = -10000.0f;
float icmOffsetX = 0.0f, icmOffsetY = 0.0f, icmOffsetZ = 0.0f;
float icmScaleX = 1.0f, icmScaleY = 1.0f, icmScaleZ = 1.0f;

// LSM303AGTR calibration variables (stored in NVS)
float lsmMinX = 1000000.0f, lsmMinY = 1000000.0f, lsmMinZ = 1000000.0f;
float lsmMaxX = -1000000.0f, lsmMaxY = -1000000.0f, lsmMaxZ = -1000000.0f;
float lsmOffsetX = 0.0f, lsmOffsetY = 0.0f, lsmOffsetZ = 0.0f;
float lsmScaleX = 1.0f, lsmScaleY = 1.0f, lsmScaleZ = 1.0f;

void loadCalibration() {
  preferences.begin("compass_cal", true); // read-only
  icmOffsetX = preferences.getFloat("icm_ox", 0.0f);
  icmOffsetY = preferences.getFloat("icm_oy", 0.0f);
  icmOffsetZ = preferences.getFloat("icm_oz", 0.0f);
  icmScaleX = preferences.getFloat("icm_sx", 1.0f);
  icmScaleY = preferences.getFloat("icm_sy", 1.0f);
  icmScaleZ = preferences.getFloat("icm_sz", 1.0f);

  lsmOffsetX = preferences.getFloat("lsm_ox", 0.0f);
  lsmOffsetY = preferences.getFloat("lsm_oy", 0.0f);
  lsmOffsetZ = preferences.getFloat("lsm_oz", 0.0f);
  lsmScaleX = preferences.getFloat("lsm_sx", 1.0f);
  lsmScaleY = preferences.getFloat("lsm_sy", 1.0f);
  lsmScaleZ = preferences.getFloat("lsm_sz", 1.0f);
  preferences.end();
  Serial.println("Calibration loaded from NVS:");
  Serial.printf("ICM Offsets: [%.2f, %.2f, %.2f] Scales: [%.3f, %.3f, %.3f]\r\n", icmOffsetX, icmOffsetY, icmOffsetZ, icmScaleX, icmScaleY, icmScaleZ);
  Serial.printf("LSM Offsets: [%.2f, %.2f, %.2f] Scales: [%.3f, %.3f, %.3f]\r\n", lsmOffsetX, lsmOffsetY, lsmOffsetZ, lsmScaleX, lsmScaleY, lsmScaleZ);
}

void saveCalibration() {
  preferences.begin("compass_cal", false); // read-write
  preferences.putFloat("icm_ox", icmOffsetX);
  preferences.putFloat("icm_oy", icmOffsetY);
  preferences.putFloat("icm_oz", icmOffsetZ);
  preferences.putFloat("icm_sx", icmScaleX);
  preferences.putFloat("icm_sy", icmScaleY);
  preferences.putFloat("icm_sz", icmScaleZ);

  preferences.putFloat("lsm_ox", lsmOffsetX);
  preferences.putFloat("lsm_oy", lsmOffsetY);
  preferences.putFloat("lsm_oz", lsmOffsetZ);
  preferences.putFloat("lsm_sx", lsmScaleX);
  preferences.putFloat("lsm_sy", lsmScaleY);
  preferences.putFloat("lsm_sz", lsmScaleZ);
  preferences.end();
  Serial.println("Calibration saved to NVS.");
}

// -----------------------------------------------------------------------------
// Wi-Fi Configuration
// -----------------------------------------------------------------------------
const char* ssid = "NicE_WiFi";
const char* password = "!Ni1001100110";

// Web Server
WiFiServer server(80);

// -----------------------------------------------------------------------------
// Heltec Wireless Tracker - Hardware Pins & Config
// -----------------------------------------------------------------------------
#define VEXT_PIN     3   // Power rail enable (HIGH = power on TFT, GNSS, etc.)
#define BL_PIN       21  // Display Backlight (HIGH = backlight ON)
#define TFT_CS       38  // Chip Select
#define TFT_RST      39  // Reset
#define TFT_DC       40  // Data/Command (RS)
#define TFT_SCLK     41  // SPI Clock
#define TFT_MOSI     42  // SPI MOSI

// Recommended I2C pins for external sensors on Wireless Tracker
#define I2C_SDA      4
#define I2C_SCL      5

// Onboard LED
#define LED_PIN      18 

// Initialize ST7735 TFT using software SPI
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// -----------------------------------------------------------------------------
// Sensor Instances
// -----------------------------------------------------------------------------
ICM_20948_I2C myICM;
LSM303AGR_MAG_Sensor Mag(&Wire);
LSM303AGR_ACC_Sensor Acc(&Wire);

bool icmReady = false;
bool lsmReady = false;

float headingICM = 0.0;
float headingLSM = 0.0;

// -----------------------------------------------------------------------------
// Global Variables
// -----------------------------------------------------------------------------
unsigned long lastUpdate = 0;
bool ledState = false;

// -----------------------------------------------------------------------------
// Webpage HTML Source (Stored in Flash Memory)
// -----------------------------------------------------------------------------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Dual Compass Comparison</title>
  <style>
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      background: #121212;
      color: #e0e0e0;
      margin: 0;
      padding: 20px;
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
    }
    h1 {
      margin-bottom: 5px;
      color: #00e6ff;
      font-weight: 300;
      text-transform: uppercase;
      letter-spacing: 2px;
      text-align: center;
    }
    .sub {
      color: #888;
      margin-bottom: 30px;
      font-size: 0.95rem;
    }
    .container {
      display: flex;
      flex-wrap: wrap;
      justify-content: center;
      gap: 30px;
      max-width: 900px;
      width: 100%;
    }
    .card {
      background: #1e1e1e;
      border: 1px solid #333;
      border-radius: 16px;
      padding: 30px;
      flex: 1;
      min-width: 280px;
      max-width: 400px;
      display: flex;
      flex-direction: column;
      align-items: center;
      box-shadow: 0 8px 30px rgba(0,0,0,0.6);
      transition: border-color 0.3s;
    }
    .card:hover {
      border-color: #555;
    }
    .card h2 {
      margin: 0 0 5px 0;
      font-size: 1.6rem;
      color: #ffd700;
    }
    .chip {
      background: #2a2a2a;
      border-radius: 20px;
      padding: 4px 12px;
      font-size: 0.75rem;
      color: #aaa;
      margin-bottom: 25px;
      text-transform: uppercase;
      letter-spacing: 1px;
    }
    .compass-wrapper {
      position: relative;
      width: 220px;
      height: 220px;
      margin-bottom: 25px;
    }
    .compass {
      width: 100%;
      height: 100%;
    }
    .needle {
      transform-origin: 100px 100px;
      transition: transform 0.15s cubic-bezier(0.25, 0.46, 0.45, 0.94);
    }
    .heading-display {
      font-size: 3rem;
      font-weight: 700;
      color: #00ff66;
      margin-top: 5px;
      font-family: monospace;
    }
    .status {
      font-size: 0.9rem;
      margin-top: 15px;
    }
    .error-msg {
      color: #ff3333;
      font-weight: bold;
    }
    .ok-msg {
      color: #00ff66;
      font-weight: bold;
    }
    .cal-section {
      margin-top: 30px;
      padding: 20px;
      background: #1e1e1e;
      border: 1px solid #333;
      border-radius: 16px;
      width: 100%;
      box-sizing: border-box;
      max-width: 860px;
      display: flex;
      flex-direction: column;
      align-items: center;
      box-shadow: 0 8px 30px rgba(0,0,0,0.6);
    }
    .cal-btn {
      padding: 12px 28px;
      font-size: 1rem;
      font-weight: bold;
      color: #121212;
      background: #00e6ff;
      border: none;
      border-radius: 30px;
      cursor: pointer;
      transition: all 0.3s ease;
      box-shadow: 0 4px 15px rgba(0, 230, 255, 0.4);
      text-transform: uppercase;
      letter-spacing: 1px;
    }
    .cal-btn:hover {
      background: #00b3cc;
      transform: translateY(-2px);
    }
    .cal-btn.calibrating {
      background: #ff3333;
      color: #fff;
      box-shadow: 0 4px 15px rgba(255, 51, 51, 0.4);
    }
    .cal-status {
      font-size: 0.95rem;
      color: #aaa;
      margin-top: 15px;
      text-align: center;
      line-height: 1.6;
    }
  </style>
</head>
<body>
  <h1>Dual Compass Comparison</h1>
  <div class="sub">Heltec Wireless Tracker Live Web Dashboard</div>
  
  <div class="container">
    <!-- ICM-20948 Card -->
    <div class="card">
      <h2>ICM-20948</h2>
      <div class="chip">InvenSense 9DoF</div>
      <div class="compass-wrapper">
        <svg class="compass" viewBox="0 0 200 200">
          <circle cx="100" cy="100" r="95" fill="#161616" stroke="#2c2c2c" stroke-width="5"/>
          <circle cx="100" cy="100" r="82" fill="none" stroke="#00e6ff" stroke-width="1.5" stroke-dasharray="3, 5"/>
          
          <!-- Dial ticks -->
          <line x1="100" y1="12" x2="100" y2="18" stroke="#ff3333" stroke-width="3"/>
          <line x1="100" y1="182" x2="100" y2="188" stroke="#eee" stroke-width="2"/>
          <line x1="12" y1="100" x2="18" y2="100" stroke="#eee" stroke-width="2"/>
          <line x1="182" y1="100" x2="188" y2="100" stroke="#eee" stroke-width="2"/>
          
          <!-- Dial Labels -->
          <text x="100" y="32" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#ff3333" text-anchor="middle">N</text>
          <text x="100" y="184" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#bbb" text-anchor="middle">S</text>
          <text x="176" y="106" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#bbb" text-anchor="middle">E</text>
          <text x="24" y="106" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#bbb" text-anchor="middle">W</text>
          
          <!-- Needle -->
          <g class="needle" id="icm-needle">
            <polygon points="100,20 108,100 100,108" fill="#ff3333"/>
            <polygon points="100,180 108,100 100,108" fill="#555"/>
            <polygon points="100,20 92,100 100,108" fill="#cc0000"/>
            <polygon points="100,180 92,100 100,108" fill="#333"/>
            <circle cx="100" cy="100" r="7" fill="#ffd700" stroke="#121212" stroke-width="2"/>
          </g>
        </svg>
      </div>
      <div class="heading-display" id="icm-val">---&deg;</div>
      <div style="font-size: 0.85rem; color: #888; margin-top: 5px; margin-bottom: 5px;">
        Pitch: <span id="icm-pitch" style="font-family: monospace; color: #bbb;">0.0</span>&deg; | Roll: <span id="icm-roll" style="font-family: monospace; color: #bbb;">0.0</span>&deg;
      </div>
      <div class="status" id="icm-status">Initializing...</div>
    </div>
    
    <!-- LSM303AGTR Card -->
    <div class="card">
      <h2>LSM303AGTR</h2>
      <div class="chip">STMicroelectronics</div>
      <div class="compass-wrapper">
        <svg class="compass" viewBox="0 0 200 200">
          <circle cx="100" cy="100" r="95" fill="#161616" stroke="#2c2c2c" stroke-width="5"/>
          <circle cx="100" cy="100" r="82" fill="none" stroke="#00e6ff" stroke-width="1.5" stroke-dasharray="3, 5"/>
          
          <!-- Dial ticks -->
          <line x1="100" y1="12" x2="100" y2="18" stroke="#ff3333" stroke-width="3"/>
          <line x1="100" y1="182" x2="100" y2="188" stroke="#eee" stroke-width="2"/>
          <line x1="12" y1="100" x2="18" y2="100" stroke="#eee" stroke-width="2"/>
          <line x1="182" y1="100" x2="188" y2="100" stroke="#eee" stroke-width="2"/>
          
          <!-- Dial Labels -->
          <text x="100" y="32" font-size="18" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#ff3333" text-anchor="middle">N</text>
          <text x="100" y="184" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#bbb" text-anchor="middle">S</text>
          <text x="176" y="106" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#bbb" text-anchor="middle">E</text>
          <text x="24" y="106" font-size="16" font-family="'Segoe UI', sans-serif" font-weight="bold" fill="#bbb" text-anchor="middle">W</text>
          
          <!-- Needle -->
          <g class="needle" id="lsm-needle">
            <polygon points="100,20 108,100 100,108" fill="#ff3333"/>
            <polygon points="100,180 108,100 100,108" fill="#555"/>
            <polygon points="100,20 92,100 100,108" fill="#cc0000"/>
            <polygon points="100,180 92,100 100,108" fill="#333"/>
            <circle cx="100" cy="100" r="7" fill="#ffd700" stroke="#121212" stroke-width="2"/>
          </g>
        </svg>
      </div>
      <div class="heading-display" id="lsm-val">---&deg;</div>
      <div style="font-size: 0.85rem; color: #888; margin-top: 5px; margin-bottom: 5px;">
        Pitch: <span id="lsm-pitch" style="font-family: monospace; color: #bbb;">0.0</span>&deg; | Roll: <span id="lsm-roll" style="font-family: monospace; color: #bbb;">0.0</span>&deg;
      </div>
      <div class="status" id="lsm-status">Initializing...</div>
    </div>
  </div>
  
  <!-- Calibration Section -->
  <div class="cal-section">
    <button id="calBtn" class="cal-btn" onclick="toggleCal()">Start Calibration</button>
    <div id="calStatus" class="cal-status">Loading calibration status...</div>
  </div>
  
  <script>
    let isCalibrating = false;
    let currentIcmAngle = 0;
    let currentLsmAngle = 0;

    // Calculates the shortest angular path to prevent flipping when wrapping 360 -> 0
    function getShortestRotation(currentRotation, targetAngle) {
      targetAngle = (targetAngle % 360 + 360) % 360;
      let currentNormalized = (currentRotation % 360 + 360) % 360;
      let diff = targetAngle - currentNormalized;
      if (diff > 180) diff -= 360;
      if (diff < -180) diff += 360;
      return currentRotation + diff;
    }

    function toggleCal() {
      const cmd = isCalibrating ? 'stop' : 'start';
      fetch('/calibrate?cmd=' + cmd)
        .then(response => response.text())
        .then(res => {
          console.log('Calibration Command: ' + cmd + ' -> ' + res);
        })
        .catch(err => console.error('Calibration error:', err));
    }

    function updateData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          if (data.icm_ok) {
            // Rotates the needle to point to Magnetic North based on heading (smoothened shortest rotation)
            currentIcmAngle = getShortestRotation(currentIcmAngle, -data.icm);
            document.getElementById('icm-needle').style.transform = `rotate(${currentIcmAngle}deg)`;
            document.getElementById('icm-val').innerHTML = `${data.icm.toFixed(1)}&deg;`;
            document.getElementById('icm-pitch').innerHTML = `${data.icm_pitch.toFixed(1)}`;
            document.getElementById('icm-roll').innerHTML = `${data.icm_roll.toFixed(1)}`;
            document.getElementById('icm-status').innerHTML = '<span class="ok-msg">ONLINE</span>';
          } else {
            document.getElementById('icm-status').innerHTML = '<span class="error-msg">SENSOR NOT DETECTED</span>';
          }
          
          if (data.lsm_ok) {
            // Rotates the needle to point to Magnetic North based on heading (smoothened shortest rotation)
            currentLsmAngle = getShortestRotation(currentLsmAngle, -data.lsm);
            document.getElementById('lsm-needle').style.transform = `rotate(${currentLsmAngle}deg)`;
            document.getElementById('lsm-val').innerHTML = `${data.lsm.toFixed(1)}&deg;`;
            document.getElementById('lsm-pitch').innerHTML = `${data.lsm_pitch.toFixed(1)}`;
            document.getElementById('lsm-roll').innerHTML = `${data.lsm_roll.toFixed(1)}`;
            document.getElementById('lsm-status').innerHTML = '<span class="ok-msg">ONLINE</span>';
          } else {
            document.getElementById('lsm-status').innerHTML = '<span class="error-msg">SENSOR NOT DETECTED</span>';
          }

          // Handle Calibration State & Display
          isCalibrating = data.is_calibrating;
          const calBtn = document.getElementById('calBtn');
          const calStatus = document.getElementById('calStatus');
          
          const icmOffStr = data.icm_offset ? data.icm_offset.map(v => v.toFixed(1)).join(', ') : '0.0, 0.0, 0.0';
          const lsmOffStr = data.lsm_offset ? data.lsm_offset.map(v => v.toFixed(1)).join(', ') : '0.0, 0.0, 0.0';

          if (isCalibrating) {
            calBtn.innerText = "Stop & Save Calibration";
            calBtn.classList.add('calibrating');
            calStatus.innerHTML = `
              <strong style="color: #ff3333; text-transform: uppercase; letter-spacing: 1px;">Calibrating...</strong><br>
              <span style="color: #888; font-size: 0.85rem;">Rotate the device in all directions in a Figure-8 pattern. Live boundary values will expand.</span><br>
              <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 30px; margin-top: 15px; width: 100%;">
                <div style="text-align: left; background: #252525; padding: 12px; border-radius: 8px; min-width: 220px; border: 1px solid #444;">
                  <strong style="color: #ffd700;">ICM-20948 Live Limits</strong><br>
                  X-axis: [${data.icm_min[0].toFixed(1)}, ${data.icm_max[0].toFixed(1)}]<br>
                  Y-axis: [${data.icm_min[1].toFixed(1)}, ${data.icm_max[1].toFixed(1)}]<br>
                  Z-axis: [${data.icm_min[2].toFixed(1)}, ${data.icm_max[2].toFixed(1)}]
                </div>
                <div style="margin: 0; text-align: left; padding: 12px; background: #252525; border-radius: 8px; min-width: 220px; border: 1px solid #444;">
                  <strong style="color: #00e6ff;">LSM303AGTR Live Limits</strong><br>
                  X-axis: [${data.lsm_min[0].toFixed(1)}, ${data.lsm_max[0].toFixed(1)}]<br>
                  Y-axis: [${data.lsm_min[1].toFixed(1)}, ${data.lsm_max[1].toFixed(1)}]<br>
                  Z-axis: [${data.lsm_min[2].toFixed(1)}, ${data.lsm_max[2].toFixed(1)}]
                </div>
              </div>`;
          } else {
            calBtn.innerText = "Start Calibration";
            calBtn.classList.remove('calibrating');
            
            const icmScaleStr = data.icm_scale ? data.icm_scale.map(v => v.toFixed(3)).join(' / ') : '1.000 / 1.000 / 1.000';
            const lsmScaleStr = data.lsm_scale ? data.lsm_scale.map(v => v.toFixed(3)).join(' / ') : '1.000 / 1.000 / 1.000';

            calStatus.innerHTML = `
              <strong style="color: #00ff66; text-transform: uppercase; letter-spacing: 1px;">Live Magnetic Vectors & Soft Iron Scales</strong><br>
              <div style="display: flex; flex-wrap: wrap; justify-content: center; gap: 40px; margin-top: 15px; width: 100%;">
                <div style="text-align: left; background: #1a1a1a; padding: 12px 20px; border-radius: 8px; border: 1px solid #2a2a2a; min-width: 240px;">
                  <strong style="color: #ffd700;">ICM-20948 Live</strong><br>
                  Mag X: <span style="font-family: monospace; color: #00ff66;">${data.icm_vec[0].toFixed(2)}</span> uT<br>
                  Mag Y: <span style="color: #00ff66; font-family: monospace;">${data.icm_vec[1].toFixed(2)}</span> uT<br>
                  Mag Z: <span style="color: #00ff66; font-family: monospace;">${data.icm_vec[2].toFixed(2)}</span> uT<br>
                  Scales X/Y/Z: <span style="color: #00e6ff; font-family: monospace;">${icmScaleStr}</span>
                </div>
                <div style="text-align: left; background: #1a1a1a; padding: 12px 20px; border-radius: 8px; border: 1px solid #2a2a2a; min-width: 240px;">
                  <strong style="color: #00e6ff;">LSM303AGTR Live</strong><br>
                  Mag X: <span style="color: #00ff66;">${data.lsm_vec[0].toFixed(2)}</span> uT<br>
                  Mag Y: <span style="color: #00ff66;">${data.lsm_vec[1].toFixed(2)}</span> uT<br>
                  Mag Z: <span style="color: #00ff66;">${data.lsm_vec[2].toFixed(2)}</span> uT<br>
                  Scales X/Y/Z: <span style="color: #00e6ff; font-family: monospace;">${lsmScaleStr}</span>
                </div>
              </div>`;
          }
        })
        .catch(err => {
          console.error(err);
          document.getElementById('icm-status').innerHTML = '<span class="error-msg">OFFLINE</span>';
          document.getElementById('lsm-status').innerHTML = '<span class="error-msg">OFFLINE</span>';
        });
    }
    // Perform fast polling every 400ms for stable and responsive needle animation
    setInterval(updateData, 400);
  </script>
</body>
</html>
)rawliteral";

// Helper to draw a clean base header on the display
void drawScreenBase() {
  tft.fillScreen(ST7735_BLACK);
  tft.fillRect(0, 0, 160, 18, ST7735_BLUE);
  tft.setCursor(8, 4);
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(1);
  tft.print("DUAL COMPASS TEST");
}

// -----------------------------------------------------------------------------
// setup()
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("\n=================================");
  Serial.println("  Dual Compass Comparison Setup  ");
  Serial.println("=================================");

  loadCalibration();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Power on display & peripherals via VEXT and backlight BL
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, HIGH);
  delay(100);

  pinMode(BL_PIN, OUTPUT);
  digitalWrite(BL_PIN, HIGH);
  delay(50);

  // Initialize TFT Display
  tft.initR(INITR_MINI160x80); 
  tft.setRotation(1); 
  drawScreenBase();

  tft.setCursor(5, 24);
  tft.setTextColor(ST7735_YELLOW);
  tft.print("Initializing I2C...");
  
  // Start I2C on user recommended pins
  Wire.begin(I2C_SDA, I2C_SCL, 100000); 
  delay(100);

  // Proactive Sensor Initialization
  tft.setCursor(5, 36);
  tft.setTextColor(ST7735_CYAN);
  tft.print("Detecting sensors...");


  // 1. Initialize ICM-20948 (AK09916 Compass)
  myICM.begin(Wire, 1); // AD0 is high (0x69)
  if (myICM.status == ICM_20948_Stat_Ok) {
    icmReady = true;
    Serial.println("ICM-20948 connection: SUCCESS (0x69)");
  } else {
    Serial.println("ICM-20948 not found at 0x69. Trying 0x68...");
    myICM.begin(Wire, 0); // AD0 is low (0x68)
    if (myICM.status == ICM_20948_Stat_Ok) {
      icmReady = true;
      Serial.println("ICM-20948 connection: SUCCESS (0x68)");
    } else {
      Serial.println("ICM-20948 connection: FAILED!");
    }
  }

  // 2. Initialize LSM303AGR Sensors (Accelerometer + Magnetometer)
  Acc.begin();
  Acc.Enable();
  Mag.begin();
  Mag.Enable();
  
  // Basic validation probe to verify LSM303 Magnetometer is responding
  int32_t tempMag[3];
  if (Mag.GetAxes(tempMag) == 0) {
    lsmReady = true;
    Serial.println("LSM303AGTR Magnetometer connection: SUCCESS");
  } else {
    Serial.println("LSM303AGTR Magnetometer connection: FAILED!");
  }

  // Display Sensor status on TFT
  tft.fillRect(0, 20, 160, 60, ST7735_BLACK);
  tft.setCursor(5, 24);
  tft.setTextColor(icmReady ? ST7735_GREEN : ST7735_RED);
  tft.print("ICM-20948: ");
  tft.print(icmReady ? "OK" : "ERR");

  tft.setCursor(5, 36);
  tft.setTextColor(lsmReady ? ST7735_GREEN : ST7735_RED);
  tft.print("LSM303AGTR: ");
  tft.print(lsmReady ? "OK" : "ERR");

  delay(1500);

  // Connect to Wi-Fi
  tft.fillRect(0, 20, 160, 60, ST7735_BLACK);
  tft.setCursor(5, 24);
  tft.setTextColor(ST7735_YELLOW);
  tft.print("WiFi Connecting:");
  tft.setCursor(5, 36);
  tft.setTextColor(ST7735_WHITE);
  tft.print(ssid);


  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < 30) {
    delay(500);
    Serial.print(".");
    tft.fillRect(0, 48, 160, 20, ST7735_BLACK);
    tft.setCursor(5, 52);
    String loading = "";
    for (int i = 0; i <= (attempt % 5); i++) loading += ".";
    tft.setTextColor(ST7735_CYAN);
    tft.print(loading);
  
    attempt++;
  }

  tft.fillRect(0, 20, 160, 60, ST7735_BLACK);
  tft.setCursor(5, 24);
  if (WiFi.status() == WL_CONNECTED) {
    tft.setTextColor(ST7735_GREEN);
    tft.print("WiFi Connected!");
    tft.setCursor(5, 38);
    tft.setTextColor(ST7735_WHITE);
    tft.print("IP: ");
    tft.print(WiFi.localIP().toString());
    
    server.begin();
    Serial.println("HTTP Web Server Started on Port 80");
  } else {
    tft.setTextColor(ST7735_RED);
    tft.print("WiFi Connect Failed!");
    tft.setCursor(5, 38);
    tft.print("Offline Mode.");
  }

  delay(2500);

  // Configure Arduino OTA
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.setHostname("heltec-tracker");

    // Fast OTA updater without any heavy screen redrawing in onProgress
    ArduinoOTA.onStart([]() {
      Serial.println("OTA Update Started");
      tft.fillScreen(ST7735_BLACK);
      tft.fillRect(0, 0, 160, 18, ST7735_RED);
      tft.setCursor(12, 4);
      tft.setTextColor(ST7735_WHITE);
      tft.print("SYSTEM UPDATE");
      tft.setCursor(5, 36);
      tft.setTextColor(ST7735_YELLOW);
      tft.print("Flashing wirelessly...");
    
    });

    ArduinoOTA.onEnd([]() {
      Serial.println("\nOTA Update Finished Successfully!");
      tft.fillRect(0, 20, 160, 60, ST7735_BLACK);
      tft.setCursor(5, 36);
      tft.setTextColor(ST7735_GREEN);
      tft.print("Success! Rebooting...");
    
      delay(1500);
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      static int lastPercent = -1;
      int percentage = progress / (total / 100);
      if (percentage != lastPercent) {
        lastPercent = percentage;
        Serial.printf("Progress: %u%%\n", percentage);
      }
    });

    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]\n", error);
    });

    ArduinoOTA.begin();
  }

  drawScreenBase();
}

// -----------------------------------------------------------------------------
// loop()
// -----------------------------------------------------------------------------
void loop() {
  // 1. Handle OTA updates
  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
  }

  // 2. Continuous readings from compass sensors
  static float icmRawX = 0.0f, icmRawY = 0.0f, icmRawZ = 0.0f;
  static float icmCalX = 0.0f, icmCalY = 0.0f, icmCalZ = 0.0f;
  static float lsmRawX = 0.0f, lsmRawY = 0.0f, lsmRawZ = 0.0f;
  static float lsmCalX = 0.0f, lsmCalY = 0.0f, lsmCalZ = 0.0f;

  static float icmPitch = 0.0f;
  static float icmRoll = 0.0f;
  static float lsmPitch = 0.0f;
  static float lsmRoll = 0.0f;

  // Read ICM-20948 Magnetometer
  if (icmReady && myICM.dataReady()) {
    myICM.getAGMT();
    icmRawX = myICM.magX();
    icmRawY = myICM.magY();
    icmRawZ = myICM.magZ();

    if (isCalibrating) {
      if (icmRawX < icmMinX) icmMinX = icmRawX;
      if (icmRawY < icmMinY) icmMinY = icmRawY;
      if (icmRawZ < icmMinZ) icmMinZ = icmRawZ;
      if (icmRawX > icmMaxX) icmMaxX = icmRawX;
      if (icmRawY > icmMaxY) icmMaxY = icmRawY;
      if (icmRawZ > icmMaxZ) icmMaxZ = icmRawZ;
    }

    // Apply Hard Iron Offset & Soft Iron Scaling
    icmCalX = (icmRawX - icmOffsetX) * icmScaleX;
    icmCalY = (icmRawY - icmOffsetY) * icmScaleY;
    icmCalZ = (icmRawZ - icmOffsetZ) * icmScaleZ;

    // Calculate Pitch and Roll from Accelerometer (IMU pitch & roll)
    float ax = myICM.accX();
    float ay = myICM.accY();
    float az = myICM.accZ();
    float pitch_rad = atan2(-ax, sqrt(ay*ay + az*az));
    float roll_rad = atan2(ay, az);
    icmPitch = pitch_rad * 180.0f / PI;
    icmRoll = roll_rad * 180.0f / PI;

    float cp = cos(pitch_rad);
    float sp = sin(pitch_rad);
    float cr = cos(roll_rad);
    float sr = sin(roll_rad);

    // Apply Tilt Compensation Rotation (reprojects vectors back to flat horizontal plane)
    float xh = icmCalX * cp + icmCalY * sp * sr + icmCalZ * sp * cr;
    float yh = icmCalY * cr - icmCalZ * sr;

    // Calculate heading using tilt-compensated axes
    float heading = atan2(yh, xh) * 180.0 / PI;
    if (heading < 0) heading += 360.0;
    headingICM = heading;
  }

  // Read LSM303AGTR Magnetometer
  if (lsmReady) {
    int32_t magnetometer[3];
    if (Mag.GetAxes(magnetometer) == 0) {
      lsmRawX = (float)magnetometer[0] / 10.0f;
      lsmRawY = (float)magnetometer[1] / 10.0f;
      lsmRawZ = (float)magnetometer[2] / 10.0f;

      if (isCalibrating) {
        if (lsmRawX < lsmMinX) lsmMinX = lsmRawX;
        if (lsmRawY < lsmMinY) lsmMinY = lsmRawY;
        if (lsmRawZ < lsmMinZ) lsmMinZ = lsmRawZ;
        if (lsmRawX > lsmMaxX) lsmMaxX = lsmRawX;
        if (lsmRawY > lsmMaxY) lsmMaxY = lsmRawY;
        if (lsmRawZ > lsmMaxZ) lsmMaxZ = lsmRawZ;
      }

      // Apply Hard Iron Offset & Soft Iron Scaling
      lsmCalX = (lsmRawX - lsmOffsetX) * lsmScaleX;
      lsmCalY = (lsmRawY - lsmOffsetY) * lsmScaleY;
      lsmCalZ = (lsmRawZ - lsmOffsetZ) * lsmScaleZ;

      // Calculate Pitch and Roll from Accelerometer
      int32_t acc_raw[3];
      float ax = 0.0f, ay = 0.0f, az = 0.0f;
      if (Acc.GetAxes(acc_raw) == 0) {
        ax = (float)acc_raw[0];
        ay = (float)acc_raw[1];
        az = (float)acc_raw[2];
      }
      float pitch_rad = atan2(-ax, sqrt(ay*ay + az*az));
      float roll_rad = atan2(ay, az);
      lsmPitch = pitch_rad * 180.0f / PI;
      lsmRoll = roll_rad * 180.0f / PI;

      float cp = cos(pitch_rad);
      float sp = sin(pitch_rad);
      float cr = cos(roll_rad);
      float sr = sin(roll_rad);

      // Align LSM magnetometer axes with the ICM-20948 coordinate frame before tilt compensation:
      // X_aligned = -Y_lsm, Y_aligned = X_lsm, Z_aligned = Z_lsm
      float mx = -lsmCalY;
      float my = lsmCalX;
      float mz = lsmCalZ;

      // Apply Tilt Compensation Rotation
      float xh = mx * cp + my * sp * sr + mz * sp * cr;
      float yh = my * cr - mz * sr;

      // Calculate heading using tilt-compensated aligned axes
      float heading = atan2(yh, xh) * 180.0 / PI;
      if (heading < 0) heading += 360.0;
      headingLSM = heading;
    }
  }

  // 3. Handle Web Server Requests
  WiFiClient client = server.available();
  if (client) {
    String currentLine = "";
    String requestHeader = "";
    
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        requestHeader += c;
        
        if (c == '\n') {
          if (currentLine.length() == 0) {
            // End of HTTP Request headers
            
            // Check requested route
            if (requestHeader.indexOf("GET /data") >= 0) {
              // Respond with JSON sensor data
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: application/json");
              client.println("Access-Control-Allow-Origin: *"); // Enable CORS
              client.println("Connection: close");
              client.println();
              
              // Construct JSON
              client.print("{");
              client.print("\"icm\": " + String(headingICM, 1) + ",");
              client.print("\"icm_ok\": " + String(icmReady ? "true" : "false") + ",");
              client.print("\"icm_pitch\": " + String(icmPitch, 1) + ",");
              client.print("\"icm_roll\": " + String(icmRoll, 1) + ",");
              client.print("\"lsm\": " + String(headingLSM, 1) + ",");
              client.print("\"lsm_ok\": " + String(lsmReady ? "true" : "false") + ",");
              client.print("\"lsm_pitch\": " + String(lsmPitch, 1) + ",");
              client.print("\"lsm_roll\": " + String(lsmRoll, 1) + ",");
              client.print("\"is_calibrating\": " + String(isCalibrating ? "true" : "false") + ",");
              client.print("\"icm_offset\": [" + String(icmOffsetX, 2) + "," + String(icmOffsetY, 2) + "," + String(icmOffsetZ, 2) + "],");
              client.print("\"lsm_offset\": [" + String(lsmOffsetX, 2) + "," + String(lsmOffsetY, 2) + "," + String(lsmOffsetZ, 2) + "],");
              client.print("\"icm_scale\": [" + String(icmScaleX, 3) + "," + String(icmScaleY, 3) + "," + String(icmScaleZ, 3) + "],");
              client.print("\"lsm_scale\": [" + String(lsmScaleX, 3) + "," + String(lsmScaleY, 3) + "," + String(lsmScaleZ, 3) + "],");
              client.print("\"icm_vec\": [" + String(icmCalX, 2) + "," + String(icmCalY, 2) + "," + String(icmCalZ, 2) + "],");
              client.print("\"lsm_vec\": [" + String(-lsmCalY, 2) + "," + String(lsmCalX, 2) + "," + String(lsmCalZ, 2) + "],");
              client.print("\"icm_min\": [" + String(icmMinX, 1) + "," + String(icmMinY, 1) + "," + String(icmMinZ, 1) + "],");
              client.print("\"icm_max\": [" + String(icmMaxX, 1) + "," + String(icmMaxY, 1) + "," + String(icmMaxZ, 1) + "],");
              client.print("\"lsm_min\": [" + String(lsmMinX, 1) + "," + String(lsmMinY, 1) + "," + String(lsmMinZ, 1) + "],");
              client.print("\"lsm_max\": [" + String(lsmMaxX, 1) + "," + String(lsmMaxY, 1) + "," + String(lsmMaxZ, 1) + "]");
              client.println("}");
            } 
            else if (requestHeader.indexOf("GET /calibrate") >= 0) {
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/plain");
              client.println("Connection: close");
              client.println();
              
              if (requestHeader.indexOf("cmd=start") >= 0) {
                isCalibrating = true;
                icmMinX = 10000.0f; icmMinY = 10000.0f; icmMinZ = 10000.0f;
                icmMaxX = -10000.0f; icmMaxY = -10000.0f; icmMaxZ = -10000.0f;
                
                lsmMinX = 1000000.0f; lsmMinY = 1000000.0f; lsmMinZ = 1000000.0f;
                lsmMaxX = -1000000.0f; lsmMaxY = -1000000.0f; lsmMaxZ = -1000000.0f;
                
                Serial.println("Hard/Soft Iron Calibration STARTED for both sensors.");
                client.println("Calibration started");
              } 
              else if (requestHeader.indexOf("cmd=stop") >= 0) {
                isCalibrating = false;
                
                // Calculate and save ICM offsets & scales if min/max are valid
                if (icmMaxX >= icmMinX && icmMinX < 9999.0f) {
                  icmOffsetX = (icmMaxX + icmMinX) / 2.0f;
                  icmOffsetY = (icmMaxY + icmMinY) / 2.0f;
                  icmOffsetZ = (icmMaxZ + icmMinZ) / 2.0f;
                  
                  float rx = icmMaxX - icmMinX;
                  float ry = icmMaxY - icmMinY;
                  float rz = icmMaxZ - icmMinZ;
                  if (rx > 0.1f && ry > 0.1f) {
                    if (rz > 5.0f) { // 3D Calibration
                      float avgRange = (rx + ry + rz) / 3.0f;
                      icmScaleX = avgRange / rx;
                      icmScaleY = avgRange / ry;
                      icmScaleZ = avgRange / rz;
                    } else { // 2D Calibration
                      float avgRange = (rx + ry) / 2.0f;
                      icmScaleX = avgRange / rx;
                      icmScaleY = avgRange / ry;
                      icmScaleZ = 1.0f;
                    }
                  } else {
                    icmScaleX = 1.0f; icmScaleY = 1.0f; icmScaleZ = 1.0f;
                  }
                }
                
                // Calculate and save LSM offsets & scales if min/max are valid
                if (lsmMaxX >= lsmMinX && lsmMinX < 9999.0f) {
                  lsmOffsetX = (lsmMaxX + lsmMinX) / 2.0f;
                  lsmOffsetY = (lsmMaxY + lsmMinY) / 2.0f;
                  lsmOffsetZ = (lsmMaxZ + lsmMinZ) / 2.0f;
                  
                  float rx = lsmMaxX - lsmMinX;
                  float ry = lsmMaxY - lsmMinY;
                  float rz = lsmMaxZ - lsmMinZ;
                  if (rx > 0.1f && ry > 0.1f) {
                    if (rz > 5.0f) { // 3D Calibration
                      float avgRange = (rx + ry + rz) / 3.0f;
                      lsmScaleX = avgRange / rx;
                      lsmScaleY = avgRange / ry;
                      lsmScaleZ = avgRange / rz;
                    } else { // 2D Calibration
                      float avgRange = (rx + ry) / 2.0f;
                      lsmScaleX = avgRange / rx;
                      lsmScaleY = avgRange / ry;
                      lsmScaleZ = 1.0f;
                    }
                  } else {
                    lsmScaleX = 1.0f; lsmScaleY = 1.0f; lsmScaleZ = 1.0f;
                  }
                }
                
                saveCalibration();
                Serial.println("Hard/Soft Iron Calibration STOPPED and saved to NVS.");
                client.println("Calibration stopped and saved");
              }
              else {
                client.println("Invalid calibration command");
              }
            }
            else {
              // Respond with main HTML Webpage
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: text/html");
              client.println("Connection: close");
              client.println();
              client.println(index_html);
            }
            break;
          } else {
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
  }

  // 4. Update TFT screen and LED (every 300ms)
  if (millis() - lastUpdate >= 300) {
    lastUpdate = millis();

    // Output raw and calibrated data over serial for plotting/analysis
    Serial.printf("ICM_RAW:[%.1f,%.1f,%.1f] ICM_CAL:[%.1f,%.1f,%.1f] Head:%0.1f | LSM_RAW:[%.1f,%.1f,%.1f] LSM_CAL:[%.1f,%.1f,%.1f] Head:%0.1f\r\n",
                  icmRawX, icmRawY, icmRawZ, icmCalX, icmCalY, icmCalZ, headingICM,
                  lsmRawX, lsmRawY, lsmRawZ, lsmCalX, lsmCalY, lsmCalZ, headingLSM);

    ledState = !ledState;
    digitalWrite(LED_PIN, LOW); // Keep physical LED off

    // Render data on local ST7735 screen
    // We NO LONGER clear the screen with fillRect to prevent flickering.
    // Instead, we overwrite existing text by specifying a background color in setTextColor().
    tft.setTextSize(1);

    // Web server IP status
    tft.setCursor(5, 24);
    if (WiFi.status() == WL_CONNECTED) {
      tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
      tft.print("Web: http://");
      tft.print(WiFi.localIP().toString());
      tft.print("   "); // Trailing spaces to overwrite any old longer text
    } else {
      tft.setTextColor(ST7735_RED, ST7735_BLACK);
      tft.print("Web Server: OFFLINE    ");
    }

    // ICM-20948 heading
    tft.setCursor(5, 42);
    tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    tft.print("ICM-20948: ");
    if (icmReady) {
      tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
      tft.print(headingICM, 1);
      tft.write(0xF7); // Degrees symbol in ST7735 font
      tft.print("   "); // Trailing spaces
    } else {
      tft.setTextColor(ST7735_RED, ST7735_BLACK);
      tft.print("ERR       ");
    }

    // LSM303AGTR heading
    tft.setCursor(5, 58);
    tft.setTextColor(ST7735_CYAN, ST7735_BLACK);
    tft.print("LSM303AGR: ");
    if (lsmReady) {
      tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
      tft.print(headingLSM, 1);
      tft.write(0xF7);
      tft.print("   "); // Trailing spaces
    } else {
      tft.setTextColor(ST7735_RED, ST7735_BLACK);
      tft.print("ERR       ");
    }

    // Heartbeat box
    if (ledState) {
      tft.fillRect(146, 5, 8, 8, ST7735_GREEN);
    } else {
      tft.drawRect(146, 5, 8, 8, ST7735_WHITE);
    }
  }
}
