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
#include <LittleFS.h>
#include <Adafruit_Sensor.h>
#include <RoboCompute.h>
#include "main.h"
#include "io_sub.h"
#include "compass.h"
#include "datastorage.h"
#include "pidrudspeed.h"
#include "subwifi.h"

// Global instances
WebServer subServer(80);
AsyncUDP udp;
extern float global_speed_bb;
extern float global_speed_sb;
extern bool icm_ready;
extern float global_hdg;
extern RoboStruct mainData;
extern SemaphoreHandle_t mainDataMutex;
extern QueueHandle_t compassIn;
extern float last_raw_x, last_raw_y, last_raw_z;
extern uint8_t bno_cal_sys, bno_cal_gyro, bno_cal_accel, bno_cal_mag;
extern String global_cal_msg;
extern String global_cal_load, global_cal_ver;
extern uint32_t global_loop_cnt;
uint32_t global_params_rev = 0;


static RoboStruct subWifiOut;
static RoboStruct subWifiIn;
QueueHandle_t udpOut = NULL;
QueueHandle_t udpIn = NULL;
String global_mac_str = "";

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

static String indexHtmlCache = "";

/**
 * @brief Core 0 Task: Manages WiFi, WebServer, and UDP communications.
 */
void WiFiTask(void *arg) {
    Serial.println("WiFiTask: Starting...");
    
    if(!LittleFS.begin(true)){
        Serial.println("WiFiTask: LittleFS Mount Failed");
    } else {
        Serial.println("WiFiTask: LittleFS Mounted");
        File file = LittleFS.open("/index.html", "r");
        if(file) {
            indexHtmlCache = file.readString();
            file.close();
            Serial.println("WiFiTask: Cached index.html in RAM (" + String(indexHtmlCache.length()) + " bytes)");
        } else {
            Serial.println("WiFiTask: Failed to open /index.html for caching");
        }
    }

    WiFiManager wm; 
    Serial.println("WiFiTask: Connecting WiFi...");
    if (!wm.autoConnect("NicE-Buoy-Sub")) {
        Serial.println("WiFiTask: Failed to connect and hit timeout");
        delay(3000);
        ESP.restart();
    }
    Serial.print("WiFiTask: Connected. IP: ");
    Serial.println(WiFi.localIP());
    
    // Broadcast/Listen for other buoys on port 1001
    if (udp.listen(1001)) {
        Serial.println("WiFiTask: UDP Listening on 1001");
        udp.onPacket([](AsyncUDPPacket p) {
            RoboStruct d; 
            String s=String((const char*)p.data(),p.length()); 
            rfDeCode(s,&d);
            if(d.IDs!=-1 && d.IDs!=espMac()){
                xQueueSend(udpIn,(void*)&d,10);
            }
        });
    }

    // Dashboard Endpoints served from RAM cache
    subServer.on("/", [](){
        if(indexHtmlCache.length() > 0) {
            subServer.send(200, "text/html", indexHtmlCache);
        } else {
            subServer.send(404, "text/plain", "File not found in RAM cache. Please check LittleFS filesystem.");
        }
    });
    subServer.on("/savecal", [](){ int c=34; xQueueSend(compassIn,(void*)&c,10); subServer.send(200,"text/plain","OK"); });
    
    // Parameter Update API (optimized to hold mutex for minimal duration)
    subServer.on("/setparam", [](){
        if(!subServer.hasArg("p")||!subServer.hasArg("v")){subServer.send(400,"text/plain","Err");return;}
        String p=subServer.arg("p"); float v=subServer.arg("v").toFloat();
        bool paramUpdated = false;
        
        if(mainDataMutex && xSemaphoreTake(mainDataMutex, pdMS_TO_TICKS(500))){
            if(p=="kpr"){mainData.Kpr=v; paramUpdated = true;}
            else if(p=="kir"){mainData.Kir=v; paramUpdated = true;}
            else if(p=="kdr"){mainData.Kdr=v; paramUpdated = true;}
            else if(p=="kps"){mainData.Kps=v; paramUpdated = true;}
            else if(p=="kis"){mainData.Kis=v; paramUpdated = true;}
            else if(p=="kds"){mainData.Kds=v; paramUpdated = true;}
            else if(p=="coff"){mainData.compassOffset=v; paramUpdated = true;}
            else if(p=="pvspd"){mainData.pivotSpeed=v; paramUpdated = true;}
            else if(p=="holdrad"){
                if(v < 1.5f) v = 1.5f; // Safety: Must be > Pivot range (1.0m) + 0.5m buffer
                mainData.holdRad=v; paramUpdated = true;
            }
            else if(p=="minspd"){mainData.minSpeed=(int)v; paramUpdated = true;}
            else if(p=="maxspd"){mainData.maxSpeed=(int)v; paramUpdated = true;}
            else if(p=="revbb"){mainData.revBB=(v>0.5); paramUpdated = true;}
            else if(p=="revsb"){mainData.revSB=(v>0.5); paramUpdated = true;}
            else if(p=="tswap"){mainData.swap_BB_SB=(v>0.5); paramUpdated = true;}
            
            if (paramUpdated) {
                global_params_rev++;
            }
            xSemaphoreGive(mainDataMutex);
        }
        
        if (paramUpdated) {
            // Write parameter updates to NVS flash storage and initialize PID loops outside the mutex
            if(p=="kpr" || p=="kir" || p=="kdr"){pidRudderParameters(&mainData,SET);initRudPid(&mainData);}
            else if(p=="kps" || p=="kis" || p=="kds"){pidSpeedParameters(&mainData,SET);initSpeedPid(&mainData);}
            else if(p=="coff"){CompasOffset(&mainData,SET);}
            else if(p=="pvspd"){speedMaxMin(&mainData,SET);initSpeedPid(&mainData);initRudPid(&mainData);}
            else if(p=="holdrad"){computeParameters(&mainData,SET);initSpeedPid(&mainData);initRudPid(&mainData);}
            else if(p=="minspd" || p=="maxspd"){speedMaxMin(&mainData,SET);initSpeedPid(&mainData);initRudPid(&mainData);}
            else if(p=="revbb" || p=="revsb"){thrusterInversion(&mainData,SET);}
            else if(p=="tswap"){thrusterSwap(&mainData,SET);}
        }
        subServer.send(200,"text/plain","OK");
    });

    // Telemetry and Parameter Read API (optimized to format string outside of mutex lock)
    subServer.on("/params", [](){
        static String last_params = "{\"kpr\":1.0,\"kir\":0.0,\"kdr\":0.0,\"kps\":1.0,\"kis\":0.0,\"kds\":0.0,\"coff\":0.0,\"pvspd\":0.5,\"minspd\":0,\"maxspd\":100,\"holdrad\":2.0,\"revbb\":0,\"revsb\":0,\"tswap\":0}";
        float kpr = 1.0, kir = 0.0, kdr = 0.0, kps = 1.0, kis = 0.0, kds = 0.0, coff = 0.0, pvspd = 0.5, holdrad = 2.0;
        int revbb = 0, revsb = 0, tswap = 0, minspd = 0, maxspd = 100;
        
        if(mainDataMutex && xSemaphoreTake(mainDataMutex, pdMS_TO_TICKS(100))){
            kpr = mainData.Kpr;
            kir = mainData.Kir;
            kdr = mainData.Kdr;
            kps = mainData.Kps;
            kis = mainData.Kis;
            kds = mainData.Kds;
            coff = (float)mainData.compassOffset;
            revbb = mainData.revBB ? 1 : 0;
            revsb = mainData.revSB ? 1 : 0;
            tswap = mainData.swap_BB_SB ? 1 : 0;
            pvspd = (float)mainData.pivotSpeed;
            minspd = mainData.minSpeed;
            maxspd = mainData.maxSpeed;
            holdrad = (float)mainData.holdRad;
            xSemaphoreGive(mainDataMutex);
        }
        
        char buf[500];
        snprintf(buf, sizeof(buf), 
            "{\"kpr\":%.3f,\"kir\":%.3f,\"kdr\":%.3f,\"kps\":%.3f,\"kis\":%.3f,\"kds\":%.3f,\"coff\":%.1f,\"revbb\":%d,\"revsb\":%d,\"tswap\":%d,\"pvspd\":%.2f,\"minspd\":%d,\"maxspd\":%d,\"holdrad\":%.1f}",
            kpr, kir, kdr, kps, kis, kds, coff, revbb, revsb, tswap, pvspd, minspd, maxspd, holdrad
        );
        last_params = String(buf);
        subServer.send(200,"application/json", last_params);
    });

    subServer.on("/data", [](){
        float icm = global_hdg;
        int sbb = 0, ssb = 0;
        double ir = 0, ip = 0;
        
        if (mainDataMutex && xSemaphoreTake(mainDataMutex, pdMS_TO_TICKS(100))) {
            sbb = (int)mainData.speedBb; 
            ssb = (int)mainData.speedSb;
            ir = mainData.ir;
            ip = mainData.ip;
            xSemaphoreGive(mainDataMutex);
        }

        String j="{"; 
        j+="\"icm\":"+String(icm,2)+",";
        j+="\"speed_bb\":"+String(sbb)+",";
        j+="\"speed_sb\":"+String(ssb)+",";
        j+="\"ir\":"+String(ir,2)+",";
        j+="\"ip\":"+String(ip,2)+",";
        j+="\"cal_load\":\""+global_cal_load+"\",";
        j+="\"cal_ver\":\""+global_cal_ver+"\",";
        j+="\"mac\":\""+global_mac_str+"\",";
        j+="\"cal_levels\":["+String(bno_cal_sys)+","+String(bno_cal_gyro)+","+String(bno_cal_accel)+","+String(bno_cal_mag)+"],";
        j+="\"cal_msg\":\""+global_cal_msg+"\",";
        j+="\"rev\":"+String(global_params_rev);
        j+="}";
        subServer.send(200,"application/json",j);
    });

    subServer.begin(); 
    Serial.println("WiFiTask: WebServer started");
    
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
    Serial.println("WiFiTask: OTA started");

    // Main Server Loop
    while(1){ 
        subServer.handleClient(); 
        ArduinoOTA.handle();
        if (udpOut && xQueueReceive(udpOut, (void *)&subWifiOut, 0) == pdTRUE) { 
            subWifiOut.IDs = espMac(); 
            String out = rfCode(&subWifiOut); udp.broadcast(out.c_str()); 
        }
        vTaskDelay(pdMS_TO_TICKS(2)); 
    }
}
