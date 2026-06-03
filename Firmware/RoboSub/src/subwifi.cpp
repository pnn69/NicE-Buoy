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
 * @brief Core 0 Task: Manages WiFi, WebServer, and UDP communications.
 */
void WiFiTask(void *arg) {
    if(!LittleFS.begin(true)){
        Serial.println("LittleFS Mount Failed");
    }

    WiFiManager wm; 
    
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
    subServer.on("/", [](){
        File file = LittleFS.open("/index.html", "r");
        if(!file) {
            subServer.send(404, "text/plain", "File not found. Please upload the 'data' folder using 'Upload Filesystem Image'.");
            return;
        }
        subServer.streamFile(file, "text/html");
        file.close();
    });
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
