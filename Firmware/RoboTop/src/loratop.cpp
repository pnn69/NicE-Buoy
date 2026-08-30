#include <Arduino.h>
#include <LoRa.h>
#include "loratop.h"
#include "io_top.h"
#include "main.h"
#include "topwifi.h"

// Forward declaration to resolve scope order for repeater
bool sendLora(String loraTransmitt);

static const char *TAG = "lora.cpp";
static bool loraOk = false;
static unsigned long my_id = 0;
static char loraData[MAXSTRINGLENG];
static RoboStruct loraMsgout = {};
static RoboStruct loraMsgin = {};
static RoboStruct pendingMsg[10] = {};
static RoboStruct loraStruct;
static unsigned long transmittReady = 0;
QueueHandle_t loraOut;
QueueHandle_t loraIn;
static unsigned long buoyId = 0;
static RoboStruct *pMainData = NULL;

//***************************************************************************************************
//  Lora init
//***************************************************************************************************
/**
 * @brief Initializes the LoRa radio module via SPI.
 * 
 * Configures the pins and frequency, and enables CRC.
 * 
 * @return true if initialization is successful, false otherwise.
 */
bool InitLora(void)
{

    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    // LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN); // only polling mode so no irq needed
    if (!LoRa.begin(LoRa_frequency))
    {
        Serial.println("LoRa setup Failed!");
        return false;
    }
    LoRa.enableCrc();
    buoyId = espMac();
    return true;
}

//***************************************************************************************************
//  initialise lora queue
//***************************************************************************************************
/**
 * @brief Initializes the FreeRTOS queues for LoRa Rx/Tx and calls InitLora.
 */
void initloraqueue(void)
{
    loraIn = xQueueCreate(10, sizeof(RoboStruct));
    loraOut = xQueueCreate(10, sizeof(RoboStruct));
    InitLora();
    Serial.print("#BuoyId=");
    Serial.println(espMac(), HEX);
}

//***************************************************************************************************
//  Store to ack buffer
//***************************************************************************************************
/**
 * @brief Tells whether a command changes the buoy's operating mode.
 *
 * Only one mode change can be outstanding per target: see storeAckMsg().
 *
 * @param cmd The command to classify.
 * @return true for commands that put the buoy into a different operating state.
 */
static bool isModeCmd(int cmd)
{
    switch (cmd)
    {
    case IDLE:
    case UNLOCK:
    case REMOTE:
    case LOCKING:
    case LOCKED:
    case LOCKPOS:
    case SETLOCKPOS:
    case DOCKING:
    case DOCKED:
    case DOCKPOS:
    case SETDOCKPOS:
        return true;
    default:
        return false;
    }
}

/**
 * @brief Stores a message in the pending queue awaiting an acknowledgment.
 *
 * Enforces one live entry per (target, semantic class) before inserting, because the
 * frames carry no sequence number: a receiver cannot tell a retransmit from a fresh
 * command, so it is always last-heard-wins. Without this, pressing LOCK and then IDLE
 * within the ~7 s retry window left the older LOCK entry in a lower slot, and chkAckMsg()
 * (which always returns the lowest occupied slot) kept re-sending it - dragging the buoy
 * back into the mode the operator had just left.
 *
 * Two supersede rules, both scoped to the same recipient so the SENDTRACK fan-out keeps
 * one independent entry per buoy:
 *   - same cmd  -> replace, so idempotent parameter sets stop accumulating slots;
 *   - mode vs mode -> the newer command evicts the older one.
 *
 * @param ackBuffer The message structure to store.
 */
void storeAckMsg(RoboStruct ackBuffer)
{
    bool newIsMode = isModeCmd(ackBuffer.cmd);
    int i = 0;

    /* Purge everything this message supersedes for the same target */
    while (i < 10)
    {
        if (pendingMsg[i].cmd != 0 && pendingMsg[i].IDr == ackBuffer.IDr &&
            (pendingMsg[i].cmd == ackBuffer.cmd || (newIsMode && isModeCmd(pendingMsg[i].cmd))))
        {
            // Serial.println("superseded pending msg:" + String(pendingMsg[i].cmd) + " on pos:" + String(i) + " by msg:" + String(ackBuffer.cmd));
            pendingMsg[i].ack = 0;
            pendingMsg[i].cmd = 0;
            pendingMsg[i].IDs = 0;
            pendingMsg[i].IDr = 0;
            pendingMsg[i].retry = 0;
        }
        i++;
    }

    i = 0;
    while (i < 10)
    {
        if (pendingMsg[i].cmd == 0)
        {
            memcpy(&pendingMsg[i], &ackBuffer, sizeof(ackBuffer));
            // Serial.println("message stored on pos:" + String(i) + " rettrys left:" + ackBuffer.retry + " for:" + String(ackBuffer.IDr, HEX) + " msg:" + ackBuffer.cmd);
            return;
        }
        i++;
    }
    Serial.println("#Error: LoRa ack table full, msg:" + String(ackBuffer.cmd) + " for:" + String(ackBuffer.IDr, HEX) + " dropped");
}

//***************************************************************************************************
//  Remove to ack buffer
//***************************************************************************************************
/**
 * @brief Removes a message from the pending queue when its ACK is received.
 * 
 * @param ackBuffer The acknowledgment message containing the matching cmd and sender ID.
 */
void removeAckMsg(RoboStruct ackBuffer)
{
    // Serial.println("looking for msg:" + String(ackBuffer.cmd) + "Id:" + String(ackBuffer.IDs, HEX));
    // printf("looking for msg%d of id %lld\r\n", ackBuffer.cmd, ackBuffer.macIDs);
    int i = 0;
    while (i < 10)
    {
        // Serial.println("ackBuffer.macIDr: " + String(ackBuffer.macIDs,HEX) +" pendingMsg[i].macIDs: " + String(pendingMsg[i].macIDr,HEX));
        if (pendingMsg[i].cmd == ackBuffer.cmd && (pendingMsg[i].IDr == ackBuffer.IDs || (pMainData && pendingMsg[i].IDr == pMainData->IDs)))
        {
            // Serial.println("message removed pos:" + String(i) + " rettrys left:" + pendingMsg[i].retry + " for:" + String(pendingMsg[i].IDr, HEX) + " msg:" + pendingMsg[i].cmd);
            pendingMsg[i].ack = 0;
            pendingMsg[i].cmd = 0;
            pendingMsg[i].IDs = 0;
            pendingMsg[i].IDr = 0;
            pendingMsg[i].retry = 0;
            //            printf("message removed for ack on pos:%d\r\n", i);
            return;
        }
        i++;
    }
}

//***************************************************************************************************
//  check ack buffer
//  remove data if more than n times has failed
//***************************************************************************************************
/**
 * @brief Checks the pending message queue for timeouts and decrements retries.
 * 
 * @return A RoboStruct containing a message that needs retransmission, or an empty
 * struct if nothing needs retransmitting.
 */
RoboStruct chkAckMsg()
{
    RoboStruct in;
    in.ack = 0;
    in.cmd = 0;
    int i = 0;
    while (i < 10)
    {
        if (pendingMsg[i].cmd != 0)
        {
            memcpy(&in, &pendingMsg[i], sizeof(RoboStruct));
            pendingMsg[i].retry--;
            if (pendingMsg[i].retry == 0)
            {
                pendingMsg[i].cmd = 0;
                pendingMsg[i].ack = 0;
                pendingMsg[i].IDs = 0;
                pendingMsg[i].IDr = 0;
            }
            // printf("message ack on pos:%d rettrys left %d to:%lld  msg:%d\r\n", i, pendingMsg[i].retry, pendingMsg[i].macIDr, pendingMsg[i].cmd);
            // Serial.println("message ack on pos:" + String(i) + " rettrys left:" + pendingMsg[i].retry + " for:" + String(pendingMsg[i].IDr, HEX) + " msg:" + pendingMsg[i].cmd);
            return in;
        }
        i++;
    }
    return in;
}

//***************************************************************************************************
//  Remove white space
//***************************************************************************************************
/**
 * @brief Removes all whitespace characters from a given String.
 * 
 * @param str The input string.
 * @return The string without any whitespace characters.
 */
String removeWhitespace(String str)
{
    String result = "";
    for (int i = 0; i < str.length(); i++)
    {
        if (str.charAt(i) != ' ')
        {
            result += str.charAt(i);
        }
    }
    return result;
}

//***************************************************************************************************
//  Receive and decode incoming lora message
//***************************************************************************************************
#define REPEATER_CACHE_SIZE 16
#define REPEATER_TIMEOUT_MS 20000UL // 20-second timeout

struct RepeaterCacheEntry {
    String message = "";
    unsigned long last_repeated_ms = 0;
};

static RepeaterCacheEntry repeaterCache[REPEATER_CACHE_SIZE];
static int repeaterCacheIndex = 0;

static bool checkAndRecordRepeaterMessage(const String &msg) {
    unsigned long now = millis();
    int foundIndex = -1;
    
    // Search the cache for an existing match
    for (int i = 0; i < REPEATER_CACHE_SIZE; i++) {
        if (repeaterCache[i].message == msg) {
            foundIndex = i;
            break;
        }
    }
    
    if (foundIndex != -1) {
        // Match found! Check if the 20-second window has expired
        if (now - repeaterCache[foundIndex].last_repeated_ms < REPEATER_TIMEOUT_MS) {
            // Within 20 seconds: DO NOT repeat
            return false;
        } else {
            // More than 20 seconds: allowed to repeat again. Update timestamp.
            repeaterCache[foundIndex].last_repeated_ms = now;
            return true;
        }
    } else {
        // No match found in cache. Add to circular buffer.
        repeaterCache[repeaterCacheIndex].message = msg;
        repeaterCache[repeaterCacheIndex].last_repeated_ms = now;
        
        // Advance circular index
        repeaterCacheIndex = (repeaterCacheIndex + 1) % REPEATER_CACHE_SIZE;
        return true;
    }
}

/**
 * @brief Parses an incoming LoRa packet and routes it to the correct queues.
 * 
 * @param packetSize The size of the received LoRa packet.
 */
void onReceive(int packetSize)
{
    if (packetSize == 0)
        return; // if there's no packet, return
    // read packet header bytes:
    byte incomingLength = LoRa.read(); // incoming msg length
    String incoming = "";
    while (LoRa.available())
    {
        incoming += (char)LoRa.read();
    }

    if (incomingLength != incoming.length())
    { // check length for error
        Serial.println("#error: message length does not match length");
        return; // skip rest of function
    }
    
    RoboStruct in;
    rfDeCode(incoming,&in);

    // Recognise either our physical MAC or our logical Sub-synced ID
    bool is_addressed_to_me = (in.IDr == buoyId || (pMainData && in.IDr == pMainData->IDs));

    if (is_addressed_to_me && in.ack == ACK) // A message form me so check if its a ACK message
    {
        removeAckMsg(in);
        // printf("#Lora Ack received buffer cleared\r\n");
        return;
    }
    if (is_addressed_to_me || in.IDr == BUOYIDALL || in.IDr == 0) // A message form
    {
        if (is_addressed_to_me) {
            removeAckMsg(in);
        }
        // DO NOT send an immediate zeroed ACK for GETACK. 
        // Let the main loop handle the fetch from Sub and send the real data.
        Serial.println("#Lora_i <" + incoming + ">");
        xQueueSend(loraIn, (void *)&in, 10); // send to main
    }

    // Transparent Repeater: Repeat if the message was not sent by us,
    // and is not addressed specifically to us (meaning it's either for someone else, or a broadcast!)
    bool is_from_me = (in.IDs == buoyId || (pMainData && in.IDs == pMainData->IDs));
    if (!is_from_me && !is_addressed_to_me)
    {
        if (checkAndRecordRepeaterMessage(incoming))
        {
            Serial.println("#LoRa Repeat: Forwarding <" + incoming + ">");
            sendLora(incoming);
        }
    }
}

//***************************************************************************************************
//  Lora data out
//***************************************************************************************************
/**
 * @brief Transmits a string over LoRa.
 * 
 * @param loraTransmitt The string data to send.
 * @return true if successfully pushed to the LoRa module, false otherwise.
 */
bool sendLora(String loraTransmitt)
{
    if (transmittReady < millis())
    {
        if (LoRa.beginPacket()) // start packet
        {
            LoRa.write(loraTransmitt.length());
            LoRa.print(loraTransmitt);
            LoRa.endPacket(); // finish packet and send it
            // Serial.println("#Lora_o <" + loraTransmitt + ">");
            transmittReady = millis() + 10;
            return true;
        }
    }
    return false;
}

//***************************************************************************************************
//  Lora task
//***************************************************************************************************
/**
 * @brief FreeRTOS task handling all LoRa communication.
 * 
 * Continuously polls for incoming packets, checks the out queue for new messages,
 * handles transmission, and manages the retransmission logic for unacknowledged messages.
 * 
 * @param arg Task arguments (Pointer to mainData RoboStruct).
 */
#include "udplog.h"

void LoraTask(void *arg)
{
    pMainData = (RoboStruct *)arg;
    unsigned long retransmittReady = 0;
    while (1)
    {
        crumbAt(CRUMB_LORA, 200);
        // Continuously poll for any incoming LoRa packets to prevent RX FIFO buffer overflow
        onReceive(LoRa.parsePacket()); 
        crumbAt(CRUMB_LORA, 201);
        
        // Process any telemetry messages queued for LoRa transmission
        if (xQueueReceive(loraOut, (void *)&loraMsgout, 1) == pdTRUE)
        {
            // Set the sender ID to our actual hardware MAC address if not already specified
            if (loraMsgout.IDs == 0) loraMsgout.IDs = (pMainData && pMainData->IDs != 0) ? pMainData->IDs : buoyId;
            
            // Adjust tgDist to be the distance to the actual radius (delta)
            if (loraMsgout.cmd == TOPDATA || loraMsgout.cmd == DIRDIST || loraMsgout.cmd == LOCKED || loraMsgout.cmd == DOCKED) {
                if (loraMsgout.status == LOCKED || loraMsgout.status == LOCKING || loraMsgout.status == DOCKED || loraMsgout.status == DOCKING) {
                    loraMsgout.tgDist -= loraMsgout.holdRad;
                }
            }
            
            crumbAt(CRUMB_LORA, 202);
            String loraString = rfCode(&loraMsgout);
            int retries = 0;
            crumbAt(CRUMB_LORA, 203);
            
            // Non-blocking wait and send loop:
            // Continues to listen to the radio interface while backing off to avoid missing inbound packets.
            while (sendLora(String(loraString)) != true)
            {
                crumbAt(CRUMB_LORA, 204);
                onReceive(LoRa.parsePacket()); // Keep receiving while waiting to transmit!
                vTaskDelay(pdMS_TO_TICKS(10)); // Shorter sleep (10ms instead of 50ms) to increase system responsiveness
                retries++;
                
                // Hardware Self-Healing: If transmission fails consistently for 500ms (50 * 10ms),
                // the radio transceiver may have locked up (SPI glitch or state machine stall).
                // Force a full hardware-level reset and reinitialization of the LoRa module.
                if (retries >= 50)             
                {
                    Serial.println("#Error: LoRa send failed. Hardware-resetting LoRa module...");
                    InitLora();
                    break;
                }
            }
            // If the transmission expects a receipt confirmation (GETACK/SET), store it in our retry table
            crumbAt(CRUMB_LORA, 205);
            if (loraMsgout.ack == GETACK || loraMsgout.ack == SET)
            {
                crumbAt(CRUMB_LORA, 206);
                loraMsgout.retry = 5;
                storeAckMsg(loraMsgout);                            // put data in buffer (will be removed on ack)
                retransmittReady = millis() + 900 + random(0, 150); // give some time for ack
            }
        }
        
        /* Retransmit any pending unacknowledged packets */
        if (retransmittReady < millis())
        {
            loraMsgout = chkAckMsg();
            if (loraMsgout.cmd != 0)
            {
                String loraString = rfCode(&loraMsgout);
                int retries = 0;
                
                // Retransmission loop with non-blocking packet polling and self-healing support
                while (sendLora(loraString) != true)
                {
                    onReceive(LoRa.parsePacket()); // Keep receiving while waiting to transmit!
                    vTaskDelay(pdMS_TO_TICKS(10)); // Shorter sleep (10ms instead of 50ms)
                    retries++;
                    if (retries >= 50)             // 50 * 10ms = 500ms timeout
                    {
                        Serial.println("#Error: LoRa retry failed. Hardware-resetting LoRa module...");
                        InitLora();
                        break;
                    }
                }
                retransmittReady = millis() + 1500 + random(0, 150);
            }
        }
        vTaskDelay(1); // Crucial task yield to avoid triggering watchdog timers on this CPU core
    }
}
