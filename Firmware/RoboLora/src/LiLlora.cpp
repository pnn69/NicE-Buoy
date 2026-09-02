/*
https://github.com/sandeepmistry/arduino-LoRa/blob/master/examples/LoRaDuplex/LoRaDuplex.ino
*/
#include <Arduino.h>
#include <SPI.h> // include libraries
#include <LoRa.h>
#include <WebSocketsServer.h>
#include "io.h"
#include "LiLlora.h"
#include <RoboCompute.h>
#include "sercom.h"
#include "controlwifi.h"

int counter = 0;

const int csPin = 18;    // LoRa radio chip select
const int resetPin = 23; // LoRa radio reset
const int irqPin = 26;   // change for your board; must be a hardware interrupt pin

static unsigned long lasttransmission = 0;
static unsigned long transmittReady = 0;

static RoboStruct loraMsgout = {};
static RoboStruct loraMsser = {};
RoboStruct pendingMsg[10] = {};
char buffer[128]; // or however long your messages are

String outgoing;          // outgoing message
byte msgCount = 0;        // count of outgoing messages
byte localAddress = 0xFE; // address of this device
byte destination = 0x01;  // destination to send to
QueueHandle_t loraOut;
QueueHandle_t loraIn;
QueueHandle_t loraToMain;
QueueHandle_t loraRawOutQueue;

static unsigned long buoyId = 0;
// struct loraDataType loraIn;
// struct loraDataType loraOut;

//***************************************************************************************************
//  Lora init
//***************************************************************************************************
bool InitLora(void)
{
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN);
    Serial.print("LoRa setup ");
    LoRa.setPins(RADIO_CS_PIN, RADIO_RST_PIN, RADIO_DIO0_PIN);
    if (!LoRa.begin(LoRa_frequency))
    {
        Serial.println(" failed!");
        return false;
    }
    LoRa.enableCrc();
    Serial.println(" Succes!");
    return true;
}

//***************************************************************************************************
//  initialise lora queue
//***************************************************************************************************
void initloraqueue(void)
{
    loraIn = xQueueCreate(10, sizeof(RoboStruct));
    loraOut = xQueueCreate(10, sizeof(RoboStruct));
    loraToMain = xQueueCreate(10, sizeof(RoboStruct));
    loraRawOutQueue = xQueueCreate(10, 160 * sizeof(char));
    InitLora();
    buoyId = espMac();
    Serial.print("#BuoyId=");
    Serial.println(buoyId, HEX);
}

//***************************************************************************************************
//  Store to ack buffer
//***************************************************************************************************
void storeAckMsg(RoboStruct ackBuffer)
{
    int i = 0;
    while (i < 10)
    {
        if (pendingMsg[i].cmd == 0)
        {
            memcpy(&pendingMsg[i], &ackBuffer, sizeof(ackBuffer));
            pendingMsg[i] = ackBuffer;
            // Serial.println("message stored on pos:" + String(i) + " rettrys left:" + ackBuffer.retry + " for:" + String(ackBuffer.IDr, HEX) + " msg:" + ackBuffer.cmd);
            return;
        }
        i++;
    }
}

//***************************************************************************************************
//  Remove to ack buffer
//***************************************************************************************************
void removeAckMsg(RoboStruct ackBuffer)
{
    // Serial.println("looking for msg:" + String(ackBuffer.cmd) + " Id:" + String(ackBuffer.IDs, HEX));
    int i = 0;
    while (i < 10)
    {
        // Serial.println("Found:" + String(pendingMsg[i].cmd) + " Id:" + String(pendingMsg[i].IDr, HEX));
        if (pendingMsg[i].cmd == ackBuffer.cmd && pendingMsg[i].IDr == ackBuffer.IDs)
        {
            pendingMsg[i].ack = 0;
            pendingMsg[i].cmd = 0;
            pendingMsg[i].IDs = 0;
            pendingMsg[i].IDr = 0;
            pendingMsg[i].retry = 0;
            return;
        }
        i++;
    }
}

//***************************************************************************************************
//  check ack buffer
//  remove data if more than n times has failed
//***************************************************************************************************
RoboStruct chkAckMsg(void)
{
    RoboStruct in;
    in.ack = 0;
    in.cmd = 0;
    int i = 0;
    while (i < 10)
    {
        if (pendingMsg[i].cmd != 0)
        {
            in = pendingMsg[i];
            pendingMsg[i].retry--;
            if (pendingMsg[i].retry == 0)
            {
                pendingMsg[i].cmd = 0;
                pendingMsg[i].ack = 0;
                pendingMsg[i].IDs = 0;
                pendingMsg[i].IDr = 0;
            }
            return in;
        }
        i++;
    }
    return in;
}

//***************************************************************************************************
//  Receive and decode incoming lora message
//***************************************************************************************************
// Forward declaration to resolve scope order for the repeater, as in RoboTop.
bool sendLora(String loraTransmitt);
static void maybeRepeat(const String &incoming, const RoboStruct &in);

//***************************************************************************************************
//  Transparent repeater
//***************************************************************************************************
// Ported from RoboTop, where the design has already been through its mistakes - see the comments
// there. The short version:
//
//   dedup cache      the same frame is not relayed twice inside REPEATER_TIMEOUT_MS
//   staggered slots  the delay comes from THIS board's own MAC, so two relays are never due in the
//                    same millisecond. A synchronous relay had every node transmit at once, and a
//                    node is deaf while it transmits, so the relays collided and each node also
//                    missed whatever else was on the air.
//   cancel on heard  a pending relay is dropped when that exact frame is heard again, so in
//                    practice only the earliest node in range actually transmits it
//   unicast only     broadcasts are not relayed, except LORA_LINK. Relaying telemetry buys nothing
//                    and costs the channel dearly; a link report's whole purpose is to arrive from
//                    a node you cannot hear directly.
//
// Why this gateway wants one: it is the strongest transmitter measured on this network, and it is
// mains-portable in a way the buoys are not. Standing it between a weak handheld and the buoys
// turns a marginal one-shot command into two strong hops.
#define REPEATER_CACHE_SIZE 16
#define REPEATER_TIMEOUT_MS 20000UL
#define REPEAT_SLOTS 4
#define REPEAT_NODE_SLOTS 5
#define REPEAT_SLOT_MS 250
#define REPEAT_BASE_MS 40
#define REPEAT_JITTER_MS 60
#define REPEAT_EXPIRY_MS 3000

struct RepeaterCacheEntry { String message = ""; unsigned long last_repeated_ms = 0; };
static RepeaterCacheEntry repeaterCache[REPEATER_CACHE_SIZE];
static int repeaterCacheIndex = 0;

struct RepeatSlot {
    String message = "";
    unsigned long due_ms = 0;
    bool busy = false;
    unsigned long target_id = 0;
    int cmd = 0;
};
static RepeatSlot repeatSlots[REPEAT_SLOTS];

static bool checkAndRecordRepeaterMessage(const String &msg)
{
    unsigned long now = millis();
    for (int i = 0; i < REPEATER_CACHE_SIZE; i++)
    {
        if (repeaterCache[i].message == msg)
        {
            if (now - repeaterCache[i].last_repeated_ms < REPEATER_TIMEOUT_MS) return false;
            repeaterCache[i].last_repeated_ms = now;
            return true;
        }
    }
    repeaterCache[repeaterCacheIndex].message = msg;
    repeaterCache[repeaterCacheIndex].last_repeated_ms = now;
    repeaterCacheIndex = (repeaterCacheIndex + 1) % REPEATER_CACHE_SIZE;
    return true;
}

// This board's own slot, folded from its MAC: stable, needs no configuration, and cannot collide
// with another node unless the two fold to the same value.
static unsigned long repeatDelayMs(void)
{
    uint32_t id = (uint32_t)espMac();
    id ^= id >> 16;
    id ^= id >> 8;
    int slot = (int)(id % REPEAT_NODE_SLOTS);
    return REPEAT_BASE_MS + (unsigned long)slot * REPEAT_SLOT_MS + random(0, REPEAT_JITTER_MS);
}

static void queueRepeat(const String &msg, unsigned long target_id, int cmd)
{
    for (int i = 0; i < REPEAT_SLOTS; i++)
    {
        if (!repeatSlots[i].busy)
        {
            repeatSlots[i].message = msg;
            repeatSlots[i].due_ms = millis() + repeatDelayMs();
            repeatSlots[i].target_id = target_id;
            repeatSlots[i].cmd = cmd;
            repeatSlots[i].busy = true;
            return;
        }
    }
    Serial.println("#LoRa Repeat: slots full, frame not relayed");
}

// Somebody beat us to it, or the sender resent it - either way ours is no longer needed.
static void cancelRepeat(const String &msg)
{
    for (int i = 0; i < REPEAT_SLOTS; i++)
        if (repeatSlots[i].busy && repeatSlots[i].message == msg)
        {
            repeatSlots[i].message = "";
            repeatSlots[i].busy = false;
            return;
        }
}

// The node it was meant for has answered, so relaying it now would only add noise.
static void cancelRepeatOnAck(const RoboStruct *in)
{
    for (int i = 0; i < REPEAT_SLOTS; i++)
        if (repeatSlots[i].busy && repeatSlots[i].cmd == in->cmd && repeatSlots[i].target_id == in->IDs)
        {
            repeatSlots[i].message = "";
            repeatSlots[i].busy = false;
            return;
        }
}

static void drainRepeats(void)
{
    unsigned long now = millis();
    for (int i = 0; i < REPEAT_SLOTS; i++)
    {
        if (repeatSlots[i].busy && (long)(now - repeatSlots[i].due_ms) >= 0)
        {
            if (sendLora(repeatSlots[i].message))
            {
                Serial.println("#LoRa Repeat: forwarding <" + repeatSlots[i].message + ">");
                repeatSlots[i].message = "";
                repeatSlots[i].busy = false;
            }
            else if ((long)(now - repeatSlots[i].due_ms) > REPEAT_EXPIRY_MS)
            {
                Serial.println("#LoRa Repeat: expired, dropping relay");
                repeatSlots[i].message = "";
                repeatSlots[i].busy = false;
            }
            return;
        }
    }
}

//***************************************************************************************************
//  LoRa link quality
//***************************************************************************************************
// Same measurement the Tops and the handheld make - see LORA_LINK in RoboCompute.h. Having the
// gateway report too is what makes a transmitter comparison possible: put this and the handheld in
// the same place and the Tops' readings of the two say which radio is the weaker, which no single
// node can tell you about itself.
#define LINK_PEERS_TRACKED 12
#define LINK_REPORT_MS 60000UL
#define LINK_FORGET_MS 300000UL
#define LINK_DEDUP_MS 1200UL
#define LINK_DEDUP_SLOTS 8

struct LinkPeer
{
    uint32_t id = 0;
    long rssiSum = 0;
    int16_t rssiWorst = 0;
    uint16_t count = 0;
    int16_t lastRssi = 0;
    unsigned long lastHeard = 0;
    bool used = false;
};

static LinkPeer linkPeer[LINK_PEERS_TRACKED];
static String linkSeen[LINK_DEDUP_SLOTS];
static unsigned long linkSeenAt[LINK_DEDUP_SLOTS] = {0};
static int linkSeenIdx = 0;
static unsigned long linkReportDue = 0;

// A relayed frame carries the original sender's id but arrives at the relay's signal strength, so
// only the first copy of a frame is measured. See the same guard on the Top.
static bool linkFirstCopy(const String &msg)
{
    unsigned long now = millis();
    for (int i = 0; i < LINK_DEDUP_SLOTS; i++)
        if (linkSeen[i] == msg && (now - linkSeenAt[i]) < LINK_DEDUP_MS) return false;
    linkSeen[linkSeenIdx] = msg;
    linkSeenAt[linkSeenIdx] = now;
    linkSeenIdx = (linkSeenIdx + 1) % LINK_DEDUP_SLOTS;
    return true;
}

static void linkNote(uint32_t id, int rssi)
{
    if (id == 0) return;
    int free_slot = -1;
    for (int i = 0; i < LINK_PEERS_TRACKED; i++)
    {
        if (linkPeer[i].used && linkPeer[i].id == id)
        {
            linkPeer[i].rssiSum += rssi;
            if (rssi < linkPeer[i].rssiWorst) linkPeer[i].rssiWorst = (int16_t)rssi;
            if (linkPeer[i].count < 65535) linkPeer[i].count++;
            linkPeer[i].lastRssi = (int16_t)rssi;
            linkPeer[i].lastHeard = millis();
            return;
        }
        if (!linkPeer[i].used && free_slot < 0) free_slot = i;
    }
    if (free_slot < 0) return;
    linkPeer[free_slot].used = true;
    linkPeer[free_slot].id = id;
    linkPeer[free_slot].rssiSum = rssi;
    linkPeer[free_slot].rssiWorst = (int16_t)rssi;
    linkPeer[free_slot].count = 1;
    linkPeer[free_slot].lastRssi = (int16_t)rssi;
    linkPeer[free_slot].lastHeard = millis();
}

static bool linkAlive(const LinkPeer &p)
{
    return p.used && p.lastHeard && (millis() - p.lastHeard) < LINK_FORGET_MS;
}

static int linkMean(const LinkPeer &p)
{
    return p.count ? (int)(p.rssiSum / (long)p.count) : (int)p.lastRssi;
}

// Relay a frame addressed to somebody else, in case that somebody is out of the sender's range but
// in ours. Called at the end of onReceive().
static void maybeRepeat(const String &incoming, const RoboStruct &in)
{
    bool is_from_me = (in.IDs == espMac());
    bool is_addressed_to_me = (in.IDr == buoyId || in.IDr == 0x99);
    bool is_broadcast = (in.IDr == BUOYIDALL || in.IDr == 0);
    bool relay_worthy = !is_broadcast || in.cmd == LORA_LINK;
    if (is_from_me || is_addressed_to_me || !relay_worthy) return;
    if (checkAndRecordRepeaterMessage(incoming)) queueRepeat(incoming, in.IDr, in.cmd);
}

void linkReportService(void)
{
    if (linkReportDue == 0) linkReportDue = millis() + LINK_REPORT_MS;
    if ((long)(millis() - linkReportDue) < 0) return;
    linkReportDue = millis() + LINK_REPORT_MS;

    int order[LINK_PEERS_TRACKED];
    int n = 0;
    for (int i = 0; i < LINK_PEERS_TRACKED; i++)
        if (linkAlive(linkPeer[i])) order[n++] = i;

    RoboStruct msg = {};
    msg.cmd = LORA_LINK;
    msg.ack = INF;
    msg.IDr = BUOYIDALL;
    msg.IDs = espMac();

    // Worst first: if anything has to be left out it should be the healthy links.
    for (int a = 0; a < n - 1; a++)
        for (int b = a + 1; b < n; b++)
            if (linkMean(linkPeer[order[b]]) < linkMean(linkPeer[order[a]]))
            {
                int t = order[a]; order[a] = order[b]; order[b] = t;
            }

    int put = 0;
    for (; put < n && put < LORA_LINK_MAX_PEERS; put++)
    {
        const LinkPeer &p = linkPeer[order[put]];
        msg.linkPeerId[put] = p.id;
        msg.linkRssi[put] = (int16_t)linkMean(p);
        msg.linkCount[put] = p.count;
    }
    msg.linkPeers = (uint8_t)put;
    msg.linkOmitted = (uint8_t)(n > put ? n - put : 0);

    // Goes out even with nothing to report: the frame is also the only thing that lets the Tops
    // measure THIS node, and "heard nobody" is itself worth transmitting.
    xQueueSend(loraOut, (void *)&msg, 0);

    for (int i = 0; i < LINK_PEERS_TRACKED; i++)
    {
        linkPeer[i].rssiSum = 0;
        linkPeer[i].rssiWorst = 0;
        linkPeer[i].count = 0;
    }
}

void onReceive(int packetSize)
{
    if (packetSize == 0)
        return;                        // if there's no packet, return
    byte incomingLength = LoRa.read(); // incoming msg length
    String incoming = "";
    digitalWrite(LED_PIN, HIGH); // turn on led
    while (LoRa.available())
    {
        incoming += (char)LoRa.read();
    }
    digitalWrite(LED_PIN, LOW); // turn off led
    if (incomingLength != incoming.length())
    { // check length for error
        Serial.println("#error: message length does not match length");
        return; // skip rest of function
    }
    Serial.println(incoming);
    
    // Broadcast incoming LoRa telemetry safely and thread-safe to WebSockets
    extern QueueHandle_t wsOutQueue;
    if (wsOutQueue != NULL) {
        char packetBuf[160];
        memset(packetBuf, 0, sizeof(packetBuf));
        int rssi = LoRa.packetRssi();
        sprintf(packetBuf, "LORA:%d:", rssi);
        int prefixLen = strlen(packetBuf);
        int len = incoming.length();
        if (len > (159 - prefixLen)) {
            len = 159 - prefixLen; // Leave room for null terminator
        }
        for (int i = 0; i < len; i++) {
            packetBuf[prefixLen + i] = incoming[i];
        }
        packetBuf[prefixLen + len] = '\0';
        xQueueSend(wsOutQueue, (void *)packetBuf, 10);
    }

    RoboStruct in = {};
    rfDeCode(incoming, &in);
    in.loralstmsg = LoRa.packetRssi();

    // Link accounting, before any of the addressing tests below can drop the frame - hearing
    // somebody else's traffic measures the path just as well as hearing our own.
    if (in.IDs != espMac() && linkFirstCopy(incoming))
        linkNote((uint32_t)in.IDs, in.loralstmsg);

    // If we were about to relay this exact frame we no longer need to - somebody beat us to it, or
    // the sender resent it. Ahead of the addressing tests so it applies to frames meant for us too.
    cancelRepeat(incoming);
    if (in.ack == ACK) cancelRepeatOnAck(&in);
    if ((in.IDr == buoyId || in.IDr == 0x99) && in.ack == ACK) // A message form me so check if its a ACK message
    {
        removeAckMsg(in);
        // printf("#Lora Ack received buffer cleared\r\n");
        return;
    }
    if (in.IDr == buoyId || in.IDr == BUOYIDALL || in.IDr == 0x99) // A message form me so check if its a ACK message
    {
        if (in.ack == GETACK) // on ack request send ack back
        {
            // IDr,IDs,ACK,MSG
            loraMsgout.IDr = in.IDs;
            loraMsgout.IDs = buoyId;
            loraMsgout.cmd = in.cmd;
            loraMsgout.ack = ACK;
            xQueueSend(loraOut, (void *)&loraMsgout, 10); // send ACK out
        }
    }
    xQueueSend(loraToMain, (void *)&in, 10); // send to main

    // Last, so relaying can never delay handling a frame that was for us.
    maybeRepeat(incoming, in);
}

//***************************************************************************************************
//  Lora data out: Handles low-level transmission of formatted telemetry payloads.
//  Implements safety margins, LED indicators, and explicit end-of-packet validation.
//***************************************************************************************************
bool sendLora(String loraTransmitt)
{
    // Ensure the required transmission cooldown delay has elapsed
    if (transmittReady < millis())
    {
        if (LoRa.beginPacket()) // Initialize the hardware packet buffer
        {
            digitalWrite(LED_PIN, HIGH); // Turn on transmission indicator LED
            LoRa.write(loraTransmitt.length());
            LoRa.print(loraTransmitt);
            
            // Validate packet transmission success. LoRa.endPacket() returns 1 if packet sent successfully.
            if (LoRa.endPacket() == 1)
            {
                digitalWrite(LED_PIN, LOW); // Turn off transmission indicator LED
                Serial.printf("Lora sent: %s\r\n", loraTransmitt.c_str());
                // Enforce a brief post-transmission silence window (10ms) to allow channel recovery
                transmittReady = millis() + 10;
                return true; 
            }
            else
            {
                digitalWrite(LED_PIN, LOW); // Safeguard indicator LED state
                Serial.println("#Error: LoRa.endPacket() failed during transmission.");
            }
        }
        else
        {
            Serial.println("#Error: LoRa.beginPacket() failed.");
        }
    }
    return false;
}

//***************************************************************************************************
//  Lora task: Orchestrates main outgoing queues, automated retries, and self-healing.
//  Runs pinned to its own core to guarantee low-latency RF performance.
//***************************************************************************************************
void LoraTask(void *arg)
{
    unsigned long retransmittReady = 0;
    delay(500);
    while (1)
    {
        // Continuously poll the LoRa FIFO buffer to process incoming packets
        onReceive(LoRa.parsePacket());

        // Put out any relay whose backoff has expired - see queueRepeat().
        drainRepeats();

        // One link report a minute - see linkReportService().
        linkReportService();

        // Process any outgoing telemetry messages queued by the main firmware loop
        static RoboStruct txMsg;
        if (xQueueReceive(loraOut, (void *)&txMsg, 1) == pdTRUE)
        {
            // Preserve webpage (0x99) and screen (0x98) client identities so the Top buoy allows saving local docking setup parameters
            if (txMsg.IDs != 0x99 && txMsg.IDs != 0x98) {
                txMsg.IDs = espMac(); // Attach the local device MAC address as sender ID
            }
            String loraString = rfCode(&txMsg);

            int attempts = 0;
            const int maxAttempts = 3;
            bool sent = false;
            
            // Retry Loop: Attempt to transmit up to 3 times in case of congestion or collision
            while (attempts < maxAttempts)
            {
                if (sendLora(String(loraString)))
                {
                    sent = true;
                    break;
                }
                attempts++;
                // Wait 150ms before retrying, during which we keep polling to avoid RX FIFO overflows
                for (int d = 0; d < 15; d++) 
                {
                    onReceive(LoRa.parsePacket()); 
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }

            // Lock Recovery / Self-Healing: If 3 transmission attempts failed, the SPI LoRa module
            // may have entered an unstable lock state. Trigger a hardware-level reinitialization.
            if (!sent)
            {
                Serial.println("#Error: Failed to transmit LoRa packet after 3 attempts. Transceiver may be locked up.");
                Serial.println("#Attempting LoRa transceiver self-healing...");
                if (InitLora())
                {
                    Serial.println("#LoRa self-healing successful!");
                }
                else
                {
                    Serial.println("#LoRa self-healing failed!");
                }
            }

            Serial.println(loraString);
            // If the message requests a remote receipt confirmation (GETACK/SET), store it for tracking
            if (txMsg.ack == GETACK || txMsg.ack == 3) // 3 is Python's GETACK equivalence
            {
                txMsg.retry = 5;
                storeAckMsg(txMsg);                                 // Insert into pending acknowledgements list
                retransmittReady = millis() + 500 + random(0, 150); // Schedule retry check with random jitter
            }
        }

        /* Retransmit any pending unacknowledged packets */
        if (retransmittReady < millis())
        {
            static RoboStruct retryMsg;
            retryMsg = chkAckMsg();
            if (retryMsg.cmd != 0)
            {
                String loraString = rfCode(&retryMsg);
                int attempts = 0;
                const int maxAttempts = 3;
                bool sent = false;
                
                // Retry Transmission loop for pending acknowledgements
                while (attempts < maxAttempts)
                {
                    if (sendLora(loraString))
                    {
                        sent = true;
                        break;
                    }
                    attempts++;
                    // Short wait with continuous FIFO polling
                    for (int d = 0; d < 5; d++) 
                    {
                        onReceive(LoRa.parsePacket()); 
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                }

                if (!sent)
                {
                    Serial.println("#Error: Failed to retransmit ACK-pending LoRa packet. Transceiver may be locked up.");
                    Serial.println("#Attempting LoRa transceiver self-healing...");
                    if (InitLora())
                    {
                        Serial.println("#LoRa self-healing successful!");
                    }
                    else
                    {
                        Serial.println("#LoRa self-healing failed!");
                    }
                }
                retransmittReady = millis() + 900 + random(0, 150); // Reschedule retry check
            }
            else
            {
                // No pending messages to retry, idle check schedule
                retransmittReady = millis() + 500;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // Critical: yield CPU core control to prevent watchdog timeout triggers
    }
}
