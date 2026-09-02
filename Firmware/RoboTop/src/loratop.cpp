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
    // Round-robin, not "lowest occupied slot".
    //
    // This returns ONE entry per call and the caller only calls it every ~1.5 s, so always
    // starting the scan at 0 meant slot 0 consumed all five of its retransmits before slot 1 was
    // ever looked at - 7.5 s of the channel to itself. A COMPUTE STARTLINE files one SETLOCKPOS
    // per buoy, so the SECOND buoy's waypoint got a single attempt and then nothing for seven
    // seconds. Whichever buoy happened to land in the higher slot was the one that never sailed.
    static int scanStart = 0;
    for (int n = 0; n < 10; n++)
    {
        int i = (scanStart + n) % 10;
        if (pendingMsg[i].cmd != 0)
        {
            scanStart = (i + 1) % 10;
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

//***************************************************************************************************
//  Deferred repeat slots
//***************************************************************************************************
// A relay is queued here rather than transmitted from inside onReceive().
//
// It used to be sent synchronously, the instant the frame came off the radio. Every Top that heard
// the same frame therefore relayed it in the same millisecond, so the relays collided with each
// other - and a node is deaf while it transmits, so each one also missed whatever else was on the
// air. Nothing about the old design could have worked with more than one relay in the field.
//
// Each queued relay carries a due time, and the delay is derived from OUR OWN MAC so that two
// Tops holding the same frame are never due at the same moment. Pure randomness would still
// collide now and then; a MAC-derived slot cannot, as long as the two boards fold to different
// slots. A small random jitter is added on top purely to break the tie if they do collide.
//
// The slot is wider than the longest frame we relay, so the node in slot 0 is finished before the
// node in slot 1 begins - and because a relay that has already been heard is cancelled (see
// cancelRepeat), in practice only the lowest-slot node in range actually transmits. The others
// hear it, drop their copy, and the frame reaches the far node exactly once.
//
// Slot count and width are bounded by the retry interval, not chosen freely: 4 * 250 + 40 + 60 =
// 1100 ms, comfortably inside the ~1500 ms the sender's retry table waits. See cancelRepeat().
//
// Two boards CAN fold to the same slot - with three Tops and five slots it is not unlikely. That
// is not fatal: both relay at once, the relay is lost to the collision, and the sender's next
// retransmit produces a fresh attempt with newly drawn jitter. The stagger removes the guaranteed
// collision the old synchronous relay had; it does not have to remove every possible one.
#define REPEAT_SLOTS 4          // how many relays may be waiting at once
#define REPEAT_NODE_SLOTS 5     // how many distinct per-MAC time slots
#define REPEAT_SLOT_MS 250      // width of one slot: > airtime of any frame we relay
#define REPEAT_BASE_MS 40       // dead time before slot 0, lets the original sender's tail clear
#define REPEAT_JITTER_MS 60     // tie-break when two MACs fold to the same slot
#define REPEAT_EXPIRY_MS 3000   // give up on a relay the radio will not accept

struct RepeatSlot {
    String message = "";
    unsigned long due_ms = 0;
    bool busy = false;
    unsigned long target_id = 0;  // who the frame was addressed to
    int cmd = 0;                  // what it was asking for
};

static RepeatSlot repeatSlots[REPEAT_SLOTS];

/**
 * @brief Schedules a frame for relay after a randomised backoff.
 *
 * @param msg The verbatim frame to put back on the air.
 */
/**
 * @brief How long THIS board waits before relaying, from its own MAC.
 *
 * buoyId is espMac(); its low bytes differ between any two boards, so folding it down gives a
 * stable per-node slot with nothing to configure and no coordination between nodes.
 *
 * @return Delay in milliseconds.
 */
static unsigned long repeatDelayMs(void)
{
    uint32_t id = (uint32_t)buoyId;
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
    // Dropped rather than queued unbounded: if four relays are already waiting, the channel is
    // busier than the relay is worth and the original sender still has its own retry table.
    Serial.println("#LoRa Repeat: slots full, frame not relayed");
}

/**
 * @brief Drops a pending relay because that exact frame has just been heard on the air again.
 *
 * This is what makes the per-MAC stagger worth having. Every Top in range of the sender queues the
 * same relay; the one in the lowest slot transmits first, and the rest hear their own pending frame
 * come back and discard it. So a unicast that needs relaying is relayed ONCE, by whichever node
 * happens to be earliest, instead of once per node.
 *
 * A transparent repeater cannot tell another node's relay from the original sender's own
 * retransmit - the frame is byte-identical either way - so this cancels on both. That is safe
 * only because of the timing: the longest relay delay is REPEAT_BASE_MS + 3 slots + jitter, about
 * 1100 ms, while the sender's retry table waits ~1500 ms. Our relay has therefore always gone out
 * (and freed its slot) before a retransmit could arrive, so a retransmit never cancels a relay
 * that was still needed. Widening the slots past ~450 ms would break that and starve a genuinely
 * out-of-range node - see the bound noted at REPEAT_SLOT_MS.
 *
 * @param msg The frame just received, verbatim.
 */
static void cancelRepeat(const String &msg)
{
    for (int i = 0; i < REPEAT_SLOTS; i++)
    {
        if (repeatSlots[i].busy && repeatSlots[i].message == msg)
        {
            repeatSlots[i].message = "";
            repeatSlots[i].busy = false;
            return;
        }
    }
}

/**
 * @brief Drops a pending relay because the node it was meant for has just answered.
 *
 * If we can hear the addressee acknowledging the very command we were about to relay for it, then
 * it heard the original perfectly well and the relay is pure channel load. This is the normal case
 * in a compact field where every node is in range of every other: a COMPUTE STARTLINE sends one
 * unicast SETLOCKPOS per buoy, and without this the buoys NOT addressed would each relay both of
 * them after they had already been delivered.
 *
 * @param in A decoded ACK frame just received.
 */
static void cancelRepeatOnAck(const RoboStruct *in)
{
    for (int i = 0; i < REPEAT_SLOTS; i++)
    {
        if (repeatSlots[i].busy && repeatSlots[i].cmd == in->cmd &&
            repeatSlots[i].target_id == in->IDs)
        {
            repeatSlots[i].message = "";
            repeatSlots[i].busy = false;
            return;
        }
    }
}

/**
 * @brief Transmits any relay whose backoff has expired. Called from LoraTask.
 *
 * One per pass, so a burst of relays is spread over successive loops instead of being pushed out
 * back to back.
 */
static void drainRepeats(void)
{
    unsigned long now = millis();
    for (int i = 0; i < REPEAT_SLOTS; i++)
    {
        if (repeatSlots[i].busy && (long)(now - repeatSlots[i].due_ms) >= 0)
        {
            if (sendLora(repeatSlots[i].message))
            {
                Serial.println("#LoRa Repeat: Forwarding <" + repeatSlots[i].message + ">");
                repeatSlots[i].message = "";
                repeatSlots[i].busy = false;
            }
            else if ((long)(now - repeatSlots[i].due_ms) > REPEAT_EXPIRY_MS)
            {
                // The radio has refused this for three seconds - by now the relay is stale enough
                // to be useless, and holding the slot would block every later one. Freeing it is
                // also what stops a wedged transceiver from silently disabling the repeater.
                Serial.println("#LoRa Repeat: expired, dropping relay");
                repeatSlots[i].message = "";
                repeatSlots[i].busy = false;
            }
            // Otherwise sendLora()'s inter-packet guard is still holding. Leave the slot busy and
            // try again next pass rather than losing the relay, which is what the old
            // fire-and-forget call did on every guarded return.
            return;
        }
    }
}

/**
 * @brief Parses an incoming LoRa packet and routes it to the correct queues.
 * 
 * @param packetSize The size of the received LoRa packet.
 */
//***************************************************************************************************
//  LoRa link quality
//***************************************************************************************************
// What we hear, and how well, keyed on WHO SENT IT. One reading per ordered pair builds a matrix,
// and it is the asymmetry in that matrix that separates a deaf antenna from a long path: a link
// that is weak in both directions is distance or an obstruction, one that is weak in a single
// direction points at one end. An unattributed pile of signal strengths cannot show either.
//
// RSSI is only half of it. It describes the frames that ARRIVED - a link can sit at a comfortable
// -70 dBm and drop most of what is sent to it, and nothing in the signal strength would say so. So
// the count of frames heard per minute goes out beside it.
#define LINK_PEERS_TRACKED 12
#define LINK_REPORT_MS 60000UL

// A relayed frame carries the ORIGINAL sender's id but arrives at the RELAY's signal strength.
// Counting one would invent a link that does not physically exist, at a strength belonging to a
// different pair - worse than no reading, because it looks plausible. So only the FIRST copy of a
// given frame is measured.
//
// The window is derived from the repeater's own timing rather than picked: a relay cannot be
// earlier than REPEAT_BASE_MS nor later than the last slot plus jitter. A sender's own retransmit
// is ~1500 ms behind, outside this window, so it still counts - and it should, being a genuine
// second transmission over the air. Frames inside the window that are NOT relays are rare enough,
// and undercounting is the safe direction: it can only make a link look worse than it is.
#define LINK_DEDUP_MS (REPEAT_BASE_MS + REPEAT_SLOTS * REPEAT_SLOT_MS + REPEAT_JITTER_MS + 100)
#define LINK_DEDUP_SLOTS 8

// A peer that transmits once a minute lands in a one-minute window perhaps half the time, so a
// table that only ever showed the current window made such a peer blink in and out of existence.
// The window stats are what the report carries; lastRssi and lastHeard are kept alongside so a
// slow peer still reads as "heard, 90 s ago at -95" instead of vanishing. Genuinely gone is a
// different statement, and LINK_FORGET_MS is when we make it.
#define LINK_FORGET_MS 300000UL

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
static uint8_t linkRotate = 0;

// Has this exact frame already been measured recently? See LINK_DEDUP_MS.
static bool linkFirstCopy(const String &msg)
{
    unsigned long now = millis();
    for (int i = 0; i < LINK_DEDUP_SLOTS; i++)
    {
        if (linkSeen[i] == msg && (now - linkSeenAt[i]) < LINK_DEDUP_MS) return false;
    }
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
    if (free_slot < 0) return;   // more peers than slots: the table is already the interesting ones
    linkPeer[free_slot].used = true;
    linkPeer[free_slot].id = id;
    linkPeer[free_slot].rssiSum = rssi;
    linkPeer[free_slot].rssiWorst = (int16_t)rssi;
    linkPeer[free_slot].count = 1;
    linkPeer[free_slot].lastRssi = (int16_t)rssi;
    linkPeer[free_slot].lastHeard = millis();
}

// Still worth reporting? Heard at all, and not so long ago that it has plainly gone.
static bool linkAlive(const LinkPeer &p)
{
    return p.used && p.lastHeard && (millis() - p.lastHeard) < LINK_FORGET_MS;
}

// The mean, rounded, for a peer with at least one sample.
static int linkMean(const LinkPeer &p)
{
    return p.count ? (int)(p.rssiSum / (long)p.count) : (int)p.lastRssi;
}

// One report a minute, LoRa only. Never queued to udpOut: this is a measurement of the LoRa path,
// and a copy arriving over WiFi would say nothing about it. It IS broadcast, so that every listener
// gets the same picture - see the LORA_LINK exception in the repeater below for how it reaches a
// node that cannot hear us directly.
void linkReportService(void)
{
    if (linkReportDue == 0) linkReportDue = millis() + LINK_REPORT_MS;
    if ((long)(millis() - linkReportDue) < 0) return;
    linkReportDue = millis() + LINK_REPORT_MS;

    // Collect and sort worst mean first: if anything has to be left out of the frame it should be
    // the healthy links, not the marginal one being hunted.
    int order[LINK_PEERS_TRACKED];
    int n = 0;
    for (int i = 0; i < LINK_PEERS_TRACKED; i++)
        if (linkAlive(linkPeer[i])) order[n++] = i;
    if (n == 0) return;

    for (int a = 0; a < n - 1; a++)
        for (int b = a + 1; b < n; b++)
            if (linkMean(linkPeer[order[b]]) < linkMean(linkPeer[order[a]]))
            {
                int t = order[a]; order[a] = order[b]; order[b] = t;
            }

    RoboStruct msg = {};
    msg.cmd = LORA_LINK;
    msg.ack = INF;
    msg.IDr = BUOYIDALL;
    msg.IDs = (pMainData && pMainData->IDs != 0) ? pMainData->IDs : buoyId;
    msg.status = pMainData ? pMainData->status : 0;

    int put = 0;
    if (n <= LORA_LINK_MAX_PEERS)
    {
        for (; put < n; put++)
        {
            const LinkPeer &p = linkPeer[order[put]];
            msg.linkPeerId[put] = p.id;
            msg.linkRssi[put] = (int16_t)linkMean(p);
            msg.linkCount[put] = p.count;
        }
        msg.linkOmitted = 0;
    }
    else
    {
        // The worst few always, then one rotating slot through the rest - so a peer that is
        // permanently eighth-best still gets reported, just less often, instead of never.
        int fixed = LORA_LINK_MAX_PEERS - 1;
        for (; put < fixed; put++)
        {
            const LinkPeer &p = linkPeer[order[put]];
            msg.linkPeerId[put] = p.id;
            msg.linkRssi[put] = (int16_t)linkMean(p);
            msg.linkCount[put] = p.count;
        }
        int spare = fixed + (linkRotate % (n - fixed));
        const LinkPeer &p = linkPeer[order[spare]];
        msg.linkPeerId[put] = p.id;
        msg.linkRssi[put] = (int16_t)linkMean(p);
        msg.linkCount[put] = p.count;
        put++;
        linkRotate++;
        msg.linkOmitted = (uint8_t)(n - put);
    }
    msg.linkPeers = (uint8_t)put;

    xQueueSend(loraOut, (void *)&msg, 0);

    // Fresh window. The counts mean "per minute" or they mean nothing - but lastRssi and lastHeard
    // survive, so a peer that simply had a quiet minute still reads as heard rather than gone.
    for (int i = 0; i < LINK_PEERS_TRACKED; i++)
    {
        linkPeer[i].rssiSum = 0;
        linkPeer[i].rssiWorst = 0;
        linkPeer[i].count = 0;
    }
}

// The same table as JSON, for this Top's own web page. Not a second transport for the report - it
// is served on request over HTTP and puts nothing on the air.
String linkReportJson(void)
{
    String out = "[";
    bool first = true;
    for (int i = 0; i < LINK_PEERS_TRACKED; i++)
    {
        if (!linkAlive(linkPeer[i])) continue;
        if (!first) out += ",";
        first = false;
        out += "{\"id\":\"" + String(linkPeer[i].id, HEX) + "\"";
        out += ",\"rssi\":" + String(linkMean(linkPeer[i]));
        out += ",\"worst\":" + String(linkPeer[i].count ? linkPeer[i].rssiWorst : linkPeer[i].lastRssi);
        out += ",\"count\":" + String(linkPeer[i].count);
        out += ",\"age\":" + String((millis() - linkPeer[i].lastHeard) / 1000) + "}";
    }
    out += "]";
    return out;
}

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
    
    int packetRssi = LoRa.packetRssi();

    RoboStruct in;
    rfDeCode(incoming,&in);

    // Measure the link, whoever the frame was addressed to - hearing somebody else's traffic says
    // just as much about the path as hearing our own. Our own echo is skipped, and only the first
    // copy of a frame counts: see linkFirstCopy().
    {
        bool from_self = (in.IDs == buoyId || (pMainData && in.IDs == pMainData->IDs));
        if (!from_self && linkFirstCopy(incoming)) linkNote((uint32_t)in.IDs, packetRssi);
    }

    // Before anything else: if we were about to relay this exact frame, we no longer need to -
    // somebody beat us to it, or the original sender resent it. Ahead of the early returns below
    // so it still applies to frames addressed to us.
    cancelRepeat(incoming);
    if (in.ack == ACK) cancelRepeatOnAck(&in);

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

    // Transparent Repeater: relay a frame that is addressed to SOMEONE ELSE, in case that
    // someone is out of the sender's range but in ours.
    //
    // Broadcasts are deliberately NOT relayed. They used to be, because a broadcast is not
    // "addressed specifically to us" and so fell through into this branch - and all periodic
    // telemetry is broadcast (handleTimerRoutines sets IDr = BUOYIDALL). Every Top therefore
    // rebroadcast every other Top's telemetry, putting each frame on the air once per Top in
    // the field. Measured at the library default SF7/125 kHz, where TOPDATA is 116 bytes and
    // 195 ms of airtime: three locked buoys sending one frame a second came to ~1950 ms of
    // airtime per second, i.e. about 195% channel occupancy. The channel could not carry it,
    // and what got lost was the traffic that mattered - a COMPUTE STARTLINE would compute and
    // its SETLOCKPOS frames would never arrive.
    //
    // Relaying a broadcast buys nothing anyway: a node close enough to hear our relay is
    // almost always close enough to have heard the original, and every buoy broadcasts its own
    // telemetry regardless. So the rule is unicast only.
    bool is_from_me = (in.IDs == buoyId || (pMainData && in.IDs == pMainData->IDs));
    bool is_broadcast = (in.IDr == BUOYIDALL || in.IDr == 0);
    // LORA_LINK is the one broadcast that IS relayed. The rule above exists because relaying
    // telemetry buys nothing - a node close enough to hear the relay had almost certainly heard the
    // original, and every buoy broadcasts its own anyway. Neither holds here: the whole point of a
    // link report is to arrive from a node you CANNOT hear directly, and nobody else can produce it
    // on that node's behalf. One frame a minute per Top is ~0.2% of the channel, and the existing
    // dedup cache stops it going round in circles.
    bool relay_worthy = !is_broadcast || in.cmd == LORA_LINK;
    if (!is_from_me && !is_addressed_to_me && relay_worthy)
    {
        if (checkAndRecordRepeaterMessage(incoming))
        {
            queueRepeat(incoming, in.IDr, in.cmd);
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
        // Put out any relay whose backoff has expired - see queueRepeat().
        drainRepeats();
        // One link report a minute - see linkReportService().
        linkReportService();
        
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
