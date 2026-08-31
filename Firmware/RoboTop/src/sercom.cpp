#include "sercom.h"
#include "io_top.h"
#include "main.h"
#include "topwifi.h"
#include <HardwareSerial.h>
#include "robotone.h"
#include "buzzer.h"
#include <AsyncUDP.h>

QueueHandle_t serOut;
QueueHandle_t serIn;
static RoboStruct serDataOut;
static RoboStruct serDataIn;
RoboStruct pendingMsg[10] = {};
static unsigned long lastSerMsg = 0;
static unsigned long retransmittReady = 0;
static unsigned long mac;

/**
 * @brief Initializes the FreeRTOS queues for serial communication.
 */
void initserqueue(void)
{
    serOut = xQueueCreate(10, sizeof(RoboStruct));
    serIn = xQueueCreate(10, sizeof(RoboStruct));
    mac = espMac();
}

//***************************************************************************************************
//  Store to ack buffer
//***************************************************************************************************
void SerstoreAckMsg(RoboStruct ackBuffer)
{
    for (int i = 0; i < 10; i++)
    {
        if (pendingMsg[i].cmd == ackBuffer.cmd && pendingMsg[i].IDr == ackBuffer.IDr)
        {
            pendingMsg[i] = ackBuffer;
            pendingMsg[i].retry = 5;
            // printf("ACK_STORE: Updated msg cmd=%d for IDr=%X at pos=%d\r\n", ackBuffer.cmd, ackBuffer.IDr, i);
            return;
        }
    }

    for (int i = 0; i < 10; i++)
    {
        if (pendingMsg[i].cmd == 0)
        {
            pendingMsg[i] = ackBuffer;
            // printf("ACK_STORE: Stored msg cmd=%d for IDr=%X at pos=%d\r\n", ackBuffer.cmd, ackBuffer.IDr, i);
            return;
        }
    }
    // printf("ACK_STORE: ERROR - Buffer full!\r\n");
}

//***************************************************************************************************
//  Remove to ack buffer
//***************************************************************************************************
void SerremoveAckMsg(RoboStruct ackBuffer)
{
    bool found = false;
    for (int i = 0; i < 10; i++)
    {
        if (pendingMsg[i].cmd != 0 && pendingMsg[i].cmd == ackBuffer.cmd)
        {
            // printf("ACK_REMOVE: Removed msg cmd=%d from pos=%d\r\n", pendingMsg[i].cmd, i);
            pendingMsg[i].ack = 0;
            pendingMsg[i].cmd = 0;
            pendingMsg[i].IDs = 0;
            pendingMsg[i].IDr = 0;
            pendingMsg[i].retry = 0;
            found = true;
        }
    }
    if (!found) {
        // printf("ACK_REMOVE: No pending msg found for cmd=%d\r\n", ackBuffer.cmd);
    }
}

//***************************************************************************************************
//  check ack buffer
//***************************************************************************************************
RoboStruct SerchkAckMsg(void)
{
    RoboStruct in;
    in.ack = 0;
    in.cmd = 0;
    for (int i = 0; i < 10; i++)
    {
        if (pendingMsg[i].cmd != 0)
        {
            in = pendingMsg[i];
            pendingMsg[i].retry--;
            if (pendingMsg[i].retry == 0)
            {
                // printf("ACK_RETRY: Failed after max retries cmd=%d\r\n", pendingMsg[i].cmd);
                pendingMsg[i].cmd = 0;
                pendingMsg[i].ack = 0;
                pendingMsg[i].IDs = 0;
                pendingMsg[i].IDr = 0;
            } else {
                // printf("ACK_RETRY: Resending cmd=%d to IDr=%X, retries left=%d\r\n", in.cmd, in.IDr, pendingMsg[i].retry);
            }
            return in;
        }
    }
    return in;
}

/**
 * @brief FreeRTOS task handling all half-duplex serial communication.
 */
#include "udplog.h"

// Physical wakeup pulse for the Sub: drop the UART, hold the shared TX line HIGH for 500 ms, then
// bring the UART back up. The Sub watches for that level and powers its rails back on. 500 ms is
// the figure this has always used and it is known good on the hardware.
//
// Costs nothing if the Sub was awake all along - it just sees half a second of idle line.
static void subWakePulse(void)
{
    Serial.println("Waking up Sub...");
    Serial1.end();
    pinMode(COM_PIN_TX, OUTPUT);
    digitalWrite(COM_PIN_TX, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(COM_PIN_TX, LOW);
    Serial1.begin(BAUDRATE, SERIAL_8N1, COM_PIN_RX, COM_PIN_TX, LEVEL);
}

// --- Waking a sleeping Sub on demand -----------------------------------------------------------
//
// The Sub refreshes its own POWEROFFTIME timer on every serial frame it RECEIVES, and after 60
// minutes without one it pulls PWRENABLE low and switches itself off. Nothing here used to bring it
// back when we simply wanted to TALK to it: the only triggers were boot, a status change out of
// IDLE, and the serial watchdog - and the watchdog is gated on isSerConnected AND clears that flag
// after firing, so it goes off exactly once and then disarms until the Sub speaks. A Sub that is
// asleep never speaks, so the only way back was the button on the Top.
//
// So: pulse it awake when there is actually something to send. On demand rather than on a timer,
// which is what keeps this from defeating the power saving - no traffic, no wake, and the buoy is
// left to sleep.
//
// Readiness is decided by the Sub's OWN data rather than a fixed settle delay: it streams compass
// and voltage at a high rate, so the first frame after the pulse is proof it is up and listening.
// No guessed constant, and it adapts to however long the boot actually takes.
#define SUB_QUIET_MS          3000   // Silent this long => assume asleep. It normally streams.
#define SUB_WAKE_SETTLE_MAX   4000   // Give up waiting for its first frame and send anyway
#define SUB_WAKE_MIN_GAP     15000   // Never pulse more often than this - a Sub that is simply
                                     // unplugged must not have every outgoing frame stall the task

void SercomTask(void *arg)
{
    unsigned long lastRx = millis();
    mac = espMac();
    delay(2000);
    beep(5, buzzer);
    Serial1.begin(BAUDRATE, SERIAL_8N1, COM_PIN_RX, COM_PIN_TX, LEVEL);
    Serial1.setTimeout(100);
    Serial.setTimeout(100);
    while (1)
    {
        crumbAt(CRUMB_SER, 300);
        if (Serial.available())
        {
            crumbAt(CRUMB_SER, 301);
            String serStringIn = Serial.readStringUntil('\n');
            serStringIn.trim();
            if (serStringIn.length() > 0)
            {
                RoboStruct serDataIn;
                rfDeCode(serStringIn, &serDataIn);
                if (serDataIn.IDs != -1 && serDataIn.IDs != mac && serDataIn.IDs != 0x99)
                {
                    printf("SER_PC_IN CMD=%d\n", serDataIn.cmd); 
                    xQueueSend(serIn, (void *)&serDataIn, 10);
                }
            }
        }

        crumbAt(CRUMB_SER, 302);
        if (Serial1.available())
        {
            crumbAt(CRUMB_SER, 303);
            String serStringIn = Serial1.readStringUntil('\n');
            serStringIn.trim();
            if (serStringIn.length() > 0)
            {
                RoboStruct serDataIn;
                rfDeCode(serStringIn, &serDataIn);
                // printf("DEBUG_SERCOM_IN: %s\r\n", serStringIn.c_str());
                // Echo Prevention inside Half-Duplex RS-485 / Single-Wire Serial Line:
                // Since the TX and RX lines are physically tied together in half-duplex configurations,
                // any packet we transmit is immediately echoed back and received on our own RX buffer.
                // We identify and ignore these echoes by matching the sender ID with our Top MAC address
                // and validating if the command intent represents a request (GET, GETACK, or SET).
                bool is_echo = (serDataIn.IDs == mac && (serDataIn.ack == GET || serDataIn.ack == GETACK || serDataIn.ack == SET));
                
                if (serDataIn.IDs != -1 && serDataIn.IDs != 0x99 && !is_echo)
                {
                    lastRx = millis();
                    if (serDataIn.ack == ACK)
                    {
                        // Explicit Acknowledgement received from the Sub; remove the matching message from retries
                        SerremoveAckMsg(serDataIn);
                    }
                    else
                    {
                        // IMPLICIT ACKNOWLEDGEMENT:
                        // Any valid unsolicited response or status message (like MsgType.INF) received from the Sub
                        // implies the Sub is awake, operational, and has processed our request.
                        // We safely remove any matching pending retry messages from our buffer.
                        SerremoveAckMsg(serDataIn);
                        xQueueSend(serIn, (void *)&serDataIn, 10);
                        lastSerMsg = millis();

                        if (serDataIn.ack == GETACK)
                        {
                            // If the Sub explicitly requested an acknowledgement, construct and send the ACK reply
                            serDataIn.IDr = serDataIn.IDs;
                            serDataIn.IDs = mac;
                            serDataIn.ack = ACK;
                            xQueueSend(serOut, (void *)&serDataIn, 10);
                        }
                    }
                }
            }
        }

        // Process any outgoing serial commands destined for the Sub unit.
        //
        // A frame that arrives while the Sub looks asleep is HELD here, not sent: the Sub is
        // pulsed awake and the frame goes out as soon as it starts talking (or after
        // SUB_WAKE_SETTLE_MAX, best effort). Nothing new is dequeued while one is held, so
        // outbound telemetry backs up in serOut and the oldest is dropped - which is the right
        // thing while the Sub is off, because there is nobody to receive it anyway.
        static RoboStruct subHeldFrame;
        static bool subWakePending = false;
        static unsigned long subWakeSentMs = 0;
        static unsigned long lastWakePulseMs = 0;
        bool haveFrame = false;

        if (subWakePending)
        {
            // Signed subtraction, so this still works across the millis() rollover.
            bool subSpoke = ((long)(lastRx - subWakeSentMs) > 0);
            if (subSpoke || millis() - subWakeSentMs > SUB_WAKE_SETTLE_MAX)
            {
                if (!subSpoke) Serial.println("Sub did not answer the wakeup - sending anyway");
                serDataOut = subHeldFrame;
                subWakePending = false;
                haveFrame = true;
            }
        }
        else if (xQueueReceive(serOut, (void *)&serDataOut, 0) == pdTRUE)
        {
            haveFrame = true;
        }

        if (haveFrame)
        {
            if (serDataOut.cmd == WAKEUP)
            {
                // An explicitly queued wakeup - boot, a status change out of IDLE, the watchdog.
                subWakePulse();
                lastWakePulseMs = millis();
            }
            else if (millis() - lastRx > SUB_QUIET_MS &&
                     millis() - lastWakePulseMs > SUB_WAKE_MIN_GAP)
            {
                // Something to say to a Sub that has gone quiet: wake it first, then hold this
                // frame until it answers. This is the path that used to need the button.
                subWakePulse();
                lastWakePulseMs = millis();
                subWakeSentMs = millis();
                subHeldFrame = serDataOut;
                subWakePending = true;
            }
            else
            {
                // Enforce line silence margin (20ms) after last reception to prevent packet collisions
                while (lastRx + 20 > millis())
                    vTaskDelay(pdMS_TO_TICKS(1));

                serDataOut.IDs = mac;
                
                String out = rfCode(&serDataOut);
                Serial1.println(out);
                /* 
                 * High-Throughput Performance Tuning:
                 * Commented out 'DEBUG_SERCOM_OUT' to prevent excessive terminal printing 
                 * which can lead to serial buffer overflows and timing jitter in the 500ms 
                 * real-time telemetry loop.
                 */
                // printf("DEBUG_SERCOM_OUT: %s\r\n", out.c_str());

                // If the outgoing command is GETACK or SET, record it in our retransmission list
                if (serDataOut.ack == GETACK || serDataOut.ack == SET)
                {
                    serDataOut.retry = 5;
                    SerstoreAckMsg(serDataOut);
                    retransmittReady = millis() + 1000;
                }
            }
        }

        if (retransmittReady != 0 && retransmittReady < millis())
        {
            serDataOut = SerchkAckMsg();
            if (serDataOut.cmd != 0)
            {
                while (lastRx + 20 > millis())
                    delay(1);
                String out = rfCode(&serDataOut);
                Serial1.println(out);
                // printf("SER_SUB_RETRY>%s<\r\n", out.c_str());
                retransmittReady = millis() + 1000;
            } else {
                retransmittReady = 0;
            }
        }
        delay(1);
    }
}
