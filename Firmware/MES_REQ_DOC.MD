RoboBuoy ESP-Mesh-Lite Architecture and Requirements
=====================================================

PURPOSE

This document defines the target communication architecture and non-negotiable
requirements for the RoboBuoy system.

The system consists of:

1. RoboCYD
2. RoboLink
3. RoboTop, currently maximum 3 units
4. RoboSub, one connected to each RoboTop
5. PC/laptop for monitoring, diagnostics and control

The communication architecture will progressively adopt ESP-Mesh-Lite.

The most important design rule is:

    THE BUOYS MUST REMAIN AUTONOMOUS.

Neither RoboCYD, RoboLink, the PC, Wi-Fi, nor the availability of an operator
interface may become a required dependency for local buoy navigation, control
or buoy-to-buoy coordination.


======================================================================
1. ROBOCYD
======================================================================

ROLE

RoboCYD is the operator console and the field Wi-Fi access point.

RoboCYD provides:

- touchscreen operator interface
- fleet status display
- field Wi-Fi access point
- web dashboard
- WebSockets
- UDP communication
- OTA updates
- LoRa communication during the migration period
- direct visibility of network/IP status to the operator


HARDWARE

RoboCYD currently uses:

- ESP32 CYD
- 240 x 320 ILI9341 display
- XPT2046 resistive touchscreen
- SX127x 433 MHz LoRa
- 4 MB flash


CURRENT ESP-IDF STATUS

RoboCYD is already migrated and hardware-tested with:

- ESP-IDF 5.5.5
- Arduino-ESP32 3.3.12 as an ESP-IDF component
- original RoboCYD application
- original screens and menus
- TFT_eSPI
- XPT2046_Bitbang_Slim
- correct BGR display colour order
- Wi-Fi
- ArduinoOTA
- HTTP
- WebSockets
- UDP
- SPIFFS
- LoRa
- live RoboTop telemetry
- ESP-IDF to ESP-IDF OTA

Known-good Git tag:

    esp-idf-full-system-v1.1

Known-good commit:

    542c7fae


FLASH LIMIT

Current RoboCYD application:

    OTA application partition: 0x140000
    firmware size:             approximately 0x129170
    remaining:                 approximately 93.8 KB
    remaining percentage:      approximately 7%

Because of this limited application partition headroom, ESP-Mesh-Lite should
NOT be added directly to RoboCYD unless a future redesign provides a compelling
reason.


======================================================================
2. ROBOCYD FIELD ACCESS POINT
======================================================================

RoboCYD remains the field Wi-Fi access point.

This is intentional because RoboCYD has a display and can show the operator:

- Wi-Fi state
- SSID
- IP address
- network diagnostics
- connection state


FIELD OPERATION

There is NO assumption that an external Wi-Fi router exists on the lake.

When NicE_WiFi is unavailable, RoboCYD creates:

    SSID: Robo_WiFi
    IP:   192.168.4.1

A PC, phone and RoboLink can connect to Robo_WiFi.

Typical field network:

               PC / phone
                    |
                    | Wi-Fi
                    |
               Robo_WiFi
                    |
                RoboCYD
               192.168.4.1
                AP + UI
                    |
             ordinary Wi-Fi
                    |
                RoboLink
                    |
              ESP-Mesh-Lite
             /      |       \
         RoboTop1 RoboTop2 RoboTop3


HOME OPERATION

When NicE_WiFi is available, RoboCYD may join NicE_WiFi exactly as the existing
working firmware does.

The existing automatic home/field network behaviour should be preserved until
the replacement architecture has been fully hardware-tested.


======================================================================
3. ROBOLINK
======================================================================

ROLE

RoboLink is the optional communications interface between:

- PC/laptop
- ordinary Wi-Fi
- ESP-Mesh-Lite
- RoboTop fleet
- optional LoRa during migration/testing

RoboLink replaces the old RoboLora role/name.

The name RoboLink is intentionally transport-independent.


ROBOLINK RESPONSIBILITIES

RoboLink should provide:

- PC/laptop communications interface
- serial/USB PC interface where useful
- ordinary Wi-Fi connection toward RoboCYD/home network
- ESP-Mesh-Lite participation
- Mesh diagnostics
- Robo protocol forwarding
- monitoring/logging
- possibly network OTA distribution in the future
- LoRa during the migration period if useful


ROBOLINK IS NOT A FLEET CONTROLLER

This rule is critical:

    RoboLink MUST NOT become a required router or controller
    for autonomous fleet operation.

RoboLink is infrastructure.

RoboLink improves:

- PC connectivity
- monitoring
- diagnostic visibility
- Mesh connectivity
- communication range
- operator access

But removal of RoboLink must NOT stop:

- RoboTop-to-RoboTop communication
- RoboTop-to-RoboSub UART communication
- local buoy navigation
- waypoint exchange
- start-line operation
- position keeping
- docking
- autonomous control


======================================================================
4. ROBOTOP
======================================================================

There can currently be up to 3 RoboTop units.

Each RoboTop:

- contains an ESP32
- controls its corresponding RoboSub through hardware UART
- contains fleet-level navigation/control logic
- communicates with other RoboTop units
- should eventually become an ESP-Mesh-Lite peer
- should retain LoRa during Mesh development until Mesh has been proven
- must remain autonomous


ROBOTOP PEER COMMUNICATION

RoboTop units must communicate with each other WITHOUT requiring RoboLink.

They must also communicate without RoboCYD when necessary.

For example:

    RoboTop1 <------> RoboTop2

or with multi-hop:

    RoboTop1 <------> RoboTop2 <------> RoboTop3

RoboTop1 and RoboTop3 do not need to be directly within radio range if RoboTop2
can provide an intermediate Mesh path.


======================================================================
5. ROBOSUB
======================================================================

Each RoboTop controls one RoboSub through hardware UART.

RoboSub performs lower-level buoy functions including:

- ESC control
- navigation/control execution
- position control
- compass/control functions
- docking execution
- other existing low-level autonomous functions

RoboSub does NOT need to become an ESP-Mesh-Lite node if RoboTop acts as the
network proxy.

The existing RoboTop to RoboSub hardware UART architecture should remain.


======================================================================
6. AUTONOMY REQUIREMENT
======================================================================

THIS IS THE MOST IMPORTANT REQUIREMENT.

The following operating condition MUST be supported:

    RoboCYD  = OFF or absent
    RoboLink = OFF or absent
    PC       = OFF or absent

while the RoboTop/RoboSub fleet remains operational.

The fleet must continue performing autonomous tasks.


REQUIRED AUTONOMOUS FUNCTIONS INCLUDE

- position holding
- navigation
- local control loops
- waypoint following
- start-line calculation
- waypoint exchange between RoboTop units
- docking
- RoboTop to RoboSub UART communication
- ESC/control execution
- peer fleet coordination required by the existing system


======================================================================
7. LOCAL START-LINE OPERATION
======================================================================

The existing RoboTop short-press + long-press functionality is a specific
acceptance requirement.

Example:

RoboLink is completely absent.

RoboCYD may also be absent.

An operator performs:

    one short button press
    followed by
    one long button press

on a RoboTop.

The system must still do the following:

1. detect the button sequence

2. calculate the new start line using the existing algorithm

3. calculate the required new waypoint(s)

4. apply the result locally where required

5. transmit the new waypoint to the other RoboTop/buoy

6. have the receiving RoboTop process the waypoint

7. communicate the required information to the corresponding RoboSub over UART

8. continue autonomous navigation/control


ROBOLINK MUST NOT BE INVOLVED IN THIS PROCESS.

RoboLink may observe or log the operation when present.

RoboLink may assist communication when present.

But RoboLink must never become mandatory for this behaviour.


======================================================================
8. COMMUNICATION ARCHITECTURE
======================================================================

NORMAL FIELD CONFIGURATION

                         PC
                          |
                        Wi-Fi
                          |
                     Robo_WiFi
                          |
                       RoboCYD
                 operator console/AP
                          |
                    ordinary Wi-Fi
                          |
                       RoboLink
                          |
                     Mesh network
                    /     |      \
                   /      |       \
              RoboTop1 RoboTop2 RoboTop3
                  |       |        |
                UART    UART     UART
                  |       |        |
              RoboSub1 RoboSub2 RoboSub3


HOWEVER:

RoboLink MUST NOT become the path for every RoboTop-to-RoboTop packet.


PEER OPERATION WITHOUT ROBOLINK

With RoboLink absent:

                 RoboTop1
                 /      \
            RoboTop2   RoboTop3
                |          |
              UART       UART
                |          |
            RoboSub2    RoboSub3


PEER OPERATION WITHOUT ROBOLINK AND ROBOCYD

                X RoboCYD X
                X RoboLink X

                    RoboTop1
                    /      \
               RoboTop2   RoboTop3
                   |          |
                 UART       UART
                   |          |
               RoboSub2    RoboSub3

Autonomous operation must continue.


======================================================================
9. MESH DESIGN PRINCIPLE
======================================================================

ESP-Mesh-Lite is a transport/network layer.

ESP-Mesh-Lite must NOT become the Robo application protocol.

The application-level Robo protocol should remain transport-independent.

Conceptually:

                   Robo protocol
                         |
             +-----------+-----------+
             |           |           |
            UART        LoRa        Mesh
             |           |           |
             +-----------+-----------+
                         |
                    application


The Robo application should ideally not care how a valid Robo packet travelled
between two units.


======================================================================
10. EXISTING ROBO PROTOCOL
======================================================================

Preserve the existing Robo protocol.

Existing concepts that should remain reusable include:

- RoboStruct
- sender ID
- destination/receiver ID
- command
- status
- sequence number where applicable
- ACK information where applicable
- CRC
- RoboCode()
- RoboDecode()
- packet encoding
- packet decoding

Existing Robo packets have already been transported over:

- UART
- UDP
- LoRa

ESP-Mesh-Lite should become another transport underneath this protocol.

Do NOT redesign commands such as TOPDATA, BUOYPOS, SETUPDATA, etc. solely
because Mesh is introduced.


======================================================================
11. DIRECT PEER OPERATION
======================================================================

RoboTop communication must not conceptually become:

    Top1 -> RoboLink -> Top2

Instead it should be:

    Top1 ----------> Top2
           Mesh

If direct RF range is insufficient:

    Top1 -> Top2 -> Top3

may provide connectivity between Top1 and Top3.

The application on Top1 should not have to implement individual physical
radio hops.


======================================================================
12. ROBOLINK FAILURE REQUIREMENT
======================================================================

Explicit acceptance test:

1. Start the fleet normally.

2. Establish communication between RoboTop units.

3. Verify autonomous functions.

4. Completely power RoboLink off.

5. Verify RoboTop units continue communicating.

6. Verify RoboSub UART communication continues.

7. Perform the RoboTop short + long start-line button sequence.

8. Verify a new start line is calculated.

9. Verify the waypoint reaches the appropriate other buoy.

10. Verify corresponding RoboSubs continue control/navigation.

11. Verify docking remains possible.

12. Verify no buoy resets purely because RoboLink disappeared.


LOSS OF ROBOLINK MAY REMOVE

- PC monitoring through RoboLink
- RoboLink diagnostics
- RoboLink command injection
- RoboLink logging

LOSS OF ROBOLINK MUST NOT REMOVE

- autonomous control
- navigation
- waypoint exchange
- start-line operation
- docking
- position control
- Top/Sub communication
- Top-to-Top communication


======================================================================
13. ROBOCYD FAILURE REQUIREMENT
======================================================================

RoboCYD must not be a single point of failure for autonomous operation either.

If RoboCYD disappears:

- RoboTop units continue operating
- RoboTop-to-RoboTop communication continues where the Mesh permits
- RoboSubs continue their active tasks
- waypoint exchange remains possible
- start-line operation remains possible locally
- docking remains possible
- local control remains active

The unavailable functions may include:

- touchscreen operator interface
- RoboCYD HTTP dashboard
- RoboCYD WebSockets
- PC access through RoboCYD's AP if no alternative access exists

These are operator/infrastructure losses, NOT losses of autonomous control.


======================================================================
14. PUBLIC FIELD NETWORK
======================================================================

No external AP/router is assumed to exist on the lake.

RoboCYD remains responsible for providing:

    Robo_WiFi

for:

- laptop
- phone
- RoboLink
- diagnostics
- operator access

RoboCYD should continue showing the current network/IP address on the display.

This is an operational requirement.


======================================================================
15. ROBOLINK NETWORK BEHAVIOUR
======================================================================

At home:

    NicE_WiFi
       |
       +---- RoboCYD
       |
       +---- RoboLink

In the field:

    RoboCYD
      |
      +---- Robo_WiFi
              |
              +---- PC
              |
              +---- RoboLink

RoboLink then participates in the Mesh network.

RoboLink may provide Mesh visibility to the PC and RoboCYD.

RoboLink should NOT be configured so that its disappearance destroys the
RoboTop peer network.


======================================================================
16. LORA MIGRATION
======================================================================

LoRa should remain active during Mesh development.

Do NOT remove LoRa merely because Mesh-Lite starts working.

Recommended migration:

Stage 1:

    existing UDP
    existing LoRa

Stage 2:

    existing UDP
    existing LoRa
    Mesh-Lite added

Stage 3:

    Mesh-Lite becomes the preferred peer transport
    LoRa remains available as backup
    UDP architecture may be simplified if appropriate

Stage 4:

    evaluate LoRa removal only after extensive field testing

There is no requirement to remove LoRa if keeping LoRa provides useful
independent redundancy.


======================================================================
17. REDUNDANT COMMUNICATION
======================================================================

An ideal final arrangement may provide:

                    RoboTop
                   /       \
                  /         \
            Mesh/IP         LoRa
                |             |
                |             |
             peer fleet / RoboCYD

Mesh and LoRa should be considered independent communication paths.

A Mesh failure should not automatically destroy LoRa operation.

A LoRa failure should not automatically destroy Mesh operation.


======================================================================
18. PC INTERFACE
======================================================================

RoboLink should provide the communications interface toward a PC/laptop.

Potential RoboLink PC functions include:

- receive all appropriate Robo protocol messages
- display/log traffic
- send operator/debug commands
- display Mesh topology
- display node identities
- display Mesh parent/level information
- display signal information
- diagnostic logging
- firmware/version visibility
- potentially OTA orchestration in the future

However, PC connectivity is optional.

PC absence must have no effect on autonomous navigation/control.


======================================================================
19. UDP AND BROADCAST HANDLING
======================================================================

Existing Robo software uses UDP broadcasts.

Do NOT assume Layer-2/subnet broadcasts automatically propagate through
different routed or Mesh network segments.

RoboLink may require an explicit application-level Robo packet relay.

Example:

    packet received from ordinary Wi-Fi
                 |
                 v
             RoboLink
                 |
                 v
            Mesh transport

Reverse:

    packet received from Mesh
                 |
                 v
             RoboLink
                 |
                 v
        ordinary Wi-Fi / PC

Avoid:

- relay loops
- duplicate storms
- endlessly repeated broadcasts

Use existing Robo packet information such as:

- sender ID
- receiver ID
- sequence information
- ACK state

for duplicate handling where practical.


======================================================================
20. MULTI-HOP REQUIREMENT
======================================================================

The fleet will operate on a lake.

Expected physical characteristics:

- clear line-of-sight in many situations
- communication over water
- external approximately +6 dBi antennas where appropriate
- potentially substantial distances between buoys

Do NOT assume all RoboTop units are always in direct radio range.

Mesh validation must include genuine multi-hop.

Example:

    RoboTop1 <----> RoboTop2 <----> RoboTop3

Test conditions must ensure RoboTop1 cannot directly communicate reliably with
RoboTop3 while the path through RoboTop2 still works.

Only then has actual Mesh functionality been proven.


======================================================================
21. SELF-HEALING TEST
======================================================================

After multi-hop works:

    Top1 -> Top2 -> Top3

remove or power off Top2 while another possible parent/path is available.

Measure:

- failure detection time
- network recovery time
- lost messages
- application behaviour
- navigation continuity
- LoRa continuity
- effect on RoboSub UART/control

The loss of one Mesh path must never crash/reset the autonomous buoy controller.


======================================================================
22. HARDWARE SAFETY PRINCIPLE
======================================================================

Transport failures must remain transport failures.

They must NOT propagate into failures of:

- navigation
- control loops
- ESC outputs
- stored mission state
- docking state
- compass state
- RoboTop/RoboSub communication

The network is not the real-time control loop.


======================================================================
23. SOFTWARE DEVELOPMENT STRATEGY
======================================================================

Do NOT perform a big-bang networking rewrite.

Use small hardware-tested steps.


RECOMMENDED ORDER

1. Preserve current RoboCYD known-good state.

2. Keep esp-idf-full-system-v1.1 as the RoboCYD rollback point.

3. Begin work on RoboLink.

4. Preserve existing RoboLink functionality first.

5. Establish RoboLink <-> RoboCYD ordinary Wi-Fi communication.

6. Keep LoRa active.

7. Add ESP-Mesh-Lite to RoboLink/native ESP-IDF environment.

8. Test Mesh with a disposable/spare ESP32 first.

9. Verify ordinary IP communication.

10. Send one existing Robo packet over Mesh.

11. Add Mesh to one RoboTop.

12. Verify RoboLink <-> RoboTop communication.

13. Add a second RoboTop.

14. Verify RoboTop <-> RoboTop communication.

15. Verify actual multi-hop.

16. Remove RoboLink and verify peer communication continues.

17. Remove RoboCYD and verify autonomous operation continues.

18. Perform local short + long button start-line operation without RoboLink.

19. Verify waypoint transmission to another RoboTop.

20. Verify RoboSub navigation/control.

21. Test docking.

22. Test Mesh self-healing.

23. Compare Mesh and LoRa behaviour.

24. Only then evaluate removing older transports.


======================================================================
24. ROBOCYD GIT BASELINE
======================================================================

Known-good RoboCYD branch:

    feature/ESP_IDF_Migration

Known-good corrected-colour tag:

    esp-idf-full-system-v1.1

Known-good commit:

    542c7fae

Experimental ESP-IDF Mesh branch:

    feature/ESP_IDF_Mesh_Lite

There is also an older pre-ESP-IDF branch:

    feature/ESP_Mesh_Lite

Do not confuse the two Mesh branches.


======================================================================
25. KNOWN ROBOCYD MESH DEPENDENCY ISSUE
======================================================================

Do NOT blindly add:

    espressif/mesh_lite=1.0.2

to RoboCYD.

This was already tested.

Dependency resolution failed because:

    Arduino-ESP32 3.3.12
        requires esp_modem ^2.0.2

while:

    Mesh-Lite 1.0.2
        requires IoT-Bridge 1.0.1
            requires esp_modem 1.*

Do NOT downgrade RoboCYD's known-good ESP-IDF/Arduino environment simply to
force this old Mesh-Lite dependency set to resolve.

This is another reason Mesh functionality belongs primarily in RoboLink and
RoboTop rather than RoboCYD.


======================================================================
26. ROLE DEFINITIONS
======================================================================

Use these definitions consistently:

RoboCYD
    Operator console and field Wi-Fi AP.

RoboLink
    Optional PC/network/Mesh communications interface.
    NOT a fleet master.
    NOT required for autonomy.

RoboTop
    Autonomous fleet peer.
    Mesh participant.
    Controls one RoboSub through UART.

RoboSub
    Autonomous low-level buoy controller.
    Controlled locally by RoboTop over UART.

PC
    Optional monitoring/control/diagnostic terminal.
    Never required for autonomous operation.


======================================================================
27. NON-NEGOTIABLE DESIGN RULES
======================================================================

1. RoboCYD remains the visible field Wi-Fi AP.

2. RoboCYD must show network/IP information to the operator.

3. RoboLink is the PC/network/Mesh interface.

4. RoboLink is optional.

5. RoboLink must never become a required path for Top-to-Top communication.

6. RoboTop units must continue communicating when RoboLink is absent.

7. RoboTop/RoboSub units must continue autonomous operation when RoboCYD is
   absent.

8. RoboSub remains connected to RoboTop through hardware UART.

9. Mesh-Lite is a transport/network layer, not the Robo application protocol.

10. Preserve the existing Robo protocol.

11. Preserve LoRa while Mesh is being developed and proven.

12. Do not assume an external Wi-Fi router exists in the field.

13. RoboCYD continues providing Robo_WiFi when required.

14. The short + long RoboTop start-line operation must work without RoboLink.

15. The new waypoint must reach the other buoy without RoboLink.

16. Docking/navigation must work without RoboLink.

17. Network failures must not stop local real-time control.

18. Changes must be incremental and hardware-tested.

19. OTA recovery paths must not be sacrificed without a proven replacement.

20. Avoid introducing any new central single point of failure.


======================================================================
FINAL SUCCESS CRITERION
======================================================================

With all infrastructure present:

                        PC
                         |
                      RoboCYD
                    AP + operator UI
                         |
                      RoboLink
                    PC/Mesh interface
                         |
                   ESP-Mesh-Lite
                  /      |       \
             RoboTop1 RoboTop2 RoboTop3
                 |       |        |
               UART    UART     UART
                 |       |        |
             RoboSub1 RoboSub2 RoboSub3

the operator should have excellent visibility and communications.

But the system must degrade gracefully.


MOST IMPORTANT FAILURE TEST:

    RoboCYD  = OFF
    RoboLink = OFF
    PC       = OFF

The autonomous fleet must still be capable of:

    RoboTop1 <------ peer/Mesh ------> RoboTop2
       |                                 |
      UART                              UART
       |                                 |
    RoboSub1                          RoboSub2

and must continue:

- buoy control
- position holding
- navigation
- local start-line calculation
- start-line waypoint exchange
- autonomous waypoint following
- docking
- Top/Sub UART communication


FINAL DESIGN PHILOSOPHY:

    RoboCYD provides operator access.

    RoboLink provides connectivity and visibility.

    RoboTop and RoboSub provide autonomy.

    Mesh improves communication.

    LoRa provides independent redundancy where useful.

    Neither RoboCYD nor RoboLink may become necessary
    for the fleet to remain autonomous.