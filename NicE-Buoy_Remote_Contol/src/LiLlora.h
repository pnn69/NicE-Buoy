#ifndef LORA_H_
#define LORA_H_
#include "Arduino.h"
/*
    [Designator][Reciever][MSG_ID][lengt_msg][msg]
*/

struct loraDataType
{
    byte destination;
    byte sender;
    int recipient;
    byte id;
    byte messagelength;
    String message;
    int rssi;
    float snr;
};

extern loraDataType loraIn;
extern loraDataType loraOut;

extern bool loraOK;
bool InitLora(void);
int polLora(void);
bool sendLora(int buoy);
void sendLoraSetTargetPosition(int buoy);
void sendLoraSetDocPosition(int buoy);
void sendLoraGoToDocPosition(int buoy);
void sendLoraSetSailDirSpeed(int buoy, int tgdir, int speed);
void sendLoraStatus(int buoy, int status);
void sendLoraSetGetPosition(int buoy);
void sendLoraSetIdle(int buoy);
void sendLoraReset(int buoy);

#endif /* LORA_H_ */
