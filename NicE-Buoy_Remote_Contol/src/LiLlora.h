#ifndef LORA_H_
#define LORA_H_

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
bool sendLora(void);
void sendLoraSetTargetPosition(void);
void sendLoraReset(void);

#endif /* LORA_H_ */
