#ifndef LORA_H_
#define LORA_H_

/*
    MSG_ID (127 msg possible)
    [Designator][Reciever][MSG_ID][lengt_msg][msg][rssi_reciever][snr_reciever]
    msb msg_ID is answer or request. 1 means request. zerro answer from divice
    rssi and snr are 1 and -1 at request (No rssi known)

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
void sendLoraPos(int id, double lat, double lon);
void sendLoraAnchorPos(bool);
void sendLoraDirDistanceTarget(unsigned long tgdir, unsigned long tgdistance);
void sendLoraDirDistanceSbSpeedBbSpeedTarget(unsigned long tgdir, unsigned long tgdistance, int sp, int sb, int bb, int heading);
bool InitLora(void);
void sendLora(void);
int polLora(void);

#endif /* LORA_H_ */
