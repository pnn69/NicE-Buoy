#ifndef LORA_H_
#define LORA_H_

/*
    MSG_ID (127 msg possible)
    [Designator][Reciever][MSG_ID][lengt_msg][msg][rssi_reciever][snr_reciever]
    msb msg_ID is answer or request. 1 means request. zerro answer from divice
    rssi and snr are 1 and -1 at request (No rssi known)

*/
#define POSITION 1
#define BATTERY_LEVEL 2
#define ANCHER_POSITION 3
#define GET_DIR_DISTANSE_ANCHER_POSITION 4
#define SET_ANCHER_POSITION 5
#define SET_CURREND_POSITION_AS_ANCHER_POSITION 6
#define DOC_POSITION 7
#define SET_DOC_POSITION 8
#define BUOY_ID 9
#define SET_BUOY_ID 10
#define SET_SAIL_DIR_SPEED 11
#define SET_SAIL_SPEED 12
#define SET_SAIL_DIR 13
#define SET_SBpwr_BBpwr 14
#define BUOY_MODE 15

#define SAIL_TO_DOC_POSITION 20

#define SYSTEM_STASTUS 90

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

#endif /* LORA_H_ */
