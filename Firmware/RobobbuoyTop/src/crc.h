#ifndef CRC_H_
#define CRC_H_
#include <Arduino.h>

String addCRCToString(String input);
bool verifyCRC(String input);

#endif /* CRC_H_ */
