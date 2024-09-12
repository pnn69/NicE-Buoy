#ifndef ROBOCALC_H
#define ROBOCALC_H
#include <Arduino.h>

String addCRCToString(String input);
bool verifyCRC(String input);


#endif
