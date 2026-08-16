#ifndef MAIN_H_
#define MAIN_H_
#include "esp_log.h"
#include <RoboCompute.h>
#include <RoboTone.h>
#include "../../RoboDependency\RobobuoyVersion.h"
#include "topwifi.h"

// Implemented in main.cpp, which owns the serial-link state to our own Sub.
bool subSerialAlive(void);

#endif /* MAIN_H_ */