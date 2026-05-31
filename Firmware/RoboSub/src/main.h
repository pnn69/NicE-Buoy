#ifndef MAIN_H_
#define MAIN_H_

#include <RoboCompute.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

extern SemaphoreHandle_t mainDataMutex;
extern float global_imon_v;

#include "../../RoboDependency\RobobuoyVersion.h"

#endif /* MAIN_H_ */