#ifndef COMPASS_H_
#define COMPASS_H_

#include <Arduino.h>
#include <RoboCompute.h>

extern QueueHandle_t compass;
extern QueueHandle_t compassIn;

bool InitCompass(void);
void initcompassQueue(void);
bool CalibrateCompass(void);
float GetHeading(void);
float GetHeadingRaw(void);
float CompassAverage(float in);
void CompassTask(void *arg);
int linMagCalib(int *corr);

#endif /* COMPASS_H_ */