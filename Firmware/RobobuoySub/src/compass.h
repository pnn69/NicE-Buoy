#ifndef COMPASS_H_
#define COMPASS_H_

extern QueueHandle_t compass;

void initGpsQueue(void);
bool InitCompass(void);
bool CalibrateCompass(void);
void calibrateMagneticNorth();
float GetHeading(void);
float GetHeadingRaw(void);
float CompassAverage(float);
float GetHeadingAvg(void);
void CompassTask(void *arg);
#endif /* COMPASS_H_ */