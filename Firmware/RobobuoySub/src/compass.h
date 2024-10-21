#ifndef COMPASS_H_
#define COMPASS_H_

extern QueueHandle_t compass;

void initGpsQueue(void);
bool InitCompass(void);
bool CalibrateCompass(void);
void calibrateMagneticNorth();
double GetHeading(void);
double GetHeadingRaw(void);
double CompassAverage(float);
double GetHeadingAvg(void);
void CompassTask(void *arg);
#endif /* COMPASS_H_ */