#ifndef COMPASS_H_
#define COMPASS_H_

extern QueueHandle_t compass;
extern QueueHandle_t compassIn;

void calibrateMagneticNorth(void);
bool InitCompass(void);
void initcompassQueue(void);
void calibrateNorthCompas(void);
void calibrateParametersCompas(void);
bool CalibrateCompass(void);
double GetHeading(void);
double GetHeadingRaw(void);
double GetHeadingRaw(int x, int y, int z);
double CompassAverage(double);
double GetHeadingAvg(void);
void CompassTask(void *arg);
#endif /* COMPASS_H_ */