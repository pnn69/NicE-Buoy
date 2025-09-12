#ifndef COMPASS_H_
#define COMPASS_H_

extern QueueHandle_t compass;
extern QueueHandle_t compassIn;

extern double magHard[3];
extern double magSoft[3][3];

void calibrateMagneticNorth(void);
void InitCompass(void);
void initcompassQueue(void);
void calibrateNorthCompas(void);
void calibrateParametersCompas(void);
bool CalibrateCompass(void);
double GetHeadingRaw(void);
double CompassAverage(double);
double GetHeadingAvg(void);
void CompassTask(void *arg);
#endif /* COMPASS_H_ */