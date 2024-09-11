#ifndef COMPASS_H_
#define COMPASS_H_

// void CompassTask(void *arg);
bool InitCompass(void);
bool CalibrateCompass(void);
void calibrateMagneticNorth();
float GetHeading(void);
float GetHeadingRaw(void);
float CompassAverage(float);
float GetHeadingAvg(void);
void GpsAverage(double *lat, double *lon);
#endif /* COMPASS_H_ */