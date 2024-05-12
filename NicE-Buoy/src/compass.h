#ifndef COMPASS_H_
#define COMPASS_H_

// void CompassTask(void *arg);
bool InitCompass(void);
bool CalibrateCompass(void);
void callibratCompassOfest(int magCorrection);
float GetHeading(void);
float GetHeadingRaw(void);
float CompassAverage(float);
void GpsAverage(double *lat, double *lon);
int linMagCalib(int *corr);
#endif /* COMPASS_H_ */