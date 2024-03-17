#ifndef COMPASS_H_
#define COMPASS_H_

// void CompassTask(void *arg);
bool InitCompass(void);
bool CalibrateCompass(void);
void callibratCompassOfest(int magCorrection);
void callibratCompassOfest(void);
float GetHeading(void);
float GetHeadingRaw(void);
float CompassAverage(float);
void GpsAverage(double *lat, double *lon);
#endif /* COMPASS_H_ */