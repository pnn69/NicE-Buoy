#ifndef COMPASS_H_
#define COMPASS_H_

void CompassTask(void* arg);
bool InitCompass(void);
float GetHeading(void);
float CompassAverage(float);

#endif /* COMPASS_H_ */