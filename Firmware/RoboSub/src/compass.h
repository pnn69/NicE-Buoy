#ifndef COMPASS_H_
#define COMPASS_H_

#include <Arduino.h>
#include <RoboCompute.h>

extern QueueHandle_t compass;
extern QueueHandle_t compassIn;

template <typename T>
struct vector_t
{
    T x, y, z;
};

// Stores min and max magnetometer values from calibration
extern vector_t<float> m_max;
extern vector_t<float> m_min;
extern vector_t<float> icm_max;
extern vector_t<float> icm_min;

bool InitCompass(void);
void initcompassQueue(void);
bool CalibrateCompass(void);
float GetHeading(void);
float GetHeadingRaw(void);
float CompassAverage(float in);
void CompassTask(void *arg);
int linMagCalib(int *corr);

#endif /* COMPASS_H_ */