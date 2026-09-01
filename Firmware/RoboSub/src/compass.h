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

bool InitCompass(void);
void initcompassQueue(void);
bool CalibrateCompass(void);
float GetHeading(void);
float GetHeadingNoOffset(void);
float GetHeadingRaw(void);
float CompassAverage(float in);
void CompassTask(void *arg);
int linMagCalib(int *corr);

// Guided eight point compass calibration. See the block comment in compass.cpp: the rules live
// there once, and every interface drives the same session rather than repeating the arithmetic.
void cal8Begin(void);
void cal8Cancel(void);
int  cal8Set(void);          // captures the direction being asked for, returns its index or -1
bool cal8Save(void);         // commits offset and table together, or nothing
extern bool cal8_active;
extern int cal8_next;        // 0..7 while running, 8 when every direction is captured
extern float cal8_captured[8];

#endif /* COMPASS_H_ */