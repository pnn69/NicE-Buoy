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
// expect_leg guards against a retried press capturing the wrong leg; -1 means "whatever is next".
int  cal8Set(int expect_leg = -1);
bool cal8Save(void);         // commits offset and table together, or nothing
extern bool cal8_active;

// The heading reference lock. A calibration run in progress claims it, and while it is held nothing
// may move compassOffset - that offset is the domain the correction table is indexed by, so moving
// it mid-run puts the measurements taken before and after in different frames. See compass.cpp.
void cal8Lock(bool on);
bool cal8RefLocked(void);
extern bool cal8_ref_locked;
extern int cal8_next;        // 0..7 while running, 8 when every direction is captured
extern float cal8_captured[8];

// The one door every eight point table goes through. Forces entry 0 to zero, folding the rotation
// into compassOffset so north stays true with the correction switched off, writes both to NVS, and
// re-enables the correction. Returns false if the stored table is not usable (not in increasing
// order) - it is stored anyway, so the caller can report the truth rather than silently leaving the
// buoy uncorrected. See the block comment in compass.cpp.
bool storeInterpolationTable(const float *eight);

#endif /* COMPASS_H_ */