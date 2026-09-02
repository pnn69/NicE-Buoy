#ifndef DATASTORAGE_H_
#define DATASTORAGE_H_

// Direction flags for the storage API below.
// NOTE: these deliberately do NOT reuse the names GET/SET. Those are msg_t enum constants
// (GET=1, SET=2) used for the protocol ack field, and defining them as macros here silently
// rewrote every ack assignment in this project to true/false -> ack=1/ack=0 instead of 1/2.
#define MEM_GET true
#define MEM_PUT false
#include <RoboCompute.h>

void initMemory(void);
void memDockPos(RoboStruct *buoy, bool get);
void memBuoyId(uint64_t *id, bool get);
void CompassCalibrationFactors(RoboStruct *buoy, bool get);
void CompasOffset(RoboStruct *buoy, bool get);
void computeParameters(RoboStruct *buoy, bool get);
void speedMaxMin(RoboStruct *buoy, bool get);
void pidSpeedParameters(RoboStruct *buoy, bool get);
void pidRudderParameters(RoboStruct *buoy, bool get);
void thrusterInversion(RoboStruct *buoy, bool get);
void thrusterSwap(RoboStruct *buoy, bool get);
void apParameters(String *ap, String *ww, bool get);
void hardIron(RoboStruct *buoy, bool get);
void softIron(RoboStruct *buoy, bool get);
void CompassCalibrationFactorsFloat(float *MaxX, float *MaxY, float *MaxZ, float *MinX, float *MinY, float *MinZ, bool get);
void CompassOffsetCorrection(double *offset, bool get);
void memBnoCalib(uint8_t *data, bool get);
void memIcmCalib(float *hi, float *si, bool get);
void memCompassAvg(int *avg, bool get);
void memCompassTrim(float *trim, bool *enabled, bool get);
void memPrDamping(float *damping, bool get);
void memDampingFactors(float *acc, float *gyro, float *mag, float *att, bool get);
void memInterpolationTable(float *angles, bool get);

// The ICM filtering mode the stored table was measured in. See the comment in datastorage.cpp:
// the mode is the table's input, so a table measured in one mode is meaningless in another.
void memInterpTableMode(int *mode, bool get);
void memInterpTableRev(int *rev, bool get);
void memMountLevel(float *pitch, float *roll, bool get);
void memEscNeutral(int *bb, int *sb, bool get);

#endif /* DATASTORAGE_H_ */
