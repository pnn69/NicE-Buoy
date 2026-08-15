#ifndef DATASTORAGE_H_
#define DATASTORAGE_H_

// Direction flags for the storage API below.
// NOTE: these deliberately do NOT reuse the names GET/SET. Those are msg_t enum constants
// (GET=1, SET=2) used for the protocol ack field, and defining them as macros here silently
// rewrote every ack assignment in this project to true/false -> ack=1/ack=0 instead of 1/2.
#define MEM_GET true
#define MEM_PUT false

void initMemory(void);
void memBuoyId(int8_t *id, bool get);
void apParameters(String *ap, String *ww, bool get);
void CompassCallibrationFactorsFloat(float *MaxX, float *MaxY, float *MaxZ, float *MinX, float *MinY, float *MinZ, bool get);
void CompasOffset(RoboStruct *buoy, bool get);
void CompassOffsetCorrection(int *delta, bool get);
void MechanicalCorrection(double *correction, bool get);
void memDockPos(RoboStruct *buoy, bool get);
void memDockApproach(RoboStruct *buoy, bool get);
void thrusterInversion(RoboStruct *buoy, bool get);
void computeParameters(RoboStruct *buoy, bool get);
void pidSpeedParameters(RoboStruct *buoy, bool get);
void pidRudderParameters(RoboStruct *buoy, bool get);
void thrusterSwap(RoboStruct *buoy, bool get);
void defautls(RoboStruct *buoy);

#endif /* DATASTORAGE_H_ */
