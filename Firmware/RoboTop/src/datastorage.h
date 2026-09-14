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
void CompassCalibrationFactorsFloat(float *MaxX, float *MaxY, float *MaxZ, float *MinX, float *MinY, float *MinZ, bool get);
void memDockPos(RoboStruct *buoy, bool get);
void memDockApproach(RoboStruct *buoy, bool get);
// Whether the automatic thruster cleaning is armed. Top-owned, like the dock approach above, and
// carried in SETUPDATA so every front end can read and change it - see cleanEnabled in
// RoboCompute.h.
void memCleanEnabled(RoboStruct *buoy, bool get);

#endif /* DATASTORAGE_H_ */
