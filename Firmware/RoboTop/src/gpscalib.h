#ifndef GPSCALIB_H_
#define GPSCALIB_H_
#include "main.h"

// GPS-based Fourier compass calibration, see doc/GPS_FOURIER_CALIBRATION_SPEC.md.
//
// Drives the buoy along eight straight legs, compares the heading it was told to steer with the
// bearing GPS says it actually travelled, and turns the residual into a new 8-point interpolation
// table for the Sub's Fourier correction. Hard-iron, soft-iron, gyro bias and the filter settings
// are left alone.

// Called every pass of the main loop. Does nothing unless status is GPS_FOURIER_CALIBRATE.
void handleGpsFourierCalibration(RoboStruct *cal);

// Fed by handleSerialData() with every STORE_INTERPOLATION_TABLE frame the Sub sends back.
void gpsCalibTableReply(const RoboStruct *in);

// True while a run is in progress - used for the button light and the dashboard.
bool gpsCalibActive(void);

// One-line human readable progress, e.g. "LEG 3/8 180 deg - running 62m". Empty when idle.
const char *gpsCalibProgress(void);

#endif /* GPSCALIB_H_ */
