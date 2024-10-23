#ifndef MAIN_H_
#define MAIN_H_
#include <RoboCodeDecode.h>
#include <RoboCalc.h>
#include "../../RobobuoyDependency\RobobuoyMsg.h"
#include "../../RobobuoyDependency\RobobuoyDefaults.h"
#include "../../RobobuoyDependency\RobobuoyVersion.h"

#define MAXSTRINGLENG 150

struct extBuoyParameters
{
    double mac = 0;                    // id of the buoy is the macaddres of the top unit
    unsigned int status = 0;           // status
    double lat, lon, mDir, wDir, sStd; // dif parameters
    int thrusterPwr;                   // power to thrusters 0-100%
    int psBatt, ptBatt;                // Battery percentaqge 0-100% from sub and top
};

#endif /* MAIN_H_ */