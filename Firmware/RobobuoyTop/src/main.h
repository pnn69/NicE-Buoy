#ifndef MAIN_H_
#define MAIN_H_
#include <RoboCompute.h>
#include <RoboTone.h>
#include "../../RobobuoyDependency\RobobuoyMsg.h"
#include "../../RobobuoyDependency\RobobuoyDefaults.h"
#include "../../RobobuoyDependency\RobobuoyVersion.h"

#define MAXSTRINGLENG 150
#define BUFLENMHRG 20 // 60 samples

struct extBuoyParameters
{
    double mac = 0;                    // id of the buoy is the macaddres of the top unit
    unsigned int status = 0;           // status
    double lat, lon, mDir, wDir, sStd; // dif parameters
    int thrusterPwr;                   // power to thrusters 0-100%
    int psBatt, ptBatt;                // Battery percentaqge 0-100% from sub and top
};

struct RoboWindStruct
{
    double winddir[3 + BUFLENMHRG]; // winddir[0]=avarage winddir[1]=pionter winddir[2]=standarddeviation winddir[3..BUFLENWINDSPEED + 3)]=data
};

#endif /* MAIN_H_ */