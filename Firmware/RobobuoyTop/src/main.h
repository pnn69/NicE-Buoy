#ifndef MAIN_H_
#define MAIN_H_
#include <RoboCodeDecode.h>
#include "../../RobobuoyDependency\RobobuoyMsg.h"
#include "../../RobobuoyDependency\RobobuoyDefaults.h"
#include "../../RobobuoyDependency\RobobuoyVersion.h"

struct BuoyDataType
{
    unsigned int status = IDLE;
};

extern RoboStruct roboData;

#endif /* MAIN_H_ */