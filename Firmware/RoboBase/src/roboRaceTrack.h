#ifndef ROBORACTRACK_H_
#define ROBORACTRACK_H_

bool AddDataToBuoyBase(RoboStruct dataIn, RoboStruct buoyPara[3]);
RoboStruct GetDataFromBuoyBase(uint64_t id,RoboStruct buoyPara[3]);

#endif /* ROBORACTRACK_H_ */
