#ifndef ROBORACTRACK_H_
#define ROBORACTRACK_H_

bool AddDataToBuoyBase(RoboStruct dataIn);
RoboStruct GetDataFromBuoyBase(uint64_t id);
void ComputeTrackPosition(RoboStruct buoyPara[3]);

#endif /* ROBORACTRACK_H_ */
