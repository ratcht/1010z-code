#ifndef FLYWHEEL_H
#define FLYWHEEL_H

#include "vex.h"
#include "pidf/pidf.h"
#include "helpers/rate-limiter.h"



extern bool flyWheelOn;
extern float flyWheelPower;
extern float flyWheelErrorPower;
inline float flyWheelDesiredPower;

extern float flyWheelPotentialRPM[3];
extern float flyWheelDesiredRPM;

extern void FlyWheelSwap();
extern void FlyWheelIncrease();
extern void FlyWheelDecrease();
extern void SetFlyWheelPower(float setting);
//inline float kFlyWheel[4] = {0.08, 0.001, 0.0001, 0.0}; //Distance sensor vals

extern PIDF flyWheelVals;




//Flywheel thread
extern int FlyWheelThread();


#endif