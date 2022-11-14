#ifndef FLYWHEEL)H
#define FLYWHEEL_H

#include "vex.h"
#include "pidf/pidf.h"
#include "helpers/rate-limiter.h"



extern bool flyWheelOn;
extern float flyWheelPower;
extern float flyWheelErrorPower;

extern float flyWheelPotentialRPM[3];
extern float flyWheelDesiredRPM;

extern void FlyWheelSwap();
extern void SetFlyWheelPower(int setting);



//Flywheel thread
extern int FlyWheelThread();


#endif