#ifndef INTAKE_H
#define INTAKE_H

#include "vex.h"
#include "helpers/rate-limiter.h"


extern float rollerSpeed;
extern float intakeSpeed;
extern bool intakeOn;
extern bool newTakeOn;
extern bool shootOn;
extern bool intakeReverse;


extern void IntakeSwap();
extern void IntakeReverseSwap();



extern void ShooterSwap();


//Intake thread
extern int IntakeThread();

#endif