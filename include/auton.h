#ifndef AUTON_H
#define AUTON_H

#include "vex.h"
#include "subsystems/drivetrain.h"
#include "motion-profile.h"
#include "subsystems/pistons.h"
#include "subsystems/flywheel.h"
#include "subsystems/intake.h"
#include "subsystems/pure-pursuit.h"

#include "helpers/rate-limiter.h"
#include "pidf/pidf.h"
#include "helpers/calc-funcs.h"

int brainPrint();

extern void LeftAuto();
extern void LeftAWP();
extern void RightAuto();
extern void BetterRightAuto();
extern void SkillsAuto();
extern void Left();
extern void PureTest();



#endif