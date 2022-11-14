#ifndef CHASSISINFO_H
#define CHASSISINFO_H

#include "structs/path-struct.h"

//--------__Parameters__--------//

/*
Units:

length : in
velocity : ms^-1
accel : ms^-2

*/

//_odom
extern float wheelDiameter; //wheel diameter 
extern float hTCentre; // Left Tracking Wheel distance to centre: Sl
extern float pTCentre; // Back Tracking Wheel distance to centre: Ss

extern float trackWidth; // Track Width
extern float gearRatio; //base gearRatio

//_motionprofile 
//extern float maxVel;
//extern float maxAccel;

extern Point finalPosition;

extern Point AWPStart;

extern Point Goal;


//----------------DRIVE SETTINGS------------------
extern float turnImportance;


extern float rLookAhead;

#endif