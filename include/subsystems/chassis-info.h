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
extern double wheelDiameter; //wheel diameter 
extern double hTCentre; // Left Tracking Wheel distance to centre: Sl
extern double pTCentre; // Back Tracking Wheel distance to centre: Ss

extern double trackWidth; // Track Width
extern double gearRatio; //base gearRatio

//_motionprofile 
//extern double maxVel;
//extern double maxAccel;

extern Point finalPosition;

extern Point AWPStart;

extern Point Goal;


//----------------DRIVE SETTINGS------------------
extern double turnImportance;

#endif