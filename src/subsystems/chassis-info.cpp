#include "subsystems/chassis-info.h"

double wheelDiameter = 2.75; //wheel diameter 
double hTCentre = 0; // Left Tracking Wheel distance to centre: Sl
double pTCentre = 1.5; // Back Tracking Wheel distance to centre: Ss
double trackWidth = 12; // Track Width

double gearRatio = 3/4; //36 to 48
 
//_motionprofile 
//Robot: 
//72 ins^-1 = 12v

//double maxVel = 50; //inches per second


//double maxAccel = 20; // inches per second^2

//Odom FinalPos
Point finalPosition({ 0,0 });


//Fixed Values
Point AWPStart({13.07, 122.5});
Point Goal({135.2,135.6});


//-------------DRIVE SETTINGS-------------------
double turnImportance = 0.1;