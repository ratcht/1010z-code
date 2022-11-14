#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "vex.h"
#include "subsystems/chassis-info.h"
#include "helpers/calc-funcs.h"


extern void SpinDrive(float right, float left);

//----New Odom-----
inline float arcRadiusL, arcRadiusR, arcRadius;
inline float arcRadiusF, arcRadiusB;
inline float offset, xOffset, yOffset;
inline float offsetH, offsetP;
inline float rPolar, thetaPolar;

inline float horzWheelPos, paraWheelPos;
inline float horzWheelTrack, paraWheelTrack;

inline float prevHVal, prevPVal; //inches

inline float prevOrientation = 0;
inline float absOrientation = 0;
inline float deltaOrientation = 0;

inline float deltaHT = 0, deltaPT = 0; //Tracking wheel distance travelled since cycle
inline float totalDeltaHT = 0, totalDeltaPT = 0; //Tracking wheel total change since reset


inline float h; // The hypotenuse of the triangle formed by the middle of the robot on the starting position and ending position and the middle of the circle it travels around
inline float i; // Half on the angle that I've traveled
inline float h2; // The same as h but using the back instead of the side wheels
inline float r, r2; // The radius of the circle the robot travel's around with the right side of the robot
inline float sinI;
//---Funcs----
void getWheelVals();
void odom();
void odomTracking();
void initOdom();



#endif