#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include "pidf/pidf.h"
#include "structs/vector-struct.h"
#include "helpers/calc-funcs.h"
#include "helpers/rate-limiter.h"
#include "subsystems/chassis-info.h"
#include "subsystems/drivetrain.h"

#include <vex.h>
#include <cmath>
#include <algorithm>


//------VARS--------
inline Path* finalPath;
inline Path desPath1, desPath2;
inline Point* lookaheadPoint;
inline Point* pclosePoint;
inline float curvature;
inline float angularVel;


//------PURE PURSUIT LOOP VARIABLES-------
inline float pathSize;
inline float shortestDistance;
inline float prevIndex;
inline float closeIndex;

inline float signedCurvature;

inline limiter l;


inline float lookaheadDistance = 15;
//-----CALC VELOCITY--------

inline float targetVel;
inline float targetRW, targetLW; //target right/left wheel velocities
inline float kMotorP[4] = {0.2, 0.0, 0.0, 0.25}; //Starting Vals (No goal being lifted)
inline float maxSpeed = 30;

inline PIDF rightDrive(kMotorP), leftDrive(kMotorP);

//-----Path Numbers---------

inline std::vector<Path> paths;


//Functions
extern void initPure();
extern void findClosestPoint();
extern float calcIntersection(Vector* d, Vector* f, float r);
extern float calcFractionalT(Path* path, Point* robotPos, double lookahead, double startingLargest);
extern void findLookaheadPoint();
extern float findCurvature();
extern void calculateWheelVelocities();
extern void FollowPath(Path* followedPath, float timeOut, float error);

extern int RunPure();
inline bool isPureActive = false;

inline float offFromLast = INT8_MAX;

#endif