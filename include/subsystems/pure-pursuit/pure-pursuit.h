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


inline float lookaheadDistance = 5;

//-----CALC VELOCITY--------

inline float targetVel;
inline float targetRW, targetLW; //target right/left wheel velocities


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

extern int RunPure();
extern bool isPureActive = false;

inline float offFromLast = INT8_MAX;

#endif