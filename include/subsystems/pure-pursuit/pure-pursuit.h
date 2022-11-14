#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include "pidf/pidf.h"
#include "structs/vector-struct.h"
#include "helpers/calc-funcs.h"
#include "helpers/rate-limiter.h"
#include <vex.h>
#include <cmath>
#include <algorithm>


//------VARS--------
inline Path desiredPath;
inline Path finalPath;
inline Point lookaheadPoint;
inline Point* pclosePoint;
inline float curvature;
inline float angularVel;


//------PURE PURSUIT LOOP VARIABLES-------
inline float pathSize;
inline float shortestDistance;
inline float prevIndex;
inline float closeIndex;

inline float aCurvatureSlope;
inline float bCurvatureSlope;
inline float cCurvatureSlope;
inline float relativeX;
inline float side;
inline float crossProduct;
inline float signedCurvature;

inline limiter l;


//-----CALC VELOCITY--------

inline float targetVel;
inline float targetRW, targetLW; //target right/left wheel velocities


//-----Path Numbers---------

inline std::vector<Path> paths;


//Functions
extern void initPure();
extern void findClosestPoint();
extern float calcFractionalT(Vector* d, Vector* f, float r);
extern void findLookaheadPoint();


#endif