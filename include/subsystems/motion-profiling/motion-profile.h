#ifndef MOTIONPROFILE_H
#define MOTIONPROFILE_H

#include "subsystems/chassis-info.h"
#include "helpers/calc-funcs.h"
#include <cmath>



class MotionProfile {
  public:
    Point targetPoint;
    Point startPoint;
    float distanceFromPoint, distance1, distance2;
    float theta;
    float maxVel, maxAccel;
    float attainableVelocity, topReachableSpeed;
    float t1, t2, t2cul, t3cul;
    float numPoints;
    Path desPath;

    void determineTopSpeed();
    void determineTimes();
    void generatePointsinPath();

    
    MotionProfile(Point targetPoint, Point startPoint, float maxVel, float maxAccel);
    MotionProfile();

};



#endif