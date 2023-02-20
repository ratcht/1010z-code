#ifndef CALC-FUNCS_H
#define CALC-FUNCS_H

#include "vex.h"
#include <vector>
#include <cmath>
#include <deque>
#include <string>
#include "/structs/path-struct.h"
#include "/subsystems/chassis-info.h"

inline float fieldAngleOffset = 0;

//clamp function
inline float clamp(float d, float min, float max) {
  const float t = d < min ? min : d;
  return t > max ? max : t;
}

inline float toRads(float deg) {
  return deg * M_PI/180;
}

inline float toDeg(float rads) {
  return rads * 180/M_PI;
}

inline float restrictDomainRad(float rads) {
  if (-M_PI < rads && rads <= M_PI) return rads;
  if (M_PI < rads) return rads = -(rads - M_PI);
  

}

inline float hypotenuse(float a, float b) {
  return sqrt((a*a) + (b*b));
}

inline float getInertialReading() {
  float in2 = toRads(Inertial2.rotation());
 // float in3 = Inertial3.rotation() * M_PI/180;
 //Add Field Offset
  in2 += fieldAngleOffset;
  return in2;
}

inline float IStoRPM(float v) {
  //w = v/r
  //[rpm] = 2pi/(w*60)
  //Therefore, [rpm] = 2pi/( (v/r) * 60 )
  return (2*M_PI/((v/(0.5*wheelDiameter))*60));
}


//takes a directional vector
inline float getHeadingAngle(float x, float y){
  float refAngle = atan(fabs(x)/fabs(y));
  float thetaHeading;
  //quadrant 1:  x > 0 & y > 0    
    if (x > 0 && y > 0) {
      thetaHeading = refAngle;
    }

    //quadrant 2:  x > 0 & y < 0    
    else if (x < 0 && y > 0) {
      thetaHeading = 2*M_PI - refAngle;
    }

    //quadrant 3:  x1 > x2 & y1 > y2    
    else if (x < 0 && y < 0) {
      thetaHeading = M_PI + refAngle;
    }

    //quadrant 4:  x1 > x2 & y2 > y1    
    else if (x > 0 && y < 0) {
      thetaHeading = M_PI - refAngle;
    }

    //quadrant 5: straight up y2 > y1 && x2 == x1
    else if (x == 0 && y > 0) {
      thetaHeading = 0;
    }
    
    //quadrant 6: straight right y2 == y1 && x2 > x1
    else if (x > 0 && y == 0) {
      thetaHeading = M_PI/2;
    }

    //quadrant 7: straight down y1 > y2 && x2 = x1
    else if (x == 0 && y < 0 ) {
      thetaHeading = M_PI;
    }

    //quadrant 8: straight left x1 > x2 && y2 = y1
    else if (x < 0 && y == 0) {
      thetaHeading = 3 * M_PI / 2;
    }

    else {
      thetaHeading = 0;
    }

  return thetaHeading;
}

inline float getHeadingBetweenPoints(Point x, Point y) {
  float xd = y.x - x.x;
  float yd = y.y - x.y;
  return getHeadingAngle(xd, yd);
}

inline float roundOneDP(float num) {
  float x10 = num * 10;
  x10 = round(x10);
  x10 = x10 / 10;
  return x10;
}

inline Point parseString(std::string split) {
  std::string delimiter = ",";
  std::string s1 = split.substr(0, split.find(delimiter)); // x
  std::string s2 = s1.substr(0, s1.find(delimiter)); // y
  std::string s3 = s2.substr(0, s2.find(delimiter)); // curvature
  std::string s4 = s3.substr(0, s3.find(delimiter)); // targetVel
  std::string s5 = s4.substr(0, s4.find(delimiter)); // distanceFromStart

  
  Point p({static_cast<float>(atof(s1.c_str())), static_cast<float>(atof(s2.c_str())) });

  // p.setCurvature(atof(s3.c_str()));
  // p.setTargetVelocity(atof(s4.c_str()));
  // p.setDistance(atof(s5.c_str()));

  return p;
}

inline Point pointDAway(Point start, float dis, float theta) {
  float x = start.x + dis*sin(theta);
  float y = start.y + dis*cos(theta);

  return Point({x,y});
} 

#endif