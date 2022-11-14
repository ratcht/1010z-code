#ifndef POINT-STRUCT_H
#define POINT-STRUCT_H

#include <vector>
#include <cmath>
#include <deque>

/*

..Units..

Lengths : Inches
Angles : Radians

*/

struct Point {
  float x, y;

  float distanceFromStart;
  float curvature;

  float globalHeading;
  float targetVelocity;

  float timeAtPoint;

  void setCoordinates (std::vector<float> points) {
    x = points.at(0);
    y = points.at(1);
  }

  void setDistance (float distanceFromStart){
    this -> distanceFromStart = distanceFromStart;
  }

  void setCurvature (float curvature){
    this -> curvature = curvature;
  }

  void setTargetVelocity (float targetVelocity){
    this -> targetVelocity = targetVelocity;
  }

  void setTime (float t) {
    this -> timeAtPoint = t;
  }

  Point(std::vector<float> point){
    x = point.at(0);
    y = point.at(1);
  }

  Point() {
    //empty constructor
  }

};

//return the magnitude of a vector: c^2 = a^2 + b^2
inline float getMagnitude(Point point) {
  return sqrt( pow(point.x, 2) + pow(point.y, 2) );
}


//calculates the distance between 2 points
inline float getDistance(Point p0, Point p1) {
  return getMagnitude(Point({p1.x - p0.x, p1.y - p0.y}));
}

//calculates the distance between 2 points
inline float getDistanceP(Point* p0, Point* p1) {
  return getMagnitude(Point({p1->x - p0->x, p1->y - p0->y}));
}

#endif