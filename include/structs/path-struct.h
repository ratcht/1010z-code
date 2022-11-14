#ifndef PATH-STRUCT_H
#define PATH-STRUCT_H

#include <vector>
#include <cmath>
#include <deque>
#include "structs/point-struct.h"

/*

..Units..

Lengths : Inches
Angles : Radians

*/


struct Path {
  //deque is used here instead of vector as it makes it easy to push points to the front
  //point --> a vector of [x,y]
  std::deque<Point> points;

  //returns a point
  Point getPoint(float t) {
    return points.at(t);
  }
  
  Point* getPointP(float t) {
    return &points.at(t);
  }

  Point* getPointinTime (float t){
    if (t > (points.size()-1)*0.1) return getPointinTime((points.size()-1) * 0.1);
    
    return getPointP(t/0.1);
  }

  //adds a point
  void addPoint(Point point){
    points.push_back(point);
  }

  void addPointVector(std::vector<float> vPoint){
    Point point(vPoint);
    points.push_back(point);
  }
};


struct Segment {
  std::vector<float> a, b, c, d;
};

#endif