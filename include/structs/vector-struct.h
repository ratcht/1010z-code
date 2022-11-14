#ifndef VECTOR-STRUCT_H
#define VECTOR-STRUCT_H

#include <vector>
#include <cmath>
#include <deque>
#include "helpers/calc-funcs.h"
#include "structs/point-struct.h"


struct Vector {
  float magnitude;
  float thetaHeading;
  float refAngle;
  int quadrant;
  Point* startPoint;
  Point* endPoint;
  Point dirVector;

  float Dot(Vector* b) {
    return (b->dirVector.x*dirVector.x)+(b->dirVector.y*dirVector.y);
  }

  Vector(Point* p1, Point* p2) {
    //__Start & End__
    startPoint = p1;
    endPoint = p2;
    dirVector = Point({p2->x - p1->x, p2->y - p1->y});
    //__Vector magnitude
    magnitude = getDistanceP(p1, p2);

    thetaHeading = getHeadingAngle(dirVector.x, dirVector.y);

  }

};

#endif