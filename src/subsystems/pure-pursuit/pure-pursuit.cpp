#include "subsystems/pure-pursuit/pure-pursuit.h"

void initPure() {
  prevIndex = 0;
  shortestDistance = getDistance(finalPosition, finalPath->getPoint(finalPath->points.size()-1));
  closeIndex = finalPath->points.size()-1;
  l.reset();
  targetLW = 0;
  targetRW = 0;
}



//-------------------------------------------
//Follow path functions
//-------------------------------------------

void findClosestPoint() {
  //______Find closest point______
  //Start at prev point index +1

  //by default, set shortest distance to distance from last point
  //loop through all points ahead of prevClosestPoint
  for (float i = fmin(prevIndex + 1, finalPath->points.size()-1); i < finalPath->points.size(); i++) {   
    float robotDistance = getDistanceP(&finalPosition, finalPath->getPointP(i));

    if (robotDistance < shortestDistance) {
      shortestDistance = robotDistance;
      closeIndex = i;
    }
  }

  pclosePoint = finalPath->getPointP(closeIndex);
  prevIndex = closeIndex;
}


float calcFractionalT(Vector* d, Vector* f, float r) {
  float a = d->Dot(d) ;
  float b = 2*f->Dot(d) ;
  float c = f->Dot( f ) - r*r ;

  float discriminant = b*b-4*a*c;
  if( discriminant < 0 ) return -1;
  
  // ray didn't totally miss sphere,
  // so there is a solution to
  // the equation.
  
  discriminant = sqrt( discriminant );

  // either solution may be on or off the ray so need to test both
  // t1 is always the smaller value, because BOTH discriminant and
  // a are nonnegative.
  float t1 = (-b - discriminant)/(2*a);
  float t2 = (-b + discriminant)/(2*a);

  // 3x HIT cases:
  //          -o->             --|-->  |            |  --|->
  // Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit), 

  // 3x MISS cases:
  //       ->  o                     o ->              | -> |
  // FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)
  
  if( t1 >= 0 && t1 <= 1 ){
    // t1 is the intersection, and it's closer than t2
    // (since t1 uses -b - discriminant)
    // Impale, Poke
    return t1 ;
  }

  // here t1 didn't intersect so we are either started
  // inside the sphere or completely past it
  if( t2 >= 0 && t2 <= 1 ){
    // ExitWound
    return t2 ;
  }
  
  // no intn: FallShort, Past, CompletelyInside
  return -1 ;
  
}


void findLookaheadPoint() {
  float runningT = 0; 

  //Loop through all points in path
  for (float i = 0; i < finalPath->points.size()-2; i++) {
    //d: vector from line segment start to end
    //f: vector from centre of circle to start
    Vector d(finalPath->getPointP(i),finalPath->getPointP(i+1));
    Vector f(&finalPosition, finalPath->getPointP(i));
    float calcT = calcFractionalT(&d, &f, rLookAhead) + i;

    runningT = (calcT > runningT) ? calcT : runningT;
  }

}