#include "subsystems/pure-pursuit.h"

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
  shortestDistance = finalPath->points.at(finalPath->points.size()-1).distanceFromStart;
  for (float i = 0; i < finalPath->points.size(); i++) {
    float robotDistance = getDistanceP(&finalPosition, finalPath->getPointP(i));

    if (robotDistance < shortestDistance) {
      shortestDistance = robotDistance;
      closeIndex = i;
    }
  }

  pclosePoint = finalPath->getPointP(closeIndex);
  prevIndex = closeIndex;
}

float calcIntersection(Vector* d, Vector* f, float r) {
  float a = d->Dot(d);
  float b = 2 * f->Dot(d);
  float c = f->Dot(f) - r * r;

  float discriminant = b * b - (4 * a * c);
  if (discriminant < 0) return -1;

  // ray didn't totally miss sphere,
  // so there is a solution to
  // the equation.

  discriminant = sqrt(discriminant);

  // either solution may be on or off the ray so need to test both
  // t1 is always the smaller value, because BOTH discriminant and
  // a are nonnegative.
  float t1 = (-b - discriminant) / (2 * a);
  float t2 = (-b + discriminant) / (2 * a);

  // 3x HIT cases:
  //          -o->             --|-->  |            |  --|->
  // Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit), 

  // 3x MISS cases:
  //       ->  o                     o ->              | -> |
  // FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

  if (t1 >= 0 && t1 <= 1) {
      // t1 is the intersection, and it's closer than t2
      // (since t1 uses -b - discriminant)
      // Impale, Poke
      return t1;
  }

  // here t1 didn't intersect so we are either started
  // inside the sphere or completely past it
  if (t2 >= 0 && t2 <= 1) {
      // ExitWound
      return t2;
  }

  // no intn: FallShort, Past, CompletelyInside
  return -1;

}


//Calculate tVal + index
float calcFractionalT(Path* path, Point* robotPos, double lookahead, double startingLargest) {

  float runningT = 0;

  //Loop through all points in path
  for (float i = 0; i < path->points.size() - 2; i++) {
    //d: vector from line segment start to end
    //f: vector from centre of circle to start
    Vector d(path->getPointP(i), path->getPointP(i + 1));
    Vector f(robotPos, path->getPointP(i));
    float calcT = calcIntersection(&d, &f, lookaheadDistance);
    if (calcT == -1) continue;
    calcT += i;
    runningT = (calcT > runningT) ? calcT : runningT;
  }

  return runningT;
}

void findLookaheadPoint() {

  float calcT = calcFractionalT(finalPath, &finalPosition, lookaheadDistance, closeIndex);
  float index = round(calcT);

  lookaheadPoint = finalPath->getPointP(index);
}



float findCurvature() {
  //find curvature of current robot arc
  //curvature = 2x/L^2
  //where x is horizontal distance to the point and L is the lookahead

  float aCurvatureSlope = -(atan(absOrientation));
  float bCurvatureSlope = 1;
  float cCurvatureSlope = (atan(absOrientation) * finalPosition.x) - finalPosition.y;

  float relativeX = abs(aCurvatureSlope * lookaheadPoint->x + bCurvatureSlope * lookaheadPoint->y + cCurvatureSlope) / sqrt(pow(aCurvatureSlope, 2) + pow(bCurvatureSlope, 2));

  //get signed curvature
  //side = signum(cross product) = signum((By − Ry) * (Lx − Rx) − (Bx − Rx) * (Ly − Ry))
  //Bx = Rx + cos(robot angle)
  //By = Ry + sin(robot angle)


  //sign = signum(cross product)
  float crossProduct = (cos(absOrientation) * (lookaheadPoint->x - finalPosition.x)) - (sin(absOrientation) * (lookaheadPoint->y - finalPosition.y));

  //signum ternary operator
  float side = (crossProduct > 0) ? 1 : ((crossProduct < 0) ? -1 : 0);

  float signedCurvature = ((2 * relativeX) / pow(lookaheadDistance, 2)) * side;

  return signedCurvature;

  
}


void calculateWheelVelocities() {
  //calculate wheel velocities
  /*
  V = target robot velocity
  L = target left wheel’s speed
  R = target right wheel’s speed
  C = curvature of arc
  W = angular velocity of robot
  T = track width

  V = (L + R)/2
  W = (L − R)/T
  V = W/C
  */
  targetVel = pclosePoint->targetVelocity;

  targetLW = targetVel * (2 + (signedCurvature * trackWidth)) / 2;
  targetRW = targetVel * (2 - (signedCurvature * trackWidth)) / 2;

}

int RunPure(){
  while(isPureActive){
    findClosestPoint();
    findLookaheadPoint();
    signedCurvature = findCurvature();
    calculateWheelVelocities();

    float kMotorZ[4] = {0.2, 0.0, 0.0, 0.3}; //Starting Vals (No goal being lifted)
    leftDrive.changePID(kMotorZ);
    rightDrive.changePID(kMotorZ);

    float powerFL = leftDrive.calculateForwardPower(targetLW);
    float powerFR = rightDrive.calculateForwardPower(targetRW);


    SpinDrive(powerFR, powerFL);

    //calculate error
    offFromLast = getDistanceP(&finalPosition, &finalPath->points.at(finalPath->points.size()-1));

    wait(20, msec);
  }

  return 1;
}

void FollowPath(Path* followedPath, float timeOut, float error){
    prevIndex = 0;

  Brain.Timer.reset();
  isPureActive = true;
  offFromLast = INT8_MAX;
  finalPath = followedPath;
  vex::task runPure(RunPure);
  // elapsedTime = roundOneDP(Brain.Timer.value());
  // while (elapsedTime <= timeOut || offFromLast >= error){
  //   wait(20, msec);
  // }  // elapsedTime = roundOneDP(Brain.Timer.value());
  // while (elapsedTime <= timeOut || offFromLast >= error){
  //   wait(20, msec);
  // }
 // isPureActive = false;
}

