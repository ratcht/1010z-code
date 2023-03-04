#include "subsystems/motion-profiling/motion-profile.h"




//------generate profile---------
void MotionProfile::determineTopSpeed() {
  distanceFromPoint = getDistance(startPoint, targetPoint);
  //V attainable = sqrt(acceleration*distance)
  topReachableSpeed = sqrt(maxAccel * distanceFromPoint);
  attainableVelocity = std::min(topReachableSpeed, maxVel);
}


void MotionProfile::determineTimes() {
  //t1 = t3
  //d1 = d3
  //tTotal = t1+t2+t3

  distance1 = (pow(attainableVelocity, 2) / (2 * maxAccel));
  t1 = roundOneDP(attainableVelocity / maxAccel);
  
  if (topReachableSpeed > maxVel) {
      distance2 = fabs(distanceFromPoint - (2 * distance1));
      t2 = roundOneDP(distance2 / attainableVelocity);
  } else {
      distance2 = 0;
      t2 = 0;
  }

  t2cul = t1 + t2;
  t3cul = t1 + t2 + t1;
}


void MotionProfile::generatePointsinPath() {   //generate path with correct number of points desPath

  //calc diff
  theta = std::atan2(targetPoint.x - startPoint.x, targetPoint.y - startPoint.y);
  numPoints = round(t3cul / 0.1);
  float distanceFromStart = 0;

  float vel = 0, prevVel = 0, accel = 0;
  float xDiff = 0, yDiff = 0, displacement = 0;
  float prevX = startPoint.x, prevY = startPoint.y;

  for (float i = 0; i <= numPoints; i++) {
    //add point

    //Point to add
    Point pointToAdd;
    float t = i * 0.1;  //convert i to time value (increments of 10ms)
    pointToAdd.setTime(t);

    t -= 0.000000001;  //subtract small amount from t

    if (t > 0) {
      //if t is not 0 --> set previous values
      prevVel = (desPath.getPoint(i - 1).targetVelocity);
      prevX = (desPath.getPoint(i - 1).x);
      prevY = (desPath.getPoint(i - 1).y);
    }

    if (t <= t1 && t >0) {
      //1st part --> acceleration
      vel = prevVel + (maxAccel * 0.1);

      accel = maxAccel;
    } else if (t <= t2cul && t > t1) {
      //2nd part (optional) --> constant
      vel = prevVel;

      accel = 0;
    } else if (t > t2cul && t <= t3cul) {
      //3rd part --> deacceleration
      vel = prevVel - (maxAccel * 0.1);

      accel = -maxAccel;
    } 

    //calculate displacement based on previous speed and acceleration
    displacement = (prevVel * 0.1) + (0.5 * accel * 0.1 * 0.1);
    distanceFromStart += displacement;


    xDiff = displacement * sin(theta);
    yDiff = displacement * cos(theta);
  
    pointToAdd.setCoordinates({ prevX + xDiff, prevY + yDiff });
    pointToAdd.setTargetVelocity(vel);
    pointToAdd.setDistance(distanceFromStart);

    desPath.addPoint(pointToAdd);

  }

}


MotionProfile::MotionProfile(Point targetPoint, Point startPoint, float maxVel, float maxAccel) {
  this -> targetPoint = targetPoint;
  this -> startPoint = startPoint;
  this -> maxVel = maxVel;
  this -> maxAccel = maxAccel;


  determineTopSpeed();
  determineTimes();
  generatePointsinPath();
  
}





MotionProfile::MotionProfile() {}




