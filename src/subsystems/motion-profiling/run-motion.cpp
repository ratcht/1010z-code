#include "pidf/pidf.h"

int DriverPIDF(){
  Brain.Timer.reset();
  driveVals.resetPID();
  turnVals.resetPID();
  limiter lTurn;

  while(enableDriverPID) {

    //Reset values switch
    if (resetDriveSensors) {
      resetDriveSensors = false;
      //reset motors
    }

    if (resetPID) {
      resetPID = false;
      turnVals.resetPID();
    }

    if (resetBrainTimer) {
      resetBrainTimer = false;
      Brain.Timer.reset();
    }


    //1. get elapsed time
    elapsedTime = roundOneDP(Brain.Timer.value());


    //2. get desiredPoint
    Point* desiredPoint = desiredPath->getPointinTime(elapsedTime);

    //3. set Desired Point
    driveVals.desiredPoint = desiredPoint;


    //4. pass current XY values
    driveVals.updatePIDValsPoint(finalPosition);

    //5. Get error motor power
    double pidP = driveVals.calculateErrorPower();
    
    
    //get forward power
    double TargVel = desiredPoint->targetVelocity;

    double fPower = driveVals.calculateForwardPower(TargVel);

    //===============================================================
        


    //===============================================================
    //-------------TURN PID-------------//

    double actualHeading = toDeg(getInertialReading());
    turnVals.desiredValue = desiredHeading;

    //inertial sensor switch --> inertial must be turned off when using vision
    turnVals.updatePIDValsSingle(actualHeading); //passes inertial sensor val
    double turnMotorPower = turnVals.calculateErrorPower(); 



    //===============================================================

    //Set up error stoppage
    offFromTarget = getDistanceP(&finalPosition, finalPoint);

    SpinDrive(fPower + pidP + turnMotorPower, fPower + pidP - turnMotorPower);


    vex::task::sleep(20);
  }

  return 1;
}