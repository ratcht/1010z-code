#include "subsystems/drivetrain.h"

void SpinDrive(float right, float left) {
  //SPIN BASE (Values in RPM)
  //Values are not adjusted to motor values
  
  
  // LeftDrive.spin(forward, left/gearRatio, velocityUnits::rpm);
  // LeftTop.spin(forward, left/gearRatio, velocityUnits::rpm);
  // RightDrive.spin(forward, right/gearRatio, velocityUnits::rpm);
  // RightTop.spin(forward, right/gearRatio, velocityUnits::rpm);
  LeftDrive.spin(forward, left, voltageUnits::volt);
  LeftTop.spin(forward, left, voltageUnits::volt);
  RightDrive.spin(forward, right, voltageUnits::volt);
  RightTop.spin(forward, right, voltageUnits::volt);

}

void SpinDriveVel(float right, float left) {
  //SPIN BASE (Values in RPM)
  //Values are not adjusted to motor values
  
  
  // LeftDrive.spin(forward, left/gearRatio, velocityUnits::rpm);
  // LeftTop.spin(forward, left/gearRatio, velocityUnits::rpm);
  // RightDrive.spin(forward, right/gearRatio, velocityUnits::rpm);
  // RightTop.spin(forward, right/gearRatio, velocityUnits::rpm);
  right = (40*right/M_PI)/(wheelDiameter/2);
  left = (40*left/M_PI)/(wheelDiameter/2);

  LeftDrive.spin(forward, left, velocityUnits::rpm);
  LeftTop.spin(forward, left, velocityUnits::rpm);
  RightDrive.spin(forward, right, velocityUnits::rpm);
  RightTop.spin(forward, right, velocityUnits::rpm);

}

void initOdom() {
  prevOrientation = 0;
  absOrientation = 0;
  deltaOrientation = 0;
  Point finalPosition({0,0});

  deltaHT = 0, deltaPT = 0; //Tracking wheel distance travelled since cycle
  totalDeltaHT = 0, totalDeltaPT = 0;; //Tracking wheel total change since reset

  EncoderHorz.setPosition(0, deg);
  EncoderPara.setPosition(0, deg);

  arcRadiusL = 0, arcRadiusR = 0, arcRadius = 0;
  offset = 0, xOffset = 0, yOffset = 0;

  horzWheelPos = 0, paraWheelPos = 0;
  horzWheelTrack = 0, paraWheelTrack = 0;

  prevHVal = 0, prevPVal = 0; //inches
}


void getWheelVals() {
  //Motor Wheel Vals
  horzWheelPos = toRads(EncoderHorz.position(degrees));
  paraWheelPos = toRads(EncoderPara.position(degrees));

  horzWheelTrack = horzWheelPos * (wheelDiameter/2) * (-1);
  paraWheelTrack = paraWheelPos * (wheelDiameter/2) * (-1);

  deltaHT = horzWheelTrack - prevHVal;
  deltaPT = paraWheelTrack - prevPVal;

  prevHVal = horzWheelTrack;
  prevPVal = paraWheelTrack;

  totalDeltaHT += deltaHT;
  totalDeltaPT += deltaPT;
}

void odomTracking() {
  getWheelVals();
  
	absOrientation = getInertialReading();
  deltaOrientation = absOrientation - prevOrientation; // deltaorientation The angle that I've traveled
  prevOrientation = absOrientation;

	if (deltaOrientation) {
		r = deltaPT / deltaOrientation; // The radius of the circle the robot travel's around with the right side of the robot
		i = deltaOrientation / 2.0;
	  sinI = sin(i);
		h = (r * sinI) * 2.0;

		r2 = deltaHT / deltaOrientation; // The radius of the circle the robot travel's around with the back of the robot
		h2 = (r2 * sinI) * 2.0;
	
  } else {
		h = deltaPT;
		i = 0;

		h2 = deltaHT;
	}
	float p = i + absOrientation; // The global ending angle of the robot
	float cosP = cos(p);
	float sinP = sin(p);

	// Update the global position
	finalPosition.y += h * cosP;
	finalPosition.x += h * sinP;

	finalPosition.y += h2 * (-sinP); // -sin(x) = sin(-x)
	finalPosition.x += h2 * cosP; // cos(x) = cos(-x)
}


int DriverPIDF(){
  Brain.Timer.reset();
  driveVals.resetPID();
  limiter lTurn;

  while(enableDriverPID) {


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
    Point* pDesiredPoint = targetPath.getPointinTime(elapsedTime);
    //set desiredvalue in pid
    driveVals.desiredValue = pDesiredPoint->distanceFromStart;

    targVel = pDesiredPoint->targetVelocity;
    fPower = driveVals.calculateForwardPower(targVel);

    fromStart = getDistance(targetPath.getPoint(0), finalPosition);

    //update pidf
    driveVals.updatePIDF(fromStart);
    float pidP = driveVals.calculateErrorPower();

    offFromTarget = getDistance(finalPosition, targetPath.points.at(targetPath.points.size()-1));

    //turning
    float turnError = fixedHeading - toDeg(getInertialReading());
    if(reversed){
      turnError *=(-1);
    }
    float pidT = turnError*0.8;
    float RPower = fPower-pidT+pidP;
    float LPower= fPower+pidT+pidP;

    if(reversed){
      RPower *= (-1);
      LPower *= (-1);
    }
    SpinDrive(RPower,LPower );

    if(completed == true){
      break;
    }
    vex::task::sleep(20);
  }

  return 1;
}

int TurnOnlyPIDF(){

  Brain.Timer.reset();

  while(enableTurnPID) {
    
    if (resetBrainTimer) {
      resetBrainTimer = false;
      Brain.Timer.reset();
    }


    //1. get elapsed time
    elapsedTime = roundOneDP(Brain.Timer.value());

    //===============================================================
    //-------------TURN PID-------------//

    float actualHeading = toDeg(getInertialReading());
  

    

    //inertial sensor switch --> inertial must be turned off when using vision
    turnVals.updatePIDF(actualHeading); //passes inertial sensor val
    offFromTarget =abs(turnVals.error);

    //===============================================================
    if(enableTurnPID != true){
      break;
    }
    SpinDrive(-turnVals.calculateErrorPower(),turnVals.calculateErrorPower());    
    
    vex::task::sleep(20);
  }
  return 1;
}


int Stopper() { 
  while (enableDriverPID || enableTurnPID) {
    
    if (offFromTarget <= errorRange || timeOut <= elapsedTime) {
      wait(100, msec);
      turnVals.resetPID();
      driveVals.resetPID();
      enableDriverPID = false;
      fPower = 0;
      SpinDrive(0,0);
      enableTurnPID = false;
      completed = true;
    }



    wait (20, msec);
  }
  return 1;
}


int OdomThread() {
  while (enableOdom){
    odomTracking();
    wait(20, msec);
  }
  return 1;
}

//==========================
//Go to Points
//==========================

void GoToPoint(MotionProfile pProfile, float error, float _timeOut) {
  
  float finalHeading = getHeadingBetweenPoints(finalPosition, pProfile.targetPoint);
  //set global variables to arguments
  desiredHeading = finalHeading;
  targetPath = pProfile.desPath;


  timeOut = _timeOut;

  errorRange = error;

  //enable pid
  enableDriverPID = true;

  completed = false;
  vex::task driverPIDF(DriverPIDF);
  vex::task stopper(Stopper);

  while(!completed) {
    wait(20, msec);
  }
  
}

void TurnToAngle (float finalHeading, float error, float _timeOut) {

  vex::task turnOnlyPIDF(TurnOnlyPIDF);

  //take motion profile
  turnVals.desiredValue = finalHeading;
  enableTurn = true;
  enableTurnPID = true;
  completed = false;

//TIMEOUT = ERROR
  errorRange = error;

  timeOut = _timeOut;

  //vex::task stopper(Stopper);
  
}
