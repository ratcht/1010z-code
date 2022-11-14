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
  paraWheelTrack = paraWheelPos * (wheelDiameter/2);

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