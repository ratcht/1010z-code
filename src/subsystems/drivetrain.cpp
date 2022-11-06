#include "subsystems/drivetrain.h"

void SpinDrive(double right, double left) {
  //SPIN BASE (Values in RPM)
  //Values are not adjusted to motor values
  
  
  LeftDrive.spin(forward, left/gearRatio, velocityUnits::rpm);
  LeftTop.spin(forward, left/gearRatio, velocityUnits::rpm);
  RightDrive.spin(forward, right/gearRatio, velocityUnits::rpm);
  RightTop.spin(forward, right/gearRatio, velocityUnits::rpm);


}