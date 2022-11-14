#include "subsystems/flywheel.h"

bool flyWheelOn = false;
float flyWheelPower = 12;
float flyWheelErrorPower = 0;

float flyWheelPotentialRPM[3] = {2250, 2630, 2400};
float flyWheelDesiredRPM = flyWheelPotentialRPM[0];
float flyWheelDesiredPower = 12;


void SetFlyWheelPower(int setting){
  if (setting == 0) {
    flyWheelDesiredRPM = 2250;
    flyWheelDesiredPower = 9;
  } 
  else if (setting == 1) {
    flyWheelDesiredRPM = 2400;
    flyWheelDesiredPower = 9.3;
  }
  else if (setting == 2) {
    flyWheelDesiredRPM = 2630;
    flyWheelDesiredPower = 10;
  }
  

}

void FlyWheelSwap() {
  flyWheelOn = !flyWheelOn;
}

//Flywheel thread
int FlyWheelThread() {
  limiter flyWheelLimit;
  while (1) {
    if(flyWheelOn) {
      //spin flywheel
      flyWheelPower = flyWheelDesiredPower;
      flyWheelPower = flyWheelLimit.rateLimiter(flyWheelPower, 14); //limit the acceleration of the flywheel

      //enable pid once flywheel is at 9 volts
      if (flyWheelPower != flyWheelDesiredPower) goto run;

      flyWheelVals.desiredValue = flyWheelDesiredRPM;  //desired flywheel RPM
      flyWheelVals.updatePIDValsSingle(FlyWheel.velocity(rpm)*5); //calculates error values

      if (flyWheelVals.error >= 100) flyWheelErrorPower = flyWheelVals.calculateErrorPower(); //set error power
      else flyWheelErrorPower = 0;

      goto run;

    }
    else{
      //turn off flywheel
      flyWheelPower = 0;
      flyWheelPower = flyWheelLimit.rateLimiter(flyWheelPower, 3); //limit the deacceleration of the flywheel
      flyWheelDesiredRPM = 0;

      goto run;
    }

    run:
      FlyWheel.spin(fwd, flyWheelPower + flyWheelErrorPower, voltageUnits::volt);
      vex::task::sleep(20);
  }
  return 1;
}