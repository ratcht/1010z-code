#include "subsystems/flywheel.h"

bool flyWheelOn = false;
float flyWheelErrorPower = 0;
float flyWheelPower = 0;
float flyWheelPotentialRPM[3] = {2250, 2630, 2400};
float flyWheelDesiredRPM = flyWheelPotentialRPM[0];

 

void SetFlyWheelPower(float setting){
  flyWheelDesiredPower = setting;
}

void FlyWheelIncrease(){
  flyWheelDesiredPower = 10;
}

void FlyWheelDecrease(){
  flyWheelDesiredPower = 9.5;
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
      flyWheelPower = flyWheelLimit.rateLimiter(flyWheelPower, 18); //limit the acceleration of the flywheel
      if(flyWheelPower !=flyWheelDesiredPower) goto run;
      // if( FlyWheel.velocity(rpm)*5 <= 1900  ){
      //   flyWheelPower = 11;
      // } 

      //enable pid once flywheel is at 9 volts
      goto run;

    }
    else{
      //turn off flywheel
      flyWheelPower = 0;
      flyWheelErrorPower = 0;
      flyWheelPower = flyWheelLimit.rateLimiter(flyWheelPower, 3); //limit the deacceleration of the flywheel
      flyWheelDesiredRPM = 0;
      goto run;
    }

    run:
      FlyWheel.spin(fwd, flyWheelPower, voltageUnits::volt);
      vex::task::sleep(20);
  }
  return 1;
}