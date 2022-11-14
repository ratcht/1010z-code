#include "subsystems/intake.h"

float rollerSpeed = 12;
float intakeSpeed = 12;

bool intakeOn = false;
bool newTakeOn = false;
bool shootOn = false;
bool intakeReverse = false;



void IntakeSwap() {
  intakeOn = !intakeOn;
  shootOn = false;
  intakeReverse = false;

}

void IntakeReverseSwap() {
  intakeReverse = !intakeReverse;
}




void ShooterSwap() {
  shootOn = !shootOn;
  intakeOn = false;
  intakeReverse = false;

}



//Intake thread
int IntakeThread() {
  limiter intakelimit;
  while (1) {
    if(intakeOn && !shootOn){
      intakeSpeed = 12; 
      rollerSpeed = 12;
    }
    else if (shootOn && !intakeOn) {
      intakeSpeed = 0;
      rollerSpeed = -12;
    }
    else if(intakeReverse) {
      intakeSpeed = -12;
      rollerSpeed = -12;
    }
    else {
      intakeSpeed = 0;
      rollerSpeed = 0;
    }
    
    Intake.spin(fwd, intakeSpeed, voltageUnits::volt);

    vex::task::sleep(20);
  }
  return 1;
}
