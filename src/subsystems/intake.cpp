#include "subsystems/intake.h"

float rollerSpeed = 12;
float intakeSpeed = 12;

bool intakeOn = false;
bool shootOn = false;
bool intakeReverse = false;



void IntakeSwap() {
  intakeOn = !intakeOn;
  shootOn = false;
}

void ShooterSwap() {
  shootOn = !shootOn;
  intakeOn = false;
}

void StopAll() {
  shootOn = false;
  intakeOn = false;
}




//Intake thread
int IntakeThread() {
  while (1) {
    if(intakeOn){
      intakeSpeed = -12;
    } else if (shootOn) {
      intakeSpeed = 12;
    } else {
      intakeSpeed = 0;
    }
    
    Intake.spin(fwd, intakeSpeed, voltageUnits::volt);

    vex::task::sleep(20);
  }
  return 1;
}
