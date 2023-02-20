#include "pidf/pidf.h"


PIDF::PIDF(float k[4]) {
  kP = k[0];
  kI = k[1];
  kD = k[2];
  kF = k[3];
}

void PIDF::changePID(float k[4]) {
  kP = k[0];
  kI = k[1];
  kD = k[2];
  kF = k[3];

}
    
void PIDF::updatePIDF(float actual) {

  error = desiredValue - actual;
      
  //Derivative
  derivative = error - prevError;

  //Integral
  totalError += error;

  //Derivative --> set prevError
  prevError = error;
}

float PIDF::calculateErrorPower(){
  return (error * kP) + (totalError * kI) + (derivative * kD);
}

float PIDF::calculateForwardPower(float targetVal){
  return (targetVal * kF);
}

void PIDF::resetPID() {
  error = 0, errorX = 0, errorY = 0; //SensorValue - DesiredValue : Position
  prevError = 0; //Position 20ms ago
  derivative = 0; //error - prevError : Speed
  totalError = 0;
}


