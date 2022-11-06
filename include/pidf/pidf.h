#include "vex.h"
#include <cmath>
#include "structs/path-struct.h"

using namespace vex;

class PIDF {       
  public:     

    //Settings    
    double kP;  
    double kI;
    double kD;
    double kF;

    //Calculations
    double error, errorX, errorY; //SensorValue - DesiredValue : Position
    double prevError = 0; //Position 20ms ago
    double derivative; //error - prevError : Speed
    double totalError = 0;

    //Auto Control
    Point desiredPoint;
    double desiredValue;

    PIDF(double k[4]) {
      kP = k[0];
      kI = k[1];
      kD = k[2];
      kF = k[3];
    }

    void changePID(double k[4]) {
      kP = k[0];
      kI = k[1];
      kD = k[2];
      kF = k[3];

    }
    
    void updatePIDValsSingle(double actual) {

      error = desiredValue - actual;
      
      //Derivative
      derivative = error - prevError;

      //Integral
      totalError += error;

      //Derivative --> set prevError
      prevError = error;
    }

    void updatePIDValsPoint(Point pos) {

      errorX = desiredPoint.x - pos.x;
      errorY = desiredPoint.y - pos.y; 
      
      

      // //Derivative
      // derivative = error - prevError;

      // //Integral
      // totalError += error;

      // //Derivative --> set prevError
      // prevError = error;
    }

    double calculateErrorPower(){
      return (error * kP) + (totalError * kI) + (derivative * kD);
    }

    double calculateErrorXPower(){
      return (errorX * kP) + (totalError * kI) + (derivative * kD);
    }

    double calculateErrorYPower(){
      return (errorY * kP) + (totalError * kI) + (derivative * kD);
    }

    double calculateForwardPower(double targetVal){
      return (targetVal * kF);
    }

    void resetPID() {
      error = 0, errorX = 0, errorY = 0; //SensorValue - DesiredValue : Position
      prevError = 0; //Position 20ms ago
      derivative = 0; //error - prevError : Speed
      totalError = 0;
    }

};