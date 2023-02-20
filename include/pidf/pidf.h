#ifndef PIDF_H
#define PIDF_H

#include "vex.h"
#include <cmath>
#include "helpers/calc-funcs.h"
#include "helpers/rate-limiter.h"
#include "subsystems/chassis-info.h"



class PIDF {       
  public:     

    //Settings    
    float kP;  
    float kI;
    float kD;
    float kF;

    //Calculations
    float error, errorX, errorY; //SensorValue - DesiredValue : Position
    float prevError = 0; //Position 20ms ago
    float derivative; //error - prevError : Speed
    float totalError = 0;

    //Auto Control
    Point* desiredPoint;
    float desiredValue;

    PIDF(float k[4]);

    void changePID(float k[4]);
    
    void updatePIDF(float actual);

    float calculateErrorPower();

    float calculateForwardPower(float targetVal);

    void resetPID();

};








#endif