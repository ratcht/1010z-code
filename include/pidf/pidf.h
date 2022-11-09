#ifndef PIDF_H
#define PIDF_H

#include "vex.h"
#include <cmath>
#include "helpers/calc-funcs.h"
#include "helpers/rate-limiter.h"
#include "subsystems/chassis-info.h"
#include "subsystems/drivetrain.h"



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

    PIDF(double k[4]);

    void changePID(double k[4]);
    
    void updatePIDValsSingle(double actual);

    void updatePIDValsPoint(Point pos);

    double calculateErrorPower();

    double calculateErrorXPower();

    double calculateErrorYPower();

    double calculateForwardPower(double targetVal);

    void resetPID();

};


inline double kLMotor[3] = {0.028, 0.0, 0.001}; //Starting Vals (No goal being lifted)
inline double kRMotor[3] = {0.028, 0.0, 0.001}; //Starting Vals (No goal being lifted)
inline double kInertial[3] = {0.06, 0.001, 0.0001}; //Distance sensor vals

inline double desiredMotorVal = 0;


int driverOnlyPID();

inline PIDF driveLVals(kLMotor);
inline PIDF driveRVals(kRMotor);
inline PIDF turnVals(kInertial);

inline bool enableDriverPID = true;
inline bool resetDriveSensors = false;







#endif