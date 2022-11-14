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
    
    void updatePIDValsSingle(float actual);

    void updatePIDValsPoint(Point pos);

    float calculateErrorPower();

    float calculateErrorXPower();

    float calculateErrorYPower();

    float calculateForwardPower(float targetVal);

    void resetPID();

};


inline float kLMotor[4] = {0.012, 0.0, 0.001, 0.0}; //Starting Vals (No goal being lifted)
inline float kRMotor[4] = {0.012, 0.0, 0.001, 0.0}; //Starting Vals (No goal being lifted)
inline float kInertial[4] = {0.08, 0.001, 0.0001, 0.0}; //Distance sensor vals
inline float kFlyWheel[4] = {0.08, 0.001, 0.0001, 0.0}; //Distance sensor vals


inline float desiredMotorVal = 0;
inline float lateralLMotorPower = 0;
inline float lateralRMotorPower = 0;


int driverOnlyPID();

inline PIDF driveLVals(kLMotor);
inline PIDF driveRVals(kRMotor);
inline PIDF turnVals(kInertial);
inline PIDF flyWheelVals(kFlyWheel);


inline bool enableDriverPID = true;
inline bool enableTurnPID = false;
inline bool resetDriveSensors = false;
inline bool disableDrive = false;

//Run Motion Profile
inline float kMotor[4] = {0.012, 0.0, 0.001, 0.0}; //Starting Vals (No goal being lifted)
inline float desiredHeading = 0;
inline bool resetPID = false;
inline bool resetBrainTimer = false;
inline Path* desiredPath;
inline float elapsedTime = 0;
inline PIDF driveVals(kMotor);
inline float offFromTarget;
inline Point* finalPoint;





#endif