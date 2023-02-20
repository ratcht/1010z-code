#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "vex.h"
#include "subsystems/chassis-info.h"
#include "helpers/calc-funcs.h"
#include "pidf/pidf.h"
#include "subsystems/motion-profiling/motion-profile.h"


extern void SpinDrive(float right, float left);

//----New Odom-----
inline float arcRadiusL, arcRadiusR, arcRadius;
inline float arcRadiusF, arcRadiusB;
inline float offset, xOffset, yOffset;
inline float offsetH, offsetP;
inline float rPolar, thetaPolar;

inline float horzWheelPos, paraWheelPos;
inline float horzWheelTrack, paraWheelTrack;

inline float prevHVal, prevPVal; //inches

inline float prevOrientation = 0;
inline float absOrientation = 0;
inline float deltaOrientation = 0;

inline float deltaHT = 0, deltaPT = 0; //Tracking wheel distance travelled since cycle
inline float totalDeltaHT = 0, totalDeltaPT = 0; //Tracking wheel total change since reset


inline float h; // The hypotenuse of the triangle formed by the middle of the robot on the starting position and ending position and the middle of the circle it travels around
inline float i; // Half on the angle that I've traveled
inline float h2; // The same as h but using the back instead of the side wheels
inline float r, r2; // The radius of the circle the robot travel's around with the right side of the robot
inline float sinI;

inline bool enableOdom = true;
//---Funcs----
void getWheelVals();
void odom();
void odomTracking();
void initOdom();




inline float kMotor[4] = {0.2, 0.0, 0.0, 0.25}; //Starting Vals (No goal being lifted)
inline float kInertial[4] = {0.08, 0.001, 0.1, 0.0}; 
inline float fixedHeading = 0;
//{0.08, 0.001, 0.1, 0.0}
//{0.15, 0.001, 0.1, 0.0}

inline float desiredMotorVal = 0;
inline float lateralLMotorPower = 0;
inline float lateralRMotorPower = 0;

inline bool reversed= false;


inline PIDF driveVals(kMotor);
inline PIDF turnVals(kInertial);


int DriverPIDF();
int OdomThread();
int TurnOnlyPIDF();
int Stopper();

void GoToPoint(MotionProfile pProfile, float error, float _timeOut);
void TurnToAngle (float finalHeading, float error, float _timeOut);





inline bool enableDriverPID = true;
inline bool enableTurnPID = false;
inline bool resetDriveSensors = false;
inline bool enableDrive = false;
inline bool enableTurn = false;


//Run Motion Profile
inline float desiredHeading = 0;
inline bool resetPID = false;
inline bool resetBrainTimer = false;
inline Path targetPath;
inline float fromStart;
inline float elapsedTime = 0;
inline float offFromTarget;
inline Point* finalPoint;
inline float fPower = 0;
inline float targVel = 0;

inline float autoAimSpinValue= 0;
inline bool redGoalSelected = true;
inline Point targetGoal, targetPoint;
inline float errorRange = 0;
inline bool controlFlyWheel = false, controlIntake = false;
inline bool completed = false;
inline float timeOut;

#endif