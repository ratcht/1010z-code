#include "auton.h"

MotionProfile p1;
int brainPrint() {
  while (1) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print("inertial: %f",toDeg(getInertialReading()));
    Brain.Screen.newLine();
    Brain.Screen.print("Wheel Vals HP: %f, %f", EncoderHorz.position(degrees), EncoderPara.position(degrees) );
    Brain.Screen.newLine();
    Point t = pointDAway(Point({0,0}), -30, 0);
    Brain.Screen.print("Final Pos (%f, %f)", finalPosition.x, finalPosition.y );
    Brain.Screen.newLine();
    Brain.Screen.print(" offFrom targ %f", offFromTarget );
    Brain.Screen.newLine();
    Brain.Screen.print(" FlywheelPower %f", flyWheelPower );
    Brain.Screen.newLine();
        Brain.Screen.print(" FlywheelDesPower %f", flyWheelDesiredPower );
    Brain.Screen.newLine();
        Brain.Screen.print(" RPM %f", FlyWheel.velocity(rpm)*5 );
    Brain.Screen.newLine();
    // Brain.Screen.print("PVEL %f", p1.desPath.points.at(3).targetVelocity );
    // Brain.Screen.newLine();
    Brain.Screen.print(completed );
        Brain.Screen.newLine();
    Brain.Screen.print(enableTurnPID );
            Brain.Screen.newLine();
    Brain.Screen.print( FlyWheel.temperature(temperatureUnits::celsius));

    wait(20, msec);
  }

  return 0;
}


void LeftAuto() {
  vex::task runFlywheel(FlyWheelThread);
  vex::task runIntake(IntakeThread);
  enableOdom = true;
  vex::task odomThread(OdomThread);

  MotionProfile _p1(pointDAway(Point({0,0}), -6.3, 0), Point({0,0}), 100, 100);
  MotionProfile _p2(pointDAway(Point({0,-1.5}), 8, 0), Point({0,-1.5}), 30, 30);
  MotionProfile _p3(pointDAway(Point({0.2,1.5}), 50, 52), Point({0.2, 1.5}), 17, 20);

  vex::task BrainPrint(brainPrint);


  flyWheelDesiredPower = 2;
  flyWheelOn = true;

  //Motion1
  reversed = true;
  fixedHeading = 0;

  intakeOn = true;
  GoToPoint(_p1, 3, 1);
  wait(760,msec);
  intakeOn = false;

  //motion2
  reversed = false;
  GoToPoint(_p2, 3, 100);
  wait(100, msec);

  //motionshootTurn
  float k1[4] = {0.3, 0.001, 0.1, 0.0};
  turnVals.changePID(k1);
  enableTurnPID = true;
  completed = false;
  TurnToAngle(-12, 4, 30);
  wait(800, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(200, msec);

  //shoot
  shootOn = true;
  wait(300, msec);
  shootOn = false;
  wait(1700, msec);
  shootOn = true;
  wait(600, msec);

  //motion3 Turn
  float k2[4] = {0.13, 0.001, 0.1, 0.0};
  turnVals.changePID(k1);
  enableTurnPID = true;
  completed = false;
  TurnToAngle(57, 4, 30);
  wait(1300, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);

  //motion4 intake
  intakeOn=  true;
  IntakeEngaged();
  fixedHeading = 57;
  GoToPoint(_p3, 3, 100);

  //motion5 turn
  intakeOn = false;
  ShootEngaged();
  enableTurnPID = true;
  completed = false;
  float k3[4] = {0.1, 0.001, 0.1, 0.0};
  turnVals.changePID(k3);
  TurnToAngle(-35, 4, 30);

}

void LeftAWP() {
  vex::task runFlywheel(FlyWheelThread);
  vex::task runIntake(IntakeThread);
  enableOdom = true;
  vex::task odomThread(OdomThread);

  MotionProfile _p1(   pointDAway(Point({0,0}), -5, 0),    Point({0,0}),       40,      50);
  MotionProfile _p2(   pointDAway(Point({0,-1}), 5, 0),    Point({0,-1}),       40,      50);
 

  vex::task BrainPrint(brainPrint);
 
  //Motion1
  SetFlyWheelPower(9.6);
  flyWheelOn = true;
  reversed = true;
  intakeOn = true;
  fixedHeading = 0;
  GoToPoint(_p1, 3, 0.93);

  //Motion2
  IntakeEngaged();
  reversed = false;
  fixedHeading = 0;
  GoToPoint(_p2, 1, 0.6);

  float k1[4] = {0.15, 0.001, 0.1, 0.0};
  turnVals.changePID(k1);
  TurnToAngle(-50, 4, 30);
  wait(550, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);

  MotionProfile _p3(   pointDAway(finalPosition, 6.17, Inertial2.rotation()),    finalPosition,       40,      50);
  fixedHeading = Inertial2.rotation();
  GoToPoint(_p3, 1, 0.5);
    SpinDrive(0,0);

  wait(500, msec);

  reversed = true;
  MotionProfile _p4(   pointDAway(finalPosition, -6, -50),    finalPosition,       40,      50);
  fixedHeading = -50;
  GoToPoint(_p3, 1, 0.45);
  reversed = false;



  float k2[4] = {0.065, 0.001, 0.1, 0.0};
  ShootEngaged();
  turnVals.changePID(k2);
  TurnToAngle(44, 4, 30);
  wait(850, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);

  fixedHeading = Inertial2.rotation();
  MotionProfile _p5(   pointDAway(finalPosition, 65, Inertial2.rotation()),    finalPosition,       60,      60);
  intakeOn = false;
  GoToPoint(_p5, 1, 2);

  float k3[4] = {0.16, 0.0, 0.3, 0.0};
  intakeOn = false;
  turnVals.changePID(k3);
  TurnToAngle(-44, 4, 30);
  wait(800, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(100, msec);

  shootOn = true;
  wait(260, msec);
  shootOn = false;
  wait(1400, msec);

  shootOn = true;
  wait(250, msec);
  shootOn = false;
  wait(1500, msec);
  
  shootOn = true;
  wait(330, msec);

  shootOn = false;
  TurnToAngle(45, 4, 30);
  wait(800, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);

  fixedHeading = 45;
  MotionProfile _p6(   pointDAway(finalPosition, 62, 45),    finalPosition,       60,      60);
  intakeOn = false;
  GoToPoint(_p6, 1, 1.8);
  
  float k4[4] = {0.15, 0.001, 0.3, 0.0};
  turnVals.changePID(k4);
  TurnToAngle(-90, 4, 30);
  wait(600, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);

  fixedHeading = -90;
  shootOn = true;
  reversed = true;
  MotionProfile _p7(   pointDAway(finalPosition, -15, -90),    finalPosition,       60,      60);
  GoToPoint(_p7, 1, 1);
  
  reversed = false;
  MotionProfile _p8(   pointDAway(finalPosition, 5, -90),    finalPosition,       60,      60);
  GoToPoint(_p8, 1, 1);






  

}

void Left(){
  vex::task runFlywheel(FlyWheelThread);
  vex::task runIntake(IntakeThread);
  enableOdom = true;
  vex::task odomThread(OdomThread);

  MotionProfile _p1(   pointDAway(Point({0,0}), -5, 0),    Point({0,0}),       40,      50);
  MotionProfile _p2(   pointDAway(Point({0,-1}), 5, 0),    Point({0,-1}),       40,      50);
 

  vex::task BrainPrint(brainPrint);
 
  //Motion1
  SetFlyWheelPower(10.5);
  flyWheelOn = true;
  reversed = true;
  shootOn = true;
  fixedHeading = 0;
  GoToPoint(_p1, 3, 0.45);
  shootOn = false;


  //Motion2
  IntakeEngaged();
  reversed = false;
  fixedHeading = 0;
  GoToPoint(_p2, 1, 0.6);
  intakeOn = true;
  wait(600, msec);

  float k1[4] = {0.15, 0.001, 0.1, 0.0};
  turnVals.changePID(k1);
  TurnToAngle(-50, 4, 30);
  wait(550, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);

  MotionProfile _p3(   pointDAway(finalPosition, 6.17, Inertial2.rotation()),    finalPosition,       40,      50);
  fixedHeading = Inertial2.rotation();
  GoToPoint(_p3, 1, 0.5);
    SpinDrive(0,0);
  wait(500, msec);

  reversed = true;
  MotionProfile _p4(   pointDAway(finalPosition, -2, -50),    finalPosition,       40,      50);
  fixedHeading = -50;
  GoToPoint(_p3, 1, 0.45);
  reversed = false;


  intakeOn = false;
  float k2[4] = {0.15, 0.001, 0.1, 0.0};
  ShootEngaged();
  turnVals.changePID(k2);
  TurnToAngle(-9, 4, 30);
  wait(850, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);

  fixedHeading = Inertial2.rotation();

  wait(400, msec);
  ShootEngaged();
  wait(400, msec);
    shootOn = true;
  wait(260, msec);
  shootOn = false;
  wait(1200, msec);

  shootOn = true;
  wait(250, msec);
  shootOn = false;
  wait(1200, msec);
  
  shootOn = true;
  wait(330, msec);
  shootOn = false;
  wait(200, msec);
  

}

void BetterRightAuto(){
  vex::task runFlywheel(FlyWheelThread);
  vex::task runIntake(IntakeThread);
  enableOdom = true;
  vex::task odomThread(OdomThread);

  MotionProfile _p1(   pointDAway(Point({0,0}), 26, 0),    Point({0,0}),       40,      50);

  vex::task BrainPrint(brainPrint);

   
  //Motion1
  SetFlyWheelPower(10.5);
  flyWheelOn = true;
  IntakeEngaged();
  intakeOn = true;
  fixedHeading = 0;
  GoToPoint(_p1, 1, 1.3);
  wait(200, msec);
  
  float k1[4] = {0.15, 0.001, 0.1, 0.0};
  turnVals.changePID(k1);
  TurnToAngle(31, 4, 30);
  wait(550, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  intakeOn = false;
  ShootEngaged();
  wait(1700, msec);

  
  shootOn = true;
  wait(260, msec);
  shootOn = false;
  wait(1200, msec);

  shootOn = true;
  wait(250, msec);
  shootOn = false;
  wait(1200, msec);
  
  shootOn = true;
  wait(330, msec);
  shootOn = false;
  wait(200, msec);
  
  float k2[4] = {0.1, 0.001, 0.1, 0.0};
  turnVals.changePID(k2);
  TurnToAngle(-45, 4, 30);
  wait(850, msec);
  flyWheelOn = false;
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);

  reversed = true;
  fixedHeading = -44;
  MotionProfile _p2(   pointDAway(finalPosition, -30, -44),    finalPosition,       60,      60);

  GoToPoint(_p2, 3, 1.6);

  float k3[4] = {0.15, 0.001, 0.1, 0.0};
  turnVals.changePID(k3);
  TurnToAngle(0, 4, 30);
  wait(850, msec);
  flyWheelOn = false;
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);

  reversed = true;
  shootOn = true;
  fixedHeading = 0;
  MotionProfile _p3(   pointDAway(finalPosition, -10, 0),    finalPosition,       60,      60);

  GoToPoint(_p3, 3, 0.7);

  
  reversed = false;
  shootOn = true;
  fixedHeading = 0;
  MotionProfile _p4(   pointDAway(finalPosition, 5, 0),    finalPosition,       60,      60);

  GoToPoint(_p4, 3, 0.8);

}

void SkillsAuto(){
  vex::task runFlywheel(FlyWheelThread);
  vex::task runIntake(IntakeThread);
  enableOdom = true;
  vex::task odomThread(OdomThread);
  //Goto(profile, error, time)
  //MOTIONPROFILE(start point, end point, max vel, max accel)
  MotionProfile _p1(   pointDAway(Point({0,0}), -5, 0),    Point({0,0}),       40,      50);
  MotionProfile _p2(   pointDAway(Point({0,-1}), 5, 0),    Point({0,-1}),       40,      50);
  MotionProfile _p3(   pointDAway(Point({-0.65,2.89}), 18, -51.8),    Point({-0.65,2.89}),       40,      50);

  vex::task BrainPrint(brainPrint);
  wait(100, msec);

  //Motion1
  reversed = true;
  intakeOn = true;
  fixedHeading = 0;
  GoToPoint(_p1, 3, 0.5);

  //Motion2
  reversed = false;
  fixedHeading = 0;
  GoToPoint(_p2, 1, 0.3);

  //motionshootTurn
  IntakeEngaged();
  float k1[4] = {0.3, 0.001, 0.1, 0.0};
  turnVals.changePID(k1);
  TurnToAngle(-41.9, 4, 30);
  wait(400, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  
  //Motion3
   fixedHeading = -51.8;
   GoToPoint(_p3, 1, 1);
   wait(100, msec);

  float k2[4] = {0.04, 0.001, 0.1, 0.0};
  turnVals.changePID(k2);
  TurnToAngle(88, 4, 30);
  wait(850, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(100, msec);

  //Motion1
  reversed = true;
  intakeOn = true;
  fixedHeading = 87;
  MotionProfile _p4(   pointDAway(finalPosition, -15, 87),    finalPosition,       40,      50);
  GoToPoint(_p4, 3, 0.75);

  //Motion2
  ShootEngaged();
  reversed = false;
  SetFlyWheelPower(9.7);
  flyWheelOn = true;
  fixedHeading = 87;
  MotionProfile _p5(   pointDAway(finalPosition, 5, 87.6),    finalPosition,       40,      50);
  GoToPoint(_p5, 1, 0.3);
  wait(100,msec);




 //motion
  float k3[4] = {0.055, 0.001, 0.1, 0.0};
  intakeOn = false;
  turnVals.changePID(k3);
  TurnToAngle(0, 4, 30);
  wait(850, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(100, msec);

  //motion
  fixedHeading = 0;
  MotionProfile _p6(   pointDAway(finalPosition, 56, 0),    finalPosition,       50,      60);
  GoToPoint(_p6, 1, 2);
  wait(400,msec);

  float k4[4] = {0.54, 0.001, 0.1, 0.0};
  intakeOn = false;
  turnVals.changePID(k4);
  TurnToAngle(9.5, 4, 30);
  wait(450, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(100, msec);
  shootOn = true;
  wait(1300, msec);
  shootOn = false;

  //next motion
  reversed = true;
  intakeOn = true;
  fixedHeading = 7.6;
  IntakeEngaged();
  MotionProfile _p7(   pointDAway(finalPosition, -20, 7.6),    finalPosition,       50,      60);
  GoToPoint(_p7, 3, 0.45);
  reversed = false;
  flyWheelOn = false;

  //next
  float k5[4] = {0.06, 0.001, 0.1, 0.0};
  IntakeEngaged();
  intakeOn = true;
  turnVals.changePID(k5);
  TurnToAngle(118, 4, 30);
  wait(750, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);

  //motion
  fixedHeading = 120;
  MotionProfile _p8(   pointDAway(finalPosition, 23, 120),    finalPosition,       40,      40);
  GoToPoint(_p8, 1, 1.4);
  completed = true;
  wait(200,msec);
  completed = false;

    //next
  SetFlyWheelPower(9.2);
  flyWheelOn = true;
  turnVals.changePID(k5);
  TurnToAngle(35.5, 4, 30);
  wait(750, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(100, msec);

  //motion
  fixedHeading = 37;
  MotionProfile _p9(   pointDAway(finalPosition, 33, 39),   finalPosition,       45,      40);
  GoToPoint(_p9, 1, 1.7);
  wait(600,msec);

  //next
  float k10[4] = {0.23, 0.001, 0.1, 0.0};
  turnVals.changePID(k10);
  intakeOn = false;
  ShootEngaged();
  TurnToAngle(-37, 4, 30);
  wait(750, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(300, msec);

  shootOn = true;
  wait(220, msec);
  shootOn = false;
  wait(1150, msec);
  shootOn = true;
  wait(200, msec);
  shootOn = false;
  wait(1150, msec);
  shootOn = true;
  wait(450, msec);
  shootOn = false;
  IntakeEngaged();
  SetFlyWheelPower(9.8);
    

  turnVals.changePID(k1);
  TurnToAngle(30, 4, 30);
  wait(620, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);

  reversed = true;
  fixedHeading = Inertial2.rotation();
  MotionProfile _p10(   pointDAway(finalPosition, -8, Inertial2.rotation()),   finalPosition,       60,      60);
  GoToPoint(_p10, 1, 0.4);
  reversed = false;

  intakeOn = true;
  IntakeEngaged();
  float k6[4] = {0.3, 0.001, 0.1, 0.0};
  turnVals.changePID(k6);
  TurnToAngle(0, 4, 30);
  wait(700, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(100, msec);

  fixedHeading = 0;
  MotionProfile _p11(   pointDAway(finalPosition, 47.5, 0),   finalPosition,       20,      20);
  GoToPoint(_p11, 1, 4);
  wait(600, msec);

  float k7[4] = {0.1, 0.001, 0.1, 0.0};
  intakeOn = false;
  ShootEngaged();
  turnVals.changePID(k7);
  TurnToAngle(-85, 4, 30);
  wait(950, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(300, msec);

  shootOn = true;
  wait(1000, msec);
  shootOn = false;
  wait(100, msec);

  float k8[4] = {0.3, 0.001, 0.1, 0.0};
  intakeOn = false;
  ShootEngaged();
  turnVals.changePID(k8);
  TurnToAngle(-55, 4, 30);
  wait(750, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(300, msec); 

  reversed = true;
  fixedHeading = Inertial2.rotation();
  MotionProfile _p12(   pointDAway(finalPosition, -10, Inertial2.rotation()),   finalPosition,       70,      70);
  GoToPoint(_p12, 1, 0.6);
  reversed = false;

  turnVals.changePID(k10);
  TurnToAngle(-90, 4, 30);
  wait(650, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(400, msec);
  
  reversed = true;
  fixedHeading = Inertial2.rotation();
  MotionProfile _p13(   pointDAway(finalPosition, -39, Inertial2.rotation()),   finalPosition,       60,      60);
  GoToPoint(_p13, 1, 1.5);
  reversed = false;
  wait(500, msec);

  float k9[4] = {0.1, 0.001, 0.1, 0.0};
  turnVals.changePID(k9);
  TurnToAngle(-179, 4, 30);
  wait(650, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(400, msec);

  reversed = true;
  shootOn = true;
  fixedHeading = -180;
  MotionProfile _p14(   pointDAway(finalPosition, -14, -180),   finalPosition,       70,      70);
  GoToPoint(_p14, 1, 0.8);
  reversed = false;


  MotionProfile _p15(   pointDAway(finalPosition, 8, -180),   finalPosition,       70,      70);
  GoToPoint(_p15, 1, 0.4);
  wait(200, msec);

  float k11[4] = {0.2, 0.001, 0.1, 0.0};
  shootOn = false;
  IntakeEngaged();
  turnVals.changePID(k11);
  TurnToAngle(-235, 4, 30);
  wait(650, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  intakeOn = true;
  wait(200, msec);
  
  fixedHeading = Inertial2.rotation();
  MotionProfile _p16(   pointDAway(finalPosition, 15, Inertial2.rotation()),   finalPosition,       60,      60);
  GoToPoint(_p16, 1, 1.1);

  float k12[4] = {0.07, 0.001, 0.1, 0.0};
  shootOn = false;
  IntakeEngaged();
  turnVals.changePID(k12);
  TurnToAngle(-90, 4, 30);
  wait(950, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  intakeOn = true;
  wait(200, msec);

    fixedHeading = -90;
    reversed = true;
  MotionProfile _p17(   pointDAway(finalPosition, -17, -90),   finalPosition,       60,      60);
  GoToPoint(_p17, 1, 1.5);
  reversed = false;

    MotionProfile _p18(   pointDAway(finalPosition, 12, -90),   finalPosition,       60,      60);
  GoToPoint(_p18, 1, 0.9);

    float k13[4] = {0.11, 0.001, 0.1, 0.0};
  turnVals.changePID(k13);
  TurnToAngle(-135, 4, 30);
  wait(850, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(800, msec);

  EndgameFire();
  //   //motion
  //   reversed = true;
  // fixedHeading = Inertial2.rotation();
  // MotionProfile _p11(   pointDAway(finalPosition, -22, Inertial2.rotation()),   finalPosition,       45,      40);
  // GoToPoint(_p11, 1, 1.7);
  // wait(200,msec);
  // reversed = false;

 }

 void RightAuto(){
  vex::task runFlywheel(FlyWheelThread);
  vex::task runIntake(IntakeThread);
  enableOdom = true;
  vex::task odomThread(OdomThread);

  MotionProfile _p1(pointDAway(Point({0,0}), -20, 0), Point({0,0}), 40, 50);
  MotionProfile _p2(pointDAway(Point({0,-20.6}), -11, 0), Point({0,-20.6}), 25, 50);
  MotionProfile _p3(pointDAway(Point({-6.97,-24.55}), 8, 52), Point({-6.97,-24.45}), 40, 40);
  MotionProfile _p4(pointDAway(Point({0.5, -22.27}), 85, 41), Point({0.5, -22.27}), 30, 30);

  vex::task BrainPrint(brainPrint);


  wait(200, msec);
  SetFlyWheelPower(10.7);
  flyWheelOn = true;

  //Motion1
  reversed = true;
  fixedHeading = 0;
  GoToPoint(_p1, 3, 2);

  //motionshootTurn
  float k1[4] = {0.07, 0.001, 0.1, 0.0};
  turnVals.changePID(k1);
  enableTurnPID = true;
  completed = false;
  TurnToAngle(90, 4, 30);
  wait(800, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(300, msec);

  //Motion2
  intakeOn = true;
  fixedHeading = 90;
  GoToPoint(_p2, 3, 1.18);
  intakeOn = false;

  reversed = false;
  GoToPoint(_p3, 3, 1.4);

  //motionshootTurn
  float k2[4] = {0.5, 0.001, 0.1, 0.0};
  turnVals.changePID(k2);
  enableTurnPID = true;
  completed = false;
  TurnToAngle(97, 4, 30);
  wait(400, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(300, msec);

  shootOn = true;
  wait(220, msec);
  shootOn = false;

  wait(2300, msec);
  shootOn = true;
  wait(1000, msec);
  shootOn = false;
    //motionshootTurn
  float k3[4] = {0.1, 0.001, 0.1, 0.0};
  turnVals.changePID(k3);
  enableTurnPID = true;
  completed = false;
  TurnToAngle(41, 4, 30);
  wait(600, msec);
  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(700, msec);
  SetFlyWheelPower(11);

  IntakeEngaged();
  intakeOn = true;
  fixedHeading = 41;
  GoToPoint(_p4, 3, 4);
  wait(200, msec);
  intakeOn = false;

  enableTurnPID = true;
  completed = false;
  TurnToAngle(137, 4, 30);
  wait(800, msec);

  enableTurnPID = false;
  completed = true;
  SpinDrive(0,0);
  wait(300, msec);
  ShootEngaged();
  wait(700, msec);


  shootOn = true;
  wait(200, msec);
  shootOn = false;
  wait(1200, msec);
  shootOn = true;
  wait(160, msec);
  shootOn = false;
  wait(1200, msec);
  shootOn = true;
  wait(130, msec);
  shootOn = false;
  wait(1200, msec);
  wait(170, msec);
}

//(-6.97,-25.25)