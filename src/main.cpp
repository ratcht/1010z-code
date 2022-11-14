// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Inertial2            inertial      2               
// Controller1          controller                    
// LeftDrive            motor_group   3, 4            
// LeftTop              motor         5               
// RightDrive           motor_group   6, 7            
// RightTop             motor         8               
// EndgamePiston        digital_out   A               
// EncoderHorz          encoder       G, H            
// EncoderPara          encoder       E, F            
// FlyWheel             motor         12              
// Intake               motor         18              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Inertial2            inertial      2               
// Controller1          controller                    
// LeftDrive            motor_group   3, 4            
// LeftTop              motor         5               
// RightDrive           motor_group   6, 7            
// RightTop             motor         8               
// EndgamePiston        digital_out   A               
// EncoderHorz          encoder       G, H            
// EncoderPara          encoder       E, F            
// FlyWheel             motor         12              
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Inertial2            inertial      2               
// Controller1          controller                    
// LeftDrive            motor_group   3, 4            
// LeftTop              motor         5               
// RightDrive           motor_group   6, 7            
// RightTop             motor         8               
// EndgamePiston        digital_out   A               
// EncoderHorz          encoder       G, H            
// EncoderPara          encoder       E, F            
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Inertial2            inertial      2               
// Controller1          controller                    
// LeftDrive            motor_group   3, 4            
// LeftTop              motor         5               
// RightDrive           motor_group   6, 7            
// RightTop             motor         8               
// EndgamePiston        digital_out   A               
// EncoderHorz          encoder       G, H            
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Inertial2            inertial      2               
// Controller1          controller                    
// LeftDrive            motor_group   3, 4            
// LeftTop              motor         5               
// RightDrive           motor_group   6, 7            
// RightTop             motor         8               
// EndgamePiston        digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Inertial2            inertial      2               
// Controller1          controller                    
// LeftDrive            motor_group   3, 4            
// LeftTop              motor         5               
// RightDrive           motor_group   6, 7            
// RightTop             motor         8               
// EndgamePiston        digital_out   A               
// ---- END VEXCODE CONFIGURED DEVICES ----
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Inertial2            inertial      2               
// Controller1          controller                    
// LeftDrive            motor_group   3, 4            
// LeftTop              motor         5               
// RightDrive           motor_group   6, 7            
// RightTop             motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "main.h"
#include <fstream>
#include <string>
#include <cstring>



using namespace vex;

competition Competition;


void pre_auton(void) {
  vexcodeInit();

  std::vector<Point> list;

  // Create a text string, which is used to output the text file
  std::string myText;

  // Read from the text file
  std::ifstream file("pure.txt");

  // Use a while loop together with the getline() function to read the file line by line
  while (std::getline(file, myText)){
    Point one = parseString(myText);

    list.push_back(one);
  }

  // Close the file
  file.close();


 
}



void autonomous(void) {
  enableDriverPID = true;
  resetDriveSensors = true;
  vex::task driveOnlyPID(driverOnlyPID);

  disableDrive = false;
  desiredMotorVal = 850;
  turnVals.desiredValue = 0;

  vex::task::sleep(2000);
  
  resetDriveSensors = true;
  disableDrive = true;
  desiredMotorVal = 0;
  turnVals.desiredValue = 47;

  vex::task::sleep(1550);

  resetDriveSensors = true;
  disableDrive = false;
  desiredMotorVal = -220;

  vex::task::sleep(50000);
  EndgameFire();

}


void initUserControl() {
  vex::controller Controller1 (vex::controllerType::primary);
  vex::controller Controller2 (vex::controllerType::partner);

  //---------------Settings--------------- 
  
  LeftDrive.setStopping(hold);
  LeftTop.setStopping(hold);
  RightDrive.setStopping(hold);
  RightTop.setStopping(hold);
}


void usercontrol(void) {

  limiter rpmlimit;
  initUserControl();

  Controller1.ButtonUp.pressed(EndgameFire);

  while (1) {



    //---------------Drivetrain---------------
    float fwdVal = Controller1.Axis3.position(percent);
    float turnVal = Controller1.Axis1.position(percent);

    //Volts Range:  -12 --> 12
    float turnRPM = turnVal * 0.12; //convert percentage to volts
    float fwdRPM = fwdVal * 0.12 * (1 - (std::abs(turnVal) * turnImportance)); 
    //fwdRPM = rpmlimit.rateLimiter(fwdRPM, 45);

    SpinDrive(fwdRPM + turnRPM, fwdRPM - turnRPM);

    wait(20, msec); 
  }
}










//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
