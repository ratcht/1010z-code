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

 
}



void autonomous(void) {

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

  std::vector<Point> list;

  // Create a text string, which is used to output the text file
  std::string myText;

  // Read from the text file
  std::ifstream MyReadFile("pure.txt");

  // Use a while loop together with the getline() function to read the file line by line
  while (std::getline(MyReadFile, myText)){
    Point one = parseString(myText);

    list.push_back(one);
  }

  // Close the file
  MyReadFile.close();

  limiter rpmlimit;
  initUserControl();

  while (1) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);
    Brain.Screen.print(list.size());
    Brain.Screen.newLine();
    Brain.Screen.print("%f, %f", list.at(3).x, list.at(3).y);
    Brain.Screen.newLine();


    //Brain.Screen.print(point.c_str());



    //---------------Drivetrain---------------
    double fwdVal = Controller1.Axis3.position(percent);
    double turnVal = Controller1.Axis1.position(percent);

    //Volts Range:  -12 --> 12
    double turnRPM = turnVal * 4.5; //convert percentage to volts
    double fwdRPM = fwdVal * 4.5 * (1 - (std::abs(turnVal) * turnImportance)); 
    fwdRPM = rpmlimit.rateLimiter(fwdRPM, 45);

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
