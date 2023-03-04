#include "main.h"
#include <fstream>
#include <string>
#include <cstring>



using namespace vex;

competition Competition;


void pre_auton(void) {
  vexcodeInit();

  std::deque<Point> list;

  // Create a text string, which is used to output the text file
  std::string myText;

  // Read from the text file
  std::ifstream file("pathone.txt");

  // Use a while loop together with the getline() function to read the file line by line
  while (std::getline(file, myText)){
    Point one = parseString(myText);

    list.push_back(one);
  }

  // Close the file
  file.close();
  desPath1.points = list;


//_______________________________________________

  std::deque<Point> list2;

  // Create a text string, which is used to output the text file
  std::string myText2;

  // Read from the text file
  std::ifstream file2("pathtwo.txt");

  // Use a while loop together with the getline() function to read the file line by line
  while (std::getline(file2, myText2)){
    Point one = parseString(myText2);

    list2.push_back(one);
  }

  // Close the file
  file.close();
  desPath2.points = list2;

}


int autonToRun = 3;

void autonomous(void) {
  if(autonToRun == 0) {
    LeftAuto();
  } else if (autonToRun == 1) {
    LeftAWP();
  } else if(autonToRun==2) {
    RightAuto();
  } else if(autonToRun==3) {
    SkillsAuto();
  } else if(autonToRun==4) {
    BetterRightAuto();
  } else if(autonToRun==5) {
    Left();
  } else if(autonToRun==6) {
    PureTest();
  }
}


void initUserControl() {
  vex::controller Controller1 (vex::controllerType::primary);
  vex::controller Controller2 (vex::controllerType::partner);

  //---------------Settings--------------- 
  
  LeftDrive.setStopping(hold);
  LeftTop.setStopping(hold);
  RightDrive.setStopping(hold);
  RightTop.setStopping(hold);

  SetFlyWheelPower(9.5);
}



void usercontrol(void) {

  vex::task runFlywheel(FlyWheelThread);
  vex::task runIntake(IntakeThread);

  enableOdom = true;
  vex::task odomThread(OdomThread);
  vex::task BrainPrint(brainPrint);


  Controller1.ButtonA.pressed(FlyWheelSwap);

  Controller1.ButtonR1.pressed(IntakeSwap);
  Controller1.ButtonR1.pressed(IntakeEngaged);
  Controller1.ButtonR2.pressed(ShooterSwap);


  limiter rpmlimit, turnlimit;
  initUserControl();

  Controller1.ButtonL1.pressed(ShootEngaged);
  Controller1.ButtonL2.pressed(StopAll);
  Controller1.ButtonRight.pressed(EndgameFire);

  Controller1.ButtonY.pressed(BlooperToggle);
  Controller1.ButtonX.pressed(FlyWheelIncrease);
  Controller1.ButtonB.pressed(FlyWheelDecrease);


  while (1) {

    //---------------Drivetrain---------------
    float fwdVal = Controller1.Axis3.position(percent);
    float turnVal = Controller1.Axis1.position(percent);


    //Volts Range:  -12 --> 12
    float turnRPM = turnVal * 0.12; //convert percentage to volts
    turnRPM = turnlimit.rateLimiter(turnRPM, 75);

    float fwdRPM = fwdVal * 0.12 * (1 - (std::abs(turnRPM/12) * turnImportance)); 
    fwdRPM = rpmlimit.rateLimiter(fwdRPM, 95);

    SpinDrive(fwdRPM - turnRPM, fwdRPM + turnRPM);

    wait(20, msec); 


  }
}





class Button
{
  public:
    int x, y, width, height;
    std::string text;
    vex::color buttonColor, textColor;
    
    Button(int x, int y, int width, int height, std::string text, vex::color buttonColor, vex::color textColor)
    : x(x), y(y), width(width), height(height), text(text), buttonColor(buttonColor), textColor(textColor){}

    void render()
    {
      Brain.Screen.drawRectangle(x, y, width, height, buttonColor);
      Brain.Screen.printAt(x + 10, y + 10, false, text.c_str());
    }

    bool isClicked()
    {
      if(Brain.Screen.pressing() && Brain.Screen.xPosition() >= x && Brain.Screen.xPosition() <= x + width &&
      Brain.Screen.yPosition() >= y && Brain.Screen.yPosition() <= y + width) return true;
      return false;
    }
};

Button autonButtons[] = {
  Button(10, 10, 150, 50, "Auton Red 1", vex::green, vex::black),
  Button(170, 10, 150, 50, "Auton Red 2", vex::white, vex::black),
  Button(10, 70, 150, 50, "Auton Blue 1", vex::white, vex::black),
  Button(170, 70, 150, 50, "Auton Blue 2", vex::white, vex::black)
};



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
