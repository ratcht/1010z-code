#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
inertial Inertial2 = inertial(PORT14);
controller Controller1 = controller(primary);
motor LeftDriveMotorA = motor(PORT8, ratio6_1, true);
motor LeftDriveMotorB = motor(PORT19, ratio6_1, false);
motor_group LeftDrive = motor_group(LeftDriveMotorA, LeftDriveMotorB);
motor LeftTop = motor(PORT7, ratio6_1, false);
motor RightDriveMotorA = motor(PORT4, ratio6_1, false);
motor RightDriveMotorB = motor(PORT5, ratio6_1, true);
motor_group RightDrive = motor_group(RightDriveMotorA, RightDriveMotorB);
motor RightTop = motor(PORT20, ratio6_1, true);
digital_out MagazinePiston = digital_out(Brain.ThreeWirePort.C);
encoder EncoderHorz = encoder(Brain.ThreeWirePort.E);
encoder EncoderPara = encoder(Brain.ThreeWirePort.G);
motor FlyWheel = motor(PORT2, ratio6_1, false);
motor Intake = motor(PORT11, ratio18_1, false);
digital_out EndgamePiston = digital_out(Brain.ThreeWirePort.A);
digital_out BlooperPiston = digital_out(Brain.ThreeWirePort.D);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}