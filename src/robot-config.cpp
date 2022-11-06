#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
inertial Inertial2 = inertial(PORT2);
controller Controller1 = controller(primary);
motor LeftDriveMotorA = motor(PORT3, ratio6_1, false);
motor LeftDriveMotorB = motor(PORT4, ratio6_1, false);
motor_group LeftDrive = motor_group(LeftDriveMotorA, LeftDriveMotorB);
motor LeftTop = motor(PORT5, ratio6_1, false);
motor RightDriveMotorA = motor(PORT6, ratio6_1, false);
motor RightDriveMotorB = motor(PORT7, ratio6_1, false);
motor_group RightDrive = motor_group(RightDriveMotorA, RightDriveMotorB);
motor RightTop = motor(PORT8, ratio6_1, false);

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