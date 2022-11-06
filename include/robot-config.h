using namespace vex;

extern brain Brain;

// VEXcode devices
extern inertial Inertial2;
extern controller Controller1;
extern motor_group LeftDrive;
extern motor LeftTop;
extern motor_group RightDrive;
extern motor RightTop;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );