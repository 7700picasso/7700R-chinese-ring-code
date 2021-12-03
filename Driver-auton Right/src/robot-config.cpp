#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftDrive1 = motor(PORT7, ratio6_1, true);
motor leftDrive2 = motor(PORT3, ratio6_1, true);
motor rightDrive1 = motor(PORT6, ratio6_1, false);
motor rightDrive2 = motor(PORT2, ratio6_1, false);
motor amogo = motor(PORT18, ratio36_1, false);
motor lift1 = motor(PORT1, ratio36_1, true);
digital_out pis1 = digital_out(Brain.ThreeWirePort.B);
inertial Inertial21 = inertial(PORT21);
digital_out pis2 = digital_out(Brain.ThreeWirePort.A);
inertial Gyro = inertial(PORT20);
motor intakes = motor(PORT10, ratio18_1, false);
motor chain = motor(PORT12, ratio18_1, false);

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