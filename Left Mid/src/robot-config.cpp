#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftDrive1 = motor(PORT4, ratio18_1, true);
motor leftDrive2 = motor(PORT6, ratio18_1, true);
motor leftmiddle = motor(PORT5, ratio18_1, false);
motor rightDrive1 = motor(PORT1, ratio18_1, false);
motor rightDrive2 = motor(PORT3, ratio18_1, true);
motor rightmiddle = motor(PORT2, ratio18_1, false);
motor Lift = motor(PORT9, ratio36_1, false);
inertial Gyro = inertial(PORT10);
gps GPS = gps(PORT8, 220.00, 50.00, mm, 90);
digital_out MogoTilt = digital_out(Brain.ThreeWirePort.C);
motor Rings = motor(PORT20, ratio6_1, false);
digital_out claw1 = digital_out(Brain.ThreeWirePort.E);
/*vex-vision-config:begin*/
signature Vision__MOGO_RED = signature (1, 6279, 10719, 8498, -1039, -261, -650, 1.6, 0);
signature Vision__MOGO_BLUE = signature (2, -3231, -2263, -2746, 6987, 11713, 9350, 3.3, 0);
signature Vision__MOGO_YELLOW = signature (3, -1, 2239, 1118, -4399, -2741, -3570, 2.2, 0);
vision Vision = vision (PORT17, 50, Vision__MOGO_RED, Vision__MOGO_BLUE, Vision__MOGO_YELLOW);
/*vex-vision-config:end*/

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