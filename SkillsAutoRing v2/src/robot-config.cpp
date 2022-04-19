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
inertial Gyro = inertial(PORT18);
gps GPS = gps(PORT8, -160.00, -200.00, mm, -90);
distance DistFront = distance(PORT15);
distance DistBack = distance(PORT16);
distance DistClaw = distance(PORT17);
digital_out MogoTilt = digital_out(Brain.ThreeWirePort.C);
digital_out Forklift = digital_out(Brain.ThreeWirePort.F);
motor Rings = motor(PORT20, ratio6_1, false);
digital_out claw1 = digital_out(Brain.ThreeWirePort.E);
/*vex-vision-config:begin*/
signature Vision__MOGO_RED = signature (1, 8195, 10569, 9382, -967, -223, -594, 2.1, 0);
signature Vision__MOGO_BLUE = signature (2, -2311, -1047, -1678, 7295, 10595, 8944, 2.6, 0);
signature Vision__MOGO_YELLOW = signature (3, 1861, 3163, 2512, -3873, -3263, -3568, 5.8, 0);
vision Vision = vision (PORT19, 50, Vision__MOGO_RED, Vision__MOGO_BLUE, Vision__MOGO_YELLOW);
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