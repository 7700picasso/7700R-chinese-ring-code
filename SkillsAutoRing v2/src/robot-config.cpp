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
gps GPS = gps(PORT8, -175.00, -110.00, mm, -90);
distance DistFront = distance(PORT7);
digital_out MogoTilt = digital_out(Brain.ThreeWirePort.C);
digital_out Forklift = digital_out(Brain.ThreeWirePort.F);
motor Rings = motor(PORT20, ratio6_1, false);
digital_out claw1 = digital_out(Brain.ThreeWirePort.E);
/*vex-vision-config:begin*/
signature Vision__MOGO_RED = signature (1, 8017, 9845, 8931, 393, 1153, 773, 5.2, 0);
signature Vision__MOGO_BLUE = signature (2, -2897, -1905, -2401, 10269, 14393, 12331, 4.2, 0);
signature Vision__MOGO_YELLOW = signature (3, 1117, 2153, 1636, -2853, -2167, -2510, 4.3, 0);
signature Vision__SIG_4 = signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
vision Vision = vision (PORT19, 50, Vision__MOGO_RED, Vision__MOGO_BLUE, Vision__MOGO_YELLOW, Vision__SIG_4);
/*vex-vision-config:end*/
/*vex-vision-config:begin*/
vision VisionBack = vision (PORT12, 50);
/*vex-vision-config:end*/
gps GPSR = gps(PORT18, 177.00, -110.00, mm, 90);

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