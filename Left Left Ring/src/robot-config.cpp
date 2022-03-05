#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftDrive1 = motor(PORT1, ratio6_1, true);
motor leftDrive2 = motor(PORT3, ratio6_1, true);
motor leftmiddle = motor(PORT5, ratio6_1, true);
motor rightDrive1 = motor(PORT6, ratio6_1, false);
motor rightDrive2 = motor(PORT8, ratio6_1, false);
motor rightmiddle = motor(PORT4, ratio6_1, false);
motor Lift = motor(PORT10, ratio36_1, false);
inertial Gyro = inertial(PORT19);
gps GPS = gps(PORT21, 220.00, 50.00, mm, 90);
distance DistFront = distance(PORT15);
distance DistBack = distance(PORT16);
distance DistClaw = distance(PORT17);
digital_out MogoTilt = digital_out(Brain.ThreeWirePort.A);
digital_out ClashRoyal1 = digital_out(Brain.ThreeWirePort.D);
digital_out ClashRoyal2 = digital_out(Brain.ThreeWirePort.E);
motor Rings = motor(PORT9, ratio6_1, false);
digital_out claw1 = digital_out(Brain.ThreeWirePort.B);
digital_out claw2 = digital_out(Brain.ThreeWirePort.C);

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