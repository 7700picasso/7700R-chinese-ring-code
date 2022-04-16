using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor leftDrive1;
extern motor leftDrive2;
extern motor leftmiddle;
extern motor rightDrive1;
extern motor rightDrive2;
extern motor rightmiddle;
extern motor Lift;
extern inertial Gyro;
extern gps GPS;
extern distance DistFront;
extern distance DistBack;
extern distance DistClaw;
extern digital_out MogoTilt;
extern digital_out ClashRoyal1;
extern digital_out ClashRoyal2;
extern motor Rings;
extern digital_out claw1;
extern digital_out tall;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );