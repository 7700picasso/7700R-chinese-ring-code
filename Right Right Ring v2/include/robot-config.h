using namespace vex;

extern brain Brain;

using signature = vision::signature;

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
extern digital_out Forklift;
extern motor Rings;
extern digital_out claw1;
extern distance Stalker;
extern signature Vision__SIG_1;
extern signature Vision__SIG_2;
extern signature Vision__SIG_3;
extern signature Vision__SIG_4;
extern signature Vision__SIG_5;
extern signature Vision__SIG_6;
extern signature Vision__SIG_7;
extern vision Vision;
extern signature VisionBack__SIG_1;
extern signature VisionBack__SIG_2;
extern signature VisionBack__SIG_3;
extern signature VisionBack__SIG_4;
extern signature VisionBack__SIG_5;
extern signature VisionBack__SIG_6;
extern signature VisionBack__SIG_7;
extern vision VisionBack;
extern limit Trigger1;
extern limit Trigger2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );