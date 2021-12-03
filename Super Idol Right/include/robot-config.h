using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor leftDrive1;
extern motor leftDrive2;
extern motor rightDrive1;
extern motor rightDrive2;
extern motor amogo;
extern motor lift1;
extern digital_out pis1;
extern inertial Inertial21;
extern digital_out pis2;
extern inertial Gyro;
extern motor intakes;
extern motor chain;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );