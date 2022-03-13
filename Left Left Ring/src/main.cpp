/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Benjamin and Sean                                         */
/*    Created:      sometime                                                  */
/*    Description:  7700R code 2021-2022  Skills                              */
/*               Benjamin 2-11-22                                             */
/*----------------------------------------------------------------------------*/
//7700R
//Benjamin
//Sean
//Avery
//Matthew
//Jaehoon
//Saif
//Josh
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// leftDrive1           motor         1               
// leftDrive2           motor         3               
// leftmiddle           motor         5               
// rightDrive1          motor         6               
// rightDrive2          motor         2               
// rightmiddle          motor         4               
// Lift                 motor         10              
// Gyro                 inertial      19              
// GPS                  gps           8               
// DistFront            distance      15              
// DistBack             distance      16              
// DistClaw             distance      17              
// MogoTilt             digital_out   A               
// ClashRoyal1          digital_out   D               
// ClashRoyal2          digital_out   E               
// Rings                motor         9               
// claw1                digital_out   B               
// tall                 digital_out   C               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <math.h>

using namespace vex;

// A global instance of competition, dont touch either
competition Competition;

// define your global Variables here
char *str = "";
const long double pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825; // much more accurate than 3.14. accurate enough to go across the universe and be within an atom of error

#define Diameter 3.25 * 3 / 5
#define UNITSIZE 23.75 // tile size
#define MOGO_DIST 5
#define NOTE str = 
#define INF 4294967295
#define CLAW_OPEN false
#define TILT_OPEN false
#define LIFT_UP 85
#define DIAG sqrt(2)
#define High_Open false

// for red comments

void pre_auton(void) {
  vexcodeInit();
  Gyro.calibrate();
  GPS.calibrate();
  //picasso.set(false);
	claw1.set(CLAW_OPEN);
  MogoTilt.set(TILT_OPEN);
  ClashRoyal1.set(false);
  ClashRoyal2.set(false);
  wait(2000, msec);
  tall.set(High_Open);


  // All activities that occur before the competition starts
  // gets pistons down before match
  // dont touch this 
	
	// BOOOOOOP whoops i touched it
}

void drive(int lspeed, int rspeed, int wt){
  leftDrive1.spin(forward, lspeed, pct);
  leftDrive2.spin(forward, lspeed, pct);
  leftmiddle .spin(forward, lspeed, pct);
  rightDrive1.spin(forward, rspeed, pct);
  rightDrive2 .spin(forward, rspeed, pct);
  rightmiddle .spin(forward, rspeed, pct);
  wait(wt, msec);
}
//use to go forward,backwards,left right etc,turning if your stupid
//use mmDrive to go forward and backwards,use gyro to turn

double minRots() {    
  double rots[] = {
    leftDrive1.position(rev), leftDrive2.position(rev), leftmiddle.position(rev),
    rightDrive1.position(rev), rightDrive2.position(rev), rightmiddle.position(rev)
  };
  double min = rots[0];
  
  for (int i = 1; i < 6; i++) {
    if (rots[i] < min) {
      min = rots[i];
    }
  }
  return min;
}

double mod(double a, double b) {
  return (a < 0) * b + a - b * (floor(a / b) + (a < 0));
}

int8_t sgn(double x) {
  return x > 0 ? 1 : x < 0 ? -1 : 0;
}

// get the angle at which the robot needs to turn to point towards point (x,y)
double degToTarget(double x1, double y1, double x2, double y2, bool Reverse = false, double facing = Gyro.rotation(degrees)) { 
	// First formula then second formula until the % operator so we dont have to use this formula multiple times.
	// If Reverse then we'd be adding 180 then subtracting it again at the start of formula 2. This avoids that.
  return mod(atan2(x2 - x1, y2 - y1) * 180 / pi - mod(facing, 360) - 180 * !Reverse, 360) - 180;
	/*
	EXPLAINATION:
	Formula for some value of error when current direction is 'facing' (assume atan2 is in radians so we must convert it to degrees):
	error(facing) = atan2(x2 - x1, y2 - y1) * 180 / pi + 180 * Reverse - facing. (formula 1)
	This formula can return a value greater than 180, and we don't want the robot turning more than it needs to.
	So we covert it to an angle between -180 and 180 with the formula (which I derived myself):
	angle(theta) = (theta - 180) % 360 - 180. (formula 2)
	Unfortunately, C++ doesn't allow % operator for non-int types, so we have our own formula for a % b:
	a % b = (a < 0) * b + a - b * int(a / b).
	Idk if int(x) will work the way it needs to, so we'll make our own:
	INT(x) = floor(x) + (x < 0).
	Noting that a < 0 iff a / b < 0 for all a and b ≠ 0, our modulo formula becomes:
	a % b = (a < 0) * b + a - b * (floor(a / b) + (a < 0)). (modulo formula)
	*/
}




void brakeDrive() {
  leftDrive1.stop(brake);
  leftDrive2.stop(brake);
  leftmiddle.stop(brake);
  rightDrive1.stop(brake);
  rightDrive2.stop(brake);
  rightmiddle.stop(brake);
  //break code for the gryo balance code
  // actually change brake type
  leftDrive1.setStopping(brake);
  leftDrive2.setStopping(brake);
  leftmiddle.setStopping(brake);
  rightDrive1.setStopping(brake);
  rightDrive2.setStopping(brake);
  rightmiddle.setStopping(brake);
}

void coastDrive() {
  leftDrive1.stop(coast);
  leftDrive2.stop(coast);
  leftmiddle.stop(coast);
  rightDrive1.stop(coast); //coast drive
  rightDrive2.stop(coast); //put notes here with examples 
  rightmiddle.stop(coast);
  // actually set brake stop type to coast
  leftDrive1.setStopping(coast);
  leftDrive2.setStopping(coast);
  leftmiddle.setStopping(coast);
  rightDrive1.setStopping(coast); 
  rightDrive2.setStopping(coast);
  rightmiddle.setStopping(coast);
}


void liftDeg(double angle, int WT = -1, int speed = 100) {
  Lift.setVelocity(speed, percent);
  Lift.setStopping(hold);

  Lift.spinFor(forward, 7 * angle, degrees, WT == -1);
  
  if (WT != -1) {
    wait(WT,msec);
  }
  else {
    Lift.stop(hold);
  }                                            //more math 
}

void liftTo(double angle, int WT = -1, int8_t speed = 100) {
  Lift.setVelocity(speed, percent);
  Lift.setStopping(hold);

  Lift.spinFor(forward, 7 * angle - Lift.position(degrees), degrees, WT == -1);
  
  if (WT != -1) {
    wait(WT,msec);
  }
  else {
    Lift.stop(hold);
  }
}

void liftTime(int speed, int duration, bool stop = false) {
  Lift.spin(forward,speed,percent);
  wait(duration,msec);
  if (stop) {
    Lift.stop(hold);
  }
}

void liftWait(double target, uint32_t maxTime = INF) {
  uint32_t startTime = vex::timer::system();
  while (Lift.position(degrees) * 7 > target && vex::timer::system() - startTime < maxTime) {
    wait(10, msec);
  }
}

//makes lift go up or down
// its the lift speed then wait time
//example lift(-100,1200);  so lift 100% for 1200 msc
// 100 is up and -100 is down,or other way around,you can figure that out

void rings(bool on, int speed = 87) {
  if (on) {
    Rings.spin(forward, on * speed, percent);
  }
  else {
    Rings.stop(hold);
  }
}

void Claw(bool open) {
  //wait(50 * open, msec);
  claw1.set(open);
  //wait(50 * !open, msec);
}

//idk maybe opens and closes the claw
//true is open and false is close
//examples
//claw.set(true);    open
//claw.set(false);   close

void mogoTilt(bool state) {
  MogoTilt.set(state);
}

void tallmogo(bool state) {
  tall.set(state);
}

void clashRoyal(bool state) {
  ClashRoyal1.set(state);
  ClashRoyal2.set(state);
}

void inchDrive(double target, double accuracy = 0.25) {
  leftDrive1.setPosition(0,  rev);
  leftDrive2.setPosition(0,  rev); // might only need 1 of 3 of these but im a dumbass so leave it 
  leftmiddle.setPosition(0,  rev);

  double 
    speed,
    error = target,
    olderror = error,
    Kp = 50 / 3, // about 16.667, was previously 10
    Ki = 1, // to increase speed if its taking too long. Adds a bit over 50% speed when 12 inches left.
    Kd = 40 / 3; // about 13.333, was previously 20.0

  double sum = 0;
  double decay = 0.5;

  /*
  dont use the drive function you dumbass AND DONT USE "//" for multiple lines
  use inchdrive,this took me a while to code :(
  its target/inches/amount then speed/percentage
  examples
  inchDrive(55, 100); go 55in forward at 100%
  inchDrive(-55, 100); go 55in backwards at 100%
  */

  while(fabs(error) > accuracy){
    // did this late at night but this while is important 
    // fabs = absolute value
    error = target - leftmiddle.position(rev) * Diameter * pi; //the error gets smaller when u reach ur target
    sum = sum * decay + error;
    speed = Kp * error + Ki * decay + Kd * (error - olderror); // big error go fast slow error go slow 
    drive(speed, speed, 10);
    olderror = error;
  }
  brakeDrive();
}

//if gyro needs calibrating add a 10ms wait or something, gyro cal takes about 1.5 sec
//1 sec if your good

void balance() { // WIP
  Brain.Screen.clearScreen();
  double Kp = 1.2;
  double Kt = 0.15; // constant for tip counts. This acts like Ki.
  volatile double speed;
  volatile double pitch = Gyro.pitch(degrees);

  const uint8_t maxPitchIdx = 2;
  volatile double oldPitches[maxPitchIdx + 1] = {pitch, pitch, pitch};
  volatile double oldpitch = pitch;

  double step = 1, oldStep = step;
  double upper = 100, lower = -100;
  double backCoef = 1;
  uint8_t inclineDir = sgn(pitch);
  uint8_t overCount = 0, underCount = 0; // over is how many times it went too far back. Under is how many times it didn't back up enough.

  uint8_t lastTip = sgn(pitch);
  bool wasTipping = true;
  volatile uint32_t startTime = vex::timer::system();
  double stopAng = 20;
  while (vex::timer::system() - startTime < 1300) {
    pitch = Gyro.pitch(degrees);
    speed = Kp * pitch;

    if (fabs(pitch) > 5) {
      // if its at the bottom of the platform and it wont go up, then someone didn't use this function correctly.
      if (fabs(pitch) > stopAng) {
        drive(speed, speed, 30);
        inclineDir = sgn(pitch);
      }
      else if (sgn(pitch * oldpitch) == 1) {
        speed *= Kt * (underCount - overCount) - backCoef;
        drive(speed, speed, 30); // Back up bc it has already overshot
      }
      startTime = vex::timer::system();
    }
    else {
      brakeDrive();
      wait(30, msec);
      oldpitch = pitch;
    }

    // correct backCoef
    bool tipping = fabs(pitch) > 10 && fabs(pitch) > fabs(oldpitch);

    if (tipping && !wasTipping) {
      int8_t sign = -sgn(pitch * inclineDir); // if pitch and incline have same sign then their product is positve. otherwise, its negative. neither can be 0 :)
      step /= !(sign * step + backCoef > lower && sign * step + backCoef < upper) + 1;
      backCoef += sign * step;
      upper = (upper != 100 || sign == -1) ? backCoef + step : upper; // if step it decreasing then lower upper
      lower = (lower != -100 || sign == 1) ? backCoef - step : lower; // if step is increasing then raise lower
      // update tip counters
      if (sign == 1) {
        overCount++;
      }
      else {
        underCount++;
      }
    }

    // update "old" variables
    for (uint8_t i = maxPitchIdx; i > 0; i++) {
      oldPitches[i] = oldPitches[i - 1];
    }
    oldPitches[0] = pitch;
    oldpitch = oldPitches[maxPitchIdx];
    oldStep = step;

    wasTipping = fabs(pitch) > 10;
    lastTip = !wasTipping ? lastTip : sgn(pitch);
  }
  Brain.Screen.printAt(1, 150, "i am done ");
}
void unitDrive(double target, bool endClaw = false, double clawDist = 1, uint32_t maxTime = INF, double maxSpeed = 100, bool raiseMogo = false, double accuracy = 0.25) {
	double Kp = 5; // was previously 50/3
	double Ki = 1; // to increase speed if its taking too long.
	double Kd = 20; // was previously 40/3
	double decay = 0.5; // integral decay
	
	target *= UNITSIZE; // convert UNITS to inches
	
	volatile double speed;
	volatile double error = target;
	volatile double olderror = error;
	 
  leftDrive1.setPosition(0, rev);
	leftDrive2.setPosition(0, rev);
  leftmiddle.setPosition(0, rev);
  rightDrive1.setPosition(0, rev);
  rightDrive2.setPosition(0, rev);
  rightmiddle.setPosition(0, rev);
	 
  volatile double sum = 0;
  uint32_t startTime = vex::timer::system();
  bool isOpen;
	 
  while((fabs(error) > accuracy || fabs(speed) > 10) && vex::timer::system() - startTime < maxTime) {
    // did this late at night but this while is important 
    error = target - minRots() * Diameter * pi; //the error gets smaller when u reach ur target
    sum = sum * decay + error;
    speed = Kp * error + Ki * sum + Kd * (error - olderror); // big error go fast slow error go slow 
    speed = !(fabs(speed) > maxSpeed) ? speed : maxSpeed * sgn(speed);
    drive(speed, speed, 10);
    olderror = error;
    isOpen = target > 0 ? claw1.value() == CLAW_OPEN : MogoTilt.value() == TILT_OPEN;
    if (endClaw && isOpen && (/*DistClaw.objectDistance(inches) < 2 ||*/ fabs(error) <= clawDist)) { // close claw b4 it goes backwards.
	    if (target > 0) Claw(!CLAW_OPEN);
      else mogoTilt(!TILT_OPEN);
    }
    if (raiseMogo && !isOpen) {
      Lift.spin(forward,100,percent);
    }
  }
	brakeDrive();
  if (endClaw && isOpen) {
    if (target > 0) Claw(!CLAW_OPEN);
    else mogoTilt(!TILT_OPEN);
  }
}

void gyroturn(double target, double &idealDir,double accuracy = 1) { // idk maybe turns the robot with the gyro,so dont use the drive function use the gyro
  double Kp = 0.8; // was 2.0
  double Ki = 0.1; // adds a bit less than 50% when there is 90° left.
  double Kd = 1.5; // was 16.0
 
  double currentDir = Gyro.rotation(degrees);
  double speed = 100;
  double error = target;
  double olderror = error;

  double lambda = 0.5; // exponential decay rate

  double sum = 0;
  
  idealDir += target;
  target = currentDir + idealDir - Gyro.rotation(degrees);
  
  while(fabs(error) > accuracy || fabs(speed) > 10){ //fabs = absolute value while loop again
    currentDir = Gyro.rotation(degrees);
    error = target - currentDir; //error gets smaller closer you get,robot slows down
    sum = sum * lambda + error;
    speed = Kp * error + Ki * sum + Kd * (error - olderror); // big error go fast slow error go slow 
    drive(speed, -speed, 10);
    Brain.Screen.printAt(1, 40, "heading = %0.2f    degrees", currentDir); //math thing again,2 decimal places
    Brain.Screen.printAt(1, 60, "speed = %0.2f    degrees", speed);
    //all ths print screen helps test to make sure gyro not broke
    olderror = error;
  }
  brakeDrive();
  currentDir = Gyro.rotation(degrees); //prints the gyro rotation degress
  Brain.Screen.printAt(1, 40, "heading = %0.2f    degrees", currentDir);
}

void turnTo(double target, double accuracy = 1) {
  double facing = Gyro.rotation(degrees);
  gyroturn(mod(target - facing - 180, 360) - 180, facing, accuracy);
}


void auton() {
	NOTE"            R             RRRR            RRRR  RRRRRRRRRRRRRRRRRR       RRRRRRRR       ";
	NOTE"           RRR            RRRR            RRRR  RRRRRRRRRRRRRRRRRR    RRRRRRRRRRRRRR    ";
	NOTE"          RRRRR           RRRR            RRRR         RRRR          RRRRR      RRRRR   ";
	NOTE"         RRRRRRR          RRRR            RRRR         RRRR         RRRR          RRRR  ";
	NOTE"        RRRR RRRR         RRRR            RRRR         RRRR         RRRR          RRRR  ";
	NOTE"       RRRR   RRRR        RRRR            RRRR         RRRR         RRRR          RRRR  ";
	NOTE"      RRRR     RRRR       RRRR            RRRR         RRRR         RRRR          RRRR  ";
	NOTE"     RRRRRRRRRRRRRRR      RRRR            RRRR         RRRR         RRRR          RRRR  ";
	NOTE"    RRRRRRRRRRRRRRRRR     RRRR            RRRR         RRRR         RRRR          RRRR  ";
	NOTE"   RRRR           RRRR     RRRRR        RRRRR          RRRR          RRRRR      RRRRR   ";
	NOTE"  RRRR             RRRR      RRRRRRRRRRRRRR            RRRR           RRRRRRRRRRRRRR    ";
	NOTE" RRRR               RRRR        RRRRRRRR               RRRR              RRRRRRRR       ";

	Claw(CLAW_OPEN); // open claw
  mogoTilt(TILT_OPEN);
  clashRoyal(false);

	//runningAuto = true;
  double facing = 11.3;
  Gyro.setRotation(11.3, deg);
  double mogoStopDist = 6; // STOP THIS MANY UNITS BEFORE A MOGO. FEEL FREE TO CHANGE

	while (Gyro.isCalibrating() || GPS.isCalibrating()) { // dont start until gyro and GPS are calibrated
		wait(10, msec);
	}
  "SIDE-RINGS-MID";
  "START AT THE HALFWAY LINE NEXT TO THE ALLIANCE GOAL, FACING IT WITH THE CLAW. SEE https://cdn.discordapp.com/attachments/875890646714576906/944007778526175282/Left_auto_SIDE-PICASSO.png FOR REFERENCE";
	// SIDE
  unitDrive(2.5,true,3); // GO TO THE MOGO
  liftDeg(15,0);
  unitDrive(-2);
  // ALLIANCE GOAL
  gyroturn(-23, facing); // POINT MOGO LIFT AT THE ALLIANCE GOAL. change the 30.
  unitDrive(-0.667,true,3,1500,50); // GET MOGO INTO MOGO LIFT.
  // RINGS
  //unitDrive(1.75); // align. this probably wont work
  //turnTo(90);
  rings(true);
  unitDrive(1.5);
  turnTo(87);
  unitDrive(2,false,0,INF,50);
  unitDrive(-1.5);
  turnTo(0);
  unitDrive(-1);
  /*inchDrive(2.5 * UNITSIZE);
  Claw(CLAW_OPEN);
  // MID
  gyroturn(-90,facing);
  inchDrive(1 * UNITSIZE);
  Claw(!CLAW_OPEN);
  inchDrive(-1 * UNITSIZE);
  wait(300,msec);*/
}


//driver controls,dont change unless your jaehoon or sean
void driver() {
  // User control code here, inside the loop
  //2 joy sticks
  //rstick is axis 2 and lstick is axis 3,why its 2,3 and not 1,2 idk ask vex
  coastDrive(); // set drive motors to coast

  while (Gyro.isCalibrating() || GPS.isCalibrating()) { // dont start until gyro is calibrated
    wait(10, msec);
  }
  Gyro.setRotation(GPS.rotation(degrees) - 90, degrees);
  //Controller1.Screen.print("%0.3f", Gyro.rotation(deg));
  bool r2Down = false;
  bool r1Down = false;

  bool ringsOn = false;

  while (true) {
    // drive control
		int rstick=Controller1.Axis2.position();
		int lstick=Controller1.Axis3.position();
		drive(lstick, rstick,10);
		int8_t tmp, ringSpeed = 87;
    // mogoTilt controls
    if (!Controller1.ButtonR2.pressing()) {
      r2Down = false;
    }
    else if (!r2Down) {
      mogoTilt(!MogoTilt.value());
      r2Down = true;
    }
    // ring controls
    if (!Controller1.ButtonR1.pressing()) {
      r1Down = false;
    }
    else if (!r1Down) {
      ringsOn = !ringsOn;
      r1Down = true;
    }
		if (Controller1.ButtonY.pressing()) { // turn off rings
			ringsOn = false;
		}
		else if (Controller1.ButtonB.pressing()) { // reverse rings
      ringSpeed = -100;
		}
    rings(ringsOn || ringSpeed < 0,ringSpeed);
		// lift control
		tmp = 100 * (Controller1.ButtonL1.pressing() - Controller1.ButtonL2.pressing());
		if (tmp == 0) {
			Lift.stop(hold);
		}
		else {
			Lift.spin(forward, tmp, percent);
		}
  

		if (Controller1.ButtonX.pressing()) { // claw close
      Claw(!CLAW_OPEN);
		}
		else if (Controller1.ButtonA.pressing()) { //claw open
      Claw(CLAW_OPEN);
		}
    // tall controls
    if (Controller1.ButtonUp.pressing()) { 
      tallmogo(!High_Open);
		}
		else if (Controller1.ButtonRight.pressing()) { 
      tallmogo(High_Open);
		}
		wait(20, msec); // dont waste air 
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(auton);
  Competition.drivercontrol(driver);

  // Run the pre-autonomous function.
  pre_auton();

  // Stops main from exiting in the infinite loop.
  while (true) {
    wait(100, msec);
  }
}
