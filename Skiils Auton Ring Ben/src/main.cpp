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
// leftDrive1           motor         7               
// leftDrive2           motor         3               
// leftmiddle           motor         5               
// rightDrive1          motor         6               
// rightDrive2          motor         2               
// rightmiddle          motor         4               
// Lift                 motor         1               
// claw                 digital_out   A               
// Gyro                 inertial      19              
// GPS                  gps           9               
// DistFront            distance      15              
// DistBack             distance      16              
// DistClaw             distance      17              
// MogoTilt1            digital_out   B               
// MogoTilt2            digital_out   C               
// ClashRoyal1          digital_out   D               
// ClashRoyal2          digital_out   E               
// Rings                motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <math.h>

using namespace vex;

// A global instance of competition, dont touch either
competition Competition;

// define your global Variables here
std::string str = "";
const long double pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825; // much more accurate than 3.14. accurate enough to go across the universe and be within an atom of error

#define Diameter 3.25
#define UNITSIZE 23.75 // tile size
#define NOTE str = 
#define INF 4294967295
#define CLAW_OPEN true
#define TILT_OPEN false
#define LIFT_UP 85

// for red comments

void pre_auton(void) {
  vexcodeInit();
  Gyro.calibrate();
  GPS.calibrate();
	claw.set(CLAW_OPEN);
  MogoTilt1.set(TILT_OPEN);
  MogoTilt2.set(TILT_OPEN);
  ClashRoyal1.set(false);
  ClashRoyal2.set(false);
  wait(2000, msec);

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

void rings(bool on, int speed = 66) { // i think 100 is a bit fast
  if (on) {
    Rings.spin(forward, on * speed, percent);
  }
  else {
    Rings.stop(hold);
  }
}

void Claw(bool open) {
  //wait(50 * open, msec);
  claw.set(open);
  //wait(50 * !open, msec);
}

//idk maybe opens and closes the claw
//true is open and false is close
//examples
//claw.set(true);    open
//claw.set(false);   close

void mogoTilt(bool state) {
  MogoTilt1.set(state);
  MogoTilt2.set(state);
}

void clashRoyal(bool state) {
  ClashRoyal1.set(state);
  ClashRoyal2.set(state);
}

void unitDrive(double target, bool endClaw = false, double clawDist = 1, uint32_t maxTime = INF, double maxSpeed = 100, double accuracy = 0.25) {
	double Kp = 10; // was previously 50/3
	double Ki = 2; // to increase speed if its taking too long.
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
    isOpen = target > 0 ? claw.value() == CLAW_OPEN : MogoTilt1.value() == TILT_OPEN;
    if (endClaw && isOpen && (/*DistClaw.objectDistance(inches) < 2 ||*/ fabs(error) <= clawDist)) { // close claw b4 it goes backwards.
	    if (target > 0) Claw(!CLAW_OPEN);
      else mogoTilt(!TILT_OPEN);
    }
  }
	brakeDrive();
  if (endClaw && isOpen) {
    if (target > 0) Claw(!CLAW_OPEN);
    else mogoTilt(!TILT_OPEN);
  }
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

void gyroturn(double target, double accuracy = 1) { // idk maybe turns the robot with the gyro,so dont use the drive function use the gyro
  double Kp = 1.1;
  double Ki = 0.2;
  double Kd = 1.25;
	double decay = 0.5; // integral decay
	
  volatile double sum = 0;

  volatile double speed;
  volatile double error = target;
  volatile double olderror = error;

  target += Gyro.rotation(degrees);

  while(fabs(error) > accuracy || fabs(speed) > 1) { //fabs = absolute value while loop again
    error = target - Gyro.rotation(degrees);; //error gets smaller closer you get,robot slows down
    sum = sum * decay + error; 
    speed = Kp * error + Ki * sum + Kd * (error - olderror); // big error go fast slow error go slow 
    drive(speed, -speed, 10);
    Brain.Screen.printAt(1, 60, "speed = %0.2f    degrees", speed);
    olderror = error;
  }
}

void turnTo(double target, double accuracy = 1) {
  gyroturn(mod(target - Gyro.rotation(degrees) - 180, 360) - 180, accuracy);
}

void pointAt(double x2, double y2, bool Reverse = false, double x1 = -GPS.yPosition(inches), double y1 = GPS.xPosition(inches), uint32_t maxTime = INF, double accuracy = 1) { 
	// point towards targetnss 
  x2 *= UNITSIZE, y2 *= UNITSIZE;
	double target = degToTarget(x1, y1, x2, y2, Reverse, Gyro.rotation(degrees)); // I dont trust the gyro for finding the target, and i dont trst the gps with spinning

	// using old values bc they faster
	double Kp = 1.1;
	double Ki = 0.2;
	double Kd = 1.25;
	double decay = 0.5; // integral sum decay.

	volatile double sum = 0;

	volatile double speed;
	volatile double error = target;
	volatile double olderror = error;
  uint32_t startTime = vex::timer::system();
	
	target += Gyro.rotation(degrees); // I trust gyro better for turning.

	while((fabs(error) > accuracy || fabs(error - olderror) > 0.05) && vex::timer::system() - startTime < maxTime) { // fabs = absolute value while loop again
		error = target - Gyro.rotation(degrees); //error gets smaller closer you get,robot slows down
		sum = sum * decay + error;
		speed = Kp * error + Ki * sum + Kd * (error - olderror); // big error go fast slow error go slow 
		drive(speed, -speed, 10);
		olderror = error;
	}
	brakeDrive();
}
bool runningAuto = 0;
/*void printPos() {
  while (true) {
    //Controller1.Screen.clearLine();
    if (!runningAuto) {
      Controller1.Screen.setCursor(0, 0);
      Controller1.Screen.print("(%0.2f, %0.2f), %0.2f", GPS.yPosition(inches) / -UNITSIZE, GPS.xPosition(inches) / UNITSIZE, GPS.rotation(degrees) - 90, Gyro.rotation(degrees));
    }
    this_thread::sleep_for(500);
  }
}
vex::thread POS(printPos);*/
//                                                        if this runs for 4.3 billion msecs, then skills is broken, and our battery is magical v
void driveTo(double x2, double y2, bool Reverse = false, bool endClaw = false, double offset = 0, double clawDist = 6, uint32_t maxTime = INF, double maxSpeed = 100, double accuracy = 0.25) {
  // point towards target
  wait(200, msec);
	// get positional data
  double x1 = -GPS.yPosition(inches), y1 = GPS.xPosition(inches);
  pointAt(x2, y2, Reverse, x1, y1, maxTime / 2);
  x2 *= UNITSIZE, y2 *= UNITSIZE;
  x1 = -GPS.yPosition(inches), y1 = GPS.xPosition(inches);

  // go to target
  // volatile double distSpeed = 100;
  double target = (1 - Reverse * 2) * (sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)) - offset);
  unitDrive(target / UNITSIZE, endClaw, clawDist, maxTime, maxSpeed, accuracy);

  /*return; // end cuz everything after this is experimental.

  // experimental

  double Kp = 10; // was previously 10
  double Ki = 2; // to increase speed if its taking too long.
  double Kd = 20; // was previously 20.0
  double decay = 0.5; // integral decay
  
  volatile double sum = 0;
          
  volatile double speed;
  volatile double error = target;
  volatile double olderror = error;

  volatile double dirError = 0;
  volatile double oldDirError = dirError;
  volatile double dirSpeed = 0;
  volatile double dirSum = 0;
  volatile double oldDir = Gyro.rotation(degrees);
  volatile double oldL = leftmiddle.position(rev) * pi * Diameter;
  volatile double oldR = rightmiddle.position(rev) * pi * Diameter;

  double dirKp = 1.1; // was previously 10
  double dirKi = 0.2; // to increase speed if its taking too long.
  double dirKd = 1.25; // was previously 20.0
  double dirDecay = 0.5; // integral decay
     
  leftDrive1.setPosition(0, rev);
  leftDrive2.setPosition(0, rev);
  leftmiddle.setPosition(0, rev);
  rightDrive1.setPosition(0, rev);
  rightDrive2.setPosition(0, rev);
  rightmiddle.setPosition(0, rev);
     
  while(fabs(error) > accuracy || fabs(speed) > 10) {
    bool overShot = fabs(degToTarget(x1, y1, x2, y2, Reverse)) > 100;
    target = -((Reverse + overShot) % 2) * sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)); // the error gets smaller when u reach ur target
    dirError = degToTarget(x1, y1, x2, y2, (Reverse + overShot) % 2);
    sum = sum * decay + error;
    dirSum = dirSum * dirDecay + dirError;

    speed = Kp * error + Ki * sum + Kd * (error - olderror); // big error go fast slow error go slow 
    dirSpeed = dirKp * dirError + dirKi * dirSum + dirKd * (dirError - oldDirError);
    drive(speed + dirSpeed, speed - dirSpeed, 10);
    olderror = error;
    oldDirError = dirError;
    
    // r = (∆L + ∆R) / 2(θ - θ')
    // P' = (x + r * ( sin (θ') - sin (θ) ), y + r * ( cos (θ') - cos (θ) ) )

    // to ensure nothing is undefined and stuff

    double L = leftmiddle.position(rev) * pi * Diameter;
    double R = rightmiddle.position(rev) * pi * Diameter;
    double dir = Gyro.rotation(degrees);
    if (dir == oldDir) {
      x1 += (olderror - error) * cos(dir),
      y1 += (olderror - error) * sin(dir);
    }
    else {
      double r = (L - oldL + R - oldR) / (oldDir - dir) / 2;
      x1 += r * (sin(dir) - sin(oldDir));
      y1 += r * (cos(dir) - cos(oldDir));
    }
    oldDir = dir;
    oldL = L;
    oldR = R;

    if (endClaw && error < 0 && claw.value()) { // close claw b4 it goes backwards.
      Claw(!CLAW_OPEN);
    }
  }
  brakeDrive();*/
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

	claw.set(true); // open claw
  mogoTilt(TILT_OPEN);
  clashRoyal(false);

	//runningAuto = true;

	while (Gyro.isCalibrating() || GPS.isCalibrating()) { // dont start until gyro and GPS are calibrated
		wait(10, msec);
	}
	GPS.setRotation(GPS.rotation(degrees) - 90, degrees);
	Gyro.setRotation(GPS.rotation(degrees), degrees);

	NOTE "AUTO PLAN:";
	NOTE "START ON RED SIDE LEFT";
	/*
	  1)  PICASSO LEFT BLUE
	  2)  CLAW LEFT YELLOW
	  3)  MOGO LIFT LEFT RED 
	  4)  PLATFORM LEFT YELLOW 
	  5)  DROP LEFT RED ON RED SIDE
	  6)  SHOVE TALL YELLOW TO BLUE WITH MOGO LIFT
	  7)  CLAW + PLATFORM RIGHT YELLOW
	  8)  CLAW RIGHT BLUE
	  9)  MOGO LIFT RIGHT RED + BRING IT TO RED SIDE 
  	10) PARK ON BLUE SIDE
	*/

  // prep stuff
  liftTime(-50, 0, false);
 	brakeDrive(); // set motors to brake
  // GET LEFT YELLOW
	driveTo(-1.5,0,false,true,5); // grab it
  Lift.setPosition(0, degrees);
	liftTo(LIFT_UP, 0); // raise lift
	// LEFT BLUE
  unitDrive(-1.667);
	driveTo(-1.3,-2.5,true,true,6,3,1500);
  // FILL LEFT BLUE WITH RINGS
  driveTo(-1.8,-1);
  rings(true);
  driveTo(0,-1,false,false,6,0,INF,75);
  rings(false);
	// PLATFORM LEFT YELLOW
  driveTo(-0.15,-1.8,false,false,0,0,1300); // first value must be experimented with
	Claw(CLAW_OPEN); // drop it
  unitDrive(-0.25); // back up
  // DROP LEFT BLUE 
  turnTo(90); // set it down
  liftTo(-10,0); // lower lift
  mogoTilt(TILT_OPEN);
  unitDrive(0.25);
	// GET TALL MOGO & BRING TO OTHER SIDE
  driveTo(0,0,false,true,0,6);
  liftTo(15,20); // raise lift. Reduces friction.
  driveTo(-1, 1.5);
  turnTo(90); // point away from the next goal. We need to drop the tall goal
  liftTo(-10,0); // lower lift
  liftWait(2); // wait until lift is lowered
  Claw(CLAW_OPEN); // drop the mogo
  // GET LEFT RED
  driveTo(-2.5,1.5,true,true,3,3,2000);
  liftTo(20,0);
  // FILL LEFT RED WITH RINGS 
  rings(true); // start intake
  driveTo(-2,0);
  driveTo(1.5, 0, false, false, 8);
  // GET RIGHT YELLOW
  unitDrive(-0.25); // back up bc we're probably gonna hit the mogo or smthn if we dont
  liftTo(-10,0); // lower lift
  liftWait(2); // wait until lift is lowered
  rings(false); // turn off intake
  unitDrive(2 / 3, true, 4); // get it
  // PLATFORM RIGHT YELLOW
  liftTo(LIFT_UP, 0);
  driveTo(0.6,-1.8, false, false, 0, 0, 2500); // go to platform
	Claw(CLAW_OPEN); // drop it
  // RELOCATE LEFT RED TO CLAW
  unitDrive(-0.5); // BACK UP
  liftTo(-10, 0); // lower lift
  mogoTilt(TILT_OPEN); // drop it
  unitDrive(0.25); // GIVE ENOUGH SPACE TO TURN
  gyroturn(180); // turn around
  liftWait(5); // ensure lift is low enough
  unitDrive(0.4, true, 4); // get it with claw
  // PLATFORM LEFT RED
  liftTo(LIFT_UP, 0); // raise lift
  driveTo(0.8,-1.8, false, false, 0, 0, 1500); // go to platform 
  Claw(CLAW_OPEN); // drop the goal
  // GET LEFT BLUE
  unitDrive(-0.25); // back up
  liftTo(-10, 0); // lower lift
  pointAt(-0.75, -1.5); // point at the mogo
  liftWait(10); // ensure that the lift is low enough
  driveTo(-0.75, -1.5, false, true, 0, 3); // get the mogo
  // GET RIGHT RED
  liftTo(LIFT_UP, 0); // raise lift
  driveTo(1, 1.75, true); // align on the diagonal
  driveTo(5 / 3, 2.5, true, true, 6, 3, 1500); // get it
  unitDrive(0.25); // back up
  rings(true); // turn on intake
  driveTo(1.5,-1.75); // bring it back to red ride
  // GET RIGHT BLUE
  mogoTilt(TILT_OPEN); // drop the mogo
  driveTo(2.5, -1.5, true, true, 3, 3, 1333); // get it
  // FILL RIGHT BLUE WITH RINGS
  driveTo(2, -1.375,false,false, 0, 0, 1333); // align with rings
	driveTo(2, 2.7, false, false, 0, 0, 4000); // use wall to align with the platform.
	// ALIGN FOR PARKING
	unitDrive(-3.5 / UNITSIZE); // back up from wall
	turnTo(-90); // point straight
	unitDrive(0.4); // approach platform
  liftTo(-10, 0); // bring down the platform. wait till it's done
  liftWait(5, 2667); // wait for lift to lower. But not forever.
	liftTime(0, 0); // allow lift to get shoved a bit up.
  // PARK
  unitDrive(49 / UNITSIZE); // hopefully goes to the middle
  wait(750,msec); // wait before continuing
  balance(); // just in case its not balanced.
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
		int8_t tmp, ringSpeed = 66;
    // mogoTilt controls
    if (!Controller1.ButtonR2.pressing()) {
      r2Down = false;
    }
    else if (!r2Down) {
      mogoTilt(!MogoTilt1.value());
      r2Down = true;
    }
    // ring controls
    if (!Controller1.ButtonR1.pressing()) {
      r1Down = false;
    }
    else if (!r2Down) {
      ringsOn = !ringsOn;
      r1Down = true;
    }
    rings(ringsOn);
		// lift control
		tmp = 100 * (Controller1.ButtonL1.pressing() - Controller1.ButtonL2.pressing());
		if (tmp == 0) {
			Lift.stop(hold);
		}
		else {
			Lift.spin(forward, tmp, percent);
		}
  
		if (Controller1.ButtonY.pressing()) { // hook down
			ringsOn = false;
		}
		else if (Controller1.ButtonB.pressing()) { // hook up or maybe the opposite 
			ringsOn = true;
      ringSpeed = -66;
		}

		if (Controller1.ButtonX.pressing()) { // claw close
			claw.set(false);
		}
		else if (Controller1.ButtonA.pressing()) { //claw open
			claw.set(true);
		}

		if (Controller1.ButtonUp.pressing()) { // picasso
			clashRoyal(true);
		}
		else if (Controller1.ButtonRight.pressing()) { // un-picasso
			clashRoyal(false);
		}
    // to find the correct side for auto
    if (Controller1.ButtonDown.pressing()) {
      driveTo(-2,-1); // should be able to tell which side to go to from this.
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
