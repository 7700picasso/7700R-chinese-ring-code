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
// leftDrive1           motor         4               
// leftDrive2           motor         6               
// leftmiddle           motor         5               
// rightDrive1          motor         1               
// rightDrive2          motor         3               
// rightmiddle          motor         2               
// Lift                 motor         9               
// Gyro                 inertial      10              
// GPS                  gps           8               
// DistFront            distance      7               
// MogoTilt             digital_out   C               
// Forklift             digital_out   F               
// Rings                motor         20              
// claw1                digital_out   E               
// Vision               vision        19              
// VisionBack           vision        12              
// GPSR                 gps           18              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <math.h>
#include <array>

using namespace vex;

// A global instance of competition, dont touch either
competition Competition;

// define your global Variables here
char *str = "";
const double pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825; // much more accurate than 3.14. accurate enough to go across the universe and be within an atom of error

#define Diameter 28 / 5
#define UNITSIZE 23.75 // tile size
#define MOGO_DIST 5
#define NOTE str = 
#define INF 4294967295
#define CLAW_OPEN false
#define TILT_OPEN false
#define FORK_DOWN true
#define LIFT_UP 66
#define DIAG sqrt(2)
#define High_Open true
#define SPEED_CAP 100
#define WIDTH 11.75
#define RAD * pi / 180
#define DEG * 180 / pi
#define INFTSML 0.00000000000000000001
#define RING_SPEED 85
#define RED 1
#define BLUE 2
#define YELLOW 3

#define doThePIDThing                                              \
	sum = sum * decay + error;                                       \
	speed = Kp * error + Ki * sum + Kd * (error - olderror);

std::pair<double, double> Pos = {-1.875 * UNITSIZE, -2.5 * UNITSIZE};

// for red comments

void pre_auton(void) {
  vexcodeInit();
  Gyro.calibrate();
  GPS.calibrate();
  //picasso.set(false);
	claw1.set(CLAW_OPEN);
  MogoTilt.set(TILT_OPEN);
  wait(2000, msec);

  // All activities that occur before the competition starts
  // gets pistons down before match
  // dont touch this 
	
	// BOOOOOOP whoops i touched it
}

double wheelRevs(uint8_t idx) {    
  double rots[] = {
    leftDrive1.position(rev), leftDrive2.position(rev), leftmiddle.position(rev),
    rightDrive1.position(rev), rightDrive2.position(rev), rightmiddle.position(rev)
  };
  double counts[4] = {rots[0],rots[0],rots[0],rots[3]};
  
  for (uint8_t i = 1; i < 6; i++) {
    if (rots[i] < counts[0]) {
      counts[0] = rots[i];
    }
    if (rots[i] > counts[1]) {
      counts[1] = rots[i];
    }
  }
  for (uint8_t i = 0; i < 3; i++) {
    counts[2] += rots[i];
    counts[3] += rots[i + 3];
  }
  return counts[idx];
}

double mod(double a, double b) {
  return (a < 0) * b + a - b * (floor(a / b) + (a < 0));
}

int8_t sgn(double x) {
  return x > 0 ? 1 : x < 0 ? -1 : 0;
}
double dir(double x) {
  return mod(x - 180, 360) - 180;
}

double cot(double x) {
  return mod(x, 180) != 0 ? tan(90 - x) : INF;
}

std::array<double,2> calcArc(double dx, double dy, double theta = 90 - Gyro.rotation(deg), double w = WIDTH / 2) {
	double targetDir = atan2(dy, dx) DEG, oX, oY;
	
  if (mod(theta,180) != mod(targetDir, 180)) {
		double m = -cot(theta RAD);
		if (dy != 0) {
			double n = -dx / dy;
			double c = (dx * dx + dy * dy) / (2 * dy);

			oX = c / (m - n);
			oY = n * oX + c;
		}
		else {
			oX = dx / 2, oY = m * oX; // i calculated this limit
		}
    double r = sgn(dir(atan2(dy, dx) DEG - theta)) * sqrt(oX * oX + oY * oY);
    double deltaTheta = dir(2 * (atan2(-dy, -dx) DEG - theta));
    return {deltaTheta * (r - w) RAD, deltaTheta * (r + w) RAD};
  }
  else { // if its pointing straight at it, then we'll get some errors
    double dist = (mod(theta, 360) == mod(targetDir, 360) ? 1 : -1) * sqrt(dx * dx + dy * dy);
    return {dist, dist};
  }
}

std::pair<double, double> calcPos(double dL, double dR, double theta = 90 - Gyro.rotation(deg), double w = WIDTH / 2) {
  if (dL != dR) {
    double r = w * (dR + dL) / (dR - dL);
    double theta2 = 90 * (dR - dL) / pi / w + theta - 90;
    double oX = -r * sin(theta RAD), oY = r * cos(theta RAD);
    return {oX + r * cos(theta2 RAD), oY + r * sin(theta2 RAD)};
  }
  else {
    return {dL * cos(theta RAD), dL * sin(theta RAD)};
  }
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

double perpDist(double x1, double y1,double x2, double y2) {
  double m = -tan(Gyro.rotation(deg) RAD), n = y2 - m * x2;
  return fabs(m * x1 - y1 + n) / sqrt(m * m + n * n);
}

double xPos() {
  //return Pos.first;
  // check odometry values
  if (degToTarget((GPSR.yPosition(inches) + GPSR.yPosition(inches)) / 2, -(GPSR.xPosition(inches) + GPS.xPosition(inches)) / 2, 0, 0) >= 0) {
    return GPSR.yPosition(inches);
  }
  return GPS.yPosition(inches);
}

double yPos() {
  // check odometry values
  //return Pos.second;
  if (degToTarget((GPSR.yPosition(inches) + GPSR.yPosition(inches)) / 2, -(GPSR.xPosition(inches) + GPS.xPosition(inches)) / 2, 0, 0) >= 0) {
    return -GPSR.xPosition(inches);
  }
  return -GPS.xPosition(inches);
}

std::array<double,8> getTemp() {
  std::array<double,8> temps = {
    leftDrive1.temperature(temperatureUnits::celsius),
    leftDrive2.temperature(temperatureUnits::celsius),
    leftmiddle.temperature(temperatureUnits::celsius),
    rightDrive1.temperature(temperatureUnits::celsius),
    rightDrive2.temperature(temperatureUnits::celsius),
    rightmiddle.temperature(temperatureUnits::celsius),
    0, 0
  };
  for (uint8_t i = 0; i < 6; i++) {
    temps[6] += temps[i];
    temps[8] = std::max(temps[8],temps[i]);
  }
  temps[6]/=7;
  return temps;
}

void drive(int lspeed, int rspeed, int wt){
  double L = leftDrive1.position(deg) + leftDrive2.position(deg) + leftmiddle.position(deg);
  double R = rightDrive1.position(deg) + rightDrive2.position(deg) + rightmiddle.position(deg);

  leftDrive1.spin(forward, lspeed, pct);
  leftDrive2.spin(forward, lspeed, pct);
  leftmiddle .spin(forward, lspeed, pct);
  rightDrive1.spin(forward, rspeed, pct);
  rightDrive2 .spin(forward, rspeed, pct);
  rightmiddle .spin(forward, rspeed, pct);
  wait(wt, msec);

  L = (leftDrive1.position(deg) + leftDrive2.position(deg) + leftmiddle.position(deg) - L) / 3 * Diameter * pi; // get average change on left side
  R = (rightDrive1.position(deg) + rightDrive2.position(deg) + rightmiddle.position(deg) - R) / 3 * Diameter * pi; // get average change on right side

  // odometry
  std::pair<double,double> dPos = calcPos(L,R);
  Pos.first += dPos.first;
  Pos.second += dPos.second;
}
//use to go forward,backwards,left right etc,turning if your stupid
//use mmDrive to go forward and backwards,use gyro to turn

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
  
  if (WT != -1) {
		Lift.startSpinFor(forward, 7 * angle, degrees);
    wait(WT,msec);
  }
  else {
		Lift.spinFor(forward, 7 * angle, degrees, true);
    Lift.stop(hold);
  }                                            //more math 
}

void liftTo(double angle, int WT = -1, int8_t speed = 100) {
  Lift.setVelocity(speed, percent);
  Lift.setStopping(hold);
  
  if (WT != -1) {
		Lift.startSpinFor(forward, 7 * angle - Lift.position(deg), degrees);
    wait(WT,msec);
  }
  else {
		Lift.spinFor(forward, 7 * angle - Lift.position(deg), degrees, true);
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
  uint32_t startTime = timer::system();
  while (Lift.position(degrees) * 7 > target && timer::system() - startTime < maxTime) {
    wait(10, msec);
  }
}

//makes lift go up or down
// its the lift speed then wait time
//example lift(-100,1200);  so lift 100% for 1200 msc
// 100 is up and -100 is down,or other way around,you can figure that out

void rings(bool on, double speed = RING_SPEED) { // i think 100 is a bit fast
  if (on) {
    Rings.spin(forward, on * speed, percent);
  }
  else {
    Rings.spin(forward,0,percent);
  }
}

void Claw(bool open) {
  claw1.set(open);
}

//idk maybe opens and closes the claw
//true is open and false is close
//examples
//claw.set(true);    open
//claw.set(false);   close

void mogoTilt(bool state) {
  MogoTilt.set(state);
}

void Fork(bool state) {
  Forklift.set(state);
}

void EndClaw(uint8_t clawID, double clawDist = 0, double error = 0) {
  bool claws[] = {false, claw1.value() == CLAW_OPEN, MogoTilt.value() == TILT_OPEN, Forklift.value() == FORK_DOWN}, isOpen = claws[clawID];
  if (isOpen && fabs(error) <= clawDist) {
    switch (clawID) {
      case 1: 
        Claw(!CLAW_OPEN);
        break;
      case 2:
        mogoTilt(!TILT_OPEN);
        break;
      case 3: 
        Fork(!FORK_DOWN);
    }
  }
}

double getTrackSpeed(uint8_t trackingID = 0, bool back = false) {
  if (trackingID > 0 && trackingID < 4) {
    switch (trackingID) {
      case RED:
        if (back) {
          VisionBack.takeSnapshot(Vision__MOGO_RED);
        }
        else {
          Vision.takeSnapshot(Vision__MOGO_RED);
        }
        break;
      case BLUE:
        if (back) {
          VisionBack.takeSnapshot(Vision__MOGO_BLUE);
        }
        else {
          Vision.takeSnapshot(Vision__MOGO_BLUE);
        }
        break;
      case YELLOW: 
        if (back) {
          VisionBack.takeSnapshot(Vision__MOGO_YELLOW);
        }
        else {
          Vision.takeSnapshot(Vision__MOGO_YELLOW);
        }
        break;
    }
  }
  else {
    Brain.Screen.drawCircle(240, 136, 100,black);
    return 0;
  }
  Brain.Screen.drawCircle(240, 136, 100,red);
  double turnSpeed = 0;
  bool exists = (!back && Vision.largestObject.exists && Vision.largestObject.width > 20) || (back && Vision.largestObject.exists && Vision.largestObject.width > 20);
  if (exists) {
    // get position
    const double center = 158;
    const double Kp = 0.3;
    // Then get the turning speed with proportionality
    const double centerX = back ? VisionBack.largestObject.centerX : Vision.largestObject.centerX;
    turnSpeed = Kp * (centerX - center);
    Brain.Screen.drawCircle(240, 136, 100,green);
    //
  }
  return turnSpeed;
}

void unitDrive(double target, uint8_t endClaw = false, double clawDist = 1, uint32_t maxTime = INF, double maxSpeed = 90, bool raiseMogo = false, double mogoHeight = 100, uint8_t trackingID = 0, double accuracy = 0.25) {
	double Kp = 10; // was previously 50/3
	double Ki = 1.5; // to increase speed if its taking too long.
	double Kd = 20; // was previously 40/3
	double decay = 0.5; // integral decay
	bool lifted = false;
	
	target *= UNITSIZE; // convert UNITS to inches
	
	volatile double speed;
  double turnSpeed = 0;
	volatile double error = target;
	volatile double olderror = error;
	 
  leftDrive1.setPosition(0, rev);
	leftDrive2.setPosition(0, rev);
  leftmiddle.setPosition(0, rev);
  rightDrive1.setPosition(0, rev);
  rightDrive2.setPosition(0, rev);
  rightmiddle.setPosition(0, rev);
	 
  volatile double sum = 0;
	uint32_t startTime = timer::system();
  bool isOpen;

  while((fabs(error) > accuracy || fabs(speed) > 10) && timer::system() - startTime < maxTime) {
    // did this late at night but this while is important 
    error = target - wheelRevs(0) * Diameter * pi; //the error gets smaller when u reach ur target
    doThePIDThing
    speed = !(fabs(speed) > maxSpeed) ? speed : maxSpeed * sgn(speed);

    turnSpeed = (error - clawDist < 48 && error - clawDist > 0) * (endClaw == 2 ? -1 : 1) * getTrackSpeed(trackingID, endClaw == 2); // follow mogo if you want to

    drive(speed + turnSpeed, speed - turnSpeed, 10);
    olderror = error;
    isOpen = target > 0 ? claw1.value() == CLAW_OPEN : MogoTilt.value() == TILT_OPEN;
    EndClaw(endClaw, clawDist, error);
    if (raiseMogo && ((!isOpen && (error + 6 < clawDist)) || endClaw != 1) && !lifted) {
      liftTo(mogoHeight,0);
			lifted = true;
    }
  }
	brakeDrive();
  EndClaw(endClaw);
}

void unitArc(double target, double propLeft=1, double propRight=1, bool trueArc = true, bool endClaw = false, double clawDist = 1, uint32_t maxTime = INF, double maxSpeed = SPEED_CAP, bool raiseMogo = false, double mogoHeight = 100, double accuracy = 0.5) {
	double Kp = 10; // was previously 50/3
	double Ki = 1.5; // to increase speed if its taking too long.
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
  uint32_t startTime = timer::system();
  bool isOpen;
	bool lifted = false;
	 
  while((fabs(error) > accuracy || fabs(speed) > 10) && timer::system() - startTime < maxTime) {
    // did this late at night but this while is important 
    error = target - wheelRevs(1) * Diameter * pi; //the error gets smaller when u reach ur target
    doThePIDThing
    if (trueArc) {
      speed = !(fabs(speed) > maxSpeed) ? speed : maxSpeed * sgn(speed);
    }
    drive(speed * propLeft, speed * propRight, 10);
    olderror = error;
    isOpen = target > 0 ? claw1.value() == CLAW_OPEN : MogoTilt.value() == TILT_OPEN;
    EndClaw(endClaw,clawDist,error);
    if (raiseMogo && ((!isOpen && (error + 6 < clawDist)) || endClaw != 1) && !lifted) {
      liftTo(mogoHeight,0);
			lifted = true;
    }
  }
	brakeDrive();
  EndClaw(endClaw);
}

void arcTurn(double target, double propLeft=1, double propRight=1, bool endClaw = false, double clawDist = 1, uint32_t maxTime = INF, double maxSpeed = SPEED_CAP, bool raiseMogo = false, double mogoHeight = 100, double accuracy = 1.25) {
	double Kp = 10; // was previously 50/3
	double Ki = 1.5; // to increase speed if its taking too long.
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
  uint32_t startTime = timer::system();
  bool isOpen;
	bool lifted = false;
	 
  while((fabs(error) > accuracy || fabs(speed) > 10) && timer::system() - startTime < maxTime) {
    error = target - dir(Gyro.rotation(deg)); // the error gets smaller when u reach ur target
    doThePIDThing // call macro
    speed = !(fabs(speed) > maxSpeed) ? speed : maxSpeed * sgn(speed);
    drive(speed * propLeft, speed * propRight, 10);
    olderror = error;
    isOpen = target > 0 ? claw1.value() == CLAW_OPEN : MogoTilt.value() == TILT_OPEN;
    EndClaw(endClaw,clawDist,error);
    if (raiseMogo && ((!isOpen && (error + 6 < clawDist)) || endClaw != 1) && !lifted) {
      liftTo(mogoHeight,0);
			lifted = true;
    }
  }
	brakeDrive();
  EndClaw(endClaw);
}

void arcTo(double x2, double y2, uint8_t endClaw = false, double clawDist = 1, uint32_t maxTime = INF, double maxSpeed = SPEED_CAP, bool raiseMogo = false, double mogoHeight = 100, double accuracy = 0.25) {
  x2 *= UNITSIZE, y2 *= UNITSIZE;
  std::array<double, 2> arc = calcArc(x2, y2);
  std::pair<double, double> pos = {0, 0};
  double Kp = 10; // was previously 50/3
	double Ki = 1; // to increase speed if its taking too long.
	double Kd = 20; // was previously 40/3
	double decay = 0.5; // integral decay
	volatile double speed;
	volatile double error = (fabs(arc[0]) > fabs(arc[1]) ? arc[0] : arc[1]);// sgn(arc[0] + arc[1]) * fmax(fabs(arc[0]), fabs(arc[1]));
	volatile double olderror = error;
  volatile double sum = 0;
  double L[] = {0,0}, R[] = {0,0};
  double speedR, speedL;
  bool isOpen;
  double theta;

  /*
  𝜃 = current direction
  w = WIDTH / 2
  x = current xPos
  y = current yPos
  x' = desired xPos
  y' = desired yPos

  FOR FINDING RATIO:
  For O:
  intersection point of y=mx+b and y=nx+c: x = (b-c)/(n-m), y = nx + c )
  m = -cot(θ),
  n = (x - x') / (y' - y),
  b = y - xm,
  c = (x'^2 + y'^2 - x^2 - y^2) / 2(y' - y)
  
  O = [ (b-c) / (n-m), nx + c) ]
  r = √{(Ox-x)^2 + (Oy-y)^2}
  ∆θ = 2 * atan2(y - y', x - x') - 2θ

  ∆R = π∆θ(r + w) / 180
  ∆L = π∆θ(r - w) / 180

  FOR FINDING POSITION:
  r = w(∆R + ∆L) / (∆R - ∆L)

  θ' = 90(∆R - ∆L) / πw + θ - 90
  ∆x = r[cos(θ') - sin(θ)]
  ∆y = r[sin(θ') + cos(θ)]
  */
	 
  leftDrive1.setPosition(0, rev);
	leftDrive2.setPosition(0, rev);
  leftmiddle.setPosition(0, rev);
  rightDrive1.setPosition(0, rev);
  rightDrive2.setPosition(0, rev);
  rightmiddle.setPosition(0, rev);
	 
  uint32_t startTime = timer::system();

  while (fabs(error) > 0.5 && timer::system() - startTime < maxTime) {
    // get base speed
    doThePIDThing
    speed = fabs(speed) > maxSpeed ? sgn(speed) * maxSpeed : speed; // lower speed to maxSpeed
    
    // do the speeds
    if (fabs(arc[0]) > fabs(arc[1])) {
      speedL = speed;
      speedR = speed * arc[1] / arc[0];
    }
    else {
      speedR = speed;
      speedL = speed * arc[0] / arc[1];
    }
    // drive
    theta = 90 - Gyro.rotation(deg);
    drive(speedL, speedR, 10);
    L[0] = wheelRevs(2);
    R[0] = wheelRevs(3);
    int idx = 1;
    pos = calcPos((L[0] - L[idx]) * Diameter * pi, (R[0] - R[idx]) * Diameter * pi, theta);
    arc = calcArc(x2 - pos.first, y2 - pos.second);
    olderror = error;
    error = fabs(arc[0]) > fabs(arc[1]) ? arc[0] : arc[1];
    L[1] = L[0];
    R[1] = R[0];
    bool claws[3] = {false, claw1.value() == CLAW_OPEN, MogoTilt.value() == TILT_OPEN};
    isOpen = claws[endClaw];
    EndClaw(endClaw,clawDist,fabs((arc[0] + arc[1]) / 2));
    if (raiseMogo && !isOpen && ((fabs(arc[0] + arc[1]) / 2) + 6 < clawDist) && Lift.position(degrees) < 10) {
      liftTo(mogoHeight,0);
    }
  }
  EndClaw(endClaw);
	brakeDrive(); // then stop
}

void balance() {
  Brain.Screen.clearScreen();
  //double Kp = 2;
  //double Kt = 0.15; // constant for tip counts. This acts like Ki.
  //volatile double speed;
  volatile double pitch = Gyro.pitch(degrees);
	const double stop = 20.5;
	double back = 0, min = -6, max = 6; // these are reasonable boundaries.
	int8_t sgnPitch = sgn(pitch), sgnTip = sgnPitch;
	
	while (true) {
		pitch = Gyro.pitch(degrees);
		sgnPitch = sgn(pitch);
		if (fabs(pitch) > stop) {
      int8_t speed = sgnPitch * 100;
			drive(speed,speed,10);
			sgnTip = sgnPitch;
		}
		else {
			unitDrive(-sgnPitch * back / UNITSIZE);
      uint32_t startTime = timer::system();
			while (fabs(Gyro.pitch(degrees)) < stop) { // this should end with us waiting forever when its balanced
				wait(10,msec);
				if (fabs(Gyro.pitch(degrees)) < 3) {
					Controller1.Screen.setCursor(0,0);
					Controller1.Screen.print(back);
				}
        if (timer::system() - startTime > 1000) {
          return;
        }
				sgnPitch = sgn(Gyro.pitch(degrees));
			}
			// binary search the value for back.
			if (sgnPitch != sgnTip) { // if it didn't back up enough
				min = back;
				back = (max + back) / 2; // slice the step in half
			}
			else { // if it backed up too much
				max = back;
				back = (min + back) / 2;  // slice the step in half
			}
		}
	}
}

void gyroturn(double target, double maxSpeed = 90, uint32_t maxTime = 750, double accuracy = 2) { // idk maybe turns the robot with the gyro,so dont use the drive function use the gyro
  double Kp = 0.6;
  double Ki = 0;
  double Kd = 5;
  double decay = 0; // integral decay
	
  volatile double sum = 0;
  volatile double speed;
  volatile double error = target;
  volatile double olderror = error;
  uint32_t startTime = timer::system();

  target += Gyro.rotation(degrees);

  while (fabs(error) > accuracy && timer::system() - startTime < maxTime) {
    error = target - Gyro.rotation(degrees);; //error gets smaller closer you get,robot slows down
    doThePIDThing
    speed = !(fabs(speed) > maxSpeed) ? speed : maxSpeed * sgn(speed);
    drive(speed, -speed, 10);
    olderror = error;
  }
}

void turnTo(double target, double maxSpeed = 90, double maxTime = 750, double accuracy = 2) {
  gyroturn(mod(target - Gyro.rotation(degrees) - 180, 360) - 180, maxSpeed, maxTime, accuracy);
}

void pointAt(double x2, double y2, bool Reverse = false, uint32_t maxSpeed = 88, uint32_t maxTime = 750, double x1 = xPos(), double y1 = yPos(), double accuracy = 2) { 
	// point towards target
  x2 *= UNITSIZE, y2 *= UNITSIZE;
	double target = degToTarget(x1, y1, x2, y2, Reverse); // I dont trust the gyro for finding the target, and i dont trst the gps with spinning
	if (fabs(target) < 3) {
    return;
  }
  gyroturn(target,maxSpeed,maxTime,accuracy);
}

void driveTo(double x2, double y2, bool Reverse = false, uint8_t endClaw = false, double offset = 0, double clawDist = 3, uint32_t maxTime = 4000, double maxSpeed = 90, bool raiseMogo = false, double mogoHeight = 67, uint8_t trackingID = 0, double accuracy = 0.25) {
	// turning
  pointAt(x2, y2, Reverse, 100, 500, xPos(), yPos(),1.5); // roughly face it and let error correction take over

  // get positional data
  x2 *= UNITSIZE, y2 *= UNITSIZE;

  //const double Kp = 10, Ki = 1.5, Kd = 20, decay = 0.5, Tp = 0.4;

  double x1 = xPos(), y1 = yPos();
  //double startTime = timer::system();
  double error = (1 - Reverse * 2) * (sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)) - offset);
  /*double olderror;
  double sum = 0;
  double speed;
  double turnSpeed = 0;
  uint32_t ticks = 0;
  ;
  while (((perpDist(x1, y1, x2, y2) < accuracy && error < 3) || (claw1.value() == CLAW_OPEN && endClaw == 1)) && timer::system() - startTime > maxTime) {
    // POSITION
    x1 = GPS.yPosition(inches), y1 = -GPS.xPosition(inches);
    // sanity check
    if (fabs(x1 - Pos.first) > 8) { // 0.5 tiles off isn't too much for odometric comparison
      // use odometry values
      x1 = Pos.first;
    }
    if (fabs(y1 - Pos.second) > 8) { // 0.5 tiles off isn't too much for odometric comparison
      // use odometry values
      y1 = Pos.second;
    }
    Pos = {x1, y1}; // update odometry
    // PID
    olderror = error; // update olderror
    error = (1 - Reverse * 2) * (sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)) - offset); // update error
    doThePIDThing // do the PID thing
    speed = (fabs(speed) > maxSpeed ? sgn(speed) * maxSpeed : speed); // apply speed cap
    // PATH CORRECTION
    turnSpeed = (getTrackSpeed(trackingID, Reverse) + !trackingID * Tp * degToTarget(x1, y1, x2, y2)) * (30 + maxSpeed * 0.7) / 100; // First try tracking the goal. Otherwise follow the path. Ensure that it doesn't be too wobbly
    // LIFT & CLAW
    EndClaw(endClaw, clawDist, error); // end claw
    if (raiseMogo && ((endClaw == 1 && claw1.value() != CLAW_OPEN && error < clawDist) || endClaw != 1)) { // lift
      liftTo(mogoHeight, 0);
    }
    // FINISH TICK
    drive(speed + turnSpeed, speed - turnSpeed, 10);
    ticks++;
  }
  EndClaw(endClaw); // end claw if this has not been done already.
  brakeDrive(); // then stop*/
  unitDrive(error / UNITSIZE, endClaw, clawDist, maxTime, maxSpeed, raiseMogo, mogoHeight, trackingID, accuracy);
}

void outake() { // i did not spell this correctly but it looks better this way
  bool jammed = false;
  uint32_t startTime;
  while(true) {
    // if its still jammed after outaking then it resumes outaking
    if (this_thread::priority() != 1 && fabs(Rings.velocity(rpm)) < 2) {
      if (!jammed) {
        startTime = timer::system();
      }
      jammed = true;
      if (timer::system() - startTime > 250) {
        rings(true,-100); // outake
        this_thread::sleep_for(250); // wait a bit
        rings(true); // resume intake
      }
    }
    else {
      jammed = false;
    }
    this_thread::sleep_for(10); // wait for a bit
  }
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
  Fork(!FORK_DOWN);

	//runningAuto = true;

	while (Gyro.isCalibrating() || GPS.isCalibrating()) { // dont start until gyro and GPS are calibrated
		wait(10, msec);
	}
	Gyro.setRotation(GPS.rotation(degrees) + 90, degrees);

  if (mod(Gyro.rotation(deg), 360) - 90 > 10) { // sanity check
	  Gyro.setRotation(-90, degrees);
  }
  thread OUTAKE(outake);
  OUTAKE.thread::setPriority(1);
	Pos = {GPSR.yPosition(inches), -GPSR.xPosition(inches)};

  uint32_t startTime = timer::system();
  Lift.setPosition(0, degrees); // set lift rotations
  // GET LEFT RED
  unitDrive(-0.2,2,0); // tilt it
  liftDeg(30,0); // raise lift a bit
  rings(true); // intake on
  unitDrive(0.75,false,0,1000,33); // get rings
  OUTAKE.thread::setPriority(7);
  unitDrive(-0.25); // back up to get space
  // CLAW LEFT YELLOW
  unitArc(1,1,0); // face the goal
  Lift.spin(forward,-100,percent); // lower lift
  unitDrive(2.1,1,3,1500,87,true,80,YELLOW); // get it
  liftTo(75,0); // raise lift
  // PLATFORM LEFT YELLOW
  turnTo(20,100,500); // fix direction
  unitArc(1.2,1,0.41); // curve to face the rings
  turnTo(90);
  unitDrive(1.25,false,0,INF,50); // some rings
  unitDrive(-0.25); // some rings
  mogoTilt(TILT_OPEN); // drop the back goal
  pointAt(-0.05,2.5);
  turnTo(5);
  unitDrive(1,false,0,875);
  driveTo(0,2.5,false,0,0,0,875); // platform it
  liftTime(-100, 400);
  Claw(CLAW_OPEN); // drop it
  turnTo(20);
  liftTo(70,0); // raise lift a bit
  wait(200,msec);
  // PLATFORM LEFT RED
  unitArc(-0.5,1,0.5);//unitArc(-0.5, 1, 179 / 367, true,false,0,400); // back up
  liftTo(-10,0); // lower lift
  turnTo(-165); // face it
  unitDrive(1.25,1,3,1000,75,false,0,RED); // get it
  // platform it
  liftTo(70,0); // raise lift
  pointAt(0.25,2.5);
  driveTo(0.25,2.5,false,0,6,0,1300,75); // go there
  Claw(CLAW_OPEN); // drop it
  //liftDeg(10,0); // raise lift
  wait(200,msec); // dont fall over lol
  // GET RIGHT YELLOW
  turnTo(-20,100);
  unitArc(-0.75, 1, 0.3,true,false,0,1000); // back up + "align"
  liftTo(-10,0);
  driveTo(1.5,1,true,false,0,0,1000); // for accuracy
  turnTo(180);
  unitDrive(2.5,1,40,INF,100,false,0,YELLOW); // claw it
  // TILT RIGHT RED
  turnTo(-90);
  unitDrive(-1.5,2,1,875,75,true,20,RED); // tilt it
  liftTo(20,0); // raise lift
  turnTo(-3); // face rings
  unitDrive(1.5,0,0,INF,67); // rings
  // PLATFORM RIGHT YELLOW
  liftTo(70,0);
  pointAt(0.3,2.5);
  driveTo(0.3,2.5,false,0,0,0,1250,87); // go to platform
  wait(200,msec);
  Claw(CLAW_OPEN); // drop it
  // PLATFORM RIGHT RED
  turnTo(10);
  unitArc(-0.5,0.25, 1,true,false, 0,875,100,true,-1); // back up
  liftTo(-10,0); // lower lift
  turnTo(90,100,1000);
  // swap to claw
  unitDrive(-0.75); // plz be a better place to drop it.
  mogoTilt(TILT_OPEN); // drop it. PLEASE DONT LAND ON A RING. PLEEEEEEAAAAAASSSSSEEE
  unitDrive(0.333); // give clearance
  gyroturn(180); // face it
  unitDrive(0.75,1,3,INF,87,true, 75, RED); // get it with claw
  // now platform it
  liftTo(70,0);
  pointAt(-0.3, 2.5);
  driveTo(-0.3, 2.5, false, false, 8, 0, 1000, 67); // go to platform
  Claw(CLAW_OPEN); // drop it
  wait(200,msec);
  // CLAW MID
  unitDrive(-0.75); // back up
  liftTo(-10,0);
  pointAt(0,0);
  driveTo(0,0, false, 1, -4,1, INF, 75, true, 90, YELLOW); // claw it
  liftTo(7,0);
  // PLATFORM MID
  pointAt(-2,2.5,true); // face it once
	driveTo(-2, 2.5, true,0,0,0,1300,67); // go to platform
  turnTo(90);
  liftTo(70, true); // raise lift
  unitDrive(0.75,false,0,1000,50);
  Claw(CLAW_OPEN); // drop it
  wait(200,msec);
  unitDrive(-1,false,0,1000,75); // back up
  // TILT LEFT BLUE
  driveTo(-2, 1.5, true, 2, 8, 1, 875, 67, true, 20, BLUE); // tilt it
  // CLAW RIGHT BLUE
  driveTo(2,1); // go to the other side
	liftTo(-10,0); // lower lift
	pointAt(1.25, 2.5); // face it once
  driveTo(1.25, 2.5, false, 1, 5, 2, 1500, 50, false, 0, BLUE); // face it twice. get it
  // ALIGN FOR BALANCE
  unitDrive(-1); // back up
  driveTo(2,-3,false,0,0,0,1500,100,true,67); // go mostly there
  unitDrive(-0.1); // back up
  gyroturn(90); // point straight at the platform
  // Lower the platform
  liftTo(-10,500); // lower lift
	Lift.setMaxTorque(0,Nm); // bye bye torque
  unitDrive(1.4); // go up the platform
	Controller1.Screen.setCursor(0, 0);
  Controller1.Screen.print((timer::system() - startTime) * 0.001);
  balance();
}

//driver controls,dont change unless your jaehoon or ben
void driver() {
  // User control code here, inside the loop
  //2 joy sticks
  //rstick is axis 2 and lstick is axis 3,why its 2,3 and not 1,2 idk ask vex
  coastDrive(); // set drive motors to coast

  while (Gyro.isCalibrating() || GPS.isCalibrating()) { // dont start until gyro is calibrated
    wait(10, msec);
  }
  Gyro.setRotation(GPS.rotation(degrees) + 90, degrees);
  //Controller1.Screen.print("%0.3f", Gyro.rotation(deg));
  bool r2Down = false;
  bool r1Down = false;

  bool ringsOn = false;
  double ticks = 0;

  //thread OUTAKE(outake);

  while (true) {
    // drive control
    //OUTAKE.thread::setPriority(1);
		int rstick=Controller1.Axis2.position();
		int lstick=Controller1.Axis3.position();
		drive(lstick, rstick,10);
		double tmp, ringSpeed = RING_SPEED;
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
    // forklift controls
    if (Controller1.ButtonUp.pressing()) { 
      Forklift.set(!FORK_DOWN);
		}
		else if (Controller1.ButtonRight.pressing()) { 
      Forklift.set(FORK_DOWN);
		}
    // PROGRAM TESTING
    getTrackSpeed(2);
    if (Controller1.ButtonDown.pressing()) {
      auton();
    }
    if (Controller1.ButtonLeft.pressing() && ticks > 6.667) {
      Controller1.Screen.setCursor(0,0);
      Controller1.Screen.print("%f",getTemp());
      ticks = 0;
    }
		wait(20, msec); // dont waste air 
    ticks++;
  }
}
  
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(auton);
  Competition.drivercontrol(driver);

  // Run the pre-autonomous function.
  pre_auton();
  
  // end
  while (true) {
    wait(100,msec);
  }
}
