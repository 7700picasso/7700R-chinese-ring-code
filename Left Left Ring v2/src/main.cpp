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
// MogoTilt             digital_out   C               
// Rings                motor         20              
// claw1                digital_out   E               
// Vision               vision        17              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "Vision.h"
#include <math.h>
#include <array>

using namespace vex;

// A global instance of competition, dont touch either
competition Competition;

// define your global Variables here
std::string str;
const long double pi = 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825; // much more accurate than 3.14. accurate enough to go across the universe and be within an atom of error
bool deployed = false;

#define Diameter 28 / 5
#define UNITSIZE 23.75 // tile size
#define MOGO_DIST 5
#define NOTE str = 
#define INF 4294967295
#define CLAW_OPEN true
#define TILT_OPEN false
#define LIFT_UP 66
#define DIAG sqrt(2)
#define High_Open true
#define SPEED_CAP 100
#define WIDTH 11.75
#define RAD * pi / 180
#define DEG * 180 / pi
#define INFTSML 0.00000000000000000001
#define RING_SPEED 85
#define doThePIDThing                                              \
	sum = sum * decay + error;                                       \
	speed = Kp * error + Ki * sum + Kd * (error - olderror);

#define RED 1
#define BLUE 2
#define YELLOW 3

// for red comments

void pre_auton(void) {
  vexcodeInit();
  Gyro.calibrate();
  GPS.calibrate();
  //picasso.set(false);
	claw1.set(CLAW_OPEN);
  MogoTilt.set(TILT_OPEN);
  Lift.setStopping(hold);
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

std::array<long double,2> calcArc(double dx, double dy, double theta = 90 - Gyro.rotation(deg), double w = WIDTH / 2) {
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

std::array<double, 2> calcPos(double deltaL, double deltaR, double theta = 90 - Gyro.rotation(deg), double w = WIDTH / 2) {
  if (deltaL != deltaR) {
    double r = w * (deltaR + deltaL) / (deltaR - deltaL);
    double theta2 = 90 * (deltaR - deltaL) / pi / w + theta - 90;
    double oX = -r * sin(theta RAD), oY = r * cos(theta RAD);
    return {oX + r * cos(theta2 RAD), oY + r * sin(theta2 RAD)};
  }
  else {
    return {deltaL * cos(theta RAD), deltaL * sin(theta RAD)};
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

double getSpeed(double x, double arc = 20) {
  return sgn(x) * (arc * exp(log((100 + arc) / arc) * fabs(x) / 100) - arc);
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
  uint32_t startTime = timer::system();
  while (Lift.position(degrees) * 7 > target && timer::system() - startTime < maxTime) {
    wait(10, msec);
  }
}

//makes lift go up or down
// its the lift speed then wait time
//example lift(-100,1200);  so lift 100% for 1200 msc
// 100 is up and -100 is down,or other way around,you can figure that out

void rings(bool on, int speed = RING_SPEED) { // i think 100 is a bit fast
  if (on) {
    Rings.spin(forward, on * speed, percent);
  }
  else {
    Rings.spin(forward,0,percent);
  }
}

#define deploy(check)                                         \
  if (!deployed || !check) {                                  \
    Rings.setVelocity(100, pct);                              \
    Rings.spinFor(forward, -1/6, rev,false);                  \
  }                                                           \
  deployed = true;     

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

void EndClaw(uint8_t clawID, double clawDist = 0, double error = 0) {
  bool claws[] = {false, claw1.value() == CLAW_OPEN, MogoTilt.value() == TILT_OPEN}, isOpen = claws[clawID];
  if (isOpen && fabs(error) <= clawDist && clawID != 0) {
    switch (clawID) {
      case 1: 
        Claw(!CLAW_OPEN);
        break;
      case 2:
        mogoTilt(!TILT_OPEN);
        break;
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
    const double Kp = 0.5;
    // Then get the turning speed with proportionality
    const double centerX = back ? VisionBack.largestObject.centerX : Vision.largestObject.centerX;
    turnSpeed = Kp * (centerX - center);
    Brain.Screen.drawCircle(240, 136, 100,green);
    //
  }
  return turnSpeed;
}

void unitDrive(double target, uint8_t endClaw = false, double clawDist = 1, uint32_t maxTime = INF, double maxSpeed = INF, bool raiseMogo = false, double mogoHeight = 100, uint8_t trackingID = 0, bool rush = false, double accuracy = 0.25) {
	double Kp = 10 + 4.5 * rush; // was previously 50/3
	double Ki = 1.5; // to increase speed if its taking too long.
	double Kd = 20; // was previously 40/3
	double decay = 0.5; // integral decay
  double turnSpeed = 0;
	
	target *= UNITSIZE; // convert UNITS to inches
	
	volatile double speed;
	volatile double error = target;
	volatile double olderror = error;
	 
  leftDrive1.resetPosition();
	leftDrive2.resetPosition();
  leftmiddle.resetPosition();
  rightDrive1.resetPosition();
  rightDrive2.resetPosition();
  rightmiddle.resetPosition();
	 
  volatile double sum = 0;
  uint32_t startTime = timer::system();
  bool isOpen;
  bool lifted = false;
  
  while((fabs(error) > accuracy || fabs(speed) > 10) && timer::system() - startTime < maxTime) {
    // did this late at night but this while is important 
    if (target == 0.75 * UNITSIZE && fabs(leftDrive1.velocity(rpm)) < 10 && speed > 50) {
      target += 2.1 * UNITSIZE;
    }
    error = target - wheelRevs(0) * Diameter * pi; //the error gets smaller when u reach ur target
    doThePIDThing
    speed = !(fabs(speed) > maxSpeed) ? speed : maxSpeed * sgn(speed);

    turnSpeed = (claw1.value() == CLAW_OPEN) * (endClaw == 2 ? -1 : 1) * getTrackSpeed(trackingID); // follow mogo if you want to

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

void unitArc(double target, double propLeft=1, double propRight=1, bool trueArc = true, bool endClaw = false, double clawDist = 1, uint32_t maxTime = INF, double maxSpeed = SPEED_CAP, bool raiseMogo = false, double mogoHeight = 100, double accuracy = 0.25) {
	double Kp = 10; // was previously 50/3
	double Ki = 1.5; // to increase speed if its taking too long.
	double Kd = 20; // was previously 40/3
	double decay = 0.5; // integral decay
	
	target *= UNITSIZE; // convert UNITS to inches
	
	volatile double speed;
	volatile double error = target;
	volatile double olderror = error;
	 
  leftDrive1.resetPosition();
	leftDrive2.resetPosition();
  leftmiddle.resetPosition();
  rightDrive1.resetPosition();
  rightDrive2.resetPosition();
  rightmiddle.resetPosition();
	 
  volatile double sum = 0;
  uint32_t startTime = timer::system();
  bool isOpen;
	 
  while((fabs(error) > accuracy || fabs(speed) > 10) && timer::system() - startTime < maxTime) {
    // did this late at night but this while is important 
    error = target - wheelRevs(1) * Diameter * pi; //the error gets smaller when u reach ur target
    sum = sum * decay + error;
    speed = Kp * error + Ki * sum + Kd * (error - olderror); // big error go fast slow error go slow 
    if (trueArc) {
      speed = !(fabs(speed) > maxSpeed) ? speed : maxSpeed * sgn(speed);
    }
    drive(speed * propLeft, speed * propRight, 10);
    olderror = error;
    isOpen = target > 0 ? claw1.value() == CLAW_OPEN : MogoTilt.value() == TILT_OPEN;
    EndClaw(endClaw,clawDist,error);
    if (raiseMogo && !isOpen && (error + 6 < clawDist) && Lift.position(degrees) < 10) {
      liftTo(mogoHeight,0);
    }
  }
	brakeDrive();
  EndClaw(endClaw);
}

void arcTurn(double target, double propLeft=1, double propRight=1, bool endClaw = false, double clawDist = 1, uint32_t maxTime = INF, double maxSpeed = SPEED_CAP, bool raiseMogo = false, double mogoHeight = 100, double accuracy = 1.25) {
	double Kp = 10; // was previously 50/3
	double Ki = 1; // to increase speed if its taking too long.
	double Kd = 20; // was previously 40/3
	double decay = 0.5; // integral decay
	
	target *= UNITSIZE; // convert UNITS to inches
	
	volatile double speed;
	volatile double error = target;
	volatile double olderror = error;
	 
  leftDrive1.resetPosition();
	leftDrive2.resetPosition();
  leftmiddle.resetPosition();
  rightDrive1.resetPosition();
  rightDrive2.resetPosition();
  rightmiddle.resetPosition();
	 
  volatile double sum = 0;
  uint32_t startTime = timer::system();
  bool isOpen;
	 
  while((fabs(error) > accuracy || fabs(speed) > 10) && timer::system() - startTime < maxTime) {
    // did this late at night but this while is important 
    error = target - dir(Gyro.rotation(deg)); // the error gets smaller when u reach ur target
    sum = sum * decay + error;
    speed = Kp * error + Ki * sum + Kd * (error - olderror); // big error go fast slow error go slow 
    speed = !(fabs(speed) > maxSpeed) ? speed : maxSpeed * sgn(speed);
    drive(speed * propLeft, speed * propRight, 10);
    olderror = error;
    isOpen = target > 0 ? claw1.value() == CLAW_OPEN : MogoTilt.value() == TILT_OPEN;
    EndClaw(endClaw,clawDist,error);
    if (raiseMogo && !isOpen && (error + 6 < clawDist) && Lift.position(degrees) < 10) {
      liftTo(mogoHeight,0);
    }
  }
	brakeDrive();
  EndClaw(endClaw);
}

void arcTo(double x2, double y2, uint8_t endClaw = false, double clawDist = 1, uint32_t maxTime = INF, double maxSpeed = SPEED_CAP, bool raiseMogo = false, double mogoHeight = 100, double accuracy = 0.25) {
  std::array<long double, 2> arc = calcArc(x2,y2);
  std::array<double, 2> pos;
  double Kp = 10; // was previously 50/3
	double Ki = 1; // to increase speed if its taking too long.
	double Kd = 15; // was previously 40/3
	double decay = 0.5; // integral decay
	volatile double speed;
	volatile double error = sgn(arc[0] + arc[1]) * fmax(fabs(arc[0]), fabs(arc[1]));
	volatile double olderror = error;
  volatile double sum = 0;
  double L = 0, R = 0;
  double speedR, speedL;
  bool isOpen;
  unitArc(arc[0] / UNITSIZE,1,arc[1]/arc[0]);
  return;

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
	x2 *= UNITSIZE; // convert UNITS to inches
  y2 *= UNITSIZE; // convert UNITS to inches
	 
  leftDrive1.resetPosition();
	leftDrive2.resetPosition();
  leftmiddle.resetPosition();
  rightDrive1.resetPosition();
  rightDrive2.resetPosition();
  rightmiddle.resetPosition();
	 
  uint32_t startTime = timer::system();

  while (fabs(error) > 0.5 && timer::system() - startTime < maxTime) {
    // get base speed
    sum = decay * sum + error;
    speed = Kp * error + Ki * sum + Kd * (error - olderror);
    speed = fabs(speed) > maxSpeed ? sgn(speed) * maxSpeed : speed; // lower speed to maxSpeed
    // do the speeds
    double ratio = arc[0] / arc[1];
    if (fabs(ratio) > 0) {
      speedL = sgn(arc[0]) * speed;
      speedR = sgn(arc[1]) * speed * 1 / ratio;
    }
    else {
      speedL = sgn(arc[0]) * speed;
      speedR = sgn(arc[1]) * speed * ratio;
    }
    // drive
    drive(speedL, speedR, 30);
    pos = calcPos((wheelRevs(2) - L) * Diameter * pi, (wheelRevs(3) - R) * Diameter * pi);
    arc = calcArc(x2-pos[0], y2-pos[1]);
    olderror = error;
    error = sgn(arc[0] + arc[1]) * fmax(fabs(arc[0]), fabs(arc[1]));
    L = wheelRevs(2);
    R = wheelRevs(3);
    EndClaw(endClaw,clawDist,(arc[0] + arc[1]) / 2);
    if (raiseMogo && !isOpen && ((fabs(arc[0] + arc[1]) / 2) + 6 < clawDist) && Lift.position(degrees) < 10) {
      liftTo(mogoHeight,0);
    }
  }
	brakeDrive(); // then stop
  EndClaw(endClaw);
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
  volatile uint32_t startTime = timer::system();
  double stopAng = 20;
  while (timer::system() - startTime < 1300) {
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
      startTime = timer::system();
    }
    else {
      brakeDrive();
      wait(30, msec);
      oldpitch = pitch;
      if (startTime > 500 && MogoTilt.value() != TILT_OPEN) {
        mogoTilt(TILT_OPEN); // drop mogo because we need rings to count
      }
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

void gyroturn(double target, double maxSpeed = 80, uint32_t maxTime = 1500, double accuracy = 1.25) { // idk maybe turns the robot with the gyro,so dont use the drive function use the gyro
  double Kp = 0.667;
  double Ki = 0;
  double Kd = 7;
  double decay = 0.5; // integral decay
	
  volatile double sum = 0;
  volatile double speed;
  volatile double error = target;
  volatile double olderror = error;
  uint32_t startTime = timer::system();

  target += Gyro.rotation(degrees);

  while ((fabs(error) > accuracy || fabs(speed) > 1) && timer::system() - startTime < maxTime) { //fabs = absolute value while loop again
    error = target - Gyro.rotation(degrees);; //error gets smaller closer you get,robot slows down
    doThePIDThing
    speed = !(fabs(speed) > maxSpeed) ? speed : maxSpeed * sgn(speed);
    drive(speed, -speed, 10);
    Brain.Screen.printAt(1, 60, "speed = %0.2f    degrees", speed);
    olderror = error;
  }
}

void turnTo(double target, uint32_t maxTime = 1300, double accuracy = 1.25) {
  gyroturn(mod(target - Gyro.rotation(degrees) - 180, 360) - 180, 85, maxTime, accuracy);
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
  mogoTilt(!TILT_OPEN);
  // SIDE
  Lift.spin(forward, -100, pct); // lower lift
  unitDrive(1.7,1,1,1000,INF,false,0,YELLOW,true);
  Lift.resetPosition();
  Vision.takeSnapshot(YELLOW);
  unitDrive(-0.75,false,0,INF,INF,true,5,0,true,2);
  if (Vision.largestObject.exists && Vision.largestObject.width > 100) {
    unitDrive(-2.1, 0, 0, 2000,INF,false,0,0,true); // back up. This may take a while if we're playing tuggle war. We have plenty of time at this point.
    return;
  }
  else {
    Claw(CLAW_OPEN);
    liftTo(-10,0);
    turnTo(67,750);
    rings(true);
    unitDrive(1.75,1,1,INF, 87, false,0,YELLOW); // get mid
    unitDrive(-1.5,1,1,INF,INF,true,8);
    turnTo(25,875);
    unitDrive(-3,1,1,INF,INF);
    return;
  }
  /*// ALLIANCE
  mogoTilt(TILT_OPEN); // open tilt
  gyroturn(-80, 100,1500); // aim
  VisionBack.takeSnapshot(MOGO_BLUE); // determine which side we're on
  Rings.spin(fwd, -100, pct);
  wait(100,msec);
  Rings.stop();
  deploy(false)
  unitDrive(-1, 2, 3,1300,50,true,30,(VisionBack.largestObject.exists ? BLUE : RED)); // tilt alliance goal
  rings(true); // start intake
  wait(750,msec);
  mogoTilt(TILT_OPEN); // open tilt
  // RINGS
  unitDrive(1, false, 0, INF, 20);*/
}

//driver controls,dont change unless your jaehoon or sean
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
  unsigned int ticks = 0;

  deploy(true)

  while (true) {
    // drive control
		double rstick = getSpeed(Controller1.Axis2.position());
		double lstick = getSpeed(Controller1.Axis3.position());
		drive(lstick, rstick,10);
		int8_t tmp, ringSpeed = RING_SPEED;
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
		if (Controller1.ButtonY.pressing()) { // deploy ring hood
      deploy(false)
		}
		else if (Controller1.ButtonB.pressing()) { // reverse rings
      ringSpeed = -100;
		}
    if (!Rings.isSpinning()) {
      rings(ringsOn || ringSpeed < 0,ringSpeed);
    }
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
    
    // brakes;
    if (Controller1.ButtonDown.pressing()) {
      leftDrive1.stop(brake);
      leftDrive2.stop(brake);
      leftmiddle.stop(brake);
      rightDrive1.stop(brake);
      rightDrive2.stop(brake);
      rightmiddle.stop(brake);
    }
    else {
      leftDrive1.setStopping(coast);
      leftDrive2.setStopping(coast);
      leftmiddle.setStopping(coast);
      rightDrive1.setStopping(coast);
      rightDrive2.setStopping(coast);
      rightmiddle.setStopping(coast);
    }
    // motor temp
    if (ticks > 9) {
      Controller1.Screen.setCursor(0, 0);
      Controller1.Screen.print("%f",getTemp());
      ticks = 0;
    }
    getTrackSpeed(YELLOW);
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

 
  // Stops main from exiting in the infinite loop.
  while (true) {
    wait(100, msec);
  }
}
