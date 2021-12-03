/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Sean and Jaehoon                                                       */
/*    Created:      sometime                                           */
/*    Description:  7700R code 2021-2022                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/
//7700R
//Sean
//Avery
//Matthew
//jaehoon
//Saif
//Josh
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// leftDrive1           motor         7               
// leftDrive2           motor         3               
// rightDrive1          motor         6               
// rightDrive2          motor         2               
// amogo                motor         18              
// lift1                motor         1               
// pis1                 digital_out   B               
// Inertial21           inertial      21              
// pis2                 digital_out   A               
// Gyro                 inertial      20              
// intakes              motor         10              
// chain                motor         12              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition,dont touch either
competition Competition;

// define your global Variables here
float pi=3.14;
float Diameter = 3.25;

//dont touch
//Diameter is the wheel
//pie is pie dumbass



void pre_auton(void) {
  vexcodeInit();
  wait(2000, msec);

  // All activities that occur before the competition starts
  //gets pistons down before match
  //dont touch this 
}

void drive(int lspeed, int rspeed, int wt){
  
  leftDrive1.spin(forward, lspeed, pct);
  leftDrive2.spin(forward, lspeed, pct);
  rightDrive1.spin(forward, rspeed, pct);
 rightDrive2 .spin(forward, rspeed, pct);
  wait(wt, msec);
}
//use to go forward,backwards,left right etc,turning if your stupid
//use inchdrive to go forward and backwards,use gyro to turn



void lift(int speed, int wt ){
  lift1.spin(forward, speed, pct);
  
  wait(wt, msec);
}
//makes lift go up or down
// its the lift speed then wait time
//example lift(-100,1200);  so lift 100% for 1200 msc
// 100 is up and -100 is down,or other way around,you can figure that out




void mogo (int speed, int wt){
  amogo.spin(forward, speed, pct);
  
  
  wait(wt, msec);
}
//makes mogo go up or down
// its the lift speed then wait time
//example mogo(-100,1200);  so mogo 100% for 1200 msc
// 100 is up and -100 is down,I know this 

void chains (int speed, int wt){
  chain.spin(forward, speed, pct);
  
  
  wait(wt, msec);
}
//makes chains move forward or backwards
// its the lift speed then wait time
//example chains(-100,1200);  so chain 100% for 1200 msc
// 100 is forward and -100 is backwards,I know this 


void intake (int speed, int wt){
  intakes.spin(forward, speed, pct);
  
  
  wait(wt, msec);
}
//makes intake spin 
// its the lift speed then wait time
//example intake(-100,1200);  so intake 100% for 1200 msc
// 100 is up and -100 is down,I know this 






void piston1 (bool open)
{pis1.set (open);}
//opens and closes piston1
//true is open and false is close
//examples
//piston1.set(true);    open
//piston1.set(false);   close

void piston2 (bool open)
{pis1.set (open);}

//opens and closes piston2
//true is open and false is close
//examples
//piston2.set(true);   open
//piston2.set(false);   close




void inchDrive(float target, int speed){
  leftDrive1.setPosition(0,  rev);
    leftDrive2.setPosition(0,  rev);//might only need 1 of 3 of these but im a dumbass so leave it 
  float inches = 0.0;
  float turns= 0.0;
  float error = target; 
  float olderror = error; 
  float kp=10;
  float kd = 20.0;
  
  //dont use the drive function you dumbass
  //use inchdrive,this took me a while to code :(
  //its target/inches/amount then speed/percentage
  //examples
  //inchDrive(55, 100); go 55in forward at 100%
  //inchDrive(-55, 100); go 55in backwards at 100%


  while(fabs(error)>1.0){
    //did this late at night but this while is important 
   //fabs = absolute value
    //heading= Gyro.rotation(degrees);
  drive(speed, speed, 10);
      turns =leftDrive1.position(rev);
    inches = turns * Diameter * pi;   //took and hour to fix this I think,was off by like 10 inches lol
    olderror=error;
    error = target-inches; //the error gets smaller when u reach ur target
 //inches = turns * Diameter * pi;
    
    speed = kp*error+kd*(error-olderror); //big error go fast slow error go slow 

    Brain.Screen.printAt(1, 40, "turns = %0.2f    ", turns); //math fun
    Brain.Screen.printAt(1, 60, "speed = %0.2f    ", speed);
     Brain.Screen.printAt(1, 100, "inches = %.2f  f", inches);
     Brain.Screen.printAt(1, 120, "error = %.2f  f", error);
  }
   
  drive(0,0,0);
  }
//if gyro needs calibrating add a 10ms wait or something
 

void gyroturn(float target){ //idk maybe turns the robot with the gyro,so dont use the drive function use the gyro
 float kp=2.0;
 float kd = 16.0;
  Gyro.setRotation(0, degrees); //centers,calibrates gyro to 0
  float heading = 0.0;
  float speed = 100;
  float error = target;
  float olderror=error;
  while(fabs(error)>2.0){ //fabs = absolute value while loop again
    heading= Gyro.rotation(degrees);
    olderror=error;
    error = target-heading; //error gets smaller closer you get,robot slows down
    drive(speed, -speed, 10);
    speed = kp*error+kd*(error-olderror); //big error go fast slow error go slow 
    Brain.Screen.printAt(1, 40, "heading = %0.2f    degrees", heading); //math thing again,2 decimal places
    Brain.Screen.printAt(1, 60, "speed = %0.2f    degrees", speed);
    //all ths print screen helps test to make sure gyro not broke
  }
  drive(0,0,0);
  heading= Gyro.rotation(degrees);//prints the gyro rotation degress
  Brain.Screen.printAt(1, 40, "heading = %0.2f    degrees", heading);
}


//wow maybe the auton code,this auton is the right side auton,work in progress but works how is 
void auton() {
  






  
  









}

//driver controls,dont change unless your jaehoon or sean
void driver() {
  // User control code here, inside the loop
  //2 joy sticks
  //rstick is axis 2 and lstick is axis 3,why its 2,3 and not 1,2 idk ask vex
  while (true) {
    int rstick=Controller1.Axis2.position();
    int lstick=Controller1.Axis3.position();
    drive(lstick, rstick,10);

    if (Controller1.ButtonR1.pressing())  //mogo up
    {
      amogo.spin(directionType::fwd,100,velocityUnits::pct);
    }
    else if (Controller1.ButtonR2.pressing()) //mogo down
    {
      amogo.spin(directionType::rev,100,velocityUnits::pct);
    }
    else
    {
      amogo.stop(brakeType::hold); //hold,do nothing if nothing is being pressed
    }

    if (Controller1.ButtonL1.pressing())  //lift up
    {
      lift1.spin(directionType::fwd,100,velocityUnits::pct);
    }
    else if (Controller1.ButtonL2.pressing())  //lift down
    {
      lift1.spin(directionType::rev,100,velocityUnits::pct);
    }
    else
    {
      lift1.stop(brakeType::hold); //hold,do nothing if nothing is being pressed
    }

    if (Controller1.ButtonUp.pressing()) //piston1 open
    {
      pis1.set(false);
    }
    else if (Controller1.ButtonDown.pressing()) //piston1 close
    {
      pis1.set(true);
    }
    
    if (Controller1.ButtonX.pressing()) //piston2 open
    {
      pis2.set(false);
    }
    else if (Controller1.ButtonA.pressing()) //piston2 close
    {
      pis2.set(true);
    }

    

if (Controller1.ButtonB.pressing())  //intake spin
    {
      intakes.spin(directionType::fwd,100,velocityUnits::pct);
    }
    else
    {
      intakes.stop(brakeType::hold); //hold,do nothing if nothing is being pressed

      
      if (Controller1.ButtonLeft.pressing())  //chain backwards
    {
      chain.spin(directionType::fwd,100,velocityUnits::pct);
    }
    else if (Controller1.ButtonRight.pressing()) //chain forward
    {
      chain.spin(directionType::rev,100,velocityUnits::pct);
    }
    else
    {
      chain.stop(brakeType::hold); //hold,do nothing if nothing is being pressed
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
