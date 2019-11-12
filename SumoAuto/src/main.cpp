/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       situs84                                                   */
/*    Created:      Thu Oct 31 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "positioner.cpp"

using namespace vex;

vex::brain Brain;
vex::controller Controller1;
vex::motor RightMotor = vex::motor(vex::PORT10, true);
vex::motor LeftMotor = vex::motor(vex::PORT1, false);
//vex::motor ClawMotor = vex::motor(vex::PORT3, true);
//vex::motor ArmMotor = vex::motor(vex::PORT8, false);
//vex::bumper Bumper = vex::bumper(Brain.ThreeWirePort.A);
vex::sonar Sonar = vex::sonar(Brain.ThreeWirePort.B);
vex::line Line = vex::line(Brain.ThreeWirePort.C);

vex::directionType forward = vex::directionType::fwd;
vex::directionType reverse = vex::directionType::rev;
double slightly = 2.0;

//reset encoders for next movement.
void rotReset(){ RightMotor.resetRotation(); LeftMotor.resetRotation(); }

//averages the values of the encoders of the left and right motors
double absEncoderAverage(){
  return (abs(int(RightMotor.rotation(vex::rotationUnits::deg))) + abs(int(LeftMotor.rotation(vex::rotationUnits::deg))))/2;
}

//fetches encoder value to for the other functions to process (usually used to know when to stop)
double getEncoder(vex::motor m){
  return m.rotation(vex::rotationUnits::deg);
}

//alternate for vex::task::sleep
void wait(double t){vex::task::sleep(t);}
//default values 4.0 and 11.5 (in)
double wheelDiameter = 4.0;
double wheelDistance = 11.5;
double wheelCircumference = wheelDiameter * 3.14159;
double baseCircumference = wheelDistance * 3.14159;
//Don't edit this
vex::motor_group DriveBase = vex::motor_group(RightMotor, LeftMotor);
//instead of stop(vex::brakeType::hold)
void goBrake(vex::motor_group mg){ mg.stop(vex::brakeType::hold); }

//move in direction for distance (in inches)
void move(vex::directionType direction, double distance){
  int moveDegrees = (distance/wheelCircumference)*360.0;
  rotReset();
  while(absEncoderAverage() < abs(moveDegrees)){
    RightMotor.spin(direction);
    LeftMotor.spin(direction);
  }
  goBrake(DriveBase);
  wait(500);
}

//move in direction for distance (in inches)
void move(double distance){
  int moveDegrees = (distance/wheelCircumference)*360.0;
  rotReset();
  while(absEncoderAverage() < abs(moveDegrees)){
    RightMotor.spin(forward);
    LeftMotor.spin(forward);
  }
  RightMotor.stop(vex::brakeType::hold);
  LeftMotor.stop(vex::brakeType::hold);
  wait(500);
}

void move(vex::directionType direction){
  RightMotor.spin(direction);
  LeftMotor.spin(direction);
}

//rotates a single wheel and keeps the other one stationary
void moveMotor(vex::directionType direction, vex::motor m, double turn){
  int calculatedDegrees = ((turn/360)*baseCircumference/wheelCircumference)*360.0;
  rotReset();
  while(abs(int(getEncoder(m))) < abs(calculatedDegrees)){
    m.spin(direction);
  }
  m.stop(vex::brakeType::hold);
  wait(500);
}

void moveMotor(vex::motor m, double turn){
  int calculatedDegrees = ((turn/360)*baseCircumference/wheelCircumference)*360.0;
  rotReset();
  while(abs(int(getEncoder(m))) < abs(calculatedDegrees)){
    m.spin(forward);
  }
  m.stop(vex::brakeType::hold);
  wait(500);
}

//rotates relative to the origin (center point between the two drive wheels)
void rotateInPlace(vex::directionType direction, double turn){
  int calculatedDegrees = ((turn/360)*baseCircumference/wheelCircumference)*360.0;
  vex::directionType dominantDirection = direction;
  vex::directionType secondaryDirection = direction == reverse ? forward : reverse;
  rotReset();
  while(abs(int(RightMotor.rotation(vex::rotationUnits::deg))) < abs(calculatedDegrees/**1.25*/)){
    RightMotor.spin(secondaryDirection);
    LeftMotor.spin(dominantDirection);
  }
  goBrake(DriveBase);
  wait(500);
}

void rotateInPlace(vex::directionType direction){
  vex::directionType dominantDirection = direction;
  vex::directionType secondaryDirection = direction == reverse ? forward : reverse;
  RightMotor.spin(secondaryDirection);
  LeftMotor.spin(dominantDirection);
}
//continue in a direction (mostly reverse) until the bumper is pressed
void moveUntilButton(vex::directionType direction, vex::bumper B){
  DriveBase.resetRotation();
  while(!B.pressing()){
    DriveBase.spin(direction);
  }
  goBrake(DriveBase);
  wait(500);
}

//continue in a direction (mostly reverse) until the bumper is pressed
void moveUntilButton(vex::bumper B){
  DriveBase.resetRotation();
  while(!B.pressing()){
    DriveBase.spin(forward);
  }
  goBrake(DriveBase);
  wait(500);
}

double getSonar(){
  return Sonar.distance(distanceUnits::in);
}
//open/close claw
//void claw( vex::directionType dir ){ ClawMotor.rotateFor(dir, 1, vex::sec); ClawMotor.setBrake(vex::brakeType::hold);}

//move arm
//void moveArm(vex::directionType dir, double time, vex::timeUnits units){ ArmMotor.rotateFor(dir, time, units); ArmMotor.setBrake(vex::brakeType::hold); }
//void moveArm(vex::directionType dir, double rotation, vex::rotationUnits units){ ArmMotor.rotateFor(dir, rotation, units); ArmMotor.setBrake(vex::brakeType::hold); }



int main() {
  int currentLevel = 0;
  while(1){
    while((Sonar.distance(distanceUnits::in) > 10.0 || Sonar.distance(distanceUnits::in) < 0.0) && currentLevel != 2){
      currentLevel = currentLevel == 1 ? : 1;
      rotateInPlace(forward);
      
    }
    if(currentLevel == 1){
    goBrake(DriveBase);
    currentLevel = 0;
    }
    while(getSonar() < 14 && getSonar() > 0 && currentLevel != 1 && Line.value(analogUnits::pct) > .5){
      currentLevel = 2;
      move(reverse);
    }
    if(currentLevel == 2){
      currentLevel = 0;
      move(forward, 5);
      goBrake(DriveBase);
    }
  }
}