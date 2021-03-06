#pragma config(Hubs,  S1, HTMotor,  none,     none,     none)
#pragma config(Hubs,  S2, HTServo,  HTMotor,  none,     none)
#pragma config(Sensor, S3,     sSONAR,         sensorSONAR)
#pragma config(Sensor, S4,     sWallSONOR,     sensorSONAR)
#pragma config(Motor,  motorA,          ,              tmotorNormal, openLoop, encoder)
#pragma config(Motor,  motorB,          ,              tmotorNormal, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     rightArm,      tmotorNormal, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     leftArm,       tmotorNormal, openLoop, encoder)
#pragma config(Motor,  mtr_S2_C2_1,     rightDrive,    tmotorNormal, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C2_2,     leftDrive,     tmotorNormal, openLoop, reversed)
#pragma config(Servo,  srvo_S2_C1_1,    rightWing,            tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    leftWing,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    flag,                 tServoStandard)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "MotorControl.h"

#define TERRAIN_MODIFIER            1.0

#define DESCEND_RAMP_TIME           2565
#define TURN_TIME                   2148
#define ULTRASONIC_ROTATION_TIME    750
#define ARM_LOWER_TIME              1700
#define DRIVE_TOWARDS_BALL_TIME     800
#define ULTRASONIC_DELAY            90
#define HEAD_FOR_CORNER_DELAY       4000

#define BB_THRESHOLD_FAR   60
#define BB_THRESHOLD_CLOSE 30
#define WALL_THRESHOLD     50

bool runUltrasonic();
void descendRamp();
void turnRight();
void turnLeft();
void driveTowardsBall();
void driveTowardsBallWithUltrasonic();
void lowerArms();
void headForCorner();

task main()
{
  descendRamp();
#ifdef TURN_RIGHT
  turnRight();
#endif
#ifdef TURN_LEFT
  turnLeft();
#endif
  bool done = false;
  while (!done)
  {
    driveTowardsBall();
    done = runUltrasonic();
  }
  driveTowardsBallWithUltrasonic();
  headForCorner();
}

void descendRamp()
{
  setRightDrive(45);
  setLeftDrive(45);
  wait1Msec(DESCEND_RAMP_TIME * TERRAIN_MODIFIER);
  stopDrive();
}

void turnRight()
{
  setRightDrive(-45);
  setLeftDrive(45);
  wait1Msec(TURN_TIME * TERRAIN_MODIFIER);
  stopDrive();
}

void turnLeft()
{
  setRightDrive(45);
  setLeftDrive(-45);
  wait1Msec(TURN_TIME * TERRAIN_MODIFIER);
  stopDrive();
}

void lowerArms()
{
  servo[leftWing] = 0;
  servo[rightWing] = 255;
  setLeftArm(25);
  setRightArm(25);
  wait1Msec(ARM_LOWER_TIME);
  // This section is just in case the arms aren't lowered all the way by the last delay
  // If they aren't, this lowers them around the bowling ball
  // If they are, the decreased power reduces stress on the parts and motors
  setLeftArm(15);
  setRightArm(15);
  wait1Msec(ARM_LOWER_TIME);
  setLeftArm(0);
  setRightArm(0);
}

void driveTowardsBall()
{
  setLeftDrive(45);
  setRightDrive(45);
  wait1Msec(DRIVE_TOWARDS_BALL_TIME * TERRAIN_MODIFIER);
  stopDrive();
}

bool runUltrasonic()
{
  long timeMark = nPgmTime;
  setLeftDrive(45);
  setRightDrive(-45);
  while (timeMark + ULTRASONIC_ROTATION_TIME*TERRAIN_MODIFIER > nPgmTime)
  {
    if (SensorValue[sSONAR] < BB_THRESHOLD_FAR)
    {
      wait1Msec(ULTRASONIC_DELAY);
      stopDrive();
      return true;
    }
  }
  timeMark = nPgmTime;
  setLeftDrive(-45);
  setRightDrive(45);
  while (timeMark + 2*ULTRASONIC_ROTATION_TIME*TERRAIN_MODIFIER > nPgmTime)
  {
    if (SensorValue[sSONAR] < BB_THRESHOLD_FAR)
    {
      wait1Msec(ULTRASONIC_DELAY);
      stopDrive();
      return true;
    }
  }
  setLeftDrive(45);
  setRightDrive(-45);
  wait1Msec(ULTRASONIC_ROTATION_TIME*TERRAIN_MODIFIER);
  stopDrive();
  return false;
}

void driveTowardsBallWithUltrasonic()
{
  setRightDrive(45);
  setLeftDrive(45);
  while(SensorValue[sSONAR]>BB_THRESHOLD_CLOSE)
    {}
  wait10Msec(40);
  stopDrive();
  lowerArms();
}

void headForCorner()
{
  long timeMark = nPgmTime;
  setRightDrive(45);
  setLeftDrive(45);
  while (nPgmTime < timeMark + HEAD_FOR_CORNER_DELAY*TERRAIN_MODIFIER)
  {}
  stopDrive();
}
