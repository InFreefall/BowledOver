#pragma config(Hubs,  S1, HTMotor,  none,     none,     none)
#pragma config(Hubs,  S2, HTServo,  HTMotor,  none,     none)
#pragma config(Sensor, S3,     sTouch,              sensorTouch)
#pragma config(Motor,  mtr_S1_C1_1,     rightArm,      tmotorNormal, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     leftArm,       tmotorNormal, openLoop)
#pragma config(Motor,  mtr_S2_C2_1,     rightDrive,    tmotorNormal, openLoop, reversed)
#pragma config(Motor,  mtr_S2_C2_2,     leftDrive,     tmotorNormal, openLoop)
#pragma config(Servo,  srvo_S2_C1_1,    rightWing,            tServoStandard)
#pragma config(Servo,  srvo_S2_C1_2,    leftWing,             tServoStandard)
#pragma config(Servo,  srvo_S2_C1_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S2_C1_6,    flag,                 tServoStandard)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"	 //Include file to "handle" the Bluetooth messages.

int lastBeep = 0;
int delay = 100;
long lastWave = 0;
const int FLAG_WAVE_TIME = 750;
bool flagZero = false;

void initializeRobot()
{
  nMotorEncoder[leftArm] = 0;
  nMotorEncoder[rightArm] = 0;
  return;
}

void operateWheels()
{
  float leftDrivePower = 0;
  float rightDrivePower = 0;
  if (abs(joystick.joy1_y1) > 15)
  {
    leftDrivePower = joystick.joy1_y1*100.0/128.0;
  }
  if (abs(joystick.joy1_y2) > 15)
    rightDrivePower = joystick.joy1_y2*100.0/128.0;
  motor[leftDrive] = leftDrivePower;
  motor[rightDrive] = rightDrivePower;
}

void operateLift()
{
  int rightArmPower = 0;
  int leftArmPower = 0;
  if (joy1Btn(6))
  {
    rightArmPower = -25;
  }
  if (joy1Btn(8))
  {
    rightArmPower = 25;
  }
  if (joy1Btn(5))
  {
    leftArmPower = -25;
  }
  if (joy1Btn(7))
  {
    leftArmPower = 25;
  }
  if (!(rightArmPower > 0 && nMotorEncoder[rightArm] <= -2410))
    motor[rightArm] = rightArmPower;
  else
    motor[rightArm] = 0;
  if (!(leftArmPower > 0 && nMotorEncoder[leftArm] >= 2280))
    motor[leftArm] = leftArmPower;
  else
    motor[leftArm] = 0;

  nxtDisplayTextLine(1, "RArm: %d", nMotorEncoder[rightArm]);
  nxtDisplayTextLine(2, "LArm: %d", nMotorEncoder[leftArm]);

  int leftWingValue = ServoValue[leftWing];
  int rightWingValue = ServoValue[rightWing];

  if (joystick.joy1_TopHat == 2)
  {
    leftWingValue -= 3;
    rightWingValue += 3;
  }
  if (joystick.joy1_TopHat == 6)
  {
    leftWingValue += 3;
    rightWingValue -= 3;
  }

  servo[leftWing] = leftWingValue;
  servo[rightWing] = rightWingValue;
}

void operateBeep()
{
  if (SensorValue[sTouch] >= 1)
    PlayImmediateTone(random(3000),delay/4);
  if (nPgmTime/10 - lastBeep > delay && joy1Btn(1))
  {
    PlayImmediateTone(random(3000),delay/4);
    lastBeep = nPgmTime/10;
  }
  if (joy1Btn(2))
  {
    PlayImmediateTone(random(3000),delay/4);
  }
  if (joy1Btn(3) && !bSoundActive)
  {
    PlaySoundFile("NOOOOOO.rso");
  }
}

void waveFlag()
{
  if (joy1Btn(4))
  {
    servo[flag] = 0;
    flagZero = true;
  }
  else if (nPgmTime - lastWave > FLAG_WAVE_TIME)
  {
    if (flagZero)
      servo[flag] = 255;
    else
      servo[flag] = 0;
    lastWave = nPgmTime;
    flagZero = !flagZero;
  }
}

task main()
{
  initializeRobot();

  waitForStart();

  while (true)
  {
    getJoystickSettings(joystick);
    operateWheels();
    operateBeep();
    operateLift();
    waveFlag();
  }
}
