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

#define FLAG_WAVE_DELAY 800

#define SERVO_MIN 0
#define SERVO_MAX 255

long lastWave = 0;

task waveFlag()
{
  while (true)
  {
    if (nPgmTime - lastWave > FLAG_WAVE_DELAY)
    {
      if (ServoValue[flag] == SERVO_MIN)
        servo[flag] = SERVO_MAX;
      else
        servo[flag] = SERVO_MIN;
    }
  }
}
