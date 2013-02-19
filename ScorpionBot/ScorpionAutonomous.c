#pragma config(Sensor, S3,     sMUX,           sensorI2CCustom)
#pragma config(Hubs,  S2, HTMotor,  none,     none,     none)
#pragma config(Hubs,  S4, HTMotor,  HTMotor,  HTMotor,  HTServo)
#pragma config(Sensor, S1,     HTIRS2,         sensorI2CCustom)
#pragma config(Sensor, S2,     ,               sensorI2CMuxController)
#pragma config(Sensor, S3,     sMUX,           sensorI2CCustom)
#pragma config(Sensor, S4,     ,               sensorI2CMuxController)
#pragma config(Motor,  mtr_S2_C1_1,     wrist,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     motorE,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S4_C1_1,     motorNorth,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S4_C1_2,     motorEast,     tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S4_C2_1,     motorWest,     tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S4_C2_2,     motorSouth,    tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S4_C3_1,     elbow,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S4_C3_2,     shoulder,      tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S4_C4_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S4_C4_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S4_C4_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S4_C4_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S4_C4_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S4_C4_6,    servo6,               tServoNone)

#include "JoystickDriver.c"
#include "DriveTrain.h"
#include "RobotArmPID.h"
#include "rdpartyrobotcdr-3.2.1/drivers/hitechnic-sensormux.h"
#include "rdpartyrobotcdr-3.2.1/drivers/hitechnic-irseeker-v2.h"
#include "rdpartyrobotcdr-3.2.1/drivers/lego-ultrasound.h"
#include "rdpartyrobotcdr-3.2.1/drivers/lego-light.h"
#include "rdpartyrobotcdr-3.2.1/drivers/hitechnic-compass.h"
#include "rdpartyrobotcdr-3.2.1/drivers/hitechnic-accelerometer.h"

const tMUXSensor US = msensor_S3_2;

int baseLS;

void waitForWhiteLine()
{
	int diff = 0;
	while (abs(diff) < 130)
	{
		diff = SensorRaw[S1] - baseLS;
		nxtDisplayTextLine(0, "diff: %d",diff);
		writeDebugStreamLine("diff: %d",diff);
	}
}

typedef struct
{
	float x;
	float y;
	int wristNudge;
} armMotion;

armMotion armPaths[12];
int armPathIndex = 0;
int startIndex = 0;

void initArmPaths()
{
	armPaths[4].x = 5;
	armPaths[4].y = 17.19;
	armPaths[4].wristNudge = 0;
	armPaths[5].x = 20.32;
	armPaths[5].y = 17.69;
	armPaths[5].wristNudge = 10;
	armPaths[6].x = 29.32;
	armPaths[6].y = 16.49;
	armPaths[6].wristNudge = 10;
	armPaths[7].x = 6;
	armPaths[7].y = 9.09;
	armPaths[7].wristNudge = 0;

	armPaths[0].x = 4;
	armPaths[0].y = 15.7;
	armPaths[0].wristNudge = 10;
	armPaths[1].x = 13;
	armPaths[1].y = 15.7;
	armPaths[1].wristNudge = 10;
	armPaths[2].x = 13;
	armPaths[2].y = 7.3;
	armPaths[2].wristNudge = 0;
	armPaths[3].x = 4;
	armPaths[3].y = 7.2;
	armPaths[3].wristNudge = 0;

	armPaths[8].x = 14;
	armPaths[8].y = 17.3;
	armPaths[8].wristNudge = 0;
	armPaths[9].x = 25.7;
	armPaths[9].y = 17.3;
	armPaths[9].wristNudge = 0;
	armPaths[10].x = 25;
	armPaths[10].y = 10;
	armPaths[10].wristNudge = 0;
	armPaths[11].x = 5;
	armPaths[11].y = 12;
	armPaths[11].wristNudge = 0;
}

void pathStep(armMotion motion)
{
	moveToXY(motion.x, motion.y);
	wristNudge = motion.wristNudge;
}

task main()
{
	servo[servo6] = 253;
	servo[servo1] = 128;
	init();
	initEncoders();
	initArmPaths();
	SensorType[S3] = sensorI2CCustom;

	waitForStart();

	motor[wrist] = -55;

	SensorType[S1] = sensorLightActive;
	wait1Msec(50);
	baseLS = 0;
	for (int i = 0; i < 10; i++)
		baseLS += SensorRaw[S1];//LSvalRaw(LIGHT);
	baseLS /= 10;
	writeDebugStreamLine("baseLS: %d",baseLS);

	motor[motorEast] = 80;
	motor[motorWest] = 80;
	wait1Msec(100);
	motor[wrist] = 0;
	waitForWhiteLine();
	wait1Msec(100);
	stopMotors();

	wait1Msec(500);
	int ir1 = HTIRS2readACDir(msensor_S3_1);
	int ir2 = HTIRS2readACDir(msensor_S3_3);
	writeDebugStreamLine("IR: %d,%d",ir1,ir2);
	if ((ir1 >= 6) && ir2 >= 5)  // Then the IR beacon is on the right
	{
		motor[motorNorth] = -85;
		motor[motorSouth] = -85;
		wait1Msec(1000);
		while (USreadDist(US) > 60) {writeDebugStreamLine("US: %d",USreadDist(US));}
		motor[motorNorth] = 45;
		motor[motorSouth] = 45;
		wait1Msec(1600);
		stopMotors();
		servo[servo6] = 183;
		motor[motorEast] = 90;
		motor[motorWest] = 90;
		waitForWhiteLine();
		wait10Msec(15);
		stopMotors();

		armPathIndex = 0;
	}
	else if (ir1 == 5) // IR beacon is in Middle
	{
		/*motor[motorEast] = 60;
		motor[motorWest] = 60;
		wait1Msec(100);
		stopMotors();*/

		armPathIndex = 4;
	}
	else // IR beacon is on the left
	{
		motor[motorEast] = -80;
		motor[motorWest] = -80;
		wait1Msec(1100);
		stopMotors();
		motor[motorNorth] = 55;
		motor[motorSouth] = 55;
		wait1Msec(870);
		stopMotors();
		motor[motorEast] = 45;
		motor[motorWest] = 45;
		servo[servo6] = 70;
		wait1Msec(1320);
		stopMotors();

		motor[motorNorth] = 55;
		motor[motorSouth] = 55;
		waitForWhiteLine();

		motor[motorNorth] = -55;
		motor[motorSouth] = -55;
		wait10Msec(125);
		stopMotors();

		armPathIndex = 8;
	}


	pathStep(armPaths[armPathIndex]);
	startIndex = armPathIndex;
	while (true) // Arm movement loop
	{
		if (setMotorPowersForXY())
		{
			writeDebugStreamLine("Next!\n");
			armPathIndex++;
			if (armPathIndex - startIndex >= 4)
				break;
			pathStep(armPaths[armPathIndex]);
		}
	}
}
