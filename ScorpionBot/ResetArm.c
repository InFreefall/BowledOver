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

// Values in inches. See drawings in engineer's notebook for details.
const float a = 15.3;
const float b = 15.3;
const float shoulderHeight = 14.6;

float startX = 9;
float startY = 18.5;

float targetX = startX;
float targetY = startY;

int wristNudge = 0;

bool shouldPowerWrist = true;

const float H_TIER_1 = 11.12;
const float H_TIER_2 = 31.6;
const float H_TIER_3 = 42.2;

struct
{
  int powerMax;
  int direction;
  float degreesPerEncoder;
  float degreesAtZero;
  int closeEnough;

  // PID variables
  float Kp, Kd, Ki;
  float integral;
  float difference;
} Joint;

Joint jShoulder;
Joint jElbow;
Joint jWrist;

void moveToXY(float x, float y)
{
	targetX = x;
	targetY = y;
}

float degreesToEncoder(Joint joint, float degrees)
{
	return (degrees - joint.degreesAtZero) / joint.degreesPerEncoder;
}

float encoderToDegrees(Joint joint, float encoder)
{
	return encoder * joint.degreesPerEncoder + joint.degreesAtZero;
}

void initEncoders()
{
	nMotorEncoder[shoulder] = 0;
	nMotorEncoder[elbow] = 0;
	nMotorEncoder[wrist] = 0;
	wait1Msec(300);
}

void init()
{
	jShoulder.powerMax = 90;
	jShoulder.direction = -1;
	jShoulder.degreesPerEncoder = .0101221;
	jShoulder.degreesAtZero = 308.44555;
	jShoulder.closeEnough = 100;
	jShoulder.Kp = .019;
	jShoulder.Ki = .015;
	jShoulder.Kd = .0;
	jShoulder.difference = 0;
	jShoulder.integral = 0;

	jElbow.powerMax = 90;
	jElbow.direction = 1;
	jElbow.degreesPerEncoder = -.030213;
	jElbow.degreesAtZero = 8.3734;
	jElbow.closeEnough = 100;
	jElbow.Kp = .021;
	jElbow.Ki = .009;
	jElbow.Kd = 0;
	jElbow.difference = 0;
	jElbow.integral = 0;

	jWrist.powerMax = 60;
	jWrist.direction = 1;
	jWrist.degreesPerEncoder = -.1309;
	jWrist.degreesAtZero = -1.078;
	jWrist.closeEnough = 40;
	jWrist.Kp = .26;
	jWrist.Ki = .12;
	jWrist.Kd = 0;
	jWrist.difference = 0;
	jWrist.integral = 0;
}

int powerForJoint(Joint joint, int encoderValue, int targetEncoder)
{
	if (abs(encoderValue - targetEncoder) <= joint.closeEnough)
	{
		return 0;
	}

	int error = targetEncoder - encoderValue;
	float proportional = error*joint.Kp;

	joint.integral += error;
	joint.integral *= joint.Ki;

	int rateOfChange = error-joint.difference;
	float derivitive = rateOfChange*joint.Kd;
	joint.difference = error;

	int power = proportional + joint.integral + derivitive;

	if (power > joint.powerMax)
		power = joint.powerMax;
	else if (power < -joint.powerMax)
		power = -joint.powerMax;
	power *= joint.direction;
	return power;
}

void setMotorPowersForXY()
{
	float adjustedY = targetY;

	float theta = PI/2 + atan((adjustedY-shoulderHeight)/(targetX));
	float c = sqrt(pow(adjustedY-shoulderHeight,2)+pow(targetX,2));
	float angleC = acos( (pow(c,2)-pow(b,2)-pow(a,2)) / (-2*a*b) );
	float angleA = asin( (a * sin(angleC) / c) );

	int shoulderTarget = 0;
	int elbowTarget = 0;

	motor[shoulder] = -powerForJoint(jShoulder, nMotorEncoder[shoulder], shoulderTarget);
	motor[elbow] = powerForJoint(jElbow, nMotorEncoder[elbow], elbowTarget);

	float shoulderAngle = encoderToDegrees(jShoulder, nMotorEncoder[shoulder]);
	float elbowAngle = encoderToDegrees(jElbow, nMotorEncoder[elbow]);
	float targetWristAngle = shoulderAngle + elbowAngle - 180 + wristNudge;
	int wristTarget = 0;
	writeDebugStreamLine("%f,   %d",targetWristAngle,wristTarget);
	//writeDebugStreamLine("%f",shoulderAngle);
	nxtDisplayTextLine(0,"<shoulder: %f",shoulderAngle);
	nxtDisplayTextLine(3,"<elbow:    %f",elbowAngle);
	if (shouldPowerWrist)
		motor[wrist] = powerForJoint(jWrist, nMotorEncoder[wrist], wristTarget);

	nxtDisplayTextLine(1,"targetX: %f",targetX);
	nxtDisplayTextLine(2,"targetY: %f",targetY);
}

task main()
{
	init();
	while (true) {setMotorPowersForXY();}
}
