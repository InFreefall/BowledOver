#pragma config(Sensor, S3,     accelerometer,        sensorI2CCustom)
#pragma config(Sensor, S2, compass, sensorI2CCustom)

#include "rdpartyrobotcdr-3.1/drivers/hitechnic-accelerometer.h"
#include "rdpartyrobotcdr-3.1/drivers/hitechnic-compass.h"

typedef struct
{
  float x;
  float y;
} Vector;

const int NUMBER_CALIBRATION_READINGS = 30;
const int NUMBER_AVERAGE_READINGS = 1;
const int ACCELERATION_THRESHOLD = 3;

Vector readings[NUMBER_AVERAGE_READINGS];
int currentReading = 0;

Vector position;
Vector velocity;
Vector baseAcceleration;
Vector lastAcceleration;
Vector currentAcceleration;

long lastUpdate = -1;

void DR_init()
{
	position.x = 0;
	position.y = 0;
	velocity.x = 0;
	velocity.y = 0;
	baseAcceleration.x = 0;
	baseAcceleration.y = 0;
	int x,y,z; // Throwaway variable, but I don't want to pass NULL to HTACreadAllAxes

	for (int i = 0; i < NUMBER_CALIBRATION_READINGS; i++)
	{
		if (!HTACreadAllAxes(accelerometer, x, y, z))
		{
			writeDebugStreamLine("Error reading from accelerometer");
		}
		baseAcceleration.x += x;
		baseAcceleration.y += y;
	}
	baseAcceleration.x /= NUMBER_CALIBRATION_READINGS;
	baseAcceleration.y /= NUMBER_CALIBRATION_READINGS;

	HTMCsetTarget(compass);
}

bool isOutlier(float value, float mean, float stdDev)
{
	float max = mean + 2*stdDev;
	float min = mean - 2*stdDev;
	return (value < max && value > min);
}

void DR_filter_readings()
{
	  lastAcceleration = currentAcceleration;

		// Calculate mean of readings
		currentAcceleration.x = currentAcceleration.y = 0;
		for (int i = 0; i < NUMBER_AVERAGE_READINGS; i++)
		{
			currentAcceleration.x += readings[i].x;
			currentAcceleration.y += readings[i].y;
		}
		currentAcceleration.x /= NUMBER_AVERAGE_READINGS;
		currentAcceleration.y /= NUMBER_AVERAGE_READINGS;

		// Calculate standard deviation
		Vector accelStdDev;
		accelStdDev.x = accelStdDev.y = 0;
		for (int i = 0; i < NUMBER_AVERAGE_READINGS; i++)
		{
			accelStdDev.x += pow((readings[i].x - currentAcceleration.x),2);
			accelStdDev.y += pow((readings[i].y - currentAcceleration.y),2);
		}
		accelStdDev.x /= NUMBER_AVERAGE_READINGS;
		accelStdDev.y /= NUMBER_AVERAGE_READINGS;

		int nOutliers = 0;
		// Find outliers
		for (int i = 0; i < NUMBER_AVERAGE_READINGS; i++)
		{
			if (isOutlier(readings[i].x, currentAcceleration.x, accelStdDev.x) ||
				  isOutlier(readings[i].y, currentAcceleration.y, accelStdDev.y))
				{
					// Discard this reading, it is an outlier
					readings[i].x = -1;
					readings[i].y = -1;
					nOutliers++;
				}
		}

		// Recalculate mean, excluding outliers
		currentAcceleration.x = 0;
		currentAcceleration.y = 0;
		for (int i = 0; i < NUMBER_AVERAGE_READINGS; i++)
		{
			if (readings[i].x != -1 && readings[i].y != -1)
			{
				currentAcceleration.x += readings[i].x;
				currentAcceleration.y += readings[i].y;
			}
		}
		currentAcceleration.x /= (NUMBER_AVERAGE_READINGS - nOutliers);
		currentAcceleration.y /= (NUMBER_AVERAGE_READINGS - nOutliers);
}

bool DR_take_reading()
{
	int x,y,z;
	HTACreadAllAxes(accelerometer, x, y, z);
	readings[currentReading].x = x - baseAcceleration.x;
	readings[currentReading].y = y - baseAcceleration.y;
	if (readings[currentReading].x < ACCELERATION_THRESHOLD)
		readings[currentReading].x = 0;
	if (readings[currentReading].y < ACCELERATION_THRESHOLD)
		readings[currentReading].y = 0;

	// Adjust for rotation of the robot
	float dTheta = HTMCreadRelativeHeading(compass);
	float theta = atan2(readings[currentReading].x,readings[currentReading].y);
	float magnitude = sqrt(pow(readings[currentReading].x,2)+pow(readings[currentReading].y,2));
	theta += dTheta;
	readings[currentReading].x = cos(theta)*magnitude;
	readings[currentReading].y = sin(theta)*magnitude;

	currentReading++;
	if (currentReading == NUMBER_AVERAGE_READINGS)
	{
		currentReading = 0;
		DR_filter_readings();
		return true;
	}
	return false;
}

void DR_tick()
{
	if (!DR_take_reading())
		return;
	Vector currentVelocity;

	if (lastUpdate == -1)
	{
		lastUpdate = nPgmTime;
		return;
	}

	float dt = (nPgmTime - lastUpdate) / 1000;

	float dxVelocity = 0;
	float dyVelocity = 0;
	float dx = 0;
	float dy = 0;

	dxVelocity = (lastAcceleration.x + currentAcceleration.x)/2*dt;
	dyVelocity = (lastAcceleration.y + currentAcceleration.y)/2*dt;
	currentVelocity.x = velocity.x + dxVelocity;
	currentVelocity.y = velocity.y + dyVelocity;
	dx = (velocity.x + currentVelocity.x)/2*dt;
	dy = (velocity.y + currentVelocity.y)/2*dt;
	position.x += dx;
	position.y += dy;

	lastAcceleration = currentAcceleration;
	velocity = currentVelocity;
}
