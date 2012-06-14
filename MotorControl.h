void setLeftDrive(int power);
void setRightDrive(int power);

void setLeftArm(int power);
void setRightArm(int power);

void stopDrive();

#define VOLTAGE_FULL 12230

float powerMultiplier()
{
  float percentUnder = (VOLTAGE_FULL - externalBatteryAvg) / VOLTAGE_FULL;
  writeDebugStreamLine("%f",percentUnder);
  return 1.0/(1.0-percentUnder);
}

void setLeftDrive(int power)
{
  motor[leftDrive] = power * powerMultiplier();
}

void setRightDrive(int power)
{
  motor[rightDrive] = ((float)power)*.65 * powerMultiplier();
}

void setRightArm(int power)
{
  motor[rightArm] = ((float)power)*1 * powerMultiplier();
}

void setLeftArm(int power)
{
  motor[leftArm] = ((float)power)* 1.2 * powerMultiplier();
}

void zeroEncoders()
{
    nMotorEncoder[leftArm] = 0;
    nMotorEncoder[rightArm] = 0;
}

void stopDrive()
{
  motor[rightDrive] = 0;
  motor[leftDrive] = 0;
}
