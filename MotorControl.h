void setLeftDrive(int power);
void setRightDrive(int power);

void setLeftArm(int power);
void setRightArm(int power);

void setLeftDrive(int power)
{
  motor[leftDrive] = power;
}

void setRightDrive(int power)
{
  motor[rightDrive] = power;
}

void setRightArm(int power)
{
  motor[rightArm] = power;
}

void setLeftArm(int power)
{
  motor[leftArm] = ((float)power)* 1.2;
}
