void turn();

task main()
{
  motor[motorA] = 25;
  while (true) {
    turn();
    //PlaySound(soundShortBlip);
  }
}

void turn()
{
    bool soundPlayed = false;
  while (abs(nMotorEncoder[motorA]) < 340)
  {
    if (abs(nMotorEncoder[motorA]) > 340/2 && !soundPlayed)
    {
      PlaySound(soundShortBlip);
      soundPlayed = true;
    }
    }
  nMotorEncoder[motorA] = 0;
  motor[motorA] *= -1;
}
