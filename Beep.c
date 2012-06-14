task main()
{
  while (true)
  {
    PlayTone(100,10);
    wait10Msec(100);
    nxtDisplayCenteredTextLine(3,"NXTBAT:%d",externalBatteryAvg);
  }
}
