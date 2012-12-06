#pragma config(Sensor, S3,     compass,        sensorI2CCustom)

#include "rdpartyrobotcdr-v2.4/drivers/HTMC-driver.h"

task main()
{
  HTMCsetTarget(compass,0);
  while (true)
  {
    int heading = HTMCreadHeading(compass);
    writeDebugStreamLine("%d",heading);
    nxtDisplayCenteredTextLine(1,"%d",heading);
  }
}
