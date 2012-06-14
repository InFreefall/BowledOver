#pragma config(Sensor, S3,     sound,          sensorSoundDB)
task main()
{
  while (true)
    writeDebugStreamLine("%d",SensorValue[sound]);
}
