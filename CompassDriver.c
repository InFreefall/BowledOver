//////////////////////////////////////////////////////////////////////////////////////////
//               Hitechnic Compass Sensor Device Driver Task
//               ===========================================
//
// This file contains device driver for the HiTechnic compass. It is expected
// that this program will be "#include" in user application program to provide
// compass support to that application program.
//
// The driver includes integrity checking on the I2C link. The driver will
// reset and recover if:
//
// 1. An error occurs on a I2C message.
// 2. The compass cable is disconnected and reconnected from the NXT.
//
////////////////////////////////////////////////////////////////////////////////////////

//
// This program will only work on the NXT. Have the compiler check that this
// is the current platform in use.
//
#pragma platform(NXT)

//
// Definition of sensor sub-types. This will eventually migrate into a standard
// RobotC header file.
//
typedef enum
{
  subTypeNone                = 0,

  subTypeHiTechnicCompass    = 1,
  subTypeHiTechnicRcxIR      = 2,

  subTypeMindsensorsCompass  = 20,
  subTypeMindsensorsRcxIR    = 21,
  subTypeMindsensorsPSX      = 22,
  subTypeMindsensorsMotorMux = 22,
} TSensorSubTypes;


//
// Declaration for the compass device driver state machine
//
typedef enum
{
	stateNotCompass,
	stateCompassInitSend,
	stateCompassInitWaitReply,
	stateCompassPollSend,
	stateCompassPollWaitReply,
} TCompassState;


//
// A couple of useful variables
//
TCompassState nDriverState[4] =
{
	stateNotCompass, stateNotCompass, stateNotCompass, stateNotCompass
};

tSensors nDDPortIndex;

bool deviceDriverCompass();

#if defined(_DEBUG)
  int nBusErrors = 0;
#endif

//////////////////////////////////////////////////////////////////////////////////////////
//                           Main Device Driver Task
//
// User application program should start this task as part of its initialization
// sequence. The purpose of this task is to continually check all the sensors
// to see if any of them are configured as Hitechnic Compass. If so, the compass
// device driver handler is then called.
//
//////////////////////////////////////////////////////////////////////////////////////////

task taskCompassDeviceDriver()
{
  //
  // Give this task high priority to ensure that it runs frequently
  //
  // NOTE: this task makes frequent use of 'wait' functions to ensure that it does not
  //       hog the available CPU cycles. If you don't do this, then the higher priority
  //       task will consume all the CPU cycles and lower priority tasks will not run!
  //
  nSchedulePriority = 200;

  //
  // Loop forever, checking one sensor about every 5 milliseconds to see if it is a
  // Hitechnic sensor
  //
  while (true)
  {
  	for (nDDPortIndex = S1; nDDPortIndex <= S4; ++nDDPortIndex)
  	{
  		switch (SensorSubType[nDDPortIndex])
  		{
  			case subTypeHiTechnicCompass:
  				//
  				// We've found a digital compass. Keep calling the device driver to poll it
  				// until the device driver returns true indicating that poll has completed!
  				//
  				while (!deviceDriverCompass())
  				{
  					wait1Msec(1);
  				}
  				break;

				//
				// Checks for other Hitechnic sensors can be inserted here as more
				// sensor types are developed. Single file / task can then support multiple
				// devices.
				//

  			default:
  				break;
  		}
  	}
  	//
  	// Wait 20 msec between every polling cycle.
  	//
  	wait1Msec(20);
  }
  return;
}


//////////////////////////////////////////////////////////////////////////////////////////
//
//               Hitechnic Digital Compass Device Driver
//
// The device driver for a Hitechnic Digital Compass.
//
// Driver returns 'true' when a single polling cycle has been completed. It
// returns false to indicate that the current cycle needs more time.
//
// The driver is written as a state machine. It has three main stages:
//
//     1. One time setup of the sensor. State is also entered when the
//        sensor needs to be reset because of a communications error.
//
//     2. Send a poll request to the compass to read the current value.
//
//     3. Wait for the poll to complete and read the results. Then go to state
//        two.
//
//////////////////////////////////////////////////////////////////////////////////////////

bool deviceDriverCompass()
{
  typedef struct
  {
  	byte nMsgSize;
  	byte nDeviceAddress;
  	byte nLocationPtr;
  	byte nData;
  } TI2C_Output;


  TI2C_Output sOutput;
  byte  nReplyBytes[2];

	switch (nDriverState[nDDPortIndex])
	{
	case stateNotCompass:
		//
		// First time. Need to initialize the compass
		//
		nDriverState[nDDPortIndex] = stateCompassInitSend;
		//
		// RobotC firmware can send I2C messages at standard (1 byte per msec, ~9600 baud) or fast (5 bytes
		// per msec, ~50K baud). Most sensors (except for LEGO Sonar sensor) can use fast mode. Set the
		// compass to fast mode.
		//
		SensorType[nDDPortIndex]   = sensorI2CCustomFast;
		// fall through below

	case stateCompassInitSend:
	  //
	  // Build and send I2C message to initialize the compass to "measurement" mode
	  //

	  if (nI2CStatus[nDDPortIndex] == STAT_COMM_PENDING)
	  	return false;

	  // Flush the input buffer in case there are any bytes left from previous I2C messages.

	  while (nI2CBytesReady[nDDPortIndex] > 0)
	    readI2CReply(nDDPortIndex, nReplyBytes[0], 1);

	  sOutput.nMsgSize 				= 3;
	  sOutput.nDeviceAddress 	= 0x02;
	  sOutput.nLocationPtr 		= 0x41;
  	sOutput.nData           = 0x00;

  	sendI2CMsg(nDDPortIndex, sOutput.nMsgSize, 0);
		nDriverState[nDDPortIndex] = stateCompassInitWaitReply;
		return false;

	case stateCompassInitWaitReply:
	  //
  	// Wait for sensor initialization to complete
  	//
  	switch (nI2CStatus[nDDPortIndex])
  	{
		case NO_ERR:
	  	nDriverState[nDDPortIndex] = stateCompassPollSend;
	  	return true;

		case STAT_COMM_PENDING:
			// Keep waiting for reply. I2C messaging is not complete
			return false;

		default:
		case ERR_COMM_BUS_ERR:
	  	// re-initialize sensor. An I2C messaging error occurred.
	  	nDriverState[nDDPortIndex] = stateCompassInitSend;
#if defined(_DEBUG)
	  	++nBusErrors;
#endif
      return true;
		}
		break;

	case stateCompassPollSend:
	  //
	  // Build I2C message to initialize the compass to "measurement" mode
	  //

	  if (nI2CStatus[nDDPortIndex] == STAT_COMM_PENDING)
	  	return false;

	  sOutput.nMsgSize 				= 2;
	  sOutput.nDeviceAddress 	= 0x02;
	  sOutput.nLocationPtr 		= 0x44;
		sendI2CMsg(nDDPortIndex, sOutput.nMsgSize, 2);
		nDriverState[nDDPortIndex] = stateCompassPollWaitReply;
		return false;

	case stateCompassPollWaitReply:
	  //
  	// Wait for sensor initialization to complete
  	//
  	switch (nI2CStatus[nDDPortIndex])
  	{
		case NO_ERR:
	  	readI2CReply(nDDPortIndex, nReplyBytes[0], 2);
	  	SensorValue[nDDPortIndex] = nReplyBytes[1] * 256 + (uword) nReplyBytes[0];
      nDriverState[nDDPortIndex] = stateCompassPollSend;
	  	return true;

		case STAT_COMM_PENDING:
			// Keep waiting for reply. I2C messaging is not complete
			return false;

		default:
		case ERR_COMM_BUS_ERR:
	  	// re-initialize sensor. An I2C messaging error occurred.
	  	nDriverState[nDDPortIndex] = stateCompassInitSend;
#if defined(_DEBUG)
	  	++nBusErrors;
#endif
      return true;
		}
		break;
  }
  return true;
}
