#pragma config(Sensor, S1,     LEGOLS,              sensorLightInactive)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
 * $Id: DFLEX-test1.c 48 2011-02-13 20:35:38Z xander $
 */

/**
 * DFLEX-driver.h provides an API for the Dexter Industries dFlex Sensor.  This program
 * demonstrates how to use that API to calibrate the sensor.
 *
 * Changelog:
 * - 0.1: Initial release
 *
 * Credits:
 * - Big thanks to Dexter Industries for providing me with the hardware necessary to write and test this.
 *
 * License: You may use this code as you wish, provided you give credit where it's due.
 *
 * THIS CODE WILL ONLY WORK WITH ROBOTC VERSION 2.00 AND HIGHER.
 * Xander Soldaat (mightor_at_gmail.com)
 * 23 June 2010
 * version 0.1
 */

#include "drivers/DFLEX-driver.h"

task main() {
  int raw = 0;
  int nrm = 0;

  nNxtButtonTask  = -2;

  eraseDisplay();
  nxtDisplayTextLine(0, "Dexter Industries");
  nxtDisplayCenteredBigTextLine(1, "dFlex");
  nxtDisplayCenteredTextLine(3, "Test 1");
  nxtDisplayCenteredTextLine(5, "Connect sensor");
  nxtDisplayCenteredTextLine(6, "to S1");
  wait1Msec(2000);

  eraseDisplay();
  nxtDisplayTextLine(0, "dFlex Sensor");

  while (true) {
    nxtDisplayClearTextLine(5);
    nxtDisplayClearTextLine(6);

    // Get the raw value from the sensor
    raw = DFLEXvalRaw(LEGOLS);

    // Get the normalised value from the sensor
    nrm = DFLEXvalNorm(LEGOLS);

    nxtDisplayTextLine(2, "Raw:  %4d", raw);
    nxtDisplayTextLine(4, "Norm: %4d", nrm);
    wait1Msec(50);
  }
}

/*
 * $Id: DFLEX-test1.c 48 2011-02-13 20:35:38Z xander $
 */
