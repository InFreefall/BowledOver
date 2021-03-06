#pragma config(Sensor, S1,     HTIRR,               sensorI2CCustom)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
 * $Id: hitechnic-irrecv-test1.c 123 2012-11-02 16:35:15Z xander $
 */

/**
 * HTIRL-driver.h provides an API for the HiTechnic IR Receiver Sensor.  This program
 * demonstrates how to use the IR Receiver API to display the currently transmitted
 * motor powers.
 *
 * Changelog:
 * - 0.1: Initial release
 * - 0.2: More comments
 *
 * Credits:
 * - Big thanks to HiTechnic for providing me with the hardware necessary to write and test this.
 *
 * License: You may use this code as you wish, provided you give credit where it's due.
 *
 * THIS CODE WILL ONLY WORK WITH ROBOTC VERSION 3.54 AND HIGHER.
 * Xander Soldaat (xander_at_botbench.com)
 * 25 November 2009
 * version 0.2
 */

#include "drivers/hitechnic-irrecv.h"

/*
  =============================================================================
  main task with some testing code

 */
task main {
  sbyte _motA = 0;
  sbyte _motB = 0;

  nxtDisplayCenteredTextLine(0, "HiTechnic");
  nxtDisplayCenteredBigTextLine(1, "IR Recv");
  nxtDisplayCenteredTextLine(3, "Test 1");
  nxtDisplayCenteredTextLine(5, "Connect sensor");
  nxtDisplayCenteredTextLine(6, "to S1");
  wait1Msec(2000);
  eraseDisplay();

  while (true) {
    for (int i = 1; i < 5; i++) {
      // Read the motor powers sent by the remote on the specified channel
      // and display them.
      HTIRRreadChannel(HTIRR, i, _motA, _motB);
      nxtDisplayTextLine(i, "%4d, %4d", _motA, _motB);
      wait1Msec(10);
    }
  }
}

/*
 * $Id: hitechnic-irrecv-test1.c 123 2012-11-02 16:35:15Z xander $
 */
