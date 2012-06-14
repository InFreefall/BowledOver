#pragma config(Sensor, S1,     HTRCX,               sensorI2CCustom)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/*
 * $Id: HTRCX-test1.c 48 2011-02-13 20:35:38Z xander $
 */

/**
 * HTRCX-driver.h provides an API for the HiTechnic IR Link Sensor to allow
 * communication between the NXT and RCX.
 *
 * Changelog:
 * - 0.1: Initial release
 *
 * Credits:
 * - Big thanks to HiTechnic for providing me with the hardware necessary to write and test this.
 * - Big thanks to Thorsten Benter for giving me one of his RCXs to work with!
 *
 * License: You may use this code as you wish, provided you give credit where it's due.
 *
 * THIS CODE WILL ONLY WORK WITH ROBOTC VERSION 2.00 AND HIGHER.
 * Xander Soldaat (mightor_at_gmail.com)
 * 16 September 2010
 * version 0.1
 */

#include "drivers/HTRCX-driver.h"

task main () {
  short msg = 0x00FD;
  eraseDisplay();
  nxtDisplayTextLine(6, "Press [enter]");
  nxtDisplayTextLine(7, "to send msg");
  while(true) {
    // Increment the msg
    msg++;
    while(nNxtButtonPressed != kEnterButton) EndTimeSlice();
    while(nNxtButtonPressed != kNoButton) EndTimeSlice();
    // Send the message to the RCX and display
    PlaySound(soundBlip);
    nxtDisplayCenteredBigTextLine(2, "0x%04X", msg);
    HTRCXsendWord(HTRCX, msg);
  }
}

/*
 * $Id: HTRCX-test1.c 48 2011-02-13 20:35:38Z xander $
 */