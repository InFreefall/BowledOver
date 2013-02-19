
////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                 Setup for Wifi Sensor
//                 Sets up flow control
// Script for connecting to internet
//
// ate0     // Turn off the echo.
// at&k1    // Enable software flow control.
//
// at+wauth=1 // Authentication mode: open
// at+wwep1=0fb3ba79eb  // Set WEP key
// at+wa=NPX97    // Connect to the SSID
// at+ndhcp=1   // Enable DHCP <Dynamic Host Configuration Protocol>
//
// at+dnslookup=www.dexterindustries.com  // Lookup the DNS configuration.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma systemFile

#ifndef __COMMON_H__
#include "common.h"
#endif

#ifndef __RS485_H__
#include "common-rs485.h"
#endif

#define DEBUG_WIFI 1

//#ifdef DEBUG_WIFI
//#define writeRawHS(X, Y) debugnxtWriteRawHS(X, Y)
//#else
//#define writeRawHS(X, Y) nxtWriteRawHS(X, Y)
//#endif

//typedef ubyte buff_t[128];
//buff_t buffer;

long nRcvChars = 0;
ubyte BytesRead[8];
char *ssid = "legolair";
//ubyte newline[] = {0x0D};
ubyte linefeed[] = {0x0A};
ubyte endmarker[] = {27, 'E'};
char *wep_key = "0FB3BA79EB";
char *wpa_psk = "legolair";
//ubyte wpa_psk[] = {'w','l','a','n','s'};

long baudrates[] = {9600, 19200, 38400, 57600, 115200, 230400,460800, 921600};
string DWIFIscratchString;

//int RS485appendToBuff(buff_t &buf, const short index, const ubyte &pData, const short nLength)
//{
//  if (index == 0) memset(buf, 0, sizeof(buf));

//  memcpy(&buf[index], &pData, nLength);
//  return index + nLength;
//}

//void debugnxtWriteRawHS(const ubyte &pData, const short nLength)
//{
//  string tmpString;
//  ubyte buff[30];
//  memset(&buff[0], 0, 30);
//  memcpy(&buff[0], &pData, nLength);
//  StringFromChars(tmpString, &buff[0]);
//  writeDebugStream("%s", tmpString);
//  nxtWriteRawHS(&pData, nLength);
//}


////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Setup High Speed on Port 4.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////


int checkFailure() {
  ubyte currByte[] = {0};
  ubyte prevByte[] = {0};

  while (nxtGetAvailHSBytes() == 0) wait1Msec(5);

  while (nxtGetAvailHSBytes() > 0) {
    nxtReadRawHS(&currByte[0], 1);
    if ((prevByte[0] == 27) && (currByte[0] == 'F'))
      return 1;
    else if ((currByte[0] > '0') && (currByte[0] < '7'))
      return 1;
    else if (currByte[0] == '9')
      return 2;
  }
  return 0;
}

//void setupHighSpeedLink(const bool bMaster)
//{
  // Initialize port S$ to "high speed" mode.
//  nxtEnableHSPort();
//  nxtSetHSBaudRate(9600);
//  nxtHS_Mode = hsRawMode;
//  return;
//}

//void Receive(bool wait=false)
//{
//  if (wait)
//    while (nxtGetAvailHSBytes() == 0) wait1Msec(5);

//  while (nxtGetAvailHSBytes() > 0) {
//    nxtReadRawHS(&BytesRead[0], 1);
//    writeDebugStream("%c", BytesRead[0]);
//    wait1Msec(2);
//  }
//}

void setupBAUDSpeed(long baudrate) {
  int index = 0;
  string baud_cmd;

  StringFormat(baud_cmd, "ATB=%d\n", baudrate);
  index = RS485appendToBuff(RS485txbuffer, index, baud_cmd);

  RS485write(RS485txbuffer, index);
  wait1Msec(100);
  nxtDisableHSPort();
  wait1Msec(10);
  nxtEnableHSPort();
  nxtSetHSBaudRate(baudrate);
  nxtHS_Mode = hsRawMode;
  wait1Msec(10);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//      Clear Read Buffer
//      Run this to clear out the reading buffer.
//      Simply sends a carriage return, then clears the buffer out.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

//void clear_read_buffer()
//{
//  ubyte nData[] = {13};
//  writeRawHS(nData[0], 1);   // Send the carriage return
//  wait1Msec(100);
//  while(BytesRead[0] < 0){
//    nxtReadRawHS(&BytesRead[0], 1);    // Read the response.  Probably an error.
//  }
//  wait1Msec(100);
//}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//      Echo All Input Off - Turns off the echo effect on the wifi.
//      Sending the serial command "ate0" which turns off the echo effect.
//      Sends one single byte at a time, pauses.
//      Drains receiver with a read each time.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

void echo_all_input_off()
{
  writeDebugStreamLine("echo_all_input_off");
  ubyte nData[] = {'A','T','E','0',0x0D};

  //writeRawHS(nData[0], sizeof(nData));

  for(int i = 0; i < 5; i++){
    nxtWriteRawHS(&nData[i], 1);            // Send the command, byte by byte.
    nxtReadRawHS(&BytesRead[0], 8);         // Clear out the echo.
    wait10Msec(10);

  }

  // wait10Msec(100);
}

bool set_verbose(bool on)
{
  writeDebugStreamLine("set_verbose");

  if (on)
    DWIFIscratchString = "ATV1\r";
  else
    DWIFIscratchString = "ATV0\r";

  memcpy(RS485txbuffer, DWIFIscratchString, strlen(DWIFIscratchString));
  return RS485write(RS485txbuffer, strlen(DWIFIscratchString));
  // wait10Msec(100);
}

bool echo_all_input(bool on)
{
  writeDebugStreamLine("set_verbose");

  if (on)
    DWIFIscratchString = "ATE1\r";
  else
    DWIFIscratchString = "ATE0\r";

  memcpy(RS485txbuffer, DWIFIscratchString, strlen(DWIFIscratchString));
  return RS485write(RS485txbuffer, strlen(DWIFIscratchString));

}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//      Software Flow Control On
//      Send string "at&k1" and carriage return.
//      Shouldn't need the wait or read now that we've got the echo off.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

bool software_flow_control()
{
  DWIFIscratchString = "AT&K1\r";
  memcpy(RS485txbuffer, DWIFIscratchString, strlen(DWIFIscratchString));
  return RS485write(RS485txbuffer, strlen(DWIFIscratchString));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//      Scan for networks
//      Send string "AT+WS" and carriage return.
//      Don't really need to do this; gets back a tonne of data.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

bool scan_for_networks()
{
  DWIFIscratchString = "AT+ws\r";
  memcpy(RS485txbuffer, DWIFIscratchString, strlen(DWIFIscratchString));
  return RS485write(RS485txbuffer, strlen(DWIFIscratchString));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//      WIFI:  Set Authentication Mode
//             Send string "at+wauth=<n>" and carriage return.
//              n = 0 --> Off
//              n = 1 --> On
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

bool wifi_auth_mode(int state)
{
  if (state == 0)
    DWIFIscratchString = "AT+WAUTH=0\r";
  else
    DWIFIscratchString = "AT+WAUTH=1\r";

  memcpy(RS485txbuffer, DWIFIscratchString, strlen(DWIFIscratchString));
  return RS485write(RS485txbuffer, strlen(DWIFIscratchString));
  // wait10Msec(100);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//      WIFI:  Set WEP key.
//             Send string "at+WWEPn=<m>" and carriage return.
//
//                  m = WEP Key.
//                  n = type of WEP key.
//                        n= 0
//                        n= 1
//                        n= 2
//
////////////////////////////////////////////////////////////////////////////////////////////////////////

void set_ssid() {
  string ssid_cmd = "AT+WA=";
  int index = 0;

  index = RS485appendToBuff(RS485txbuffer, index, ssid_cmd);
  index = RS485appendToBuff(RS485txbuffer, index, ssid, sizeof(ssid));
  index = RS485appendToBuff(RS485txbuffer, index, newline, 1);
  RS485write(RS485txbuffer, index);
}

bool set_wep_key(int keyindex)
{
  int index = 0;
  writeDebugStreamLine("set_wep_key");

  StringFormat(DWIFIscratchString, "AT+WWEP%d=", keyindex);

  // Initalize wep_key
  index = RS485appendToBuff(RS485txbuffer, index, DWIFIscratchString);
  index = RS485appendToBuff(RS485txbuffer, index, wep_key, sizeof(wep_key));
  index = RS485appendToBuff(RS485txbuffer, index, newline, 1);
  return RS485write(RS485txbuffer, index);
}


bool set_wpa_psk()
{
  int index = 0;
  writeDebugStreamLine("set wpa key");
  DWIFIscratchString = "AT+WPAPSK=";

  index = RS485appendToBuff(RS485txbuffer, index, DWIFIscratchString);
  index = RS485appendToBuff(RS485txbuffer, index, ssid, sizeof(ssid));
  index = RS485appendToBuff(RS485txbuffer, index, ",", 1);
  index = RS485appendToBuff(RS485txbuffer, index, wpa_psk, sizeof(wpa_psk));
  index = RS485appendToBuff(RS485txbuffer, index, newline, 1);

  return RS485write(RS485txbuffer, index);
}

bool setDHCP(bool useDHCP)
{
  if (useDHCP)
    return RS485sendString("AT+NDHCP=1\r");
  else
    return RS485sendString("AT+NDHCP=0\r");

  //RS485sendString(DWIFIscratchString);

  wait10Msec(100);
}

bool getFW()
{
  writeDebugStreamLine("getFW");
  return RS485sendString("AT+VER=?\r");
  //DWIFIscratchString = {'A','T','+','V','E','R','=','?',0x0D};

  //ubyte getfw_cmd[] = {'A','T','+','V','E','R','=','?',0x0D};
  //writeRawHS(getfw_cmd[0], sizeof(getfw_cmd));       // Send the command, byte by byte.
}


bool getInfo() {
  writeDebugStreamLine("getInfo");
  return RS485sendString("AT+NSTAT=?\r");
  //ubyte status_cmd[] = {'A','T','+','N','S','T','A','T', '=', '?', 0x0D};
  //writeRawHS(status_cmd[0], sizeof(status_cmd));       // Send the command, byte by byte.
}

bool getInfoWLAN() {
  writeDebugStreamLine("getInfo");
  return RS485sendString("AT+WSTATUS\r");
  //ubyte status_cmd[] = {'A','T','+','W','S','T','A','T','U','S', 0x0D};
  //writeRawHS(status_cmd[0], sizeof(status_cmd));       // Send the command, byte by byte.
}

bool closeAllConns() {
  int len;
  writeDebugStreamLine("closeAllCons");
  return RS485sendString("AT+NCLOSEALL\r");
  //ubyte close_cmd[] = {'A','T','+','N','C','L','O','S','E','A','L','L',0x0D};
  //writeRawHS(close_cmd[0], sizeof(close_cmd));       // Send the command, byte by byte.
  //RS485read(RS485rxbuffer, len);
}

bool closeConn(int cid) {
  //int len;
  writeDebugStreamLine("closeConn");
  //cid += 48;
  StringFormat(DWIFIscratchString, "AT+NCLOSE=%d\r", cid);
  return RS485sendString(DWIFIscratchString);
  //ubyte close_cmd[] = {'A','T','+','N','C','L','O','S','E','=',cid,0x0D};
  //writeRawHS(close_cmd[0], sizeof(close_cmd));       // Send the command, byte by byte.
  //RS485read(RS485rxbuffer, len);
}

void SaveConfig() {
  int len;
  writeDebugStreamLine("save config");
  RS485sendString("AT&W0\r");
  wait1Msec(10);
  RS485sendString("AT&Y0\r");
  //ubyte save_profile_cmd[] = {'A','T','&','W','0', 13};
  //ubyte set_def_profile_cmd[] = {'A','T','&','Y','0', 13};
  //writeRawHS(save_profile_cmd[0], sizeof(save_profile_cmd));
  //RS485read(RS485rxbuffer, len);
  //wait1Msec(10);
  //writeRawHS(set_def_profile_cmd[0], sizeof(set_def_profile_cmd));
  //RS485read(RS485rxbuffer, len);
}


void startListen(long port) {
  int index = 0;
  int len = 0;

  string listen_cmd;
  StringFormat(listen_cmd, "AT+NSTCP=%d\n", port);
  index = RS485appendToBuff(RS485txbuffer, index, listen_cmd);
  RS485write(RS485txbuffer, index);
  RS485read(RS485rxbuffer, len, 100);
}

long scanBaudRate() {
  ubyte tmpbuff[8];
  string tmpString;
  ubyte attention[] = {'+','+','+',13};
  for (int i = 0; i < 8; i++) {
    memset(tmpbuff, 0, sizeof(tmpbuff));
    nxtDisableHSPort();
	  wait1Msec(10);
	  nxtEnableHSPort();
	  nxtSetHSBaudRate(baudrates[i]);
	  nxtHS_Mode = hsRawMode;
	  RS485clearRead();
	  wait1Msec(1000);
	  nxtWriteRawHS(&attention[0], sizeof(attention));
	  nxtReadRawHS(tmpbuff, 7);  // make sure last ubyte is always NULL
    StringFromChars(tmpString, &tmpbuff[0]);
    if ((StringFind(tmpString, "ERR") > -1) ||
        (StringFind(tmpString, "OK") > -1) ||
        (StringFind(tmpString, "0") > -1) ||
        (StringFind(tmpString, "2") > -1)) {
      RS485clearRead();
      return baudrates[i];
    }
	}
	RS485clearRead();
	return 0;
}

void configureWiFi()
{
  int len;
  RS485clearRead();      // Clear out the buffer and test TX/RX.
  wait10Msec(100);          // Must be run first!
  echo_all_input_off();     // Must be run first!
  wait10Msec(100);          // Must be run first!
  RS485read(RS485rxbuffer, len);
  eraseDisplay();
  software_flow_control();  // Must be run first!
  RS485clearRead();
  set_verbose(false);
  wait1Msec(100);
  RS485clearRead();
  //setupBAUDSpeed(230400);
  setupBAUDSpeed(460800);
  wait1Msec(100);
  RS485read(RS485rxbuffer, len);
  set_wpa_psk();
  RS485read(RS485rxbuffer, len);
  RS485read(RS485rxbuffer, len);
  set_ssid();
  RS485read(RS485rxbuffer, len);
  setDHCP(1);
  RS485read(RS485rxbuffer, len);
  SaveConfig();
  RS485read(RS485rxbuffer, len);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                        Receive Bytes
//
////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//                                        Main Task
//
////////////////////////////////////////////////////////////////////////////////////////////////////////
