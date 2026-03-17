/*
 Hekulex.h - Library for Dongbu Herkulex DRS-0101/DRS-0201 
 Copyright (c) 2012 - http://robottini.altervista.org
 Created by Alessandro on 09/12/2012.
 
 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 
 This library is distributed in the hope that it will be useful,  
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
 *****************************************************************************
    PLEASE START READING: Herkulex Servo Manual (http://www.hovis.co.kr/guide/herkulexeng.pdf)
 *****************************************************************************
 
 IMPORTANT:

  The library works on Arduino UNO/2009 - Arduino Mega.
  Please with Arduino UNO/2009 works with SoftwareSerial library modified with baud rate 57.600.
  Use this begin type:
		begin(57600, int rx, int tx);
 
  For Arduino Mega, please use baud rate 115.200

 *****************************************************************************
 Contact: alegiaco@gmail.com
 Web:     http://robottini.altervista.org
 Autor:   Alessandro Giacomel
 *****************************************************************************  
*/

#ifndef Herkulex_h
#define Herkulex_h
#include "Arduino.h"


// contains all motor movement information to send to servo
struct motorMoveInfo {
  uint16_t goalPos;   // raw goal position as defined in datasheets
  uint8_t ledColor;   // LED colors as defined by datasheet
  uint8_t servoID;    // ID of servo to communicate to
  uint8_t playTime;   // how long the movement should last for: overridden by explicitly defined play time for a simultaneous movement (as done in actionMoves)
};

#define DATA_SIZE	 30		// buffer for input data
#define DATA_MOVE  	 50		// max 10 servos <---- change this for more servos!
#define TIME_OUT     5   	//timeout serial communication

// SERVO HERKULEX COMMAND - See Manual p40
#define HEEPWRITE    0x01 	//Rom write
#define HEEPREAD     0x02 	//Rom read
#define HRAMWRITE	 0x03 	//Ram write
#define HRAMREAD	 0x04 	//Ram read
#define HIJOG		 0x05 	//Write n servo with different timing
#define HSJOG		 0x06 	//Write n servo with same time
#define HSTAT	 	 0x07 	//Read error
#define HROLLBACK	 0x08 	//Back to factory value
#define HREBOOT	 	 0x09 	//Reboot

#define CONVERT_PLAYTIME_TO_MS 11.2 // conversion factor for turning a playtime duration sent to and used by the servo into a millisecond duration



// HERKULEX LED - See Manual p29
typedef enum { 
  LED_OFF = 0x00,
  LED_GREEN = 0x01,
  LED_BLUE = 0x02,
  LED_RED = 0x04,
} LED_STATE;


// HERKULEX STATUS ERROR - See Manual p39
static byte H_STATUS_OK					= 0x00;
static byte H_ERROR_INPUT_VOLTAGE 		= 0x01;
static byte H_ERROR_POS_LIMIT			= 0x02;
static byte H_ERROR_TEMPERATURE_LIMIT	= 0x04;
static byte H_ERROR_INVALID_PKT			= 0x08;
static byte H_ERROR_OVERLOAD			= 0x10;
static byte H_ERROR_DRIVER_FAULT  		= 0x20;
static byte H_ERROR_EEPREG_DISTORT		= 0x40;

// HERKULEX Broadcast Servo ID
static byte BROADCAST_ID = 0xFE;

class HerkulexClass {
public:
  HerkulexClass(uint8_t busID);
  void beginSerialBus(long baud);


  void  end();

  void  initialize();
  byte  stat(int servoID);
  void  setACKPolicy(int valueACK);
  byte  checkModel();
  void  setID(int ID_Old, int ID_New);
  void  clearError(int servoID);

  void  torqueON(int servoID);
  void  torqueOFF(int servoID);

  void  queueMove(motorMoveInfo moveInfo);
  void  actionMoves(uint8_t playTime);

  void  moveOne(motorMoveInfo moveInfo);

  uint16_t getPosition(int servoID);
  int   getSpeed(int servoID);
    
  void  reboot(int servoID);
  void  setLed(int servoID, int valueLed);

  void  writeRegistryRAM(int servoID, int address, int writeByte);
  void  writeRegistryEEP(int servoID, int address, int writeByte);

  void sendData(byte* buffer, int lenght);
  void readData(int size);


  int packetSize;
  int pID;
  int cmd;
  int packetLength;
  int ck1;
  int ck2;
  byte dataEx[DATA_MOVE+8];
  byte data[DATA_SIZE]; 
  byte outputBuffer[DATA_MOVE];

// private area  
private:


  int  checksum1(byte* data, int lenghtString);
  int  checksum2(int XOR);
  
  void clearBuffer();
  void printHexByte(byte x);

  int _serialPort;
  
  
  int queuedPacketCount;
  
  int XOR;
  int playTime;
};

#endif
