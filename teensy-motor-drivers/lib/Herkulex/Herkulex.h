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

#define DATA_SIZE	 30		               // buffer for input data
#define DATA_MOVE  	 50		             // max 10 servos <---- change this for more servos!
#define SERIAL_READ_TIMEOUT_US  2000   // timeout serial communication (microseconds)

// SERVO HERKULEX COMMAND - See Manual p40
// #define HEEPWRITE    0x01 	//Rom write
// #define HEEPREAD     0x02 	//Rom read
// #define HRAMWRITE	 0x03 	//Ram write
// #define HRAMREAD	 0x04 	//Ram read
// #define HIJOG		 0x05 	//Write n servo with different timing
// #define HSJOG		 0x06 	//Write n servo with same time
// #define HSTAT	 	 0x07 	//Read error
// #define HROLLBACK	 0x08 	//Back to factory value
// #define HREBOOT	 	 0x09 	//Reboot

#define CONVERT_PLAYTIME_TO_MS 11.2 // conversion factor for turning a playtime duration sent to and used by the servo into a millisecond duration



// HERKULEX LED - See Manual p29
typedef enum { 
  LED_OFF = 0x00,
  LED_GREEN = 0x01,
  LED_BLUE = 0x02,
  LED_RED = 0x04,
  

  // MIX
  LED_PURPLE = LED_BLUE | LED_RED,
} LED_STATE;


typedef enum {

  BASE_LENGTH = 0x07,
  //setID
  HEEPWRITE_LENGTH_1 = 0x0A, 
  HEEPWRITE_DATA_LENGTH_1 = 0x03,

  //writeRegistryEEP
  HEEPWRITE_LENGTH_2 = 0x0B,    //before it was 0x0A but for 4 optional data i see 0x0B in page 36 of datasheet
  HEEPWRITE_DATA_LENGTH_2 = 0x04,

  //checkModel
  HEEPREAD_LENGTH = 0x09,
  HEEPREAD_DATA_LENGTH = 0x02,

  REGISTER_INFO_LENGTH = 0x02,
 
  //clearError, writeRegistryRAM
  HRAMWRITE_LENGTH = 0x0B,    //before it was also 0x0A, checked page 37 writeRegistryRAM
  HRAMWRITE_DATA_LENGTH = 0x04,

  //setLed, setACKPolicy, torqueFree, torqueON
  SET_ACK_POLICY_RAMWRITE_LENGTH = 0x0A,    //before it was also 0x0A, checked page 37 writeRegistryRAM
  SET_ACK_POLICY_RAMWRITE_DATA_LENGTH = 0x03,

  //getSpeed, requestPosition, 
  HRAMREAD_LENGTH = 0x09, 
  HRAMREAD_DATA_LENGTH = 0x02,

  // not used
  // HIJOG_LENGTH = 0x0A, 
  // HIJOG_DATA_LENGTH = 0x04,

  //moveOne
  HSJOG_MOVEONE_LENGTH = 0x0C,
  HSJOG_MOVEONE_DATA_LENGTH = 0x05,
  
  //actionMoves
  HSJOG_MOVEMULTIPLE_LENGTH = 0x08,
  HSJOG_MOVEMULTIPLE_DATA_LENGTH = 0x01,

  HSTAT_LENGTH = 0x07,      // STRANGE, because it would seem like the datasheet, page 42, is wrong about this one. I think that first row should have no optional data
  HSTAT_DATA_LENGTH = 0x00,

  // not used
  // HROLLBACK_LENGTH = 0x0A, 
  // HROLLBACK_DATA_LENGTH = 0x04,

  //reboot
  HREBOOT_LENGTH = 0x07, 
  HREBOOT_DATA_LENGTH = 0x00,

  
  GETPOS_RESPONSE = 13       // bytes expected back from a RAMREAD position query


} PACKET_LENGTH_BYTES;

typedef enum {
  PACKET_HEADER = 0xFF,
  ALL_SERVOS = 0xFE,
} PACKET_CONSTS;

typedef enum {
  NO_REPLY = 0x00,
  REPLY_TO_READ = 0x01, // normal operating mode
  REPLY_TO_ALL = 0x02,
} ACK_POLICY_TYPE;

typedef enum {
  BREAK_ON = 0x40,
  TORQUE_ON = 0x60,
  TORQUE_FREE = 0x00,
} TORQUE_MODE;

typedef enum {
  HEEPWRITE = 0x01,
  HEEPREAD = 0x02,
  HRAMWRITE = 0x03,
  HRAMREAD = 0x04,
  HIJOG = 0x05,
  HSJOG = 0x06,
  HSTAT = 0x07,
  HROLLBACK = 0x08,
  HREBOOT = 0x09, 
} COMMAND;

typedef enum {
  HEEPWRITE_RESPONSE = 0x41,
  HEEPREAD_RESPONSE = 0x42,
  HRAMWRITE_RESPONSE = 0x43,
  HRAMREAD_RESPONSE = 0x44,
  HIJOG_RESPONSE = 0x45,
  HSJOG_RESPONSE = 0x46,
  HSTAT_RESPONSE = 0x47,
  HROLLBACK_RESPONSE = 0x48,
  HREBOOT_RESPONSE = 0x49,
} COMMAND_RESPONSE;


typedef enum {
  ACK_POLICY = 0x01,
  STATUS_ERROR = 0x30,
  TORQUE_CONTROL = 0x34,
  LED_CONTROL = 0x35,
  CALIBRATED_POS = 0x3A,
  PWM = 0x40,
} RAM_REGISTER;

typedef enum {
  MOTOR_MODEL = 0x00,
} EEP_REGISTER;

// HERKULEX STATUS ERROR - See Manual p39
static byte H_STATUS_OK					= 0x00;
static byte H_ERROR_INPUT_VOLTAGE 		= 0x01;
static byte H_ERROR_POS_LIMIT			= 0x02;
static byte H_ERROR_TEMPERATURE_LIMIT	= 0x04;
static byte H_ERROR_INVALID_PKT			= 0x08;
static byte H_ERROR_OVERLOAD			= 0x10;
static byte H_ERROR_DRIVER_FAULT  		= 0x20;
static byte H_ERROR_EEPREG_DISTORT		= 0x40;


class HerkulexClass {
public:
  HerkulexClass();
  HerkulexClass(uint8_t serialPort);

  void beginSerialBus(long baud);
  void endSerialBus();

  void  initialize();
  bool  stat(uint8_t servoID, uint8_t* statError, uint8_t* statDetail);
  void  setACKPolicy(int valueACK);
  uint16_t checkModel(uint8_t servoID);
  void  setID(int ID_Old, int ID_New);
  void  clearError(int servoID);

  void  torqueON(int servoID);
  void  torqueFree(int servoID);

  void  queueMove(motorMoveInfo moveInfo);
  void  actionMoves(uint8_t playTime);

  void  moveOne(motorMoveInfo moveInfo);

  uint16_t getPosition(int servoID);
  void sendPosRequest(int servoID);
  uint16_t collectPosition(int servoID);

  uint16_t getSpeed(int servoID);
    
  void  reboot(int servoID);
  void  setLed(uint8_t servoID, LED_STATE valueLed);

  
  void sendData(uint8_t* buffer, uint8_t length);
  void requestRead(uint8_t length);
  void updateRead();
  boolean isReplyReady();
  
  
  private:
  
  
  int _serialPort;
  HardwareSerial* _serial = nullptr; // store pointer to serial object
  
  uint8_t calcChecksumOne();
  uint8_t calcChecksumTwo();
  
  
  void resetClassVals();
  
  void clearBuffer();
  void printHexByte(byte x);
  
  void writeToRegister(uint8_t servoID, uint8_t address, uint8_t* writeData, uint8_t writeDataLength, COMMAND cmd);
  void writeToRamRegister(uint8_t servoID, RAM_REGISTER address, uint8_t* writeData, uint8_t writeDataLength);
  void writeToEEPRegister(uint8_t servoID, EEP_REGISTER address, uint8_t* writeData, uint8_t writeDataLength);

  void requestFromRegister(uint8_t servoID, uint8_t address, uint8_t numRequestedBytes, COMMAND cmd);
  void requestFromRamRegister(uint8_t servoID, RAM_REGISTER address, uint8_t numRequestedBytes);
  void requestFromEEPRegister(uint8_t servoID, EEP_REGISTER address, uint8_t numRequestedBytes);
  bool readFromRegisterBlocking(uint8_t servoID, uint8_t address, uint8_t numRequestedBytes, uint8_t* buffer, COMMAND cmd, COMMAND_RESPONSE cmd_res);
  bool readFromRamRegisterBlocking(uint8_t servoID, RAM_REGISTER address, uint8_t numRequestedBytes, uint8_t* buffer);
  bool readFromEEPRegisterBlocking(uint8_t servoID, EEP_REGISTER address, uint8_t numRequestedBytes, uint8_t* buffer);

  void buildPacket(uint8_t servoID, uint8_t* optionalData, uint8_t optionalDataLength, COMMAND cmd);
  void sendPacket(uint8_t servoID, uint8_t* data, uint8_t dataLength, COMMAND cmd);

  bool verifyInputPacket(uint8_t* inputPacket, uint8_t inputPacketLength);
  bool readPacketReply(uint8_t servoID, uint8_t* outputBuffer, uint8_t optionalDataLength, COMMAND_RESPONSE cmd_res);
  
  uint8_t queuedPacketCount;


  // serial reading logic helpers
  boolean newDataInInputBuffer;
  boolean readPending;
  uint8_t inputLength;
  uint32_t readStartTime;

  bool readBlocking(uint8_t length);
  

  // base packet info
  uint8_t packetLength; 
  uint8_t pID;            // Servo ID
  uint8_t CMD;            // Command Type
  uint8_t checksumOne;
  uint8_t checksumTwo;

  uint8_t additionalDataLength; // length of additional data

  // servo jog "optional data"
  uint8_t playTime;
  uint8_t goalLSB; // lower 8 bits of goal
  uint8_t goalMSB; // upper 8 bits of goal : in total 16 bit goal
  uint8_t SET; // called in datasheet, it contains multiple bits of distinct info
  uint8_t ID; // seperate from pID in datasheet but same for our use case

  uint8_t packet[DATA_MOVE + PACKET_LENGTH_BYTES::HSJOG_MOVEMULTIPLE_LENGTH]; // stores full packet to send

  uint8_t packetQueue[DATA_MOVE];  // stores move packets for simulataneous jog

  uint8_t inputBuffer[DATA_MOVE]; // stores input HOW LARGE DOES THIS NEED TO BE?
  uint8_t checksumData[PACKET_LENGTH_BYTES::BASE_LENGTH];  // stores checksumdata for input validation 
};

#endif
