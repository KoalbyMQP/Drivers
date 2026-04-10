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
#define SERIAL_READ_TIMEOUT_US  5000   // timeout serial communication (microseconds)

#define CONVERT_PLAYTIME_TO_MS 11.2 // conversion factor for turning a playtime duration sent to and used by the servo into a millisecond duration



// HERKULEX LED - See Manual p29
typedef enum { 
  LED_OFF = 0x00,
  LED_GREEN = 0x01,
  LED_BLUE = 0x02,
  LED_RED = 0x04,
  

  // MIX
  LED_PURPLE = LED_BLUE | LED_RED,
  LED_ALL = LED_PURPLE | LED_GREEN,
} LED_STATE;


typedef enum {

  BASE_LENGTH = 0x07,
  //setID
  HEEPWRITE_LENGTH_1 = 0x0A, 
  HEEPWRITE_DATA_LENGTH_1 = 0x03,


  REGISTER_INFO_LENGTH = 0x02,
  HSJOG_MOVEONE_DATA_LENGTH = 0x05,
  HSJOG_MOVEMULTIPLE_DATA_LENGTH = 0x01,
  HSTAT_DATA_LENGTH = 0x00,
  HREBOOT_DATA_LENGTH = 0x00,
  
  GETPOS_RESPONSE = 13,    // bytes expected back from a RAMREAD position query

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
  SPEED_1M   = 0x01,  // 1,000,000 bps
  SPEED_667K = 0x02,  // 666,666 bps, fastest with 0201s
  SPEED_500K = 0x03,  // 500,000 bps
  SPEED_400K = 0x04,  // 400,000 bps
  SPEED_250K = 0x07,  // 250,000 bps
  SPEED_200K = 0x09,  // 200,000 bps
  SPEED_115K = 0x10,  // 115,200 bps (default)
  SPEED_57K  = 0x22,  // 57,600 bps
} BAUD_RATE;

const std::unordered_map<BAUD_RATE, uint32_t> BAUD_RATE_MAP = {
    {BAUD_RATE::SPEED_1M,   1'000'000},
    {BAUD_RATE::SPEED_667K,   666'666},
    {BAUD_RATE::SPEED_500K,   500'000},
    {BAUD_RATE::SPEED_400K,   400'000},
    {BAUD_RATE::SPEED_250K,   250'000},
    {BAUD_RATE::SPEED_200K,   200'000},
    {BAUD_RATE::SPEED_115K,   115'200},
    {BAUD_RATE::SPEED_57K,     57'600},
};

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
  BAUD_SETTING = 0x04,
  ID = 0x06,
} EEP_REGISTER;

typedef enum {
  H_STATUS_OK = 0x00,
  H_EXCEED_INPUT_VOLTAGE = 0x01,
  H_EXCEED_POT_LIMIT = 0x02,
  H_EXCEED_TEMP_LIMIT = 0x04,
  H_INVALID_PACKET = 0x08,
  H_OVERLOAD_DETECTED = 0x10,
  H_RESERVED_BIT_FIVE_ERROR = 0x20,
  H_EEP_REG_DISTORTED = 0x40,
  H_RESERVED_BIT_SEVEN_ERROR = 0x80,
} STATUS_ERROR_TYPE;

typedef enum {
  H_NO_DETAILS = 0x00,
  H_MOVING_FLAG = 0x01,
  H_INPOSITION_FLAG = 0x02,
  H_CHECKSUM_ERROR = 0x04,
  H_UNKNOWN_COMMAND = 0x08,
  H_EXCEED_REG_RANGE = 0x10,
  H_GARBAGE_DETECTED = 0x20,
  H_TORQUE_ON = 0x40,
  H_RESERVED_BIT_SEVEN_DETAIL = 0x80,
} STATUS_DETAIL;


class HerkulexClass {
public:
  HerkulexClass();
  HerkulexClass(uint8_t serialPort);

  void beginSerialBus(uint32_t baud);
  void updateSerialBaud(uint32_t baud);
  void endSerialBus();

  void  initialize();
  bool  stat(uint8_t servoID, uint8_t* statError, uint8_t* statDetail);
  void  setACKPolicy(int valueACK);
  bool checkModel(uint8_t servoID, uint16_t* model);
  void setID(uint8_t oldID, uint8_t newID);
  void clearError(int servoID);
  void setBaudRate(BAUD_RATE newBaud);

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

  uint8_t packet[DATA_MOVE + PACKET_LENGTH_BYTES::BASE_LENGTH + PACKET_LENGTH_BYTES::HSJOG_MOVEONE_DATA_LENGTH]; // stores full packet to send

  uint8_t packetQueue[DATA_MOVE];  // stores move packets for simulataneous jog

  uint8_t inputBuffer[DATA_MOVE]; // stores input HOW LARGE DOES THIS NEED TO BE?
  uint8_t checksumData[PACKET_LENGTH_BYTES::BASE_LENGTH];  // stores checksumdata for input validation 
};

#endif
