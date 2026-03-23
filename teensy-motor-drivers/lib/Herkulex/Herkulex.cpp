/*
 Hekulex.cpp - Library for Dongbu Herkulex DRS-0101/DRS-0201 
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

  02/25/2026
  The library has been edited from Arduino Uno/2009 - Arduino Mega to also work on Teensy 4.1
  The Software Serial functionality has been removed due to the lat of necessity and the abundance of UART ports on the Teensy
  Edits by Max Inman and Pau Alcolea Vila (Worcester Polytechnic Institute, 2026)

 *****************************************************************************
 Original Author:
 Contact: alegiaco@gmail.com
 Web:     http://robottini.altervista.org
 Autor:   Alessandro Giacomel
 *****************************************************************************  
*/
#include "Herkulex.h"


// Macro for the Serial port selection
#define HSerial1     1 		// Write in Serial 1 port Teensy 4.1 - Pin 00(rx) - 01(tx) 
#define HSerial2     2   	// Write in Serial 2 port Teensy 4.1 - Pin 07(rx) - 08(tx) 
#define HSerial3     3   	// Write in Serial 3 port Teensy 4.1 - Pin 15(rx) - 14(tx)
#define HSerial4     4 		// Write in Serial 4 port Teensy 4.1 - Pin 16(rx) - 17(tx) 
#define HSerial5     5   	// Write in Serial 5 port Teensy 4.1 - Pin 21(rx) - 20(tx) 
#define HSerial6     6   	// Write in Serial 6 port Teensy 4.1 - Pin 25(rx) - 24(tx)
#define HSerial7     7 		// Write in Serial 7 port Teensy 4.1 - Pin 28(rx) - 29(tx)
#define HSerial8     8 		// Write in Serial 8 port Teensy 4.1 - Pin 34(rx) - 35(tx)

// The constructor now initializes the class as an instance with a specific serial port
// This is instead of the old port, which had to be passed as an argument in beginSerialBus
// this way, the instance is related to the port as opposed to having to pass it, 
// this allows the function on the instance to be unaware of the serial port, which makes them simpler

HerkulexClass::HerkulexClass() : _serialPort(0), queuedPacketCount(0), playTime(0) {}

HerkulexClass::HerkulexClass(uint8_t serialPort){
	
	// store serial bus pointer in class private object
	switch (serialPort)
    {
        #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
			case HSerial1: _serial = &Serial1; break;
			case HSerial2: _serial = &Serial2; break;
			case HSerial3: _serial = &Serial3; break;
        #elif defined(ARDUINO_TEENSY41)
			case HSerial1: _serial = &Serial1; break;
			case HSerial2: _serial = &Serial2; break;
			case HSerial3: _serial = &Serial3; break;
			case HSerial4: _serial = &Serial4; break;
			case HSerial5: _serial = &Serial5; break;
			case HSerial6: _serial = &Serial6; break;
			case HSerial7: _serial = &Serial7; break;
			case HSerial8: _serial = &Serial8; break;
        #endif
        break;
	}

	resetClassVals();
} 

void HerkulexClass::resetClassVals(){
	queuedPacketCount = 0;
	newDataInInputBuffer = false;
	readStartTime = 0;
	packetLength = 0;
	pID = 0;
	CMD = 0;
	checksumOne = 0;
	checksumTwo = 0;
	additionalDataLength = 0;
	playTime = 0;
	goalLSB = 0;
	goalMSB = 0;
	SET = 0;
	ID = 0;
}


// Begin serial bus communications
void HerkulexClass::beginSerialBus(long baud){
	_serial->begin(baud);
}

// End serial bus communications
void HerkulexClass::endSerialBus(){
	_serial->end();
}

// initialize servos
void HerkulexClass::initialize(){
		resetClassVals();
        delay(100);       
        clearError(PACKET_CONSTS::ALL_SERVOS);	// clear error for all servos
        delay(10);
        setACKPolicy(1);						// set ACK
        delay(10);
        torqueON(PACKET_CONSTS::ALL_SERVOS);		// torqueON for all servos
        delay(10);
}

// stat
byte HerkulexClass::stat(int servoID)
{
	packetSize = PACKET_LENGTH_BYTES::HSTAT_LENGTH;
	additionalDataLength = PACKET_LENGTH_BYTES::HSTAT_DATA_LENGTH;

	pID      = servoID;			//4.Servo ID - 0XFE=All servos
	CMD      = COMMAND::HSTAT;			//5.CMD
  
	// base packet
	packet[0] = PACKET_CONSTS::PACKET_HEADER;
	packet[1] = PACKET_CONSTS::PACKET_HEADER;
	packet[2] = packetSize;
	packet[3] = pID;
	packet[4] = CMD;

	// checksum
	// because we added an additionalDataLength of 0, the checksum can be calculated with the functions
	checksumOne = calcChecksumOne();
    checksumTwo = calcChecksumTwo();

	packet[5] = checksumOne;	
	packet[6] = checksumTwo;	
	     
	sendData(packet, packetSize);
	delay(2);
	readBlocking(9); 				// read 9 bytes from serial

	// second part of the function where it reads the data
	packetSize = packet[2];       
	pID   = packet[3];        
	CMD   = packet[4];       
	checksumData[0]=packet[7];
    checksumData[1]=packet[8];
    packetLength=2;

	checksumOne = calcChecksumOne(); // old one: checksumOne = (dataEx[2]^dataEx[3]^dataEx[4]^dataEx[7]^dataEx[8]) & 0xFE; 

	checksumTwo = calcChecksumTwo();			
	
	if (checksumOne != packet[5]) return -1; //checksum verify
	if (checksumTwo != packet[6]) return -2;

	return packet[7];			// return status
}

// torque on - 
void HerkulexClass::torqueON(int servoID)
{
	packetLength = PACKET_LENGTH_BYTES::SET_ACK_POLICY_RAMWRITE_LENGTH;
	additionalDataLength = PACKET_LENGTH_BYTES::SET_ACK_POLICY_RAMWRITE_DATA_LENGTH;

	pID   = servoID;
	CMD   = COMMAND::HRAMWRITE;          
	  	
	// base packet
	packet[0] = PACKET_CONSTS::PACKET_HEADER;
	packet[1] = PACKET_CONSTS::PACKET_HEADER;
	packet[2] = packetLength;
	packet[3] = pID;
	packet[4] = CMD;
	
	// optional data
	packet[7] = 0x34; 		// Address 52
	packet[8] = 0x01; 		// Length
	packet[9] = 0x60; 		// Torque ON
	
	// checksum
	checksumOne=calcChecksumOne();
	checksumTwo=calcChecksumTwo();

	packet[5] = checksumOne;
	packet[6] = checksumTwo;			

	sendData(packet, packetLength);
}

// torque off - the torque is FREE, not Break
void HerkulexClass::torqueOFF(int servoID)
{
	packetSize = PACKET_LENGTH_BYTES::SET_ACK_POLICY_RAMWRITE_LENGTH;
	additionalDataLength = PACKET_LENGTH_BYTES::SET_ACK_POLICY_RAMWRITE_DATA_LENGTH;

	pID   = servoID;
	CMD   = COMMAND::HRAMWRITE;          
	
	checksumData[0]=0x34;               // 8. Address
	checksumData[1]=0x01;               // 9. Lenght
	checksumData[2]=0x00;               // 10. 0x00=Torque Free
  	
	// base packet
	packet[0] = PACKET_CONSTS::PACKET_HEADER;
	packet[1] = PACKET_CONSTS::PACKET_HEADER;
	packet[2] = packetSize;
	packet[3] = pID;
	packet[4] = CMD;
	
	// optional data
	packet[7] = checksumData[0]; 		// Address 52
	packet[8] = checksumData[1]; 		// Length
	packet[9] = checksumData[2]; 		// Torque Free
	
	// checksum
	checksumOne=calcChecksumOne();
	checksumTwo=calcChecksumTwo();		

	packet[5] = checksumOne;			
	packet[6] = checksumTwo;	

    sendData(packet, packetSize);
}

// ACK  - 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
void HerkulexClass::setACKPolicy(int valueACK)
{
	packetLength = PACKET_LENGTH_BYTES::SET_ACK_POLICY_RAMWRITE_LENGTH;
	additionalDataLength = PACKET_LENGTH_BYTES::SET_ACK_POLICY_RAMWRITE_DATA_LENGTH;

	pID   = PACKET_CONSTS::ALL_SERVOS;   
	CMD   = COMMAND::HRAMWRITE;      
	
  	
	// base packet
	packet[0] = PACKET_CONSTS::PACKET_HEADER;
	packet[1] = PACKET_CONSTS::PACKET_HEADER;
	packet[2] = packetLength;
	packet[3] = pID;
	packet[4] = CMD;
	
	//optional data
	packet[7] = 0x34; 		// Address 52
	packet[8] = 0x01; 		// Length
	packet[9] = valueACK; 	// Value 0=No reply, 1= Only reply to READ CMD, 2 = Always reply
	
	// checksum
	checksumOne=calcChecksumOne();	
	checksumTwo=calcChecksumTwo();					

	packet[5] = checksumOne;		
	packet[6] = checksumTwo;	

 	sendData(packet, packetLength);
}

// model - 1=0101 - 2=0201
byte HerkulexClass::checkModel()
{
	packetSize = PACKET_LENGTH_BYTES::HEEPREAD_LENGTH;
	additionalDataLength = PACKET_LENGTH_BYTES::HEEPREAD_DATA_LENGTH;

	pID   = 0xFE;	           
	CMD   = COMMAND::HEEPREAD;

	checksumData[0]=0x00;               // 8. Address
	checksumData[1]=0x01;               // 9. Lenght
  	
	// base packet
	packet[0] = PACKET_CONSTS::PACKET_HEADER;
	packet[1] = PACKET_CONSTS::PACKET_HEADER;
	packet[2] = packetSize;
	packet[3] = pID;
	packet[4] = CMD;
	
	// optional data
	packet[7] = checksumData[0]; 		// Address
	packet[8] = checksumData[1]; 		// Length

	// checksum
	checksumOne = calcChecksumOne();	
	checksumTwo = calcChecksumTwo();					

	packet[5] = checksumOne;		
	packet[6] = checksumTwo;

    sendData(packet, packetSize);

	delay(1);
	readBlocking(9);
	
	// this is the second part of te method that uses the read data
	packetSize = packet[2];           
	pID   = packet[3];           
	CMD   = packet[4];
	checksumData[0] = packet[7];         
	packetLength = 1;      
  	
	// TODO: I am unsure if these are correct
	checksumOne = calcChecksumOne();	
	checksumTwo = calcChecksumTwo();			

	if (checksumOne != packet[5]) return -1; //checksum verify
	if (checksumTwo != packet[6]) return -2;
		
	return packet[7];			// return status

}

// setID - Need to restart the servo
void HerkulexClass::setID(int ID_Old, int ID_New)
{
	packetSize = PACKET_LENGTH_BYTES::HEEPWRITE_LENGTH_1;
	// old one was : packetSize = 0x0A;  this does seem correct, so i created the HEEPWRITE_LENGTH_1, for writeRegistryEEP it's different
	additionalDataLength = PACKET_LENGTH_BYTES::HEEPWRITE_DATA_LENGTH_1; 
	//strange because the packetLength before was 3, but in the other ones it had been one less than the number of optional data

	pID   = ID_Old;
	CMD   = COMMAND::HEEPWRITE; 
	
	checksumData[0]= 0x06;               // 8. Address
	checksumData[1]= 0x01;               // 9. Length
	checksumData[2]= ID_New;             // 10. ServoID NEW
  	
	// base packet
	packet[0] = PACKET_CONSTS::PACKET_HEADER;
	packet[1] = PACKET_CONSTS::PACKET_HEADER;
	packet[2] = packetSize;
	packet[3] = pID;
	packet[4] = CMD;
	
	// optional data
	packet[7] = checksumData[0]; 		// Address 52
	packet[8] = checksumData[1]; 		// Length
	packet[9] = checksumData[2]; 		// Value
	
	// checksum
	checksumOne=calcChecksumOne();	
	checksumTwo=calcChecksumTwo();	

	packet[5] = checksumOne;
	packet[6] = checksumTwo;

	sendData(packet, packetSize);

}

// clearError
void HerkulexClass::clearError(int servoID)
{	
	packetLength = PACKET_LENGTH_BYTES::HRAMWRITE_LENGTH;
	additionalDataLength = PACKET_LENGTH_BYTES::HRAMWRITE_DATA_LENGTH;

	pID   = servoID;     		
	CMD   = COMMAND::HRAMWRITE;      
	
	// base packet
	packet[0] = PACKET_CONSTS::PACKET_HEADER;
	packet[1] = PACKET_CONSTS::PACKET_HEADER;
	packet[2] = packetLength;
	packet[3] = pID;
	packet[4] = CMD;
	
	// optional data
	packet[7] = 0x30;		// Address 48
	packet[8] = 0x02;		// Length
	packet[9] = 0x00; 		// Value1
	packet[10]= 0x00; 		// Value2
	
	// checksum
	checksumOne=calcChecksumOne();
	checksumTwo=calcChecksumTwo();

	packet[5] = checksumOne;			
	packet[6] = checksumTwo;	

	sendData(packet, packetLength);
}

void HerkulexClass::queueMove(motorMoveInfo moveInfo)
{	  
	packetQueue[queuedPacketCount++] = (uint8_t) (moveInfo.goalPos & 0xFF);        // add 8 lower bits of 16 bit goal
	packetQueue[queuedPacketCount++] = (uint8_t) (moveInfo.goalPos >> 8 & 0xFF);   // add 8 higher bits of 16 bit goal
	packetQueue[queuedPacketCount++] = moveInfo.ledColor;                          // add LED value
	packetQueue[queuedPacketCount++] = moveInfo.servoID;                           // add id of servo
}

// move all servos with the same execution time
// DO NOT USE IN MAIN: USE HerkulexMotor::actionMoves(int playTimeMs) instead
void HerkulexClass::actionMoves(uint8_t playTime)
{

	additionalDataLength = PACKET_LENGTH_BYTES::HSJOG_MOVEMULTIPLE_DATA_LENGTH;

	// length is the intro packet length (8) + the queued packet length
	packetLength = PACKET_LENGTH_BYTES::HSJOG_MOVEMULTIPLE_LENGTH + queuedPacketCount;
	pID = PACKET_CONSTS::ALL_SERVOS;
    CMD = COMMAND::HSJOG;

	// add HSJOG intro packet to the dataEx output buffer
	// base packet
	packet[0] = PACKET_CONSTS::PACKET_HEADER;
	packet[1] = PACKET_CONSTS::PACKET_HEADER;
	packet[2] = packetLength;
	packet[3] = pID;
	packet[4] = CMD;

	// optional data
	packet[7] = playTime;			// Execution time	

	// checksum
	checksumOne = calcChecksumOne();
	checksumTwo = calcChecksumTwo();			
	
	packet[5] = checksumOne;	
	packet[6] = checksumTwo;

	// copy packetQueue into packet after the HSJOG intro packet
	memcpy(&packet[8], packetQueue, queuedPacketCount);
	
	// send dataEx out onto the bus
	sendData(packet, packetLength);

	queuedPacketCount = 0; // reset counter 
}

// Builds and sends the RAMREAD position-request packet then returns immediately.
// The motor starts composing its reply; the hardware RX buffer fills on its own.
// Call this on every bus BEFORE calling collectPosition on any bus so that all
// motors can reply in parallel while the CPU is busy sending to the next bus.
void HerkulexClass::sendPosRequest(int servoID) {


	packetLength = PACKET_LENGTH_BYTES::HRAMREAD_LENGTH;
	additionalDataLength = PACKET_LENGTH_BYTES::HRAMREAD_DATA_LENGTH;

    pID          = servoID;						// Servo ID - 253=all servos
    CMD          = COMMAND::HRAMREAD;
	
	// base packet
	packet[0] = PACKET_CONSTS::PACKET_HEADER;
	packet[1] = PACKET_CONSTS::PACKET_HEADER;
	packet[2] = packetSize;
	packet[3] = pID;
	packet[4] = CMD;
	
	// optional data
	packet[7] = REGISTER::CALIBRATED_POS; // register address
	packet[8] = additionalDataLength;  	  // data length
	
	// checksum
	checksumOne = calcChecksumOne();
	checksumTwo = calcChecksumTwo();				
	
	packet[5] = checksumOne;					
	packet[6] = checksumTwo;					

    sendData(packet, packetLength);
	requestRead(PACKET_LENGTH_BYTES::GETPOS_RESPONSE);
}

boolean HerkulexClass::isReplyReady(){
	return newDataInInputBuffer;
}

// collectPosition -- Phase 2 of the parallelized two-phase position read.
// Reads and parses the 13-byte reply the motor sent after requestPosition.
// Because all motors were requested before any read is attempted, most or all
// of the reply bytes are already in the RX buffer by the time this is called,
// so the blocking wait inside readData is near-zero for all but the first bus.
//
// Returns the raw 16-bit position value, or 0xFFFF on a checksum error.
// Apply the model's posBitMask and stepsToDeg in HerkulexMotor as usual.
uint16_t HerkulexClass::collectPosition(int servoID) {
	newDataInInputBuffer = false;

	// Re-arm class fields so checksums knows what packet we expect.
	// These must match what requestPosition() set.
	packetLength = 6; // 6 magic number remove
	packetSize   = inputBuffer[2];
	pID          = inputBuffer[3];
	CMD          = inputBuffer[4];

	// Parse data from input buffer into packet for checksum
	memcpy(&packet[7], &inputBuffer[7], 6);

	checksumOne = calcChecksumOne();
	checksumTwo = calcChecksumTwo();

	if (checksumOne != inputBuffer[5]) return 0xFFFF;    // checksum error flag
	if (checksumTwo != inputBuffer[6]) return 0xFFFF;

	

	return (uint16_t)((packet[10] << 8) | packet[9]);
}

// getPosition -- original blocking API, preserved for single-motor or debug use.
// Internally calls requestPosition then collectPosition back-to-back.
// When querying multiple motors across different buses, prefer using
// SerialBusManager::requestAllPositions() + collectAllPositions() instead.
uint16_t HerkulexClass::getPosition(int servoID) {
    sendPosRequest(servoID);
    delayMicroseconds(100); // minimum turnaround time for motor to begin replying
    return collectPosition(servoID);
}

// reboot single servo - pay attention 253 - all servos doesn't work!
void HerkulexClass::reboot(int servoID) {
    packetSize = PACKET_LENGTH_BYTES::HREBOOT_LENGTH;
	additionalDataLength = PACKET_LENGTH_BYTES::HREBOOT_DATA_LENGTH;

	pID   = servoID;
	CMD   = COMMAND::HREBOOT;

	// base packet
	packet[0] = PACKET_CONSTS::PACKET_HEADER;
	packet[1] = PACKET_CONSTS::PACKET_HEADER;
	packet[2] = packetSize;
	packet[3] = pID;
	packet[4] = CMD;

	// checksum
	// before, this was done inline like so checksumOne = (packetSize ^ pID ^ CMD) &0xFE;, which is equivalent, but like this it's similar to the others

	checksumOne = calcChecksumOne();
    checksumTwo = calcChecksumTwo();
	
	packet[5] = checksumOne;	
	packet[6] = checksumTwo;
	
	sendData(packet, packetSize);

}

// LED  - see table of colors 
void HerkulexClass::setLed(int servoID, int valueLed)
{
	packetSize = PACKET_LENGTH_BYTES::SET_ACK_POLICY_RAMWRITE_LENGTH;
	additionalDataLength = PACKET_LENGTH_BYTES::SET_ACK_POLICY_RAMWRITE_DATA_LENGTH;

	pID     = servoID;            
	CMD     = COMMAND::HRAMWRITE;          

	checksumData[0] = 0x35;               // 8. Address 53
    checksumData[1] = 0x01;               // 9. Lenght
	checksumData[2] = valueLed;           // 10.LedValue
  	  	
	// base packet
	packet[0] = PACKET_CONSTS::PACKET_HEADER;
	packet[1] = PACKET_CONSTS::PACKET_HEADER;
	packet[2] = packetSize;
	packet[3] = pID;
	packet[4] = CMD;

	// optional data
	packet[7] = checksumData[0];        // Address
	packet[8] = checksumData[1];       	// Length
	packet[9] = checksumData[2];        // Value

	// checksum
	checksumOne=calcChecksumOne();	
	checksumTwo=calcChecksumTwo();	

	packet[5] = checksumOne;			// Checksum 1
	packet[6] = checksumTwo;			// Checksum 2
	
	sendData(packet, packetLength);
}

// get the speed for one servo - values betweeb -1023 <--> 1023
int HerkulexClass::getSpeed(int servoID) {
  	int speedy  = 0;

	packetSize = PACKET_LENGTH_BYTES::HRAMREAD_LENGTH;
	additionalDataLength = PACKET_LENGTH_BYTES::HRAMREAD_DATA_LENGTH;

	pID   = servoID;     	   	  
	CMD   = COMMAND::HRAMREAD;      

	checksumData[0]=0x40;               // 8. Address
	checksumData[1]=0x02;               // 9. Lenght

  
	// base packet
	packet[0] = PACKET_CONSTS::PACKET_HEADER;
	packet[1] = PACKET_CONSTS::PACKET_HEADER;
	packet[2] = packetSize;
	packet[3] = pID;
	packet[4] = CMD;
  
	//optional data
	packet[7] = checksumData[0]; 	    // Address  
	packet[8] = checksumData[1]; 		// Length
  
	// checksum
	checksumOne=calcChecksumOne();		
	checksumTwo=calcChecksumTwo();		
  
	packet[5] = checksumOne;
	packet[6] = checksumTwo;	
	
	sendData(packet, packetSize);

	delay(1);
	readBlocking(13);


	// This is the second half of the function that uses the read data
	packetSize =      inputBuffer[2];    
	pID   =           inputBuffer[3]; 
	CMD   =           inputBuffer[4];

	packetLength = 6;
	memcpy(&packet[7], &inputBuffer[7], packetLength);

	checksumOne=calcChecksumOne();	
	checksumTwo=calcChecksumTwo();	

	if (checksumOne != inputBuffer[5]) return -1;
	if (checksumTwo != inputBuffer[6]) return -1;

	speedy = ((packet[10]&0xFF)<<8) | packet[9];
	return speedy;
}

// moves one motor with the set moveInfo of goal, ID, LED color, and playTime
void HerkulexClass::moveOne(motorMoveInfo moveInfo)
{
	// set all the pre-initialized variables needed for packet building


	// length of additional data (dataEx 7 -> 11)
	additionalDataLength = PACKET_LENGTH_BYTES::HSJOG_MOVEONE_DATA_LENGTH;

	// these are needed for the base packet
	packetLength = PACKET_LENGTH_BYTES::HSJOG_MOVEONE_LENGTH;
	pID = moveInfo.servoID;
	CMD = COMMAND::HSJOG;
	

	// these are needed for the "optional data"
	// that, for this command, tells the motor its goal and playtime
	playTime = moveInfo.playTime;
	goalLSB = (uint8_t) (moveInfo.goalPos & 0X00FF);
	goalMSB = (uint8_t) ((moveInfo.goalPos & 0XFF00) >> 8);
	SET = moveInfo.ledColor << 2;  // TODO: verify against S_JOG_TAG on pg 41 of 0601 datasheet
	ID = moveInfo.servoID;


	// base packet
	packet[0] = PACKET_CONSTS::PACKET_HEADER;
	packet[1] = PACKET_CONSTS::PACKET_HEADER;
	packet[2] = packetLength;
	packet[3] = pID;
	packet[4] = CMD;

	// optional data
	packet[7] = playTime;
	packet[8] = goalLSB; 
	packet[9] = goalMSB; 
	packet[10] = SET;
	packet[11] = ID;

	// checksum has to be calculated last, but belongs in base packet
	checksumOne = calcChecksumOne();
	checksumTwo = calcChecksumTwo();

	packet[5] = checksumOne;
	packet[6] = checksumTwo;

	sendData(packet, packetLength);
}

// write registry in the RAM: one byte 
void HerkulexClass::writeRegistryRAM(int servoID, int address, int writeByte)
{
	packetSize = PACKET_LENGTH_BYTES::HRAMWRITE_LENGTH;
	// old one was : packetSize = 0x0A;  , but i think that that is wrong based on the datasheet. I changed it to 0x0B page 37
	additionalDataLength = PACKET_LENGTH_BYTES::HRAMWRITE_DATA_LENGTH;

	pID   = servoID;     
	CMD   = COMMAND::HRAMWRITE; 

	checksumData[0]=address;              // 8. Address
	checksumData[1]=0x01;               	// 9. Length
	checksumData[2]=writeByte;            // 10. Write error=0
  
	// base packet
	packet[0] = PACKET_CONSTS::PACKET_HEADER;
	packet[1] = PACKET_CONSTS::PACKET_HEADER;
	packet[2] = packetSize;
	packet[3] = pID;
	packet[4] = CMD;

	// optional data
	packet[7] = checksumData[0]; 		// Address 52
	packet[8] = checksumData[1]; 		// Length
	packet[9] = checksumData[2]; 		// Value1
	packet[10]= checksumData[3]; 		// Value2

	// checksum
	checksumOne=calcChecksumOne();
  	checksumTwo=calcChecksumTwo();

	packet[5] = checksumOne;
	packet[6] = checksumTwo;

  	sendData(packet, packetSize);
}

// write registry in the EEP memory (ROM): one byte 
void HerkulexClass::writeRegistryEEP(int servoID, int address, int writeByte)
{
	packetSize = PACKET_LENGTH_BYTES::HEEPWRITE_LENGTH_2;
	// old one was : packetSize = 0x0A;  , but i think that that is wrong based on the datasheet. I changed it to 0x0B page 36
	additionalDataLength = PACKET_LENGTH_BYTES::HEEPWRITE_DATA_LENGTH_2;
	
	pID   = servoID;     	         
	CMD   = COMMAND::HEEPWRITE;

	checksumData[0]= address;               
	checksumData[1]= 0x01;                  // 9. Length
	checksumData[2]= writeByte;             // 10. Write error=0
	// I have no idea what checksumData[3] would be...

  	// base packet
	packet[0] = PACKET_CONSTS::PACKET_HEADER;
	packet[1] = PACKET_CONSTS::PACKET_HEADER;
	packet[2] = packetSize;
	packet[3] = pID;
	packet[4] = CMD;

	// optional data
	packet[7] = checksumData[0]; 		// Address 52
	packet[8] = checksumData[1]; 		// Length
	packet[9] = checksumData[2]; 		// Value1
	packet[10]= checksumData[3]; 		// Value2

	// checksum
	checksumOne=calcChecksumOne();
  	checksumTwo=calcChecksumTwo();

	packet[5] = checksumOne;
	packet[6] = checksumTwo;

  	sendData(packet, packetSize);
}



// Private Methods //////////////////////////////////////////////////////////////

// calculated checksum1 as defined in datasheets
uint8_t HerkulexClass::calcChecksumOne()
{
	// checksum base formula is XOR packetLength, pID, CMD, and all additional data
  	checksumOne = 0 ^ packetLength ^ pID ^ CMD;
  	for (int i = 0; i < additionalDataLength; i++)
	{
		checksumOne = checksumOne ^ packet[i + 7]; // 7 puts us at the start of the additional data
	}
	return checksumOne & 0xFE;
}

// checksum2 is the bitwise complimente of ChecksumOne, which is why everything is the same except for the return
uint8_t HerkulexClass::calcChecksumTwo()
{
	checksumTwo = 0 ^ packetLength ^ pID ^ CMD;
	for (int i = 0; i < additionalDataLength; i++) 
	{
		checksumTwo = checksumTwo ^ packet[i + 7]; // 7 puts us at the start of the additional data
	}
	return (~checksumTwo) & 0xFE;
}


// write the serial data to the port
void HerkulexClass::sendData(uint8_t* buffer, uint8_t length){
	_serial->write(buffer, length);
}

void HerkulexClass::requestRead(uint8_t length){
	newDataInInputBuffer = false;
	readPending = true;
	inputLength = length;
	readStartTime = micros();
}

// update the read request
void HerkulexClass::updateRead(){
	// do nothing if no read is requested
	if(!readPending) return;

	// store data in input buffer and update flags
	if(_serial->available() >= inputLength){
		_serial->readBytes(inputBuffer, inputLength);
		readPending = false;
		newDataInInputBuffer = true;
	}

	// timeout
	else if (micros() - readStartTime >= SERIAL_READ_TIMEOUT_US){
		readPending = false;
	}
}


void HerkulexClass::readBlocking(uint8_t length){
	readStartTime = micros();
	while(_serial->available() < length){
		delayMicroseconds(50);
		if (micros() - readStartTime >= SERIAL_READ_TIMEOUT_US){
			break;
		}
		if (_serial->available() >= length){
			_serial->readBytes(inputBuffer, length);
		}
	}
}

// LEGACY
// Figure out if we can delete
//clear buffer in the serial port - better - try to do this
void HerkulexClass::clearBuffer()
{
  switch (_serialPort)
	{
	#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)
	case HSerial1:
				Serial1.flush();
				while (Serial1.available()){
				Serial1.read();
				delayMicroseconds(200);
				}

		break;
	case HSerial2:
	            Serial2.flush();
				while (Serial2.available()){
				Serial2.read();
				delayMicroseconds(200);
				}
		break;
	case HSerial3:
	            Serial3.flush();
				while (Serial3.available()){
					Serial3.read();
					delayMicroseconds(200);
				}

		break;
	#elif defined (ARDUINO_TEENSY41)
	case HSerial1:
				Serial1.flush();
				while (Serial1.available()){
				Serial1.read();
				delayMicroseconds(200);
				}

		break;
	case HSerial2:
	            Serial2.flush();
				while (Serial2.available()){
				Serial2.read();
				delayMicroseconds(200);
				}
		break;
	case HSerial3:
	            Serial3.flush();
				while (Serial3.available()){
					Serial3.read();
					delayMicroseconds(200);
				}

		break;
	case HSerial4:
				Serial4.flush();
				while (Serial4.available()){
				Serial4.read();
				delayMicroseconds(200);
				}

		break;
	case HSerial5:
	            Serial5.flush();
				while (Serial5.available()){
				Serial5.read();
				delayMicroseconds(200);
				}
		break;
	case HSerial6:
	            Serial6.flush();
				while (Serial6.available()){
					Serial6.read();
					delayMicroseconds(200);
				}

		break;
	case HSerial7:
				Serial7.flush();
				while (Serial7.available()){
				Serial7.read();
				delayMicroseconds(200);
				}

		break;
	case HSerial8:
	            Serial8.flush();
				while (Serial8.available()){
				Serial8.read();
				delayMicroseconds(200);
				}
		break;
	#endif
	}
}

void HerkulexClass::printHexByte(byte x)
{
  Serial.print("0x");
  if (x < 16) {
    Serial.print('0');
  }
    Serial.print(x, HEX);
    Serial.print(" ");

}
