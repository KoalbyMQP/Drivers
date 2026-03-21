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

HerkulexClass::HerkulexClass() : _serialPort(0), queuedPacketCount(0), XOR(0), playTime(0) {}

HerkulexClass::HerkulexClass(uint8_t busID) : _serialPort(busID), queuedPacketCount(0), XOR(0), playTime(0) {}


// Begin serial bus communications
void HerkulexClass::beginSerialBus(long baud){
	switch (_serialPort)
		{
		#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)
		case HSerial1:
			Serial1.begin(baud);
			break;
		case HSerial2:
			Serial2.begin(baud);
			break;
		case HSerial3:
			Serial3.begin(baud);
			break;
		#elif defined (ARDUINO_TEENSY41)
		case HSerial1:
			Serial1.begin(baud);
			Serial.print("starting serial port: ");
			Serial.println(_serialPort);
			break;
		case HSerial2:
			Serial2.begin(baud);
			Serial.print("starting serial port: ");
			Serial.println(_serialPort);
			break;
		case HSerial3:
			Serial3.begin(baud);
			Serial.print("starting serial port: ");
			Serial.println(_serialPort);
			break;
		case HSerial4:
			Serial4.begin(baud);
			Serial.print("starting serial port: ");
			Serial.println(_serialPort);
			break;
		case HSerial5:
			Serial5.begin(baud);
			break;
		case HSerial6:
			Serial6.begin(baud);
			break;
		case HSerial7:
			Serial7.begin(baud);
			break;
		case HSerial8:
			Serial8.begin(baud);
			break;
		#endif
		}
}

// End serial bus communications
void HerkulexClass::end()
{
	switch (_serialPort)
	{
    #if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)
	case HSerial1:
		Serial1.end();
		break;
	case HSerial2:
		Serial2.end();
		break;
	case HSerial3:
		Serial3.end();
		break;
	#elif defined (ARDUINO_TEENSY41)
	case HSerial1:
		Serial1.end();
		break;
	case HSerial2:
		Serial2.end();
		break;
	case HSerial3:
		Serial3.end();
		break;
	case HSerial4:
		Serial4.end();
		break;
	case HSerial5:
		Serial5.end();
		break;
	case HSerial6:
		Serial6.end();
		break;
	case HSerial7:
		Serial7.end();
		break;
	case HSerial8:
		Serial8.end();
		break;
	#endif
	}
}

// initialize servos
void HerkulexClass::initialize()
{
        queuedPacketCount=0;
		packetLength=0;
        delay(100);       
        clearError(BROADCAST_ID);	// clear error for all servos
        delay(10);
        setACKPolicy(1);						// set ACK
        delay(10);
        torqueON(BROADCAST_ID);		// torqueON for all servos
        delay(10);
		
}

// stat
byte HerkulexClass::stat(int servoID)
{
	packetSize = PACKET_SIZE::HSTAT_LENGTH;
	additionalDataLength = PACKET_SIZE::HSTAT_DATA_LENGTH;

	pID      = servoID;			//4.Servo ID - 0XFE=All servos
	CMD      = COMMAND::HSTAT;			//5.CMD
  
	// base packet
	dataEx[0] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[1] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[2] = packetSize;
	dataEx[3] = pID;
	dataEx[4] = CMD;

	// checksum
	// because we added an additionalDataLength of 0, the checksum can be calculated with the functions
	checksumOne = calcChecksumOne();
    checksumTwo = calcChecksumTwo();

	dataEx[5] = checksumOne;	
	dataEx[6] = checksumTwo;	
	     
	sendData(dataEx, packetSize);
	delay(2);
	readData(9); 				// read 9 bytes from serial

	// second part of the function where it reads the data
	packetSize = dataEx[2];       
	pID   = dataEx[3];        
	CMD   = dataEx[4];       
	checksumData[0]=dataEx[7];
    checksumData[1]=dataEx[8];
    packetLength=2;

	checksumOne = calcChecksumOne(); // old one: checksumOne = (dataEx[2]^dataEx[3]^dataEx[4]^dataEx[7]^dataEx[8]) & 0xFE; 

	checksumTwo = calcChecksumTwo();			
	
	if (checksumOne != dataEx[5]) return -1; //checksum verify
	if (checksumTwo != dataEx[6]) return -2;

	return dataEx[7];			// return status
}

// torque on - 
void HerkulexClass::torqueON(int servoID)
{
	packetSize = PACKET_SIZE::HRAMWRITE_LENGTH_2;
	additionalDataLength = PACKET_SIZE::HRAMWRITE_DATA_LENGTH_2;

	pID   = servoID;
	CMD   = COMMAND::HRAMWRITE;          
	
	uint8_t packetData[3] = {0x34, // memory address in servo ram
							0x01,  // # of bytes of info in data packet
							0x60}; // torque on command
	packetData[0]=0x34;               // 8. Address
	packetData[1]=0x01;               // 9. Lenght
	packetData[2]=0x60;               // 10. 0x60=Torque ON
  	
	// base packet
	dataEx[0] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[1] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[2] = packetSize;
	dataEx[3] = pID;
	dataEx[4] = CMD;
	
	// optional data
	dataEx[7] = packetData[0]; 		// Address 52
	dataEx[8] = packetData[1]; 		// Length
	dataEx[9] = packetData[2]; 		// Torque ON
	
	// checksum
	checksumOne=calcChecksumOne();
	checksumTwo=calcChecksumTwo();

	dataEx[5] = checksumOne;
	dataEx[6] = checksumTwo;			

	sendData(dataEx, packetSize);
}

// torque off - the torque is FREE, not Break
void HerkulexClass::torqueOFF(int servoID)
{
	packetSize = PACKET_SIZE::HRAMWRITE_LENGTH_2;
	additionalDataLength = PACKET_SIZE::HRAMWRITE_DATA_LENGTH_2;

	pID   = servoID;
	CMD   = COMMAND::HRAMWRITE;          
	
	checksumData[0]=0x34;               // 8. Address
	checksumData[1]=0x01;               // 9. Lenght
	checksumData[2]=0x00;               // 10. 0x00=Torque Free
  	
	// base packet
	dataEx[0] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[1] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[2] = packetSize;
	dataEx[3] = pID;
	dataEx[4] = CMD;
	
	// optional data
	dataEx[7] = checksumData[0]; 		// Address 52
	dataEx[8] = checksumData[1]; 		// Length
	dataEx[9] = checksumData[2]; 		// Torque Free
	
	// checksum
	checksumOne=calcChecksumOne();
	checksumTwo=calcChecksumTwo();		

	dataEx[5] = checksumOne;			
	dataEx[6] = checksumTwo;	

    sendData(dataEx, packetSize);
}

// ACK  - 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
void HerkulexClass::setACKPolicy(int valueACK)
{
	packetSize = PACKET_SIZE::HRAMWRITE_LENGTH_2;
	additionalDataLength = PACKET_SIZE::HRAMWRITE_DATA_LENGTH_2;

	pID   = 0xFE;	   
	CMD   = COMMAND::HRAMWRITE;      
	
	checksumData[0]= 0x34;               // 8. Address
	checksumData[1]= 0x01;               // 9. Lenght
	checksumData[2]= valueACK;           // 10.Value. 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
  	
	// base packet
	dataEx[0] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[1] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[2] = packetSize;
	dataEx[3] = pID;
	dataEx[4] = CMD;
	
	//optional data
	dataEx[7] = checksumData[0]; 		// Address 52
	dataEx[8] = checksumData[1]; 		// Length
	dataEx[9] = checksumData[2]; 		// Value
	
	// checksum
	checksumOne=calcChecksumOne();	
	checksumTwo=calcChecksumTwo();					

	dataEx[5] = checksumOne;		
	dataEx[6] = checksumTwo;	

 	sendData(dataEx, packetSize);
}

// model - 1=0101 - 2=0201
byte HerkulexClass::checkModel()
{
	packetSize = PACKET_SIZE::HEEPREAD_LENGTH;
	additionalDataLength = PACKET_SIZE::HEEPREAD_DATA_LENGTH;

	pID   = 0xFE;	           
	CMD   = COMMAND::HEEPREAD;

	checksumData[0]=0x00;               // 8. Address
	checksumData[1]=0x01;               // 9. Lenght
  	
	// base packet
	dataEx[0] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[1] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[2] = packetSize;
	dataEx[3] = pID;
	dataEx[4] = CMD;
	
	// optional data
	dataEx[7] = checksumData[0]; 		// Address
	dataEx[8] = checksumData[1]; 		// Length

	// checksum
	checksumOne = calcChecksumOne();	
	checksumTwo = calcChecksumTwo();					

	dataEx[5] = checksumOne;		
	dataEx[6] = checksumTwo;

    sendData(dataEx, packetSize);

	delay(1);
	readData(9);
	
	// this is the second part of te method that uses the read data
	packetSize = dataEx[2];           
	pID   = dataEx[3];           
	CMD   = dataEx[4];
	checksumData[0] = dataEx[7];         
	packetLength = 1;      
  	
	// TODO: I am unsure if these are correct
	checksumOne = calcChecksumOne();	
	checksumTwo = calcChecksumTwo();			

	if (checksumOne != dataEx[5]) return -1; //checksum verify
	if (checksumTwo != dataEx[6]) return -2;
		
	return dataEx[7];			// return status

}

// setID - Need to restart the servo
void HerkulexClass::setID(int ID_Old, int ID_New)
{
	packetSize = PACKET_SIZE::HEEPWRITE_LENGTH_1;
	// old one was : packetSize = 0x0A;  this does seem correct, so i created the HEEPWRITE_LENGTH_1, for writeRegistryEEP it's different
	additionalDataLength = PACKET_SIZE::HEEPWRITE_DATA_LENGTH_1; 
	//strange because the packetLength before was 3, but in the other ones it had been one less than the number of optional data

	pID   = ID_Old;
	CMD   = COMMAND::HEEPWRITE; 
	
	checksumData[0]= 0x06;               // 8. Address
	checksumData[1]= 0x01;               // 9. Length
	checksumData[2]= ID_New;             // 10. ServoID NEW
  	
	// base packet
	dataEx[0] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[1] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[2] = packetSize;
	dataEx[3] = pID;
	dataEx[4] = CMD;
	
	// optional data
	dataEx[7] = checksumData[0]; 		// Address 52
	dataEx[8] = checksumData[1]; 		// Length
	dataEx[9] = checksumData[2]; 		// Value
	
	// checksum
	checksumOne=calcChecksumOne();	
	checksumTwo=calcChecksumTwo();	

	dataEx[5] = checksumOne;
	dataEx[6] = checksumTwo;

	sendData(dataEx, packetSize);

}

// clearError
void HerkulexClass::clearError(int servoID)
{	
	packetSize = PACKET_SIZE::HRAMWRITE_LENGTH;
	additionalDataLength = PACKET_SIZE::HRAMWRITE_DATA_LENGTH;

	pID   = servoID;     		
	CMD   = COMMAND::HRAMWRITE;      

	checksumData[0]=0x30;               // 8. Address
	checksumData[1]=0x02;               // 9. Lenght
	checksumData[2]=0x00;               // 10. Write error=0
	checksumData[3]=0x00;               // 10. Write detail error=0
	
	// base packet
	dataEx[0] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[1] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[2] = packetSize;
	dataEx[3] = pID;
	dataEx[4] = CMD;
	
	// optional data
	dataEx[7] = checksumData[0]; 		// Address 52
	dataEx[8] = checksumData[1]; 		// Length
	dataEx[9] = checksumData[2]; 		// Value1
	dataEx[10]= checksumData[3]; 		// Value2
	
	// checksum
	checksumOne=calcChecksumOne();
	checksumTwo=calcChecksumTwo();

	dataEx[5] = checksumOne;			
	dataEx[6] = checksumTwo;	

	sendData(dataEx, packetSize);
}

void HerkulexClass::queueMove(motorMoveInfo moveInfo)
{	  
	outputBuffer[queuedPacketCount++] = (uint8_t) (moveInfo.goalPos & 0xFF);        // add 8 lower bits of 16 bit goal
	outputBuffer[queuedPacketCount++] = (uint8_t) (moveInfo.goalPos >> 8 & 0xFF);   // add 8 higher bits of 16 bit goal
	outputBuffer[queuedPacketCount++] = moveInfo.ledColor;                          // add LED value
	outputBuffer[queuedPacketCount++] = moveInfo.servoID;                           // add id of servo
}

// move all servos with the same execution time
// DO NOT USE IN MAIN: USE HerkulexMotor::actionMoves(int playTimeMs) instead
void HerkulexClass::actionMoves(uint8_t playTime)
{
	packetSize = PACKET_SIZE::HSJOG_LENGTH_2 + queuedPacketCount; 	// packetsize is the intro packet length (8) + the queued packet length
	additionalDataLength = PACKET_SIZE::HSJOG_DATA_LENGTH_2;
	
	pID          = 0xFE;				// Servo ID: all servos
    CMD          = COMMAND::HSJOG;		// Command Ram Write CMD SJOG Write n servo with same execution time

	// add HSJOG intro packet to the dataEx output buffer
	// base packet
	dataEx[0] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[1] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[2] = packetSize;
	dataEx[3] = pID;
	dataEx[4] = CMD;

	// optional data
	dataEx[7] = playTime;			// Execution time	

	// checksum
	checksumOne = calcChecksumOne();
	checksumTwo = calcChecksumTwo();			
	
	dataEx[5] = checksumOne;	
	dataEx[6] = checksumTwo;

	// copy outputBuffer into dataEx after the HSJOG intro packet
	memcpy(&dataEx[8], outputBuffer, queuedPacketCount);
	
	// send dataEx out onto the bus
	sendData(dataEx, packetSize);

	queuedPacketCount = 0; // reset counter 

}

// Builds and sends the RAMREAD position-request packet then returns immediately.
// The motor starts composing its reply; the hardware RX buffer fills on its own.
// Call this on every bus BEFORE calling collectPosition on any bus so that all
// motors can reply in parallel while the CPU is busy sending to the next bus.
void HerkulexClass::requestPosition(int servoID) {
	packetSize = PACKET_SIZE::HRAMREAD_LENGTH;
	additionalDataLength = PACKET_SIZE::HRAMREAD_DATA_LENGTH;

    pID          = servoID;						// Servo ID - 253=all servos
    CMD          = COMMAND::HRAMREAD;
	
	// base packet
	dataEx[0] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[1] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[2] = packetSize;
	dataEx[3] = pID;
	dataEx[4] = CMD;
	
	// optional data
	dataEx[7] = 0x3A;				// 8. Address
	dataEx[8] = 0x02;				// 9. Length
	
	// checksum
	checksumOne = calcChecksumOne();
	checksumTwo = calcChecksumTwo();				
	
	dataEx[5] = checksumOne;					
	dataEx[6] = checksumTwo;					

    sendData(dataEx, packetSize);
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

	// THIS ONE WON'T FOLLOW THE NEW FORMATTING BECAUSE IT IS READING THINGS FROM readData()

    // Re-arm class fields so checksum1() knows what packet we expect.
    // These must match what requestPosition() set.
    packetLength = 6;

    readData(GETPOS_RESPONSE_BYTES);    // blocks only as long as bytes are missing, right now it is 13 in Herkulex.h

    // Parse response into data[]
    packetSize   = dataEx[2];
    pID          = dataEx[3];
    CMD          = dataEx[4];
    checksumData[0]      = dataEx[7];
    checksumData[1]      = dataEx[8];
    checksumData[2]      = dataEx[9];
    checksumData[3]      = dataEx[10];
    checksumData[4]      = dataEx[11];
    checksumData[5]      = dataEx[12];

    checksumOne = calcChecksumOne();
    checksumTwo = calcChecksumTwo();

    if (checksumOne != dataEx[5]) return 0xFFFF;    // checksum error sentinel
    if (checksumTwo != dataEx[6]) return 0xFFFF;

    return (uint16_t)((dataEx[10] << 8) | dataEx[9]);
}

// getPosition -- original blocking API, preserved for single-motor or debug use.
// Internally calls requestPosition then collectPosition back-to-back.
// When querying multiple motors across different buses, prefer using
// SerialBusManager::requestAllPositions() + collectAllPositions() instead.
uint16_t HerkulexClass::getPosition(int servoID) {
    requestPosition(servoID);
    delayMicroseconds(100); // minimum turnaround time for motor to begin replying
    return collectPosition(servoID);
}

// reboot single servo - pay attention 253 - all servos doesn't work!
void HerkulexClass::reboot(int servoID) {
    packetSize = PACKET_SIZE::HREBOOT_LENGTH;
	additionalDataLength = PACKET_SIZE::HREBOOT_DATA_LENGTH;

	pID   = servoID;
	CMD   = COMMAND::HREBOOT;

	// base packet
	dataEx[0] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[1] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[2] = packetSize;
	dataEx[3] = pID;
	dataEx[4] = CMD;

	// checksum
	// before, this was done inline like so checksumOne = (packetSize ^ pID ^ CMD) &0xFE;, which is equivalent, but like this it's similar to the others

	checksumOne = calcChecksumOne();
    checksumTwo = calcChecksumTwo();
	
	dataEx[5] = checksumOne;	
	dataEx[6] = checksumTwo;
	
	sendData(dataEx, packetSize);

}

// LED  - see table of colors 
void HerkulexClass::setLed(int servoID, int valueLed)
{
	packetSize = PACKET_SIZE::HRAMWRITE_LENGTH_2;
	additionalDataLength = PACKET_SIZE::HRAMWRITE_DATA_LENGTH_2;

	pID     = servoID;            
	CMD     = COMMAND::HRAMWRITE;          

	checksumData[0] = 0x35;               // 8. Address 53
    checksumData[1] = 0x01;               // 9. Lenght
	checksumData[2] = valueLed;           // 10.LedValue
  	  	
	// base packet
	dataEx[0] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[1] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[2] = packetSize;
	dataEx[3] = pID;
	dataEx[4] = CMD;

	// optional data
	dataEx[7] = checksumData[0];        // Address
	dataEx[8] = checksumData[1];       	// Length
	dataEx[9] = checksumData[2];        // Value

	// checksum
	checksumOne=calcChecksumOne();	
	checksumTwo=calcChecksumTwo();	

	dataEx[5] = checksumOne;			// Checksum 1
	dataEx[6] = checksumTwo;			// Checksum 2
	
	sendData(dataEx, packetLength);
}

// get the speed for one servo - values betweeb -1023 <--> 1023
int HerkulexClass::getSpeed(int servoID) {
  	int speedy  = 0;

	packetSize = PACKET_SIZE::HRAMREAD_LENGTH;
	additionalDataLength = PACKET_SIZE::HRAMREAD_DATA_LENGTH;

	pID   = servoID;     	   	  
	CMD   = COMMAND::HRAMREAD;      

	checksumData[0]=0x40;               // 8. Address
	checksumData[1]=0x02;               // 9. Lenght

  
	// base packet
	dataEx[0] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[1] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[2] = packetSize;
	dataEx[3] = pID;
	dataEx[4] = CMD;
  
	//optional data
	dataEx[7] = checksumData[0]; 	    // Address  
	dataEx[8] = checksumData[1]; 		// Length
  
	// checksum
	checksumOne=calcChecksumOne();		
	checksumTwo=calcChecksumTwo();		
  
	dataEx[5] = checksumOne;
	dataEx[6] = checksumTwo;	
	
	sendData(dataEx, packetSize);

	delay(1);
	readData(13);


	// This is the second half of the function that uses the read data
	packetSize = dataEx[2];    
	pID   = dataEx[3]; 
	CMD   = dataEx[4];         
	checksumData[0]=dataEx[7];
	checksumData[1]=dataEx[8];
	checksumData[2]=dataEx[9];
	checksumData[3]=dataEx[10];
	checksumData[4]=dataEx[11];
	checksumData[5]=dataEx[12];
	packetLength=6;

	checksumOne=calcChecksumOne();	
	checksumTwo=calcChecksumTwo();	

	if (checksumOne != dataEx[5]) return -1;
	if (checksumTwo != dataEx[6]) return -1;

	speedy = ((dataEx[10]&0xFF)<<8) | dataEx[9];
	return speedy;
}

// moves one motor with the set moveInfo of goal, ID, LED color, and playTime
void HerkulexClass::moveOne(motorMoveInfo moveInfo)
{
	// set all the pre-initialized variables needed for packet building

	// NOTE: also, we do not need both dataEx and outputBuffer. dataEx is used as an output buffer
	// when only one motor is commanded, and outputBuffer is used when commanding multiple motors. There's no
	// need for both. But we should change last


	// length of additional data (dataEx 7 -> 11)
	additionalDataLength = PACKET_SIZE::HSJOG_DATA_LENGTH;

	// these are needed for the base packet
	packetSize = PACKET_SIZE::HSJOG_LENGTH;
	pID = moveInfo.servoID;
	CMD = COMMAND::HSJOG;
	

	// these are needed for the "optional data"
	// that, for this command, tells the motor its goal and playtime
	playTime = moveInfo.playTime;
	goalLSB = (uint8_t) (moveInfo.goalPos & 0X00FF);
	goalMSB = (uint8_t) ((moveInfo.goalPos & 0XFF00) >> 8);
	SET = 0b00000000 || (moveInfo.ledColor >> 2);  // TODO: verify against S_JOG_TAG on pg 41 of 0601 datasheet
	ID = moveInfo.servoID;


	// base packet
	dataEx[0] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[1] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[2] = packetSize;
	dataEx[3] = pID;
	dataEx[4] = CMD;

	// optional data
	dataEx[7] = playTime;
	dataEx[8] = goalLSB; 
	dataEx[9] = goalLSB; 
	dataEx[10] = SET;
	dataEx[11] = ID;

	// checksum has to be calculated last, but belongs in base packet
	checksumOne = calcChecksumOne();
	checksumTwo = calcChecksumTwo();

	dataEx[5] = checksumOne;
	dataEx[6] = checksumTwo;

	sendData(dataEx, packetSize);
}

// write registry in the RAM: one byte 
void HerkulexClass::writeRegistryRAM(int servoID, int address, int writeByte)
{
	packetSize = PACKET_SIZE::HRAMWRITE_LENGTH;
	// old one was : packetSize = 0x0A;  , but i think that that is wrong based on the datasheet. I changed it to 0x0B page 37
	additionalDataLength = PACKET_SIZE::HRAMWRITE_DATA_LENGTH;

	pID   = servoID;     
	CMD   = COMMAND::HRAMWRITE; 

	checksumData[0]=address;              // 8. Address
	checksumData[1]=0x01;               	// 9. Length
	checksumData[2]=writeByte;            // 10. Write error=0
  
	// base packet
	dataEx[0] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[1] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[2] = packetSize;
	dataEx[3] = pID;
	dataEx[4] = CMD;

	// optional data
	dataEx[7] = checksumData[0]; 		// Address 52
	dataEx[8] = checksumData[1]; 		// Length
	dataEx[9] = checksumData[2]; 		// Value1
	dataEx[10]= checksumData[3]; 		// Value2

	// checksum
	checksumOne=calcChecksumOne();
  	checksumTwo=calcChecksumTwo();

	dataEx[5] = checksumOne;
	dataEx[6] = checksumTwo;

  	sendData(dataEx, packetSize);
}

// write registry in the EEP memory (ROM): one byte 
void HerkulexClass::writeRegistryEEP(int servoID, int address, int writeByte)
{
	packetSize = PACKET_SIZE::HEEPWRITE_LENGTH_2;
	// old one was : packetSize = 0x0A;  , but i think that that is wrong based on the datasheet. I changed it to 0x0B page 36
	additionalDataLength = PACKET_SIZE::HEEPWRITE_DATA_LENGTH_2;
	
	pID   = servoID;     	         
	CMD   = COMMAND::HEEPWRITE;

	checksumData[0]= address;               
	checksumData[1]= 0x01;                  // 9. Length
	checksumData[2]= writeByte;             // 10. Write error=0
	// I have no idea what checksumData[3] would be...

  	// base packet
	dataEx[0] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[1] = PACKET_CONSTS::PACKET_HEADER;
	dataEx[2] = packetSize;
	dataEx[3] = pID;
	dataEx[4] = CMD;

	// optional data
	dataEx[7] = checksumData[0]; 		// Address 52
	dataEx[8] = checksumData[1]; 		// Length
	dataEx[9] = checksumData[2]; 		// Value1
	dataEx[10]= checksumData[3]; 		// Value2

	// checksum
	checksumOne=calcChecksumOne();
  	checksumTwo=calcChecksumTwo();

	dataEx[5] = checksumOne;
	dataEx[6] = checksumTwo;

  	sendData(dataEx, packetSize);
}



// Private Methods //////////////////////////////////////////////////////////////

// calculated checksum1 as defined in datasheets
int HerkulexClass::calcChecksumOne()
{
	// checksum base formula is XOR packetLength, pID, CMD, and all additional data
  	checksumOne = 0 ^ packetLength ^ pID ^ CMD;
  	for (int i = 0; i < additionalDataLength; i++)
	{
		checksumOne = checksumOne ^ dataEx[i + 7]; // 7 puts us at the start of the additional data
	}
	return checksumOne & 0xFE;
}

// checksum2 is the bitwise complimente of ChecksumOne, which is why everything is the same except for the return
int HerkulexClass::calcChecksumTwo()
{
	checksumTwo = 0 ^ packetLength ^ pID ^ CMD;
	for (int i = 0; i < additionalDataLength; i++) 
	{
		checksumTwo = checksumTwo ^ dataEx[i + 7]; // 7 puts us at the start of the additional data
	}
	return (~checksumTwo) & 0xFE;
}


// Sending the buffer long lenght to Serial port
void HerkulexClass::sendData(byte* buffer, int lenght)
{
		// clearBuffer(); 		//clear the serialport buffer - try to do it!
        switch (_serialPort)
		{
			#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)
			case HSerial1:
				Serial1.write(buffer, lenght);
				break;
			case HSerial2:
				Serial2.write(buffer, lenght);
				break;
			case HSerial3:
				Serial3.write(buffer, lenght);
				break;
			#elif defined (ARDUINO_TEENSY41)
			case HSerial1:
				Serial1.write(buffer, lenght);
				break;
			case HSerial2:
				Serial2.write(buffer, lenght);
				break;
			case HSerial3:
				Serial3.write(buffer, lenght);
				break;
			case HSerial4:
				Serial4.write(buffer, lenght);
				break;
			case HSerial5:
				Serial5.write(buffer, lenght);
				break;
			case HSerial6:
				Serial6.write(buffer, lenght);
				break;
			case HSerial7:
				Serial7.write(buffer, lenght);
				break;
			case HSerial8:
				Serial8.write(buffer, lenght);
				break;
			#endif
		}
}

// * Receiving the lenght of bytes from Serial port
void HerkulexClass::readData(int size)
{
	int i = 0;
    int beginsave=0;
    int Time_Counter=0;

    switch (_serialPort)
	{
	#if defined (__AVR_ATmega1280__) || defined (__AVR_ATmega128__) || defined (__AVR_ATmega2560__)
	case HSerial1:
		while((Serial1.available() < size) & (Time_Counter < TIME_OUT)){
        		Time_Counter++;
        		delayMicroseconds(1000);
		}      	
		while (Serial1.available() > 0){
      		byte inchar = (byte)Serial1.read();
			//printHexByte(inchar);
        	if ( (inchar == 0xFF) & ((byte)Serial1.peek() == 0xFF) ){
						beginsave=1;
						i=0; 						
             }
            if (beginsave==1 && i<size) {
                       dataEx[i] = inchar;
                       i++;
			}
		}
		break;
	
	case HSerial2:
	    while((Serial2.available() < size) & (Time_Counter < TIME_OUT)){
        		Time_Counter++;
        		delayMicroseconds(1000);
		}
        	
		while (Serial2.available() > 0){
			byte inchar = (byte)Serial2.read();
			if ( (inchar == 0xFF) & ((byte)Serial2.peek() == 0xFF) ){
					beginsave=1;
					i=0; 					
			}
			if (beginsave==1 && i<size) {
				   dataEx[i] = inchar;
				   i++;
			}
		}
		break;

	case HSerial3:
		while((Serial3.available() < size) & (Time_Counter < TIME_OUT)){
			Time_Counter++;
			delayMicroseconds(1000);
		}
		
		while (Serial3.available() > 0){
			byte inchar = (byte)Serial3.read();
			if ( (inchar == 0xFF) & ((byte)Serial3.peek() == 0xFF) ){
					beginsave=1;
					i=0; 
			}
			if (beginsave==1 && i<size) {
				   dataEx[i] = inchar;
				   i++;
			}
		}
		break;
	#elif defined (ARDUINO_TEENSY41)
	case HSerial1:
		while((Serial1.available() < size) & (Time_Counter < TIME_OUT)){
        		Time_Counter++;
        		delayMicroseconds(1000);
		}      	
		while (Serial1.available() > 0){
      		byte inchar = (byte)Serial1.read();
			//printHexByte(inchar);
        	if ( (inchar == 0xFF) & ((byte)Serial1.peek() == 0xFF) ){
						beginsave=1;
						i=0; 						
             }
            if (beginsave==1 && i<size) {
                       dataEx[i] = inchar;
                       i++;
			}
		}
		break;
	
	case HSerial2:
	    while((Serial2.available() < size) & (Time_Counter < TIME_OUT)){
        		Time_Counter++;
        		delayMicroseconds(1000);
		}
        	
		while (Serial2.available() > 0){
			byte inchar = (byte)Serial2.read();
			if ( (inchar == 0xFF) & ((byte)Serial2.peek() == 0xFF) ){
					beginsave=1;
					i=0; 					
			}
			if (beginsave==1 && i<size) {
				   dataEx[i] = inchar;
				   i++;
			}
		}
		break;

	case HSerial3:
		while((Serial3.available() < size) & (Time_Counter < TIME_OUT)){
			Time_Counter++;
			delayMicroseconds(1000);
		}
		
		while (Serial3.available() > 0){
			byte inchar = (byte)Serial3.read();
			if ( (inchar == 0xFF) & ((byte)Serial3.peek() == 0xFF) ){
					beginsave=1;
					i=0; 
			}
			if (beginsave==1 && i<size) {
				   dataEx[i] = inchar;
				   i++;
			}
		}
		break;
	case HSerial4:
		while((Serial4.available() < size) & (Time_Counter < TIME_OUT)){
        		Time_Counter++;
        		delayMicroseconds(1000);
		}      	
		while (Serial4.available() > 0){
      		byte inchar = (byte)Serial4.read();
			//printHexByte(inchar);
        	if ( (inchar == 0xFF) & ((byte)Serial4.peek() == 0xFF) ){
						beginsave=1;
						i=0; 						
             }
            if (beginsave==1 && i<size) {
                       dataEx[i] = inchar;
                       i++;
			}
		}
		break;
	
	case HSerial5:
	    while((Serial5.available() < size) & (Time_Counter < TIME_OUT)){
        		Time_Counter++;
        		delayMicroseconds(1000);
		}
        	
		while (Serial5.available() > 0){
			byte inchar = (byte)Serial5.read();
			if ( (inchar == 0xFF) & ((byte)Serial5.peek() == 0xFF) ){
					beginsave=1;
					i=0; 					
			}
			if (beginsave==1 && i<size) {
				   dataEx[i] = inchar;
				   i++;
			}
		}
		break;

	case HSerial6:
		while((Serial6.available() < size) & (Time_Counter < TIME_OUT)){
			Time_Counter++;
			delayMicroseconds(1000);
		}
		
		while (Serial6.available() > 0){
			byte inchar = (byte)Serial6.read();
			if ( (inchar == 0xFF) & ((byte)Serial6.peek() == 0xFF) ){
					beginsave=1;
					i=0; 
			}
			if (beginsave==1 && i<size) {
				   dataEx[i] = inchar;
				   i++;
			}
		}
		break;
	case HSerial7:
		while((Serial7.available() < size) & (Time_Counter < TIME_OUT)){
        		Time_Counter++;
        		delayMicroseconds(1000);
		}      	
		while (Serial7.available() > 0){
      		byte inchar = (byte)Serial7.read();
			//printHexByte(inchar);
        	if ( (inchar == 0xFF) & ((byte)Serial7.peek() == 0xFF) ){
						beginsave=1;
						i=0; 						
             }
            if (beginsave==1 && i<size) {
                       dataEx[i] = inchar;
                       i++;
			}
		}
		break;
	
	case HSerial8:
	    while((Serial8.available() < size) & (Time_Counter < TIME_OUT)){
        		Time_Counter++;
        		delayMicroseconds(1000);
		}
        	
		while (Serial8.available() > 0){
			byte inchar = (byte)Serial8.read();
			if ( (inchar == 0xFF) & ((byte)Serial8.peek() == 0xFF) ){
					beginsave=1;
					i=0; 					
			}
			if (beginsave==1 && i<size) {
				   dataEx[i] = inchar;
				   i++;
			}
		}
		break;
	#endif
	}
}

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
