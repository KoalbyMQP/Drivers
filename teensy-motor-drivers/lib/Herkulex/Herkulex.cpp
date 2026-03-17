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
  Edits by Pau Alcolea Vila (Worcester Polytechnic Institute, 2026)

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


HerkulexClass::HerkulexClass(uint8_t busId){
	_serialPort = busId;
}

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
			break;
		case HSerial2:
			Serial2.begin(baud);
			break;
		case HSerial3:
			Serial3.begin(baud);
			break;
		case HSerial4:
			Serial4.begin(baud);
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
	{
	packetSize    = 0x07;			//3.Packet size
	pID      = servoID;			//4.Servo ID - 0XFE=All servos
	cmd      = HSTAT;			//5.CMD
	
	ck1=(packetSize^pID^cmd)&0xFE;
        ck2=(~(packetSize^pID^cmd))&0xFE ; 
  
	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = packetSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	     
	sendData(dataEx, packetSize);
	delay(2);
	readData(9); 				// read 9 bytes from serial

	
	packetSize = dataEx[2];           // 3.Packet size 7-58
	pID   = dataEx[3];           // 4. Servo ID
	cmd   = dataEx[4];           // 5. CMD
	data[0]=dataEx[7];
    data[1]=dataEx[8];
    packetLength=2;

	
    ck1 = (dataEx[2]^dataEx[3]^dataEx[4]^dataEx[7]^dataEx[8]) & 0xFE;
	ck2=checksum2(ck1);			
	
	if (ck1 != dataEx[5]) return -1; //checksum verify
	if (ck2 != dataEx[6]) return -2;

	return dataEx[7];			// return status
}
}

// torque on - 
void HerkulexClass::torqueON(int servoID)
{
	
	uint8_t packetData[3] = {0x34, // memory address in servo ram
							 0x01,  // # of bytes of info in data packet
							 0x60}; // torque on command

	packetSize = 0x0A;               // 3.Packet size 7-58
	pID   = servoID;            // 4. Servo ID
	cmd   = HRAMWRITE;          // 5. CMD
	data[0]=0x34;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=0x60;               // 10. 0x60=Torque ON
	packetLength=3;             // lenghtData
  	
	ck1=checksum1(data,packetLength);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = packetSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = HRAMWRITE;	    // Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Torque ON

	sendData(dataEx, packetSize);
}

// torque off - the torque is FREE, not Break
void HerkulexClass::torqueOFF(int servoID)
{
	packetSize = 0x0A;               // 3.Packet size 7-58
	pID   = servoID;            // 4. Servo ID
	cmd   = HRAMWRITE;          // 5. CMD
	data[0]=0x34;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=0x00;               // 10. 0x00=Torque Free
	packetLength=3;             // lenghtData
  	
	ck1=checksum1(data,packetLength);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = packetSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Torque Free

    sendData(dataEx, packetSize);

}

// ACK  - 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
void HerkulexClass::setACKPolicy(int valueACK)
{
	packetSize = 0x0A;               // 3.Packet size 7-58
	pID   = 0xFE;	            // 4. Servo ID
	cmd   = HRAMWRITE;          // 5. CMD
	data[0]=0x34;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=valueACK;           // 10.Value. 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
	packetLength=3;             // lenghtData
  	
	ck1=checksum1(data,packetLength);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = packetSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Value

 	sendData(dataEx, packetSize);
}

// model - 1=0101 - 2=0201
byte HerkulexClass::checkModel()
{
	packetSize = 0x09;               // 3.Packet size 7-58
	pID   = 0xFE;	            // 4. Servo ID
	cmd   = HEEPREAD;           // 5. CMD
	data[0]=0x00;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	packetLength=2;             // lenghtData
  	
	ck1=checksum1(data,packetLength);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = packetSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address
	dataEx[8] = data[1]; 		// Length

    sendData(dataEx, packetSize);

	delay(1);
	readData(9);
	
	packetSize = dataEx[2];           // 3.Packet size 7-58
	pID   = dataEx[3];           // 4. Servo ID
	cmd   = dataEx[4];           // 5. CMD
	data[0]=dataEx[7];           // 8. 1st byte
	packetLength=1;              // lenghtData
  	
	ck1=checksum1(data,packetLength);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	if (ck1 != dataEx[5]) return -1; //checksum verify
	if (ck2 != dataEx[6]) return -2;
		
	return dataEx[7];			// return status

}

// setID - Need to restart the servo
void HerkulexClass::setID(int ID_Old, int ID_New)
{
	packetSize = 0x0A;               // 3.Packet size 7-58
	pID   = ID_Old;		        // 4. Servo ID OLD - original servo ID
	cmd   = HEEPWRITE;          // 5. CMD
	data[0]=0x06;               // 8. Address
	data[1]=0x01;               // 9. Lenght
	data[2]=ID_New;             // 10. ServoID NEW
	packetLength=3;             // lenghtData
  	
	ck1=checksum1(data,packetLength);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = packetSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Value

	sendData(dataEx, packetSize);

}

// clearError
void HerkulexClass::clearError(int servoID)
{
	packetSize = 0x0B;               // 3.Packet size 7-58
	pID   = servoID;     		// 4. Servo ID - 253=all servos
	cmd   = HRAMWRITE;          // 5. CMD
	data[0]=0x30;               // 8. Address
	data[1]=0x02;               // 9. Lenght
	data[2]=0x00;               // 10. Write error=0
	data[3]=0x00;               // 10. Write detail error=0
	
	packetLength=4;             // lenghtData
  	
	ck1=checksum1(data,packetLength);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = packetSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0]; 		// Address 52
	dataEx[8] = data[1]; 		// Length
	dataEx[9] = data[2]; 		// Value1
	dataEx[10]= data[3]; 		// Value2

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

	// packetsize is the intro packet length (8) + the queued packet length
    packetSize = 0x08 + queuedPacketCount;
 
    ck1 = checksum1(outputBuffer, queuedPacketCount);	//6. Checksum1
	ck2 = checksum2(ck1);				                //7. Checksum2


	// add HSJOG intro packet to the dataEx output buffer
	dataEx[0] = 0xFF;				// Packet Header
	dataEx[1] = 0xFF;				// Packet Header	
	dataEx[2] = packetSize;	 	    // Packet Size
	dataEx[3] = 0xFE;				// Servo ID: all servos
	dataEx[4] = HSJOG;				// Command Ram Write CMD SJOG Write n servo with same execution time
	dataEx[5] = ck1;				// Checksum 1
	dataEx[6] = ck2;				// Checksum 2
	dataEx[7] = playTime;			// Execution time	

	// copy outputBuffer into dataEx after the HSJOG intro packet
	memcpy(&dataEx[8], outputBuffer, queuedPacketCount);
	
	// send dataEx out onto the bus
	sendData(dataEx, packetSize);

	queuedPacketCount = 0; // reset counter 

}

// get Position
 uint16_t HerkulexClass::getPosition(int servoID) {
	uint16_t Position  = 0;

    packetSize = 0x09;               // 3.Packet size 7-58
	pID   = servoID;     	    // 4. Servo ID - 253=all servos
	cmd   = HRAMREAD;           // 5. CMD
	data[0]=0x3A;               // 8. Address
	data[1]=0x02;               // 9. Length
	
	packetLength=2;             // lenghtData
  	
	ck1=checksum1(data,packetLength);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = packetSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0];      	// Address  
	dataEx[8] = data[1]; 		// Length
	
	sendData(dataEx, packetSize);

    delayMicroseconds(100); // play with value
	readData(13);

        	
	packetSize = dataEx[2];           // 3.Packet size 7-58
	pID   = dataEx[3];           // 4. Servo ID
	cmd   = dataEx[4];           // 5. CMD
	data[0]=dataEx[7];
    data[1]=dataEx[8];
    data[2]=dataEx[9];
    data[3]=dataEx[10];
    data[4]=dataEx[11];
    data[5]=dataEx[12];
    packetLength=6;

    ck1=checksum1(data,packetLength);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

    if (ck1 != dataEx[5]) return -1;
	if (ck2 != dataEx[6]) return -1;

	Position = (dataEx[10] << 8) | dataEx[9];
        return Position;
	
}

// reboot single servo - pay attention 253 - all servos doesn't work!
void HerkulexClass::reboot(int servoID) {
        
    packetSize = 0x07;               // 3.Packet size 7-58
	pID   = servoID;     	    // 4. Servo ID - 253=all servos
	cmd   = HREBOOT;            // 5. CMD
    ck1=(packetSize^pID^cmd)&0xFE;
    ck2=(~(packetSize^pID^cmd))&0xFE ; ;	

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = packetSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	
	sendData(dataEx, packetSize);

}

// LED  - see table of colors 
void HerkulexClass::setLed(int servoID, int valueLed)
{
	packetSize   = 0x0A;               // 3.Packet size 7-58
	pID     = servoID;            // 4. Servo ID
	cmd     = HRAMWRITE;          // 5. CMD
	data[0] = 0x35;               // 8. Address 53
    data[1] = 0x01;               // 9. Lenght
	data[2] = valueLed;           // 10.LedValue
	packetLength=3;               // lenghtData
  	  	
	ck1=checksum1(data,packetLength);	//6. Checksum1
	ck2=checksum2(ck1);					//7. Checksum2

	dataEx[0] = 0xFF;			// Packet Header
	dataEx[1] = 0xFF;			// Packet Header	
	dataEx[2] = packetSize;	 		// Packet Size
	dataEx[3] = pID;			// Servo ID
	dataEx[4] = cmd;			// Command Ram Write
	dataEx[5] = ck1;			// Checksum 1
	dataEx[6] = ck2;			// Checksum 2
	dataEx[7] = data[0];        // Address
	dataEx[8] = data[1];       	// Length
	dataEx[9] = data[2];        // Value

	sendData(dataEx, packetSize);
}

// get the speed for one servo - values betweeb -1023 <--> 1023
int HerkulexClass::getSpeed(int servoID) {
  int speedy  = 0;

  packetSize = 0x09;               // 3.Packet size 7-58
  pID   = servoID;     	   	  // 4. Servo ID 
  cmd   = HRAMREAD;           // 5. CMD
  data[0]=0x40;               // 8. Address
  data[1]=0x02;               // 9. Lenght

  packetLength=2;             // lenghtData

  ck1=checksum1(data,packetLength);		//6. Checksum1
  ck2=checksum2(ck1);					//7. Checksum2

  dataEx[0] = 0xFF;			// Packet Header
  dataEx[1] = 0xFF;			// Packet Header	
  dataEx[2] = packetSize;		// Packet Size
  dataEx[3] = pID;			// Servo ID
  dataEx[4] = cmd;			// Command Ram Write
  dataEx[5] = ck1;			// Checksum 1
  dataEx[6] = ck2;			// Checksum 2
  dataEx[7] = data[0]; 	    // Address  
  dataEx[8] = data[1]; 		// Length

  sendData(dataEx, packetSize);

  delay(1);
  readData(13);


  packetSize = dataEx[2];           // 3.Packet size 7-58
  pID   = dataEx[3];           // 4. Servo ID
  cmd   = dataEx[4];           // 5. CMD
  data[0]=dataEx[7];
  data[1]=dataEx[8];
  data[2]=dataEx[9];
  data[3]=dataEx[10];
  data[4]=dataEx[11];
  data[5]=dataEx[12];
  packetLength=6;

  ck1=checksum1(data,packetLength);	//6. Checksum1
  ck2=checksum2(ck1);				//7. Checksum2

  if (ck1 != dataEx[5]) return -1;
  if (ck2 != dataEx[6]) return -1;

  speedy = ((dataEx[10]&0xFF)<<8) | dataEx[9];
  return speedy;

}

// moves one motor with the set moveInfo of goal, ID, LED color, and playTime
void HerkulexClass::moveOne(motorMoveInfo moveInfo)
{
	uint8_t posLowerBits = (uint8_t) (moveInfo.goalPos & 0X00FF);
	uint8_t posUpperBits = (uint8_t) (moveInfo.goalPos & 0XFF00) >> 8;

	uint8_t lenPacketData = 4; // as specified in datasheet
	
	uint8_t packetData[lenPacketData] = {posLowerBits, posUpperBits, moveInfo.ledColor, moveInfo.servoID};


	packetSize = 0x0C; // packet size of 12 as specified in datasheet S_JOG example

	ck1=checksum1(packetData, lenPacketData);  // Checksum1
	ck2=checksum2(ck1);			               // Checksum2

	dataEx[0] = 0xFF;			    // Packet Header
	dataEx[1] = 0xFF;			    // Packet Header	
	dataEx[2] = packetSize;	        // Packet Size
	dataEx[3] = moveInfo.servoID;   // Servo ID
	dataEx[4] = HSJOG;		        // Command type
	dataEx[5] = ck1;				// Checksum 1
	dataEx[6] = ck2;			    // Checksum 2
	dataEx[7] = moveInfo.playTime;  // Execution time	
	dataEx[8] = posLowerBits;       // lower 8 bits of goal position
	dataEx[9] = posUpperBits;       // upper 8 bits of goal position
	dataEx[10] = moveInfo.ledColor;
	dataEx[11] = moveInfo.servoID;

	sendData(dataEx, packetSize);
}

// write registry in the RAM: one byte 
void HerkulexClass::writeRegistryRAM(int servoID, int address, int writeByte)
{
  packetSize = 0x0A;               	// 3.Packet size 7-58
  pID   = servoID;     			// 4. Servo ID - 253=all servos
  cmd   = HRAMWRITE;          	// 5. CMD
  data[0]=address;              // 8. Address
  data[1]=0x01;               	// 9. Lenght
  data[2]=writeByte;            // 10. Write error=0
 
  packetLength=3;             	// lenghtData

  ck1=checksum1(data,packetLength);	//6. Checksum1
  ck2=checksum2(ck1);				//7. Checksum2

  dataEx[0] = 0xFF;			// Packet Header
  dataEx[1] = 0xFF;			// Packet Header	
  dataEx[2] = packetSize;	 	// Packet Size
  dataEx[3] = pID;			// Servo ID
  dataEx[4] = cmd;			// Command Ram Write
  dataEx[5] = ck1;			// Checksum 1
  dataEx[6] = ck2;			// Checksum 2
  dataEx[7] = data[0]; 		// Address 52
  dataEx[8] = data[1]; 		// Length
  dataEx[9] = data[2]; 		// Value1
  dataEx[10]= data[3]; 		// Value2

  sendData(dataEx, packetSize);

}

// write registry in the EEP memory (ROM): one byte 
void HerkulexClass::writeRegistryEEP(int servoID, int address, int writeByte)
{
  packetSize = 0x0A;                  // 3.Packet size 7-58
  pID   = servoID;     	         // 4. Servo ID - 253=all servos
  cmd   = HEEPWRITE;             // 5. CMD
  data[0]=address;               // 8. Address
  data[1]=0x01;                  // 9. Lenght
  data[2]=writeByte;             // 10. Write error=0
 
  packetLength=3;           	 // lenghtData

  ck1=checksum1(data,packetLength);	//6. Checksum1
  ck2=checksum2(ck1);				//7. Checksum2

  dataEx[0] = 0xFF;			// Packet Header
  dataEx[1] = 0xFF;			// Packet Header	
  dataEx[2] = packetSize;		// Packet Size
  dataEx[3] = pID;			// Servo ID
  dataEx[4] = cmd;			// Command Ram Write
  dataEx[5] = ck1;			// Checksum 1
  dataEx[6] = ck2;			// Checksum 2
  dataEx[7] = data[0]; 		// Address 52
  dataEx[8] = data[1]; 		// Length
  dataEx[9] = data[2]; 		// Value1
  dataEx[10]= data[3]; 		// Value2

  sendData(dataEx, packetSize);

}



// Private Methods //////////////////////////////////////////////////////////////

// calculated checksum1 as defined in datasheets
int HerkulexClass::checksum1(byte* data, int packetLength)
{
  XOR = 0;	
  XOR = XOR ^ packetSize;
  XOR = XOR ^ pID;
  XOR = XOR ^ cmd;
  for (int i = 0; i < packetLength; i++) 
  {
    XOR = XOR ^ data[i];
  }
  return XOR&0xFE;
}

// checksum2
int HerkulexClass::checksum2(int XOR)
{
  return (~XOR)&0xFE;
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



 HerkulexClass Herkulex;