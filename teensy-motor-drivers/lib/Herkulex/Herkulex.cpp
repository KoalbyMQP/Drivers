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
void HerkulexClass::beginSerialBus(uint32_t baud){
	if (_serial == nullptr) return;
	_serial->begin(baud);
}

// End serial bus communications
void HerkulexClass::endSerialBus(){
	if (_serial == nullptr) return;
	_serial->end();
}

void HerkulexClass::updateSerialBaud(uint32_t baud){
	if (_serial == nullptr) return;
	_serial->end();
	_serial->begin(baud);
}

// initialize servos
void HerkulexClass::initialize(){
		resetClassVals();
        delay(100);       
        setACKPolicy(ACK_POLICY_TYPE::REPLY_TO_READ);
        delay(10);
        torqueON(PACKET_CONSTS::ALL_SERVOS);    // torqueON for all servos
        delay(10);
        clearError(PACKET_CONSTS::ALL_SERVOS);	// clear error for all servos
        delay(10);
}

// stat
bool HerkulexClass::stat(uint8_t servoID, uint8_t* statError, uint8_t* statDetail)
{
	sendPacket(servoID, nullptr, 0, COMMAND::HSTAT);
	delayMicroseconds(2000);

	uint8_t buffer[2];

	if(!readPacketReply(servoID, buffer, 2, COMMAND_RESPONSE::HSTAT_RESPONSE)){
		return false;
	}

	*statError = buffer[0];
	*statDetail = buffer[1];

	return true;
}

// torque on - 
void HerkulexClass::torqueON(int servoID)
{
	uint8_t byteArray[1] = {TORQUE_MODE::TORQUE_ON};
	writeToRamRegister(servoID, RAM_REGISTER::TORQUE_CONTROL, byteArray, 1);
}

// torque to free drive mode
void HerkulexClass::torqueFree(int servoID)
{
	uint8_t byteArray[1] = {TORQUE_MODE::TORQUE_FREE};
	writeToRamRegister(servoID, RAM_REGISTER::TORQUE_CONTROL, byteArray, 1);
}

// ACK  - 0=No Replay, 1=Only reply to READ CMD, 2=Always reply
void HerkulexClass::setACKPolicy(int valueACK)
{	
	uint8_t byteArray[1] = {ACK_POLICY_TYPE::REPLY_TO_READ};
	writeToRamRegister(PACKET_CONSTS::ALL_SERVOS, RAM_REGISTER::ACK_POLICY, byteArray, 1);
}


// sets baud rate for bus, meaning all motors on bus update their baud rate
// also updates bus serial baud rate.
void HerkulexClass::setBaudRate(BAUD_RATE newBaud){
	uint8_t byteArray[1] = {newBaud};
	writeToEEPRegister(PACKET_CONSTS::ALL_SERVOS, EEP_REGISTER::BAUD_SETTING, byteArray, 1);
	delay(10);
	reboot(PACKET_CONSTS::ALL_SERVOS);
	delay(500);
	updateSerialBaud(BAUD_RATE_MAP.at(newBaud));
	initialize();
	delay(10);
}

// return full model number as specified in datasheet
bool HerkulexClass::checkModel(uint8_t servoID, uint16_t* model)
{
	uint8_t result[2];

    if (!readFromEEPRegisterBlocking(servoID, EEP_REGISTER::MOTOR_MODEL, 2, result)) {
		return false;
	}
	
	// model no is 16 bit int, shift over by 8 for correct model number
	*model = (result[0] << 8) | result[1];
    return true;

}

// setID - Need to restart the servo
void HerkulexClass::setID(uint8_t oldID, uint8_t newID)
{
	uint8_t byteArray[1] = {newID};
	writeToEEPRegister(oldID, EEP_REGISTER::ID, byteArray, 1);
	delay(100);
	reboot(oldID);
	delay(500);
}

// clearError
void HerkulexClass::clearError(int servoID)
{
	uint8_t byteArray[2] = {0x00, 0x00}; // 0s clears servo errors
	writeToRamRegister(servoID, RAM_REGISTER::STATUS_ERROR, byteArray, 2);
}

void HerkulexClass::queueMove(motorMoveInfo moveInfo)
{	  
	packetQueue[queuedPacketCount++] = (uint8_t) (moveInfo.goalPos & 0xFF);        // add 8 lower bits of 16 bit goal
	packetQueue[queuedPacketCount++] = (uint8_t) (moveInfo.goalPos >> 8 & 0xFF);   // add 8 higher bits of 16 bit goal
	packetQueue[queuedPacketCount++] = (moveInfo.ledColor << 2);                   // add LED value
	packetQueue[queuedPacketCount++] = moveInfo.servoID;                           // add id of servo
}

// move all servos with the same execution time
// DO NOT USE IN MAIN: USE HerkulexMotor::actionMoves(int playTimeMs) instead
// TODO: refactor sendPacket into buildPacket and sendPacket functions
// as we have to use buildPacket here then append the packetQueue after it, then send the whole bigass packet
// TODO: update to get rid of optionalData declaration, instead implement queuedPacketIndex and just implement logic on packetQueue
void HerkulexClass::actionMoves(uint8_t playTime)
{
	uint8_t optionalDataLength = PACKET_LENGTH_BYTES::HSJOG_MOVEMULTIPLE_DATA_LENGTH + queuedPacketCount;
    uint8_t optionalData[optionalDataLength];

    optionalData[0] = playTime;
    memcpy(&optionalData[1], packetQueue, queuedPacketCount);

    buildPacket(PACKET_CONSTS::ALL_SERVOS, optionalData, optionalDataLength, COMMAND::HSJOG);
    sendData(packet, packetLength);

    queuedPacketCount = 0;
}

// Builds and sends the RAMREAD position-request packet then returns immediately.
// The motor starts composing its reply; the hardware RX buffer fills on its own.
// Call this on every bus BEFORE calling collectPosition on any bus so that all
// motors can reply in parallel while the CPU is busy sending to the next bus.
void HerkulexClass::sendPosRequest(int servoID) {
	requestFromRamRegister(servoID, RAM_REGISTER::CALIBRATED_POS, PACKET_LENGTH_BYTES::REGISTER_INFO_LENGTH);
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
	// read data from serial port
	_serial->readBytes(inputBuffer, inputLength);

	newDataInInputBuffer = false;

	// verify input data against checksum
	if (!verifyInputPacket(inputBuffer, PACKET_LENGTH_BYTES::GETPOS_RESPONSE)){
		return 0xFFFF; // error flag
	}
	return (uint16_t)((inputBuffer[10] << 8) | inputBuffer[9]);
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

// reboots servos
void HerkulexClass::reboot(int servoID) {
	sendPacket(servoID, nullptr, PACKET_LENGTH_BYTES::HREBOOT_DATA_LENGTH, COMMAND::HREBOOT);
}

void HerkulexClass::setLed(uint8_t servoID, LED_STATE valueLed)
{
	uint8_t byteArray[1] = {valueLed};
	writeToRamRegister(servoID, RAM_REGISTER::LED_CONTROL, byteArray, 1);
}

// get the speed for one servo - values betweeb -1023 <--> 1023
uint16_t HerkulexClass::getSpeed(int servoID) {
	uint8_t dataBuffer[2];
	if (readFromRamRegisterBlocking(servoID, RAM_REGISTER::PWM, 2, dataBuffer)){
		return (((uint16_t)dataBuffer[1] << 8) | dataBuffer[0]) & 0x03FF; // build 16 bit int, then mask to 10 bits (max 1023)
	}
	return -1;
  	// int speedy  = 0;

	// packetLength = PACKET_LENGTH_BYTES::HRAMREAD_LENGTH;
	// additionalDataLength = PACKET_LENGTH_BYTES::HRAMREAD_DATA_LENGTH;

	// pID   = servoID;     	   	  
	// CMD   = COMMAND::HRAMREAD;      

	// checksumData[0]=0x40;               // 8. Address
	// checksumData[1]=0x02;               // 9. Lenght

  
	// // base packet
	// packet[0] = PACKET_CONSTS::PACKET_HEADER;
	// packet[1] = PACKET_CONSTS::PACKET_HEADER;
	// packet[2] = packetLength;
	// packet[3] = pID;
	// packet[4] = CMD;
  
	// //optional data
	// packet[7] = checksumData[0]; 	    // Address  
	// packet[8] = checksumData[1]; 		// Length
  
	// // checksum
	// checksumOne=calcChecksumOne();		
	// checksumTwo=calcChecksumTwo();		
  
	// packet[5] = checksumOne;
	// packet[6] = checksumTwo;	
	
	// sendData(packet, packetLength);

	// delay(1);
	// readBlocking(13);


	// // This is the second half of the function that uses the read data
	// packetLength =    inputBuffer[2];    
	// pID   =           inputBuffer[3]; 
	// CMD   =           inputBuffer[4];

	// packetLength = 6;
	// memcpy(&packet[7], &inputBuffer[7], packetLength);

	// checksumOne=calcChecksumOne();	
	// checksumTwo=calcChecksumTwo();	

	// if (checksumOne != inputBuffer[5]) return -1;
	// if (checksumTwo != inputBuffer[6]) return -1;

	// speedy = ((packet[10]&0xFF)<<8) | packet[9];
	// return speedy;
}

// moves one motor with the set moveInfo of goal, ID, LED color, and playTime
void HerkulexClass::moveOne(motorMoveInfo moveInfo)
{
	uint8_t optionalData[PACKET_LENGTH_BYTES::HSJOG_MOVEONE_DATA_LENGTH] = {moveInfo.playTime,
																			(uint8_t) (moveInfo.goalPos & 0X00FF),
																			(uint8_t) ((moveInfo.goalPos & 0XFF00) >> 8),
																			moveInfo.ledColor << 2,
																			moveInfo.servoID};
	
	sendPacket(moveInfo.servoID, optionalData, PACKET_LENGTH_BYTES::HSJOG_MOVEONE_DATA_LENGTH, COMMAND::HSJOG);
}




// Private Methods //////////////////////////////////////////////////////////////

// REGISTER WRITES
void HerkulexClass::writeToRamRegister(uint8_t servoID, RAM_REGISTER address, uint8_t* writeData, uint8_t writeDataLength){
	writeToRegister(servoID, address, writeData, writeDataLength, COMMAND::HRAMWRITE);
}

void HerkulexClass::writeToEEPRegister(uint8_t servoID, EEP_REGISTER address, uint8_t* writeData, uint8_t writeDataLength){
	writeToRegister(servoID, address, writeData, writeDataLength, COMMAND::HEEPWRITE);
}

// GENERAL REGISTER IMPLEMENTATION: USE writeToRamRegister OR writeToEEPRegister INSTEAD
void HerkulexClass::writeToRegister(uint8_t servoID, uint8_t address, uint8_t* writeData, uint8_t writeDataLength, COMMAND cmd){
    uint8_t optionalDataLength = PACKET_LENGTH_BYTES::REGISTER_INFO_LENGTH + writeDataLength;
    uint8_t optionalData[optionalDataLength];
    optionalData[0] = address;
    optionalData[1] = writeDataLength;
    memcpy(&optionalData[2], writeData, writeDataLength);
    sendPacket(servoID, optionalData, optionalDataLength, cmd);
}

// REGISTER READS/REQs
void HerkulexClass::requestFromRamRegister(uint8_t servoID, RAM_REGISTER address, uint8_t numRequestedBytes){
	requestFromRegister(servoID, address, numRequestedBytes, COMMAND::HRAMREAD);
}

void HerkulexClass::requestFromEEPRegister(uint8_t servoID, EEP_REGISTER address, uint8_t numRequestedBytes){
	requestFromRegister(servoID, address, numRequestedBytes, COMMAND::HEEPREAD);
}

bool HerkulexClass::readFromRamRegisterBlocking(uint8_t servoID, RAM_REGISTER address, uint8_t numRequestedBytes, uint8_t* buffer){
	return readFromRegisterBlocking(servoID, address, numRequestedBytes, buffer, COMMAND::HRAMREAD, COMMAND_RESPONSE::HRAMREAD_RESPONSE);
}

bool HerkulexClass::readFromEEPRegisterBlocking(uint8_t servoID, EEP_REGISTER address, uint8_t numRequestedBytes, uint8_t* buffer){
	return readFromRegisterBlocking(servoID, address, numRequestedBytes, buffer, COMMAND::HEEPREAD, COMMAND_RESPONSE::HEEPREAD_RESPONSE);
}

// GENERAL REGISTER IMPLEMENTATION: USE requestFromRamRegister OR requestFromEEPRegister INSTEAD
void HerkulexClass::requestFromRegister(uint8_t servoID, uint8_t address, uint8_t numRequestedBytes, COMMAND cmd){
	uint8_t optionalDataLength = PACKET_LENGTH_BYTES::REGISTER_INFO_LENGTH;
	uint8_t optionalData[optionalDataLength] = {address, numRequestedBytes};
	sendPacket(servoID, optionalData, optionalDataLength, cmd);
}

// GENERAL REGISTER IMPLEMENTATION: USE readFromRamRegisterBlocking OR ReadFromEEPRegisterBlocking INSTEAD
bool HerkulexClass::readFromRegisterBlocking(uint8_t servoID, uint8_t address, uint8_t numRequestedBytes, uint8_t* buffer, COMMAND cmd, COMMAND_RESPONSE cmd_res){
	requestFromRegister(servoID, address, numRequestedBytes, cmd);
	delayMicroseconds(2000);

	uint8_t replyOptionalLength = 4 + numRequestedBytes;
	uint8_t replyBuff[replyOptionalLength];
	
	// Since we are taking address and status echo into account, replybuff[2] is where we need to memcpy
	if (!readPacketReply(servoID, replyBuff, replyOptionalLength, cmd_res)) return false;
	memcpy(buffer, &replyBuff[2], numRequestedBytes);
    return true;

}




// these are the most basic servo packet methods: one to read and one to pull a reply from //
// ======================================================================================= //

// all packets consist of a base packet and optional data. Optional data can be empty.
void HerkulexClass::buildPacket(uint8_t servoID, uint8_t* optionalData, uint8_t optionalDataLength, COMMAND cmd){
    additionalDataLength = optionalDataLength;
    packetLength = PACKET_LENGTH_BYTES::BASE_LENGTH + additionalDataLength;
    pID = servoID;
    CMD = cmd;

    packet[0] = PACKET_CONSTS::PACKET_HEADER;
    packet[1] = PACKET_CONSTS::PACKET_HEADER;
    packet[2] = packetLength;
    packet[3] = pID;
    packet[4] = CMD;

    memcpy(&packet[7], optionalData, optionalDataLength);

    checksumOne = calcChecksumOne();
    checksumTwo = calcChecksumTwo();
    packet[5] = checksumOne;
    packet[6] = checksumTwo;
}

void HerkulexClass::sendPacket(uint8_t servoID, uint8_t* optionalData, uint8_t optionalDataLength, COMMAND cmd){
    buildPacket(servoID, optionalData, optionalDataLength, cmd);
    sendData(packet, packetLength);
}

// reads Optional Data (as specified in datasheet) into outputBuffer
bool HerkulexClass::readPacketReply(uint8_t servoID, uint8_t* outputBuffer, uint8_t optionalDataLength, COMMAND_RESPONSE cmd_res){
    packetLength = PACKET_LENGTH_BYTES::BASE_LENGTH + optionalDataLength;

    if (!readBlocking(packetLength)){
		// Serial.println("readBlocking returned nothing."); 
		return false;
	}
	// Something is going wrong in verifyInputPacket
    if (!verifyInputPacket(inputBuffer, packetLength)){
		// Serial.println("Packet cannot be verified.");
		return false;
	}

    memcpy(outputBuffer, &inputBuffer[7], optionalDataLength);
    return true;
}

bool HerkulexClass::verifyInputPacket(uint8_t* inputPacket, uint8_t inputPacketLength){
    additionalDataLength = inputPacketLength - PACKET_LENGTH_BYTES::BASE_LENGTH;
    packetLength = inputPacketLength;
    pID = inputPacket[3];
    CMD = inputPacket[4];

    packet[0] = PACKET_CONSTS::PACKET_HEADER;
    packet[1] = PACKET_CONSTS::PACKET_HEADER;
    packet[2] = packetLength;
    packet[3] = pID;
    packet[4] = CMD;
    memcpy(&packet[7], &inputPacket[7], additionalDataLength);

    checksumOne = calcChecksumOne();
    checksumTwo = calcChecksumTwo();
    return (checksumOne == inputPacket[5]) && (checksumTwo == inputPacket[6]);
}

// calculated checksum1 as defined in datasheets with class variables, ensure all are
// set properly: packetLength, pID, CMD, additionalDataLength, and all info in packet up to length packetLength
uint8_t HerkulexClass::calcChecksumOne()
{
	// checksum base formula is XOR packetLength, pID, CMD, and all additional data
  	checksumOne = 0 ^ packetLength ^ pID ^ CMD;
  	for (int i = 0; i < additionalDataLength; i++)
	{
		checksumOne = checksumOne ^ packet[i + PACKET_LENGTH_BYTES::BASE_LENGTH]; //puts us at the start of the additional data
	}
	return checksumOne & 0xFE;
}

// checksum2 is the bitwise complimente of ChecksumOne, which is why everything is the same except for the return
// calculated checksum2 as defined in datasheets with class variables, ensure all are
// set properly: packetLength, pID, CMD, additionalDataLength, and all info in packet up to length packetLength
uint8_t HerkulexClass::calcChecksumTwo()
{
	checksumTwo = 0 ^ packetLength ^ pID ^ CMD;
	for (int i = 0; i < additionalDataLength; i++) 
	{
		checksumTwo = checksumTwo ^ packet[i + PACKET_LENGTH_BYTES::BASE_LENGTH]; // puts us at the start of the additional data
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

// update the read request MUST CALL REQUEST READ BEFOREHAND
void HerkulexClass::updateRead(){
	// do nothing if no read is requested
	if(!readPending) return;

	// store data in input buffer and update flags
	if(_serial->available() >= inputLength){
		readPending = false;
		newDataInInputBuffer = true;
	}

	// timeout
	else if (micros() - readStartTime >= SERIAL_READ_TIMEOUT_US){
		readPending = false;
		newDataInInputBuffer = true;
	}
}



bool HerkulexClass::readBlocking(uint8_t length){
    readStartTime = micros();
	// Serial.print("Attempting to read ");
	// Serial.print(packetLength);
	// Serial.println(" bytes");
    while(_serial->available() < length){
        delayMicroseconds(50);
        if (micros() - readStartTime >= SERIAL_READ_TIMEOUT_US){
            return false;
        }
    }
	if(_serial->available() >= length){
		_serial->readBytes(inputBuffer, length);
		// for (uint8_t i = 0; i < length; i++) {
		// 	printHexByte(inputBuffer[i]);
		// }
		// Serial.println();
    	return true;
	} else {
		return false;
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
