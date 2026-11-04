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
#include "BoardConfig.h"


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

HerkulexClass::HerkulexClass() : _serialPort(0) {} // queuedByteCount(0), PlayTimeSJog(0) {}

HerkulexClass::HerkulexClass(uint8_t serialPort) : _serialPort(serialPort) {
	
	// This project has one logical motor bus on ESP32 hardware UART2.
	_serial = (serialPort == 1) ? &Serial2 : nullptr;

	resetClassVals();
} 

void HerkulexClass::resetClassVals(){
	queuedByteCount = 0;
	newDataInInputBuffer = false;
	readStartTime = 0;
	packetLength = 0;
	pID = 0;
	CMD = 0;
	checksumOne = 0;
	checksumTwo = 0;
	additionalDataLength = 0;
	goalLSB = 0;
	goalMSB = 0;
	SET = 0;
	ID = 0;
}


// Begin serial bus communications
void HerkulexClass::beginSerialBus(uint32_t baud){
	if (_serial == nullptr) return;
	_serial->begin(baud, SERIAL_8N1,
		BoardConfig::MOTOR_RX_PIN,
		BoardConfig::MOTOR_TX_PIN);
}

// End serial bus communications
void HerkulexClass::endSerialBus(){
	if (_serial == nullptr) return;
	_serial->end();
}

void HerkulexClass::updateSerialBaud(uint32_t baud){
	if (_serial == nullptr) return;
	_serial->end();
	beginSerialBus(baud);
}

// initialize servos
void HerkulexClass::initialize(){
		resetClassVals();
        delay(100);       
        setACKPolicy(ACK_POLICY_TYPE::REPLY_TO_READ);
        delay(100);
        clearError(PACKET_CONSTS::ALL_SERVOS);	// clear error for all servos
        delay(100);
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
bool HerkulexClass::getModel(uint8_t servoID, uint16_t* model)
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
	// Added this small delay. Don't reboot until we actually write to the EEP register. If we don't have this delay, it may not finish and reset id to default (219 in decimal)
	delay(1000);
	reboot(oldID);
	delay(1000);
}

// clearError
void HerkulexClass::clearError(int servoID)
{
	uint8_t byteArray[2] = {0x00, 0x00}; // 0s clears servo errors
	writeToRamRegister(servoID, RAM_REGISTER::STATUS_ERROR, byteArray, 2);
}

void HerkulexClass::queueMove(motorMoveInfo moveInfo)
{	  
	packetQueue[++queuedByteCount] = (uint8_t) (moveInfo.goalPos & 0X00FF);        // add 8 lower bits of 16 bit goal
	packetQueue[++queuedByteCount] = (uint8_t) ((moveInfo.goalPos & 0XFF00) >> 8); // add 8 higher bits of 16 bit goal, masked to preserve reserved last bit of MSB
	packetQueue[++queuedByteCount] = moveInfo.ledColor << 2;                       // add LED value
	packetQueue[++queuedByteCount] = moveInfo.servoID;                             // add id of servo
}

// move all servos with the same execution time
// DO NOT USE IN MAIN: USE HerkulexMotor::actionMoves(int playTimeMs) instead
void HerkulexClass::actionMoves(uint8_t playTime)
{
    packetQueue[0] = playTime; // add playtime to packetqueue
	queuedByteCount += 1;      // update count

	// build and send packet
    buildPacket(PACKET_CONSTS::ALL_SERVOS, packetQueue, queuedByteCount, COMMAND::HSJOG);
    sendData(packet, packetLength);
    queuedByteCount = 0;
}

// Builds and sends the RAMREAD position-request packet then returns immediately.
// The motor starts composing its reply; the hardware RX buffer fills on its own.
// Call this on every bus BEFORE calling collectPosition on any bus so that all
// motors can reply in parallel while the CPU is busy sending to the next bus.
void HerkulexClass::sendPosRequest(int servoID) {
	requestFromRamRegister(servoID, RAM_REGISTER::CALIBRATED_POS, PACKET_LENGTH_BYTES::REGISTER_INFO_LENGTH);

	newDataInInputBuffer = false;
	readPending = true;
	inputLength = PACKET_LENGTH_BYTES::GETPOS_RESPONSE;
	readStartTime = micros();
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

// getPositionBlocking -- original blocking API, preserved for single-motor or debug use.
// Internally calls requestPosition then collectPosition back-to-back.
// When querying multiple motors across different buses, prefer using
// SerialBusManager::requestAllPositions() + collectAllPositions() instead.
uint16_t HerkulexClass::getPositionBlocking(int servoID) {
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
}

uint8_t HerkulexClass::getQueuedBytes(){
	return queuedByteCount;
}

// moves one motor with the set moveInfo of goal, ID, LED color, and playTime
void HerkulexClass::moveOne(motorMoveInfo moveInfo)
{
	uint8_t optionalData[PACKET_LENGTH_BYTES::HSJOG_MOVEONE_DATA_LENGTH] = {moveInfo.playTime,
																			(uint8_t) (moveInfo.goalPos & 0X00FF),
																			(uint8_t) ((moveInfo.goalPos & 0XFF00) >> 8),
																			(uint8_t) (moveInfo.ledColor << 2),
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
	delayMicroseconds(4000);

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
    const uint8_t expectedLength = PACKET_LENGTH_BYTES::BASE_LENGTH + optionalDataLength;
    const uint32_t startTime = micros();
    bool sawFirstHeaderByte = false;

    // Do not assume the first byte waiting in the UART is the start of this
    // reply. A stale ACK or a noise byte must not make every later packet fail.
    while (micros() - startTime < SERIAL_READ_TIMEOUT_US) {
        if (_serial->available() == 0) {
            delayMicroseconds(50);
            continue;
        }

        const uint8_t incoming = static_cast<uint8_t>(_serial->read());

        if (!sawFirstHeaderByte) {
            sawFirstHeaderByte = (incoming == PACKET_CONSTS::PACKET_HEADER);
            continue;
        }

        if (incoming != PACKET_CONSTS::PACKET_HEADER) {
            sawFirstHeaderByte = false;
            continue;
        }

        inputBuffer[0] = PACKET_CONSTS::PACKET_HEADER;
        inputBuffer[1] = PACKET_CONSTS::PACKET_HEADER;

        // Read the length byte using the same overall deadline.
        while (_serial->available() == 0 &&
               micros() - startTime < SERIAL_READ_TIMEOUT_US) {
            delayMicroseconds(50);
        }
        if (_serial->available() == 0) return false;

        uint8_t receivedLength = static_cast<uint8_t>(_serial->read());

        // Treat a run of 0xFF bytes as overlapping header candidates. This
        // recovers cleanly if a noise byte equal to 0xFF precedes a packet.
        while (receivedLength == PACKET_CONSTS::PACKET_HEADER &&
               micros() - startTime < SERIAL_READ_TIMEOUT_US) {
            while (_serial->available() == 0 &&
                   micros() - startTime < SERIAL_READ_TIMEOUT_US) {
                delayMicroseconds(50);
            }
            if (_serial->available() == 0) return false;
            receivedLength = static_cast<uint8_t>(_serial->read());
        }
        inputBuffer[2] = receivedLength;

        if (receivedLength < PACKET_LENGTH_BYTES::BASE_LENGTH ||
            receivedLength > DATA_SIZE) {
            sawFirstHeaderByte = false;
            continue;
        }

        uint8_t received = 3;
        while (received < receivedLength &&
               micros() - startTime < SERIAL_READ_TIMEOUT_US) {
            if (_serial->available() > 0) {
                inputBuffer[received++] = static_cast<uint8_t>(_serial->read());
            } else {
                delayMicroseconds(50);
            }
        }
        if (received != receivedLength) return false;

        const bool expectedPacket =
            receivedLength == expectedLength &&
            inputBuffer[3] == servoID &&
            inputBuffer[4] == static_cast<uint8_t>(cmd_res);

        if (expectedPacket && verifyInputPacket(inputBuffer, receivedLength)) {
            memcpy(outputBuffer, &inputBuffer[7], optionalDataLength);
            return true;
        }

        // The complete frame was stale, corrupt, or for another request. Keep
        // looking for the requested response until the deadline expires.
        sawFirstHeaderByte = false;
    }

    return false;
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
    while(_serial->available() < length){
        delayMicroseconds(50);
        if (micros() - readStartTime >= SERIAL_READ_TIMEOUT_US){
            return false;
        }
    }
	if(_serial->available() >= length){
		_serial->readBytes(inputBuffer, length);
    	return true;
	} else {
		return false;
	}
}
