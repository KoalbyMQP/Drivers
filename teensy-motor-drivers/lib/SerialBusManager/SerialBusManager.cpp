#include "SerialBusManager.h"
#include "Herkulex.h"

HerkulexClass SerialBusManager::_buses[SerialBusManager::MAX_BUS_COUNT];
int SerialBusManager::_busesTracker[SerialBusManager::MAX_BUS_COUNT];


void SerialBusManager::createBus(uint8_t serialPort){
    // when called, this function creates a specific bus and marks it in the two arrays
    // _busesTracker keeps track of what buses are being used in an array that goes from 0-7
        // this is used to check that when something is called on a bus that it is properly initialized
    // _buses actaully contains the HerkulexClass instances for each of the initialized buses with the same index as their marks in _busesTracker

    // if busId is outside of set range
    if ((serialPort < 1) || (serialPort > SerialBusManager::MAX_BUS_COUNT)){
        Serial.print("not making bus");
        return;
    }
    
    SerialBusManager::_busesTracker[serialPort - 1] = 1;
    SerialBusManager::_buses[serialPort - 1] = HerkulexClass(serialPort);
    Serial.print("created bus: ");
    Serial.println(serialPort);
}

HerkulexClass& SerialBusManager::getBus(uint8_t serialPort){

    if ((serialPort < 1) || (serialPort > SerialBusManager::MAX_BUS_COUNT)) return;

    return SerialBusManager::_buses[serialPort - 1];
}

void SerialBusManager::startAllBuses(long baud){
    for (int i = 0; i < SerialBusManager::MAX_BUS_COUNT; i++){

        // if we have a serial port created
        if (SerialBusManager::_busesTracker[i] == 1){

            Serial.print("starting bus:" );
            Serial.println(i + 1);

            // then start it
            SerialBusManager::_buses[i].beginSerialBus(baud);
        }
    }
}

void SerialBusManager::endAllBuses(){
    for (int i = 0; i < SerialBusManager::MAX_BUS_COUNT; i++){

        // if we have a serial port created
        if (SerialBusManager::_busesTracker[i] == 1){

            // then end it
            SerialBusManager::_buses[i].end();
        }
    }
}

void SerialBusManager::initAllMotors(){
    for (int i = 0; i < SerialBusManager::MAX_BUS_COUNT; i++){

        // if we have a serial port created
        if (SerialBusManager::_busesTracker[i] == 1){

            // then initialize the motors on it
            SerialBusManager::_buses[i].initialize();
        }
    }
}

void SerialBusManager::actionAll(int playTimeMs){
    uint8_t playTime = (uint8_t) (playTimeMs / CONVERT_PLAYTIME_TO_MS);

    for (int i = 0; i < SerialBusManager::MAX_BUS_COUNT; i++){

        // if we have a serial port created
        if (SerialBusManager::_busesTracker[i] == 1){

            // then action the moves queued
            SerialBusManager::_buses[i].actionMoves(playTime);
        }
    }       
}

void SerialBusManager::requestAllPositions(const MotorRef* motors, uint8_t count){
    // iterate through all of the motors in the referece table (all motors we are using)
    for (uint8_t i = 0; i < count; i++){
        uint8_t busIndex = motors[i].busId - 1;  // _buses[] is 0-indexed; busId starts at 2
        // it is busIndex and not i because i is used for all of the motors, we are not iterating through the buses like the other methods
        if (SerialBusManager::_busesTracker[busIndex] == 1){
            SerialBusManager::_buses[busIndex].requestPosition(motors[i].servoId);
        }
    }
}

void SerialBusManager::collectAllPositions(const MotorRef* motors, uint16_t* results, uint8_t count){
    for (uint8_t i = 0; i < count; i++){
        uint8_t busIndex = motors[i].busId - 1;
        if (SerialBusManager::_busesTracker[busIndex] == 1){
            results[i] = SerialBusManager::_buses[busIndex].collectPosition(motors[i].servoId);
        } else {
            results[i] = 0xFFFF;    // bus not initialized
        }
    }
}
