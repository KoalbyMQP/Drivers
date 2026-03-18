#include "SerialBusManager.h"
#include "Herkulex.h"


static void createBus(uint8_t serialPort){
    // when called, this function creates a specific bus and marks it in the two arrays
    // _busesTracker keeps track of what buses are being used in an array that goes from 0-7
        // this is used to check that when something is called on a bus that it is properly initialized
    // _buses actaully contains the HerkulexClass instances for each of the initialized buses with the same index as their marks in _busesTracker

    // if busId is outside of set range
    if ((serialPort < 0) || (serialPort > SerialBusManager::MAX_BUS_COUNT)) return;
    
    SerialBusManager::_busesTracker[serialPort - 1] = 1;
    SerialBusManager::_buses[serialPort - 1] = HerkulexClass(serialPort);
}

static HerkulexClass& getBus(uint8_t serialPort){
    return SerialBusManager::_buses[serialPort];
}

static void startAllBuses(long baud){
    for (int i = 0; i < SerialBusManager::MAX_BUS_COUNT; i++){

        // if we have a serial port created
        if (SerialBusManager::_busesTracker[i] == 1){

            // then start it
            SerialBusManager::_buses[i].beginSerialBus(baud);
        }
    }
}

static void endAllBuses(){
    for (int i = 0; i < SerialBusManager::MAX_BUS_COUNT; i++){

        // if we have a serial port created
        if (SerialBusManager::_busesTracker[i] == 1){

            // then end it
            SerialBusManager::_buses[i].end();
        }
    }
}

static void initAllMotors(){
    for (int i = 0; i < SerialBusManager::MAX_BUS_COUNT; i++){

        // if we have a serial port created
        if (SerialBusManager::_busesTracker[i] == 1){

            // then initialize the motors on it
            SerialBusManager::_buses[i].initialize();
        }
    }
}

static void actionAll(int playTimeMs){
    uint8_t playTime = (uint8_t) (playTimeMs / CONVERT_PLAYTIME_TO_MS);

    for (int i = 0; i < SerialBusManager::MAX_BUS_COUNT; i++){

        // if we have a serial port created
        if (SerialBusManager::_busesTracker[i] == 1){

            // then action the moves queued
            SerialBusManager::_buses[i].actionMoves(playTime);
        }
    }       
}