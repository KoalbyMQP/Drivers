#include "SerialBusManager.h"
#include "Herkulex.h"



static void createBus(uint8_t serialPort){

    // if busId is outside of set range
    if ((serialPort < 0) || (serialPort > SerialBusManager::MAX_BUS_COUNT)) return;
    
    SerialBusManager::_busesTracker[serialPort] = 1;
    SerialBusManager::_buses[serialPort] = HerkulexClass(serialPort);
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

