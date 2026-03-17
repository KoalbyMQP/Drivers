#ifndef SerialBusManager_h
#define SerialBusManager_h
#include "Arduino.h"
#include "Herkulex.h"






class SerialBusManager {
    public:
        static constexpr uint8_t MAX_BUS_COUNT = 5;

        static void createBus(uint8_t serialPort);
        static HerkulexClass& getBus(uint8_t serialPort);

        static void actionAll(int playTimeMs);

        static HerkulexClass _buses[MAX_BUS_COUNT];
        static int _busesTracker[MAX_BUS_COUNT];

        static void startAllBuses(long baud);
        static void initAllMotors();
        static void endAllBuses();

    private:
        

};

#endif