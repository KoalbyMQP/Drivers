#ifndef SerialBusManager_h
#define SerialBusManager_h
#include "Arduino.h"
#include "Herkulex.h"

// MotorRef — lightweight descriptor used by requestAllPositions / collectAllPositions.
// Each active motor that you want to read should be registered here.
// This avoids the SerialBusManager needing to know about HerkulexMotor internals.
struct MotorRef {
    uint8_t busId;      // which serial bus this motor lives on
    uint8_t servoId;    // the motor's hardware ID on that bus
};

class SerialBusManager {
    public:
        // MAX_BUS_COUNT is 8 because that is the maximum number of serial ports the teensy can take
        // not all of them will always be in use, it's up to the main to initialize each one
        static constexpr uint8_t MAX_BUS_COUNT = 8; 

        static void createBus(uint8_t serialPort);
        static HerkulexClass& getBus(uint8_t serialPort);

        static void actionAll(int playTimeMs);

        static HerkulexClass _buses[MAX_BUS_COUNT];
        static int _busesTracker[MAX_BUS_COUNT];

        static void startAllBuses(long baud);
        static void initAllMotors();
        static void endAllBuses();

        static void requestAllPositions(const MotorRef* motors, uint8_t count);
        static void collectAllPositions(const MotorRef* motors, uint16_t* results, uint8_t count);
    private:
};


#endif