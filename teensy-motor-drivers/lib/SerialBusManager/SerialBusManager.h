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

                // getAllPositionsParallel — reads positions from all motors across all buses
        // with true cross-bus parallelism while keeping each bus strictly serial.
        //
        // Each bus can only have one request/reply in flight at a time — sending a
        // second request before collecting the first would clobber the input buffer.
        // So within each bus the sequence is serial: request motor[N] → collect
        // motor[N] → request motor[N+1] → collect motor[N+1] → ...
        //
        // Across buses, those serial sequences are overlapped: while bus 1 is waiting
        // for a reply, bus 2's request is sent and it starts replying, and so on.
        //
        // This is implemented as a round-robin state machine.  Each bus tracks whether
        // it is IDLE (ready to send the next request) or WAITING (request sent, reply
        // not yet received).  On every tick all buses are visited: IDLE buses fire
        // their next request, WAITING buses call updateRead() and collect if ready.
        // The loop exits when every bus has collected every one of its motors.
        //
        // Parameters
        //   motors  – array of MotorRef descriptors ordered however suits the caller.
        //             Motors on the same bus are grouped internally; the input order
        //             does not need to be sorted by bus.
        //   results – caller-allocated uint16_t array, length >= count.
        //             results[i] is the raw position for motors[i], or 0xFFFF on
        //             an uninitialised bus, oversubscribed bus, or reply timeout.
        //   count   – number of entries in both arrays
        static void getAllPositionsParallel(const MotorRef* motors, uint16_t* results, uint8_t count);
    private:

            // Per-bus motor queue, used internally by getAllPositionsParallel.
        // Holds the ordered list of motors to request/collect on one bus, plus the
        // state needed by the round-robin loop.
        struct BusQueue {
            // Hard cap: 10 motors per bus matches the DATA_MOVE/5 limit already
            // enforced by the Herkulex library for simultaneous jog packets.
            static constexpr uint8_t MAX_MOTORS_PER_BUS = 10;

            uint8_t resultIndices[MAX_MOTORS_PER_BUS]; // index into caller's results[]
            uint8_t servoIds[MAX_MOTORS_PER_BUS];      // servo ID for each slot
            uint8_t total;      // number of motors queued on this bus
            uint8_t nextSend;   // index of the next motor to request  (0..total)
            uint8_t nextCollect;// index of the next motor to collect  (0..total)
            bool    waiting;    // true while a request is in flight, awaiting reply

            bool allDone()     const { return nextCollect >= total; }
            bool readyToSend() const { return !waiting && nextSend < total; }
        };

        
};


#endif