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

            // then start it
            SerialBusManager::_buses[i].beginSerialBus(baud);
            delayMicroseconds(10);
        }
    }
    delayMicroseconds(100);
}

void SerialBusManager::endAllBuses(){
    for (int i = 0; i < SerialBusManager::MAX_BUS_COUNT; i++){

        // if we have a serial port created
        if (SerialBusManager::_busesTracker[i] == 1){

            // then end it
            SerialBusManager::_buses[i].endSerialBus();
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
            SerialBusManager::_buses[busIndex].sendPosRequest(motors[i].servoId);
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

// getAllPositionsParallel — cross-bus parallel, per-bus serial position read.
//
// Each Herkulex bus is a shared half-duplex line: only one request/reply
// exchange may be in flight on a bus at a time.  Calling sendPosRequest()
// twice on the same bus before collecting resets inputBuffer and loses the
// first reply, so within a bus the sequence must be strictly serial:
//
//   request motor[0] → collect motor[0] → request motor[1] → collect motor[1] ...
//
// But separate buses are fully independent — a motor on bus 2 begins its reply
// as soon as it receives its request packet, regardless of what bus 1 is doing.
// So the goal is to run each bus's serial sequence in parallel with the others.
//
// Implementation: round-robin state machine
// -----------------------------------------
// Each bus has a BusQueue with a 'waiting' flag indicating whether a request
// is currently in flight.  On every tick of the main loop, all buses are visited:
//
//   IDLE bus (waiting == false, motors remaining):
//     Fire the next sendPosRequest(), set waiting = true.
//
//   WAITING bus:
//     Call updateRead() to poll the hardware UART buffer without blocking.
//     updateRead() clears readPending when either the full reply has arrived
//     OR the timeout has elapsed — in both cases it is then safe to collect.
//     If readPending has been cleared (reply ready or timed out), call
//     collectPosition() and set waiting = false so the bus advances next tick.
//     If readPending is still set, leave waiting = true and revisit next tick.
//
// Because readPending is a private field, we infer its state indirectly:
// requestRead() always sets it before we enter the loop, and updateRead()
// always clears it (on data or timeout).  We track our own 'replyReady' flag
// by comparing serial availability before/after — but the simplest and correct
// approach is just to call updateRead() every tick and then collectPosition(),
// since collectPosition() is stateless with respect to readPending: it reads
// whatever is in inputBuffer and returns 0xFFFF on a bad checksum.  A timeout
// leaves inputBuffer with stale/zero bytes, which will fail the checksum and
// produce 0xFFFF, which is the established error sentinel for this API.
//
// The loop exits when every bus has collected all its motors.  Total wall-clock
// time is bounded by the busiest bus, not the sum across all buses.
void SerialBusManager::getAllPositionsParallel(const MotorRef* motors, uint16_t* results, uint8_t count) {

    // ------------------------------------------------------------------
    // Build per-bus queues (stack-allocated, no heap)
    // ------------------------------------------------------------------
    BusQueue queues[MAX_BUS_COUNT];
    memset(queues, 0, sizeof(queues));

    for (uint8_t i = 0; i < count; i++) {
        uint8_t busIndex = motors[i].busId - 1;

        if (busIndex >= MAX_BUS_COUNT || _busesTracker[busIndex] != 1) {
            results[i] = 0xFFFF;  // uninitialised or out-of-range bus
            continue;
        }

        BusQueue& q = queues[busIndex];
        if (q.total >= BusQueue::MAX_MOTORS_PER_BUS) {
            results[i] = 0xFFFF;  // more motors on this bus than the hard cap
            continue;
        }

        q.resultIndices[q.total] = i;
        q.servoIds[q.total]      = motors[i].servoId;
        q.total++;
    }

    // ------------------------------------------------------------------
    // Round-robin state machine
    // ------------------------------------------------------------------
    bool anyBusActive = true;
    while (anyBusActive) {
        anyBusActive = false;

        for (uint8_t busIndex = 0; busIndex < MAX_BUS_COUNT; busIndex++) {
            BusQueue& q = queues[busIndex];

            if (q.allDone()) continue;
            anyBusActive = true;

            if (q.waiting) {
                // Request is in flight — poll the UART buffer non-blocking.
                // updateRead() latches bytes into inputBuffer once the full reply
                // arrives, or clears readPending on timeout.
                _buses[busIndex].updateRead();

                // Only collect once the reply has settled (data or timeout).
                // If the bytes haven't arrived yet, leave waiting = true and
                // revisit this bus on the next tick — meanwhile other buses
                // make forward progress.
                if (_buses[busIndex].isReplyReady()) {
                    uint8_t resultIdx = q.resultIndices[q.nextCollect];
                    uint8_t servoId   = q.servoIds[q.nextCollect];
                    results[resultIdx] = _buses[busIndex].collectPosition(servoId);

                    q.nextCollect++;
                    q.waiting = false;
                    // Bus is now IDLE — next tick will send the following request.
                }

            } else if (q.readyToSend()) {
                // Bus is idle and has another motor to query.
                _buses[busIndex].sendPosRequest(q.servoIds[q.nextSend]);
                q.nextSend++;
                q.waiting = true;
            }
        }
    }
}
