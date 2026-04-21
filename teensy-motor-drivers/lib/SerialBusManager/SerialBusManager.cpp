#include "SerialBusManager.h"

HerkulexClass SerialBusManager::_buses[SerialBusManager::MAX_BUS_COUNT];
int SerialBusManager::_busesTracker[SerialBusManager::MAX_BUS_COUNT];
uint8_t SerialBusManager::busDoneCount = 0;
uint8_t SerialBusManager::activeBusCount = 0;
bool SerialBusManager::doneCollecting = false;
SerialBusManager::BusQueue SerialBusManager::queues[SerialBusManager::MAX_BUS_COUNT];


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

void SerialBusManager::infoAllMotors(const MotorRef* motors, uint8_t count){
    uint8_t stat_error;
    uint8_t stat_detail;
    uint16_t model;

    for (int i = 0; i < SerialBusManager::MAX_BUS_COUNT; i++){
        if (SerialBusManager::_busesTracker[i] == 1){
            Serial.print("\n[Bus ");
            Serial.print(i + 1);
            Serial.println("]:");

            for (int motor_idx = 0; motor_idx < count; motor_idx++) {
                if (motors[motor_idx].busId != (i + 1)) continue; // do as per-bus report

                Serial.print("  Motor ID: ");
                if (motors[motor_idx].servoId < 10){
                    Serial.print(0);
                }
                if (motors[motor_idx].servoId < 100){
                    Serial.print(0);
                }
                Serial.print(motors[motor_idx].servoId);

                if (!SerialBusManager::_buses[i].getModel(motors[motor_idx].servoId, &model)){
                    model = 0xFFFF;
                };

                delay(10);

                Serial.print(" Model: DRS-0");
                Serial.print(model, HEX);
                Serial.print(" ");


                if(SerialBusManager::_buses[i].stat(motors[motor_idx].servoId, &stat_error, &stat_detail)){
                    delay(10);
                    Serial.print(" STAT_ERROR:  0x");
                    Serial.print(stat_error, HEX);
                    if (stat_error == 0x0){
                        Serial.print(0);
                    }
                    Serial.print(" Meaning: ");
                    if (stat_error == STATUS_ERROR_TYPE::H_STATUS_OK)             Serial.print("│ No errors               │ ");
                    if (stat_error &  STATUS_ERROR_TYPE::H_EXCEED_INPUT_VOLTAGE)  Serial.print("│ Input voltage exceeded  │ ");
                    if (stat_error &  STATUS_ERROR_TYPE::H_EXCEED_POT_LIMIT)      Serial.print("│ Pot limit exceeded      │ ");
                    if (stat_error &  STATUS_ERROR_TYPE::H_EXCEED_TEMP_LIMIT)     Serial.print("│ Temperature exceeded    │ ");
                    if (stat_error &  STATUS_ERROR_TYPE::H_INVALID_PACKET)        Serial.print("│ Invalid packet          │ ");
                    if (stat_error &  STATUS_ERROR_TYPE::H_OVERLOAD_DETECTED)     Serial.print("│ Overload detected       │ ");
                    if (stat_error &  STATUS_ERROR_TYPE::H_EEP_REG_DISTORTED)     Serial.print("│ EEP register distorted  │ ");
                    
                    Serial.println();
                    Serial.print("                                 STAT_DETAIL: 0x");
                    Serial.print(stat_detail, HEX);
                    Serial.print(" Meaning: ");
                    if (stat_detail == STATUS_DETAIL::H_NO_DETAILS)               Serial.print("│ No Error                │ ");
                    if (stat_detail &  STATUS_DETAIL::H_MOVING_FLAG)              Serial.print("│ Moving                  │ ");
                    if (stat_detail &  STATUS_DETAIL::H_INPOSITION_FLAG)          Serial.print("│ In position             │ ");
                    if (stat_detail &  STATUS_DETAIL::H_CHECKSUM_ERROR)           Serial.print("│ Checksum error          │ ");
                    if (stat_detail &  STATUS_DETAIL::H_UNKNOWN_COMMAND)          Serial.print("│ Unknown command         │ ");
                    if (stat_detail &  STATUS_DETAIL::H_EXCEED_REG_RANGE)         Serial.print("│ Register range exceeded │ ");
                    if (stat_detail &  STATUS_DETAIL::H_GARBAGE_DETECTED)         Serial.print("│ Garbage detected        │ ");
                    if (stat_detail &  STATUS_DETAIL::H_TORQUE_ON)                Serial.print("│ Torque on               │ ");
                    Serial.println();
                }
                else{
                    Serial.println("NO STAT RESPONSE");
                }
            }
        }
    }
}


HerkulexClass& SerialBusManager::getBus(uint8_t serialPort){

    if ((serialPort < 1) || (serialPort > SerialBusManager::MAX_BUS_COUNT)) return;

    return SerialBusManager::_buses[serialPort - 1];
}

void SerialBusManager::updateBaudRateWithReport(const MotorRef* motors, uint8_t count, BAUD_RATE baud){
    Serial.println("PRE-baud rate update status:");
    infoAllMotors(motors, count);
    
    for (int i = 0; i < SerialBusManager::MAX_BUS_COUNT; i++) {

        // if we have a serial port created
        if (SerialBusManager::_busesTracker[i] == 1) {
            
            // update baud rate of motors
            SerialBusManager::_buses[i].setBaudRate(baud);

        }
    }

    Serial.println();
    Serial.print("POST-baud rate update status (now ");
    Serial.print(BAUD_RATE_MAP.at(baud));
    Serial.println(" bps):");
    infoAllMotors(motors, count);
}

void SerialBusManager::startAllBuses(BAUD_RATE baud){
    for (int i = 0; i < SerialBusManager::MAX_BUS_COUNT; i++){

        // if we have a serial port created
        if (SerialBusManager::_busesTracker[i] == 1){

            // then start it
            SerialBusManager::_buses[i].beginSerialBus(BAUD_RATE_MAP.at(baud));
            delay(10);
        }
    }
    delay(10);
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

void SerialBusManager::torqueOnAllMotors(){
        for (int i = 0; i < SerialBusManager::MAX_BUS_COUNT; i++){

        // if we have a serial port created
        if (SerialBusManager::_busesTracker[i] == 1){

            // then torque on the motors on it
            SerialBusManager::_buses[i].torqueON(PACKET_CONSTS::ALL_SERVOS);
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

bool SerialBusManager::isDoneCollecting(){
    return doneCollecting;
}

void SerialBusManager::tick(const MotorRef* motors, uint16_t* results, uint8_t count){
    // loop through all the serial buses
    busDoneCount = 0;
    if(!doneCollecting){
        for (uint8_t busIndex = 0; busIndex < MAX_BUS_COUNT; busIndex++) {
            // store reference to bus in q
            BusQueue& q = queues[busIndex];

            // exit if the bus is done (no more collects)
            if (q.allDone()){
                if (q.total > 0) busDoneCount++;
                continue;
            }

            // if we are waiting for a response
            if (q.waiting) {

                // update the uart read
                _buses[busIndex].updateRead();

                // if there is a reply sitting in the uart buffer
                if (_buses[busIndex].isReplyReady()) {
                    // resultIdx is the index of where to put the motor position back into results
                    uint8_t resultIdx = q.resultIndices[q.nextCollect];
                    uint8_t servoId   = q.servoIds[q.nextCollect];

                    results[resultIdx] = _buses[busIndex].collectPosition(servoId);


                    // move onto the next motor to collect
                    q.nextCollect++;
                    q.waiting = false; // no longer waiting for a response
                }

            } else if (q.readyToSend()) {
                // Bus is idle and has another motor to query.
                _buses[busIndex].sendPosRequest(q.servoIds[q.nextSend]);
                q.nextSend++;
                q.waiting = true;
            }
        }

        if(busDoneCount >= activeBusCount){
            doneCollecting = true;
        }
    }
}

// we need to give the motorrefs and the results to insert error flags into position readings
void SerialBusManager::requestAllPositions(const MotorRef* motors, uint16_t* results, uint8_t count){
    // build per-bus queues
    memset(queues, 0, sizeof(queues));

    // iterate through all of the motors sent as MotorRef objects
    for (uint8_t i = 0; i < count; i++) {

        // get bus that the motor is on
        uint8_t busIndex = motors[i].busId - 1;

        // if the bus isn't initiailized or the bus is out of range
        if (busIndex >= MAX_BUS_COUNT || _busesTracker[busIndex] != 1) {
            results[i] = 0xF0F0;  // preload results for motor with flag value
            continue;
        }

        // if the bus is all good, then get the BusQueue object for the motor's bus
        BusQueue& q = queues[busIndex];
        
        // if there are more motors on the bus than the bus can handle
        if (q.total >= BusQueue::MAX_MOTORS_PER_BUS) {
            results[i] = 0xF1F1;  // preload results for motor with flag value
            continue;
        }

        // if everything is fine, then set the motor data
        q.resultIndices[q.total] = i;
        q.servoIds[q.total] = motors[i].servoId;
        q.total++;
    }

    activeBusCount = 0;
    for (uint8_t i = 0; i < MAX_BUS_COUNT; i++){
        if (queues[i].total > 0) activeBusCount++;
    }

    doneCollecting = false;
}



// LEGACY
// not needed, tick() collects all positions
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


// LEGACY
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

    // iterate through all of the motors sent as MotorRef objects
    for (uint8_t i = 0; i < count; i++) {

        // get bus that the motor is on
        uint8_t busIndex = motors[i].busId - 1;

        // if the bus isn't initiailized or the bus is out of range
        if (busIndex >= MAX_BUS_COUNT || _busesTracker[busIndex] != 1) {
            results[i] = 0xF0F0;  // preload results for motor with flag value
            continue;
        }

        // if the bus is all good, then get the BusQueue object for the motor's bus
        BusQueue& q = queues[busIndex];
        
        // if there are more motors on the bus than the bus can handle
        if (q.total >= BusQueue::MAX_MOTORS_PER_BUS) {
            results[i] = 0xF1F1;  // preload results for motor with flag value
            continue;
        }

        // if everything is fine, then set the motor data
        q.resultIndices[q.total] = i;
        q.servoIds[q.total] = motors[i].servoId;
        q.total++;
    }

    // go through blocking state machine
    
    // we know all buses are active at beginning
    bool anyBusActive = true;
    while (anyBusActive) {

        // lower active flag and let logic in the loop control the active flag
        anyBusActive = false;

        // loop through all the serial buses
        for (uint8_t busIndex = 0; busIndex < MAX_BUS_COUNT; busIndex++) {

            // store reference to bus in q
            BusQueue& q = queues[busIndex];

            // exit if the bus is done (no more collects)
            if (q.allDone()) continue;

            // if the bus isn't done, we have an active bus
            anyBusActive = true;

            // if we are waiting for a response
            if (q.waiting) {

                // update the uart read
                _buses[busIndex].updateRead();

                // if there is a reply sitting in the uart buffer
                if (_buses[busIndex].isReplyReady()) {
                    // resultIdx is the index of where to put the motor position back into results
                    uint8_t resultIdx = q.resultIndices[q.nextCollect];
                    uint8_t servoId   = q.servoIds[q.nextCollect];

                    results[resultIdx] = _buses[busIndex].collectPosition(servoId);


                    // move onto the next motor to collect
                    q.nextCollect++;
                    q.waiting = false; // no longer waiting for a response
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
