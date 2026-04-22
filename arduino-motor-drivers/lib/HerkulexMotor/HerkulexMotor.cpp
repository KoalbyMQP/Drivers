#include "Arduino.h"
#include "HerkulexMotor.h"
#include "Herkulex.h"

// Constructor for standard range motors (DRS-0201, DRS-0601)
HerkulexMotor::HerkulexMotor(int id, MotorModel type, uint8_t bus) {
    _id = id;
    _type = type;
    _bus = bus; // Stores 1 for Serial1 or 2 for Serial2

    _zeroPos   = ModelInfo[static_cast<int>(type)].zeroSteps;
    _bounds[0] = ModelInfo[static_cast<int>(type)].minSteps;
    _bounds[1] = ModelInfo[static_cast<int>(type)].maxSteps;
}

// Constructor with custom degree boundaries
HerkulexMotor::HerkulexMotor(int id, MotorModel type, uint8_t bus, float lowerBoundDeg, float upperBoundDeg) {
    _id = id;
    _type = type;
    _bus = bus;
   
    _zeroPos = ModelInfo[static_cast<int>(type)].zeroSteps;

    int32_t upperBoundSteps = degToSteps(upperBoundDeg, type);
    int32_t lowerBoundSteps = degToSteps(lowerBoundDeg, type);

    int32_t actualUpperBound = ModelInfo[static_cast<int>(type)].maxSteps;
    int32_t actualLowerBound = ModelInfo[static_cast<int>(type)].minSteps;
    
    _bounds[1] = (upperBoundSteps > actualUpperBound) ? actualUpperBound : (uint16_t)upperBoundSteps;
    _bounds[0] = (lowerBoundSteps < actualLowerBound) ? actualLowerBound : (uint16_t)lowerBoundSteps;
}

float HerkulexMotor::getPos() {
    // Switch the global Herkulex library to this motor's specific bus
    Herkulex.setPort(_bus); 
    
    // Request position and apply the bitmask (0x03FF, 0x07FF, or 0xFFFF)
    uint16_t rawPos = Herkulex.getPosition(_id) & ModelInfo[static_cast<int>(_type)].posBitMask;
    
    return stepsToDeg(rawPos, _type);
}

void HerkulexMotor::setPos(float posDeg) {
    Herkulex.setPort(_bus); 

    int32_t rawPos = degToSteps(posDeg, _type);

    // Constrain the movement to the defined motor bounds
    if (rawPos > _bounds[1]) rawPos = _bounds[1];
    if (rawPos < _bounds[0]) rawPos = _bounds[0];

    // moveOne uses 10 for playtime and 2 for Blue LED
    Herkulex.moveOne(_id, (uint16_t)rawPos, 10, 2); 
}

void HerkulexMotor::queueMove(float posDeg) {
    int32_t rawPos = degToSteps(posDeg, _type);
    if (rawPos > _bounds[1]) rawPos = _bounds[1];
    if (rawPos < _bounds[0]) rawPos = _bounds[0];
    _pendingGoal = (uint16_t)rawPos;
    _hasPending = true;
}

void HerkulexMotor::reboot() {
    Herkulex.setPort(_bus);
    Herkulex.reboot(_id);
}

// Static function to initialize all buses
void HerkulexMotor::initialize() {
    // Initialize Bus 1
    Herkulex.setPort(1);
    Herkulex.initialize();
    
    // Initialize Bus 2
    Herkulex.setPort(2);
    Herkulex.initialize();
}

// Static function to trigger all queued moves on all buses
void HerkulexMotor::actionMoves(int pTime, HerkulexMotor** motors, uint8_t count) {
    // Bus 1 - queue all pending moves on bus 1 then fire
    Herkulex.setPort(1);
    for (uint8_t i = 0; i < count; i++) {
        if (motors[i]->_bus == 1 && motors[i]->_hasPending) {
            Herkulex.queueMoves(motors[i]->_id, motors[i]->_pendingGoal, 3);
            motors[i]->_hasPending = false;
        }
    }
    Herkulex.actionMoves(pTime);

    // Bus 2 - queue all pending moves on bus 2 then fire
    Herkulex.setPort(2);
    for (uint8_t i = 0; i < count; i++) {
        if (motors[i]->_bus == 2 && motors[i]->_hasPending) {
            Herkulex.queueMoves(motors[i]->_id, motors[i]->_pendingGoal, 3);
            motors[i]->_hasPending = false;
        }
    }
    Herkulex.actionMoves(pTime);
}

// Static function to start the hardware serial ports
void HerkulexMotor::initSerialPorts(uint32_t baudRate) {
    Herkulex.beginSerial1(baudRate); // Pins 18, 19
    Herkulex.beginSerial2(baudRate); // Pins 16, 17
}

// Internal helper: Converts float degrees to raw integer steps
int32_t HerkulexMotor::degToSteps(float deg, MotorModel type) {
    const HerkulexMotorSpec& m = ModelInfo[static_cast<int>(type)];    
    return (int32_t)(deg / m.degPerStep) + (int32_t)m.zeroSteps;
}

// Internal helper: Converts raw steps back to float degrees
float HerkulexMotor::stepsToDeg(uint16_t steps, MotorModel type) {
    const HerkulexMotorSpec& m = ModelInfo[static_cast<int>(type)];
    
    // Use signed math to handle positions "below" the zero point correctly
    int32_t centered = (int32_t)steps - (int32_t)m.zeroSteps;
    return m.degPerStep * (float)centered;
}