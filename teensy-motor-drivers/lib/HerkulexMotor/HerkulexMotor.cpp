#include "HerkulexMotor.h"

// the functions that before were just Herkulex.function are now SerialBusManager::getBus(_busId).function
// what this does is it calls the Serial Bus Manager class, which, with the motor's id, looks at the proper bus instance and calls the function properly on that
// the instancing of the buses allows the function to be called the same way and iterate through the loop of initialized serial buses.

// constructor with normal motor bounds and no zero position offset
HerkulexMotor::HerkulexMotor(int id, MotorModel type, uint8_t busId){
    _id = id;
    _busId = busId;
    _type = type;

    _zeroPos   = ModelInfo[static_cast<int>(type)].zeroSteps;
    _bounds[0] = ModelInfo[static_cast<int>(type)].minSteps;
    _bounds[1] = ModelInfo[static_cast<int>(type)].maxSteps;
}


// constructor with custom bounds and no zero position offset 
HerkulexMotor::HerkulexMotor(int id, MotorModel type, uint8_t busId, float lowerBoundDeg, float upperBoundDeg){
    _id = id;
    _busId = busId;
    _type = type;
    _zeroPos = ModelInfo[static_cast<int>(type)].zeroSteps;

    uint16_t upperBoundSteps = degToSteps(upperBoundDeg, type);
    uint16_t lowerBoundSteps = degToSteps(lowerBoundDeg, type);

    // get the upper bounds specified to each othe actual motor (this is mechanical as opposed to limb-based)
    uint16_t maxUpperBound = ModelInfo[static_cast<int>(type)].maxSteps;
    uint16_t minLowerBound = ModelInfo[static_cast<int>(type)].minSteps;
    
    // make sure that the custom bounds are not outside of the actual motor limits
    _bounds[1] = upperBoundSteps > maxUpperBound ? maxUpperBound : upperBoundSteps;
    _bounds[0] = lowerBoundSteps < minLowerBound ? minLowerBound : lowerBoundSteps;
}


// these functions wrap the core Herkulex library functions and convert to degrees (usuable units) from raw HerkuleX motor information
float HerkulexMotor::getPos(){
    // this function is no longer used with the new requestAll and collectAll but can be used for debugging
    uint16_t rawPos = SerialBusManager::getBus(_busId).getPosition(_id) & ModelInfo[static_cast<int>(_type)].posBitMask;
    return stepsToDeg(rawPos, _type);
}

void HerkulexMotor::setPos(float posDeg){
    int32_t rawPos = degToSteps(posDeg, _type);

    uint16_t boundedPos = boundPos(rawPos);

    int playTimeMs = 100;
    uint8_t playTime = (uint8_t) (playTimeMs / CONVERT_PLAYTIME_TO_MS);
    struct motorMoveInfo moveInfo = {boundedPos, LED_BLUE, _id, playTime};

    // send command to HerkulesX class
    // last argument "1" sets LED to a nice blue color.
    // Max are you sure that 1 wouldn't make the color green? since 2 is blue
    // Pau: 1 Max: 0

    // Pau, there was another issue due to how the Herkulex library (called below)
    // was handling the LED color. It used a poorly maintained switch statement that flipped
    // the logic for green and blue led. It is properly patched, so now, sending 2 (LED_BLUE)
    // will turn the LEDs blue. This is consistent to the datasheet.
    // Pau: 0 Max: 2 (goated parallelization)
    SerialBusManager::getBus(_busId).moveOne(moveInfo);
}

void HerkulexMotor::queueMove(float posDeg){
    int32_t rawPos = degToSteps(posDeg, _type);

    uint16_t boundedPos = boundPos(rawPos);

    // playTime is set to 0 as playTime is set when actionMoves is called
    struct motorMoveInfo moveInfo = {boundedPos, LED_BLUE, _id, 0};

    SerialBusManager::getBus(_busId).queueMove(moveInfo);
}

void HerkulexMotor::reboot(){
    SerialBusManager::getBus(_busId).reboot(_id);
}

MotorRef HerkulexMotor::getMotorRef() const {
    return MotorRef{_busId, _id, _type};
}

float HerkulexMotor::rawToDegs(uint16_t rawPos) {
    uint16_t masked = rawPos & ModelInfo[static_cast<int>(_type)].posBitMask;
    return stepsToDeg(masked, _type);
}


// PRIVATE METHODS
uint16_t HerkulexMotor::boundPos(int32_t rawPos){
    int32_t boundedPos = rawPos;

    boundedPos = boundedPos > _bounds[1] ? _bounds[1] : boundedPos;
    boundedPos = boundedPos < _bounds[0] ? _bounds[0] : boundedPos;

    return (uint16_t) boundedPos;
}


int32_t HerkulexMotor::degToSteps(float deg, MotorModel type){
    const HerkulexMotorSpec& m = ModelInfo[static_cast<int>(type)];    
    return int32_t(deg / m.degPerStep) + (int32_t(m.zeroSteps));
}

float HerkulexMotor::stepsToDeg(uint16_t steps, MotorModel type) {
    const HerkulexMotorSpec& m = ModelInfo[static_cast<int>(type)];
    int32_t centered = (int32_t)steps - (int32_t)m.zeroSteps;
    return m.degPerStep * (float)centered;
}