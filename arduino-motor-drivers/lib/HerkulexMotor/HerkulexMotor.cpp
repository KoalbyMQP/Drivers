#include "Arduino.h"
#include "HerkulexMotor.h"
#include "Herkulex.h"



// TODO: implement zero position offset

// constructor with normal motor bounds and no zero position offset
HerkulexMotor::HerkulexMotor(int id, MotorModel type){
    _id = id;
    _type = type;

    //place holder: but assigning motor bounds from motor limit data (in steps)
    // because we are using enum class instead of straight enum, we need to cast to an integer, it doesn't do so automatically
    // if we had normal enum doing ModelInfo[type] works (i checked by deleting class from enum Class MotorModel)
    _zeroPos   = ModelInfo[static_cast<int>(type)].zeroSteps;
    _bounds[0] = ModelInfo[static_cast<int>(type)].minSteps;
    _bounds[1] = ModelInfo[static_cast<int>(type)].maxSteps;
}


// constructor with custom bounds and no zero position offset 
HerkulexMotor::HerkulexMotor(int id, MotorModel type, float lowerBoundDeg, float upperBoundDeg){
    _id = id;
    _type = type;
   
    _zeroPos   = ModelInfo[static_cast<int>(type)].zeroSteps;

    // Upper and lower bounds based on the configuration of the motor, thes might be based on limb interferances and the such, not necessarily mechanical motor limits
    // conversion to take from degrees to raw steps
    uint16_t upperBoundSteps = int(upperBoundDeg / ModelInfo[static_cast<int>(type)].degPerStep
         - ModelInfo[static_cast<int>(type)].zeroPosOffset);
         
    uint16_t lowerBoundSteps = int(lowerBoundDeg / ModelInfo[static_cast<int>(type)].degPerStep
        - ModelInfo[static_cast<int>(type)].zeroPosOffset);

    // get the upper bounds specified to each othe actual motor (this is mechanical as opposed to limb-based)
    uint16_t actualUpperBound = ModelInfo[static_cast<int>(type)].maxSteps;
    uint16_t actualLowerBound = ModelInfo[static_cast<int>(type)].minSteps;
    
    // make sure that the custom bounds are not outside of the actual motor limits
    _bounds[1] = upperBoundSteps > actualUpperBound ? actualUpperBound : upperBoundSteps;
    _bounds[0] = lowerBoundSteps < actualLowerBound ? actualLowerBound : lowerBoundSteps;
}


// these functions wrap the core Herkulex library functions and convert to degrees (usuable units) from raw HerkuleX motor information
float HerkulexMotor::getPos(){
    
    uint16_t rawPos = Herkulex.getPosition(_id) & ModelInfo[static_cast<int>(_type)].posBitMask;

    float posDeg = ModelInfo[static_cast<int>(_type)].degPerStep *
     (rawPos - ModelInfo[static_cast<int>(_type)].zeroPosOffset - ModelInfo[static_cast<int>(_type)].zeroSteps);
    return posDeg;
}

void HerkulexMotor::setPos(float posDeg){
    // calculate raw position from degrees
    uint16_t rawPos = int(posDeg / ModelInfo[static_cast<int>(_type)].degPerStep
         + ModelInfo[static_cast<int>(_type)].zeroSteps);

    // bound raw position to motor limits
    rawPos = rawPos > _bounds[1] ? _bounds[1] : rawPos;
    rawPos = rawPos < _bounds[0] ? _bounds[0] : rawPos;


    // send command to HerkulesX class
    // last argument "1" sets LED to a nice blue color.
    //  Max are you sure that 1 wouldn't make the color green? since 2 is blue
    Herkulex.moveOne(_id, rawPos, 10, 2);
}

void HerkulexMotor::queueMove(float posDeg){
    uint16_t rawPos = int(posDeg / ModelInfo[static_cast<int>(_type)].degPerStep
         + ModelInfo[static_cast<int>(_type)].zeroSteps);

    // bound raw position to motor limits
    rawPos = rawPos > _bounds[1] ? _bounds[1] : rawPos;
    rawPos = rawPos < _bounds[0] ? _bounds[0] : rawPos;

    // use the Herkulex queuing system to add that movement to the list to be exectuted simultaneously
    // 3 makes the LED red, to differentiate
    Herkulex.queueMoves(_id, rawPos, 3);
}