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
    _bounds[0] = int(upperBoundDeg / ModelInfo[static_cast<int>(type)].degPerStep
         - ModelInfo[static_cast<int>(type)].zeroPosOffset);
    _bounds[1] = int(lowerBoundDeg / ModelInfo[static_cast<int>(type)].degPerStep
        - ModelInfo[static_cast<int>(type)].zeroPosOffset);
}


// these functions wrap the core Herkulex library functions and convert to degrees (usuable units) from raw HerkuleX motor information
float HerkulexMotor::getPos(){
    
    uint16_t rawPos = Herkulex.getPosition(_id);

    float posDeg = ModelInfo[static_cast<int>(_type)].degPerStep *
     (rawPos + ModelInfo[static_cast<int>(_type)].zeroPosOffset);

    return posDeg;

}

void HerkulexMotor::setPos(float posDeg){
    
    // calculate raw position from degrees

    uint16_t rawPos = int(posDeg / ModelInfo[static_cast<int>(_type)].degPerStep
        - ModelInfo[static_cast<int>(_type)].zeroPosOffset);


    // bound raw position to motor limits
    rawPos = rawPos > _bounds[1] ? _bounds[1] : rawPos;
    rawPos = rawPos < _bounds[0] ? _bounds[0] : rawPos;


    // send command to HerkuleX class
    Herkulex.moveOne(_id, rawPos, 10, 0);

}
