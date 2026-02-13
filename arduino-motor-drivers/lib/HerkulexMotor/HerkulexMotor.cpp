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




    uint16_t upperBoundSteps = int(upperBoundDeg / ModelInfo[static_cast<int>(type)].degPerStep
         - ModelInfo[static_cast<int>(type)].zeroPosOffset);
    _bounds[1] = int(lowerBoundDeg / ModelInfo[static_cast<int>(type)].degPerStep
        - ModelInfo[static_cast<int>(type)].zeroPosOffset);
}


// these functions wrap the core Herkulex library functions and convert to degrees (usuable units) from raw HerkuleX motor information
float HerkulexMotor::getPos(){
    
    uint16_t rawPos = Herkulex.getPosition(_id);

    float posDeg = ModelInfo[static_cast<int>(_type)].degPerStep *
     (rawPos - ModelInfo[static_cast<int>(_type)].zeroPosOffset - ModelInfo[static_cast<int>(_type)].zeroSteps);
    return posDeg;

}

void HerkulexMotor::setPos(float posDeg){
    
    // calculate raw position from degrees

    uint16_t rawPos = int(posDeg / ModelInfo[static_cast<int>(_type)].degPerStep
        + ModelInfo[static_cast<int>(_type)].zeroPosOffset + ModelInfo[static_cast<int>(_type)].zeroSteps);

    // bound raw position to motor limits
    rawPos = rawPos > _bounds[1] ? _bounds[1] : rawPos;
    rawPos = rawPos < _bounds[0] ? _bounds[0] : rawPos;


    // send command to HerkulesX class
    // last argument "1" sets LED to a nice blue color.
    Herkulex.moveOne(_id, rawPos, 10, 1);

}



// // Notes // //

// How will this play with the structure we have? are we going to want to do this instead of individually setPos?
// Do we have too many 

// //queuing up movements for different motors
// Herkulex.queueMoves(1, 200, LED_GREEN);  // Queue motor 1
// Herkulex.queueMoves(2, 800, LED_GREEN);  // Queue motor 2
// Herkulex.queueMoves(3, 512, LED_GREEN);  // Queue motor 3

// //execute them all at once
// Herkulex.actionMoves(1500);  // All motors move together, taking 1500ms