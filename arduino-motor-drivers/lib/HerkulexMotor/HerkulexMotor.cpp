#include "Arduino.h"
#include "HerkulexMotor.h"
#include "Herkulex.h"

HerkulexMotor::HerkulexMotor(int id, MotorModel type){
    _id = id;
    _type = type;

    //place holder: but assigning motor bounds from motor limit data (in steps)
    // because we are using enum class instead of straight enum, we need to cast to an integer, it doesn't do so automatically
    // if we had normal enum doing ModelInfo[type] works (i checked by deleting class from enum Class MotorModel)
    _bounds[0] = ModelInfo[static_cast<int>(type)].minPhysicalDeg;
    _bounds[1] = ModelInfo[static_cast<int>(type)].maxPhysicalDeg;
}

HerkulexMotor::HerkulexMotor(int id, MotorModel type, float lowerBoundDeg, float upperBoundDeg){
    _id = id;
    _type = type;
    
    // specific limits (in steps) -> motor steps implementaton for different motors goes here
    float modelMin = ModelInfo[static_cast<int>(type)].minPhysicalDeg;
    float modelMax = ModelInfo[static_cast<int>(type)].maxPhysicalDeg;

    _bounds[0] = (lowerBoundDeg > modelMin) ? lowerBoundDeg : modelMin;
    _bounds[1] = (upperBoundDeg < modelMax) ? upperBoundDeg : modelMax;
}


// these functions wrap the core Herkulex library functions and convert to degrees (usuable units) from raw HerkuleX motor information
float HerkulexMotor::getPos(){
    // // we are going to need
    // _id;
    // float positionDeg = 0;

    // // Singleton class pattern for HerkulexClass (extern), there is a single global instance Herkulex
    // // Herkulex.getPosition();
    
    // switch(_type) {
    //     case MotorModel::DRS_0201:
    //     break;

    //     case MotorModel::DRS_0601:
    //     break;

    //     case MotorModel::DRS_0602:
    //     break;
    // }
    // return positionDeg;
}

void HerkulexMotor::setPos(float goalPosDeg){
    // must check that the position want to set 
    // get .moveOne
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