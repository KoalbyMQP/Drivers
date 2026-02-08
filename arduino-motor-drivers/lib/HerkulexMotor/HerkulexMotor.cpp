#include "Arduino.h"
#include "HerkulexMotor.h"
#include "Herkulex.h"

HerkulexMotor::HerkulexMotor(int id, MotorModel type){
    _id = id;
    _type = type;

    //place holder: but assigning motor bounds from motor limit data (in steps)
    _bounds[0] = ModelInfo[type][4];
    _bounds[1] = ModelInfo[type][5]
}

HerkulexMotor::HerkulexMotor(int id, MotorModel type, float lowerBoundDeg, float upperBoundDeg):{
    _id = id;
    _type = type;
    // specific limits (in steps) -> motor steps implementaton for different motors goes here
}


// these functions wrap the core Herkulex library functions and convert to degrees (usuable units) from raw HerkuleX motor information
int HerkulexMotor::getPos(){

}

void HerkulexMotor::setPos(){

}
