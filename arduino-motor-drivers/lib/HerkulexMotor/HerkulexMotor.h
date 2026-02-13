#ifndef HerkulexMotor_h
#define HerkulexMotor_h

#include "Arduino.h"

// all motor models used
enum MotorModel{
    DRS_0201,
    DRS_0601,
    DRS_0602,
};


// all specs that motors have
// currently placeholders - look through documenations to determine most important params to include here
struct HerkulexMotorSpec{
    uint16_t minSteps;
    uint16_t maxSteps;
    uint16_t zeroSteps;
    uint16_t zeroPosOffset; // added because 0602 has a default calibration offset
    uint16_t posBitMask;
    float degPerStep;
};


// actual lookup table for motors
const HerkulexMotorSpec ModelInfo[] = {
  // DRS_0201
  {21, 1002, 512, 0, 0x03FF, 0.325f},

  // DRS_0601
  // bounds are recommended range from HerkuleX datasheet
  {42, 2004, 1024, 0, 0x07FF, 0.163f},

  // DRS_0602
  // bounds are for now full supported 16bit int range
  {0, 65535, 16384, 9903, 0xFFFF, 0.02778f}
};

class HerkulexMotor{
    public:
        HerkulexMotor(int id, MotorModel type);
        HerkulexMotor(int id, MotorModel type, float lowerBoundDeg, float upperBoundDeg);
        void setPos(float posDeg);
        float getPos();
        void queueMove(float posDeg);
        // for exectuting all of the queues, Herkulex.actionMoves(x), x is in how many ms
    private:
        int _id;
        MotorModel _type;
        uint16_t _zeroPos;      // zero position in steps
        uint16_t _bounds[2];    // motor bounds, in steps
};

#endif