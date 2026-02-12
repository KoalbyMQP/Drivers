#ifndef HerkulexMotor_h
#define HerkulexMotor_h

#include "Arduino.h"

// all motor models used
enum class MotorModel{
    DRS_0201,
    DRS_0601,
    DRS_0602,
};


// all specs that motors have
// currently placeholders - look through documenations to determine most important params to include here
struct HerkulexMotorSpec{
    uint16_t maxRaw;
    uint16_t centerRaw;
    float degPerTick;
    float minPhysicalDeg;
    float maxPhysicalDeg;
};


// actual lookup table for motors
// dummy data for now
const HerkulexMotorSpec ModelInfo[] = {
  // DRS_0201
  {1023, 512, -160.0f, 160.0f, 0.325f},

  // DRS_0601
  {2047, 512, -160.0f, 160.0f, 0.325f},

  // DRS_0602
  {4095, 2048, -180.0f, 180.0f, 0.088f}
};

class HerkulexMotor{
    public:
        HerkulexMotor(int id, MotorModel type);
        HerkulexMotor(int id, MotorModel type, float lowerBoundDeg, float upperBoundDeg);
        void setPos(float goalPosDeg);
        float getPos();
        void queueMove();
    private:
        int _id;
        MotorModel _type;
        int _zeroPos;
        int _bounds[2]; // we should consider whether we want these as raw values or keep them as degrees 
};

#endif