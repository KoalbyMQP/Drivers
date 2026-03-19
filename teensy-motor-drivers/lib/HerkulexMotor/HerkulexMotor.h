#ifndef HerkulexMotor_h
#define HerkulexMotor_h

#include "Arduino.h"

// all motor models used
enum MotorModel{
    DRS_0201,
    DRS_0601,
    DRS_0602,
};


// all differentiable specs for motors
struct HerkulexMotorSpec{
    uint16_t minSteps;
    uint16_t maxSteps;
    uint16_t zeroSteps;
    uint16_t posBitMask;
    float degPerStep;
};


// actual lookup table for motors
const HerkulexMotorSpec ModelInfo[] = {
  // DRS_0201
  // bounds are reccomended range from HerkuleX datasheet
  {21, 1002, 512, 0x03FF, 0.325f},

  // DRS_0601
  // bounds are recommended range from HerkuleX datasheet
  {42, 2004, 1024, 0x07FF, 0.163f},

  // DRS_0602
  // bounds are  full supported 16bit int range (max what 0602 can read)
  {0, 65535, 16384, 0xFFFF, 0.02778f}
};

class HerkulexMotor{
    public:

        // OLD: replace with constructor with busID
        // HerkulexMotor(int id, MotorModel type);
        // HerkulexMotor(int id, MotorModel type, float lowerBoundDeg, float upperBoundDeg);

        HerkulexMotor(int id, MotorModel type, uint8_t busId);
        HerkulexMotor(int id, MotorModel type, uint8_t busId, float lowerBoundDeg, float upperBoundDeg);
        
        void setPos(float posDeg);
        float getPos();
        void queueMove(float posDeg);
        void reboot();
        static void actionMoves(int playTimeMs); // here we can manage how we action the moves if we switch to multiple serial ports
        MotorRef getMotorRef() const;   // generate the MotorRef table itself so that it can be automated
        float rawToDegs(uint16_t rawPos);
    private:
        uint8_t _id;
        uint8_t _busId;
        MotorModel _type;
        uint16_t _bounds[2];    // motor bounds, in steps
        uint16_t _zeroPos;      // zero position in steps
        
        int32_t degToSteps(float deg, MotorModel type);
        uint16_t boundPos(int32_t rawPos);
        float stepsToDeg(uint16_t steps, MotorModel type);
};

#endif