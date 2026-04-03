#ifndef MotorModel_h
#define MotorModel_h

#include <stdint.h>

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


// MotorRef — lightweight descriptor used by requestAllPositions / collectAllPositions.
// Each active motor that you want to read should be registered here.
// This avoids the SerialBusManager needing to know about HerkulexMotor internals.
struct MotorRef {
    uint8_t busId;      // which serial bus this motor lives on
    uint8_t servoId;    // the motor's hardware ID on that bus
    MotorModel type;       // DRS 0601, DRS 0602... etc.,
};

#endif