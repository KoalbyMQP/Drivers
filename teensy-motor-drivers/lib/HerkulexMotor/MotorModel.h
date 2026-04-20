#ifndef MotorModel_h
#define MotorModel_h

#include <stdint.h>

// all motor models used
enum MotorModel{
    DRS_0201,
    DRS_0601,
    DRS_0602,
    DRS_0602_GEARBOX,
    UNKNOWN_MODEL,
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
  {0, 65535, 16384, 0xFFFF, 0.02778f},

  // DRS_0602_GEARBOX
  {0, 65535, 16384, 0xFFFF, 0.00926f},
};


// MotorRef — lightweight descriptor used by requestAllPositions / collectAllPositions.
// Each active motor that you want to read should be registered here.
// This avoids the SerialBusManager needing to know about HerkulexMotor internals.
struct MotorRef {
    uint8_t busId;      // which serial bus this motor lives on
    uint8_t servoId;    // the motor's hardware ID on that bus
    MotorModel type;       // DRS 0601, DRS 0602... etc.,
    uint16_t zeroPos;
    uint16_t boundsMin;
    uint16_t boundsMax;
};


// MotorGroup — a named slice of MotorRef descriptors belonging to one body segment.
// Groups a contiguous array of MotorRefs with its element count so callers can
// iterate without knowing array sizes at compile time.
//
// Typical use: populate an array of MotorGroups (one per bus / limb) and pass it
// to SerialBusManager::requestAllPositions / collectAllPositions instead of
// managing each segment's array separately.
//
// Example:
//   MotorGroup allGroups[] = {
//     { rightLegMotors, BUS_R_LEG_COUNT },
//     { leftLegMotors,  BUS_L_LEG_COUNT },
//     { chestMotors,    BUS_CHEST_COUNT  },
//   };
//
// NOTE: `motors` is a non-owning pointer — the underlying MotorRef array must
// outlive any MotorGroup that references it.
struct MotorGroup {
    MotorRef* motors;
    size_t count;
};


#endif