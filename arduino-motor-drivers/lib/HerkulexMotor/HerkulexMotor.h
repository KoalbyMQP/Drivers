#ifndef HerkulexMotor_h
#define HerkulexMotor_h

#include "Arduino.h"

enum MotorModel{
    DRS_0201,
    DRS_0601,
    DRS_0602,
};

struct HerkulexMotorSpec{
    uint16_t minSteps;
    uint16_t maxSteps;
    uint16_t zeroSteps;
    uint16_t posBitMask;
    float degPerStep;
};

const HerkulexMotorSpec ModelInfo[] = {
  {21, 1002, 512, 0x03FF, 0.325f},    // DRS_0201
  {42, 2004, 1024, 0x07FF, 0.163f},   // DRS_0601
  {0, 65535, 16384, 0xFFFF, 0.02778f} // DRS_0602
};

class HerkulexMotor{
    public:
        // Updated constructors to include bus (1 for Serial1, 2 for Serial2)
        HerkulexMotor(int id, MotorModel type, uint8_t bus);
        HerkulexMotor(int id, MotorModel type, uint8_t bus, float lowerBoundDeg, float upperBoundDeg);
        
        void setPos(float posDeg);
        float getPos();
        void queueMove(float posDeg);
        void reboot();
        
        static void initialize();
        static void actionMoves(int pTime, HerkulexMotor** motors, uint8_t count);
        static void initSerialPorts(uint32_t baudRate);

    private:
        int _id;
        MotorModel _type;
        uint8_t _bus; // New member to track the hardware bus
        uint16_t _zeroPos;
        uint16_t _bounds[2];
        
        int32_t degToSteps(float deg, MotorModel type);
        float stepsToDeg(uint16_t steps, MotorModel type);

        uint16_t _pendingGoal;
        bool _hasPending;
};

#endif