#ifndef HerkulexMotor_h
#define HerkulexMotor_h

#include "Arduino.h"
#include "SerialBusManager.h"
#include "MotorModel.h"
#include "Herkulex.h"


class HerkulexMotor{
    public:
        HerkulexMotor(int id, MotorModel type, uint8_t busId);
        HerkulexMotor(int id, MotorModel type, uint8_t busId, float lowerBoundDeg, float upperBoundDeg);
        
        void setPos(float posDeg);
        float getPos();
        void queueMove(float posDeg);
        void reboot();
        static void actionMoves(int playTimeMs); // here we can manage how we action the moves if we switch to multiple serial ports
        MotorRef getMotorRef() const;   // allows generation of the MotorRef table itself so that it can be automated
        float rawToDegs(uint16_t rawPos);

        void setLed(LED_STATE ledColor);
        uint16_t getModel();

        // getter functions
        int getId()    const { return _id; }
        int getBusId() const { return _busId; }
        MotorModel getType() const { return _type; }


        // functions on motor refs as opposed to using the motor objects
        static void motorRefSetPos(const MotorRef &ref, float posDeg);
        static void motorRefQueueMove(const MotorRef &ref, float posDeg);
        static void motorRefReboot(const MotorRef &ref);
        static float motorRefRawToDegs(const MotorRef &ref, uint16_t rawPos);
        static uint16_t boundPosFromRef(const MotorRef &ref, int32_t rawPos);


    private:
        uint8_t _id;
        uint8_t _busId;
        MotorModel _type;
        uint16_t _bounds[2];    // motor bounds, in steps
        uint16_t _zeroPos;      // zero position in steps
        
        static int32_t degToSteps(float deg, MotorModel type);
        uint16_t boundPos(int32_t rawPos);
        static float stepsToDeg(uint16_t steps, MotorModel type);
};

#endif