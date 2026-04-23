#ifndef DRV8825_h
#define DRV8825_h

#include "Arduino.h"

class DRV8825 {
    public:
        DRV8825(int sleepPin, int resetPin, int stepPin, int dirPin, int limitPin);
        void initialize();
        void runToPosition(int position);
        void moveSteps(int steps, bool direction);
        void homeMotor();
    private:
        int _sleepPin;
        int _resetPin;
        int _stepPin;
        int _dirPin; 
        int _limitPin;
        int currentPos = 0; 
        const int STEP_RANGE = 10000; // maximum steps for full range  
};

#endif