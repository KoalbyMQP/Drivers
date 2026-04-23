#include "Arduino.h"
#include "DRV8825.h"

DRV8825::DRV8825(int sleepPin, int resetPin, int stepPin, int dirPin, int limitPin) {
    _sleepPin = sleepPin;
    _resetPin = resetPin;
    _stepPin = stepPin;
    _dirPin = dirPin;
    _limitPin = limitPin;
}

void DRV8825::initialize() {
    pinMode(_sleepPin, OUTPUT);
    pinMode(_resetPin, OUTPUT);
    pinMode(_stepPin, OUTPUT);
    pinMode(_dirPin, OUTPUT);
    pinMode(_limitPin, INPUT_PULLUP); // assuming limit switch is active LOW

    digitalWrite(_sleepPin, HIGH); // wake up the driver
    digitalWrite(_resetPin, HIGH); // take the driver out of reset
}

void DRV8825::runToPosition(int position){
    if (position < 0) position = 0;
    if (position > STEP_RANGE) position = STEP_RANGE;

    int stepsToMove = position - currentPos;

    if (stepsToMove > 0) {
        moveSteps(stepsToMove, false);
    } 
    else if (stepsToMove < 0) {
        moveSteps(-stepsToMove, true);
    }

    currentPos = position;
    Serial.println("MC");
}

void DRV8825::moveSteps(int steps, bool direction){
    digitalWrite(_dirPin, direction ? HIGH : LOW);

    int stepDelay = 1000; // delay in microseconds between steps (adjust as needed)

    for (int i = 0; i < steps; i++) {
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(_stepPin, LOW);
        delayMicroseconds(stepDelay);
    }
}

void DRV8825::homeMotor() {

    digitalWrite(_dirPin, HIGH); // set direction towards limit switch

    while (digitalRead(_limitPin) == LOW) { // wait until limit switch is triggered
        digitalWrite(_stepPin, HIGH);
        delayMicroseconds(1000); // adjust speed as needed
        digitalWrite(_stepPin, LOW);
        delayMicroseconds(1000);
    }

    currentPos = 0; // reset position after homing
}