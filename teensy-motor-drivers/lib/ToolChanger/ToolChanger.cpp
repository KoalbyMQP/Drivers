#include "Arduino.h"
#include "Servo.h"
#include "ToolChanger.h"

ToolChanger::ToolChanger(int statusPin, int servoPin){
    
    _statusPin = statusPin;
    _servoPin = servoPin;

    // default positions for the servo to move to for docking, locking, and ejecting
    _servoPos[0] = 750; // dock position
    _servoPos[1] = 500; // lock position
    _servoPos[2] = 1800; // eject position

    _currentTool = ToolList[0]; // initialize current tool to empty
}

ToolChanger::ToolChanger(int statusPin, int servoPin, int dockPos, int lockPos, int ejectPos){
    
    _statusPin = statusPin;
    _servoPin = servoPin;

    // set the positions for the servo to move to for docking, locking, and ejecting based on the provided values
    _servoPos[0] = dockPos;
    _servoPos[1] = lockPos;
    _servoPos[2] = ejectPos;

    _currentTool = ToolList[0]; // initialize current tool to empty
}

int ToolChanger::getToolStatus(){
    // read the value from the status pin
    uint16_t analogValue = analogRead(_statusPin);

    // loop through the tool list to find which tool corresponds to the read value
    for (int i = 0; i < sizeof(ToolList) / sizeof(ToolList[0]); i++){
        if (analogValue >= ToolList[i].minValue && analogValue <= ToolList[i].maxValue){
            _currentTool = ToolList[i];
            return _currentTool.toolID;
        }
    }

    return -1; // return -1 as error code if no value found within bounds
}

int ToolChanger::readRawStatus(){
    return analogRead(_statusPin);
}

void ToolChanger::dockTool(){
    wrist.writeMicroseconds(_servoPos[0]);
}

void ToolChanger::lockTool(){
    wrist.writeMicroseconds(_servoPos[1]);
}

void ToolChanger::ejectTool(){
    wrist.writeMicroseconds(_servoPos[2]);
}

void ToolChanger::initialize(){
    wrist.attach(_servoPin, SERVO_MIN_US, SERVO_MAX_US); // attach the servo to the specified pin and set the min and max pulse width
    wrist.writeMicroseconds(_servoPos[0]); // move the servo to the dock position to initialize
}

