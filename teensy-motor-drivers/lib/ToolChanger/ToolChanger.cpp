#include "Arduino.h"
#include "ToolChanger.h"

ToolChanger::ToolChanger(int statusPin, int servoPin, int attachPos, int lockPos, int depositPos){
    
    _statusPin = statusPin;
    _servoPin = servoPin;

    // set the positions for the servo to move to for attaching, locking, and depositing based on the provided values
    _servoPos[0] = attachPos;
    _servoPos[1] = lockPos;
    _servoPos[2] = depositPos;

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

void ToolChanger::setServoPulse(float pulse_us){
  float duty = (pulse_us / 20000.0f) * 65535.0f;
  analogWrite(_servoPin, (uint32_t)duty);
}

void ToolChanger::attachTool(){
    setServoPulse(_servoPos[0]);
}

void ToolChanger::lockTool(){
    setServoPulse(_servoPos[1]);
}

void ToolChanger::depositTool(){
    setServoPulse(_servoPos[2]);
}

void ToolChanger::initialize(){
  analogWriteResolution(_servoPin);    // 16-bit resolution (0–65535)
  analogWriteFrequency(_servoPin, 50);  // Set servo pin to 50 Hz for servo (20ms period)
  Serial.println("ToolChanger servo pin " + String(_servoPin) + " initialized");
  ToolChanger::attachTool();
}

