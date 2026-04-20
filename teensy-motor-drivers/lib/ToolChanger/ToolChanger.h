#ifndef ToolChanger_h
#define ToolChanger_h

#include "Arduino.h"

// specs for each tool, including the analog value range that corresponds to each tool and a tool ID that can be used in the code to identify the tool
struct Tool {
    uint16_t minValue;
    uint16_t maxValue;
    uint16_t toolID;
};

// lookup table for tool IDs, based off the analog pins read from the Arduino
// each tool has an analog value that is read by the arduino, and this table will be used to 
// convert that value to a tool ID that can be used in the code
const Tool ToolList[] = {
    {0, 100, 0}, // empty or no tool
    {101, 300, 1}, // chess hand 
    {301, 500, 2}, // oximeter hand
    {501, 700, 3}, // pill bottle hand
    {701, 900, 4}, // temperature hand
    {901, 1023, 5} // precision end effector
};

class ToolChanger{
    public:
        ToolChanger(
            int statusPin, 
            int servoPin, 
            int attachPos, 
            int lockPos, 
            int depositPos);
        int getToolStatus();
        int readRawStatus();
        void attachTool();
        void lockTool();
        void depositTool();
        void initialize();
        
    private:   
        int _statusPin; // analog pin for reading tool status
        int _servoPin; // INJORA INJS2065 pin
        uint16_t _servoPos[3]; // positions for the servo to move to for attaching, locking, and depositing
        Tool _currentTool;
        void setServoPulse(float pulse_us);
};

#endif