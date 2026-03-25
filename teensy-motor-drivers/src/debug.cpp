// ============================================================================================================
// what debug should do

// we should be able to go through all of the motors in all of the serial buses and write something like:
    // motor testing
    //     id = 12
    //     motor type = 0602
    //     led = blue
    //     serial bus = Right_leg
    //     position in bus = 3rd
    //     motor position = 16 degrees
    
    // [press enter to continue to next motor]

// ============================================================================================================

#include <debug.h>
#include <HerkulexMotor.h>
#include <Herkulex.h>

// Helper function that makes the motor type readible
static const char* modelName(MotorModel t) {
    switch (t) {
        case DRS_0201: return "DRS-0201";
        case DRS_0601: return "DRS-0601";
        case DRS_0602: return "DRS-0602";
        default:       return "Unknown";
    }
}

// go through all of the motors and print the desired information
void debug_motors(HerkulexMotor* motors, int motors_size) {
    for (int i = 0; i < motors_size; i ++) {
        
        // reference of the motor instead of just making a copy
        HerkulexMotor& motor = motors[i];
        
        // Turn on the LED green
        SerialBusManager::getBus(motor.getBusId()).setLed(motor.getId(), LED_STATE::LED_GREEN); // GREEN

        Serial.print("Motor id:    "); Serial.println(motor.getId());
        Serial.print("Type:        "); Serial.println(modelName(motor.getType()));
        Serial.print("Serial bus:  "); Serial.println(motor.getBusId());
        Serial.print("Position:    "); Serial.print(motor.getPos()); Serial.println(" deg");
        Serial.println("Press any key for next motor...");
        
        // Wait to press a key and then flush the input
        while (Serial.available() == 0) {}
        while (Serial.available() > 0) Serial.read();
        
        // turn off the LED
        SerialBusManager::getBus(motor.getBusId()).setLed(motor.getId(), LED_STATE::LED_OFF);
    }

    // for (int j = 0; j < motors_size; j++) {
    //     HerkulexMotor& motor = motors[j];    
    //     SerialBusManager::getBus(motor.getBusId()).setLed(motor.getId(), LED_STATE::LED_OFF);
    // }

    // for (int k= 0; k < motors_size; k++) {
    //     HerkulexMotor& motor = motors[k];    
    //     SerialBusManager::getBus(motor.getBusId()).setLed(motor.getId(), LED_STATE::LED_OFF);
    // }

    Serial.println("All motors tested.");
}