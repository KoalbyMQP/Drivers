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
#include <MotorModel.h>

MotorModel decodeModel(int rawModel)
{
    switch (rawModel) {
        case 0x0102: return DRS_0201;
        case 0x0106: return DRS_0601;
        case 0x0206: return DRS_0602;
        default:     return UNKNOWN_MODEL;
    }
}

// Helper function that makes the motor type readable
static const char* modelName(MotorModel t) {
    switch (t) {
        case DRS_0201: return "DRS-0201";
        case DRS_0601: return "DRS-0601";
        case DRS_0602: return "DRS-0602";
        default:       return "Unknown";
    }
}

static const char* busName(int b) {
    switch (b) {
        case 1:  return "Left_leg";
        case 2:  return "Right_leg";
        case 3:  return "Chest";
        case 4:  return "Left_arm";
        case 5:  return "Right_arm";
        case 6:  return "Extra_1";
        case 7:  return "Extra_2";
        default: return "Unknown_bus";
    }
}


//find_all_motors_on_bus
int find_all_motors_on_bus(HerkulexClass& SerialBus){
    // loop through all possible pIDs (0-253), every time a motor is found, print and add to motor out
    // packet = data for servos (50) + 8 for move multiple length. See herkulex.h for more details.
    byte status;
    int motorCount = 0;

    for (uint8_t pID = 0; pID < 0xFE; pID++){
        // send packet with current pID and wait for ACK packet
        // comments for debugging
        // Serial.print("scanning: ");
        // Serial.println(pID);
        
        uint8_t error, detail;

        if (SerialBus.stat(pID, &error, &detail)) {
            // no errors and there is a motor at that id
            Serial.print("Motor at ID: ");
            Serial.println(pID);

            motorCount++;

            uint16_t modelNo;   // hold the model number returned by checkModel here
            
            if (SerialBus.checkModel(pID, &modelNo)) {
                MotorModel model = decodeModel(modelNo);

                Serial.print("Motor model: ");
                Serial.println(modelName(model));
            } else {
                Serial.println("Motor model: [READ FAILED]");
            }
            Serial.println();
        } else {
            continue;
        }
        delay(500);     // Short delay to not overwelm calls on bus
    }
    Serial.println("Done checking all of the possible ids");
    return motorCount;
}

// function that goes through every motor and tests latency for getting and sending position
void test_motor_latency(HerkulexMotor* motors, int motors_size){
    for (int i = 0; i < motors_size; i++){
        HerkulexMotor& motor = motors[i];

        unsigned long start = micros();
        float pos = motor.getPos();
        unsigned long stop = micros();

        unsigned long elapsed = stop - start;
        unsigned long totalElapsed = elapsed;

        Serial.print("Total time to read Motor ");
        Serial.print(motor.getId());
        Serial.print(" position: ");
        Serial.print(elapsed);
        Serial.println(" us");

        Serial.println("");
        Serial.print("Estimated Round Trip Time: "); Serial.println(totalElapsed);

        Serial.println("Press any key to continue to next motor...");
        while (Serial.available() == 0) {}
        while (Serial.available() > 0) Serial.read();

        Serial.println("==========================================");
    }
}

// go through all of the motors and print the desired information
void debug_motors(HerkulexMotor* motors, int motors_size) {
    
    // turn all the motors's leds off 
    for (int j = 0; j < motors_size; j++) {
        HerkulexMotor& motor = motors[j];    
        SerialBusManager::getBus(motor.getBusId()).setLed(motor.getId(), LED_STATE::LED_OFF);
    }

    // divider
    Serial.println("==========================================");

    for (int i = 0; i < motors_size; i ++) {
        
        // reference of the motor instead of just making a copy
        HerkulexMotor& motor = motors[i];
        
        // Turn on the LED green
        SerialBusManager::getBus(motor.getBusId()).setLed(motor.getId(), LED_STATE::LED_BLUE); // Blue

        Serial.print("Motor id:    "); Serial.println(motor.getId());
        Serial.print("Type:        "); Serial.println(modelName(motor.getType()));
        Serial.print("Serial bus:  "); Serial.println(busName(motor.getBusId()));
        Serial.print("Position:    "); Serial.print(motor.getPos()); Serial.println(" deg");
        Serial.println("Press any key for next motor...");
        
        // Wait to press a key and then flush the input
        while (Serial.available() == 0) {}
        while (Serial.available() > 0) Serial.read();

        Serial.println("==========================================");
        
        // turn off the LED
        SerialBusManager::getBus(motor.getBusId()).setLed(motor.getId(), LED_STATE::LED_OFF);
    }

    Serial.println("All motors tested.");
}