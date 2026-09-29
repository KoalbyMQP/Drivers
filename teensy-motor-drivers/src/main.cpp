#include <Arduino.h>
#include <HerkulexMotor.h>
#include <SerialBusManager.h>
#include <array>

/* 
 Hello (future) students,
 This code will allow you to identify every motor
 that is attached to each Teensy UART bus by printing the motor ID and model to your serial monitor. 
 You will then be able to use this information to test/move these motors. If you have everything 
 wired correctly and follow the comments, this process should (hopefully) be straightforward. If needed, 
 find the schematics for wiring a motor to a Teensy in motor wiring schemtaic.md
 Sincerely,
 Swapping End Effectors 2026-27
 */

 /*
USER ACTION:
 Adjust this value to determine your test type
  - 0 will print all motors found on all buses with their IDs and models to Serial monitor
  - 1 will test each specified motor on each specified bus
*/
uint8_t TEST_TYPE = 1; 


/*
 USER ACTION FOR: Test type 1 
  - add the Motor IDs that you want to test to each bus in TEST_MOTORS
  - define which buses will be active duirng the test in ACTIVE_BUSES
  - add the Motor Models of the motors that you want to test to each bus in TEST_MOTOR_TYPES
*/
std::vector<std::vector<uint8_t>> TEST_MOTORS = {
    {},  //bus 1
    {},  //bus 2
    {11},  //bus 3
    {},  //bus 4
    {8, 17},  //bus 5
    {},  //bus 6
    {},  //bus 7
    {}   //bus 8
};

std::vector<uint8_t> ACTIVE_BUSES = {3, 5};

std::vector<std::vector<MotorModel>> TEST_MOTOR_TYPES = {
    {},  //bus 1
    {},  //bus 2
    {DRS_0601},  //bus 3
    {},  //bus 4
    {DRS_0601, DRS_0201},  //bus 5
    {},  //bus 6
    {},  //bus 7
    {}   //bus 8
};


// Define max number of buses and motor IDs that can be detected (test type 0)
constexpr uint8_t MAX_BUSES = SerialBusManager::MAX_BUS_COUNT;
constexpr uint16_t MAX_FOUND_MOTORS = MAX_BUSES * 254U;

struct FoundMotor {
  uint8_t busId;
  uint8_t id;
  uint16_t model;
};

uint8_t activeMotorId = 0;
uint8_t activeBusId = 0;

std::vector<HerkulexMotor*> testMotors;
std::vector<MotorRef> testMotorRefs;

bool motionEnabled = false;
bool moveNegative = true;

FoundMotor foundMotors[MAX_FOUND_MOTORS];
uint16_t foundMotorCount = 0;

// Following chunks used for defining the models of each motor on a given bus (test type 0)
MotorModel decodeMotorModel(uint16_t rawModel) {
  switch (rawModel) {
    case 0x0201:
      return DRS_0201;
    case 0x0601:
      return DRS_0601;
    case 0x0602:
      return DRS_0602;
    default:
      return UNKNOWN_MODEL;
  }
}

const char* modelName(MotorModel model) {
  switch (model) {
    case DRS_0201:
      return "DRS-0201";
    case DRS_0601:
      return "DRS-0601";
    case DRS_0602:
      return "DRS-0602";
    default:
      return "Unknown";
  }
}

// find the motor IDs and models of each motor on a given bus (test type 0)
uint16_t findAllMotorIdsAndModels(uint8_t busId) {
  uint16_t busFoundCount = 0;

  for (uint16_t id = 0; id <= 253; ++id) {
    uint8_t error = 0;
    uint8_t detail = 0;

    if (!SerialBusManager::getBus(busId).stat(id, &error, &detail)) {
      continue;
    }

    uint16_t model = 0;
    if (!SerialBusManager::getBus(busId).getModel(id, &model)) {
      continue;
    }

    if (foundMotorCount >= MAX_FOUND_MOTORS) {
      break;
    }

    foundMotors[foundMotorCount].busId = busId;
    foundMotors[foundMotorCount].id = static_cast<uint8_t>(id);
    foundMotors[foundMotorCount].model = model;
    ++foundMotorCount;
    ++busFoundCount;

    MotorModel decodedModel = decodeMotorModel(model);
    Serial.print("Found servo on bus ");
    Serial.print(busId);
    Serial.print(" at ID ");
    Serial.print(id);
    Serial.print(" model=0x");
    Serial.print(model, HEX);
    Serial.print(" (");
    Serial.print(modelName(decodedModel));
    Serial.println(")");
    delay(25);
  }

  return busFoundCount;
}

// find the motor IDs and models of each motor on all buses
void findAllMotorsAcrossAllBuses() {
  foundMotorCount = 0;
  for (uint8_t busId = 1; busId <= MAX_BUSES; ++busId) {
    Serial.print("Scanning bus ");
    Serial.print(busId);
    Serial.println("...");
    findAllMotorIdsAndModels(busId);
  }


}

void setup() {

  Serial.begin(1000000);
  delay(2000);

  Serial.println("BOOT OK");
  delay(1000);

  if (TEST_TYPE == 0) {

    // create all Teensy buses (1-8)
    Serial.println("Creating buses...");
    for (uint8_t busId = 1; busId <= MAX_BUSES; ++busId) {
      SerialBusManager::createBus(busId);
    }

    // Start created Teensy buses
    Serial.println("Starting all buses...");
    SerialBusManager::startAllBuses(BAUD_RATE::SPEED_115K);

    // function callback to scan for motors accross all buses
    Serial.println("Scanning all Herkulex buses for IDs and models...");
    findAllMotorsAcrossAllBuses();

    if (foundMotorCount == 0){
      Serial.println("No motors found. Check Power + GND wiring and Baud rate.");
      Serial.println("Ensure motor(s) have power before connecting your PC to the Teensy.");
    }

    Serial.print("SCAN COMPLETE");

  }
  else if (TEST_TYPE == 1){

    // create selected Teensy buses
    Serial.println("Creating bus(es)...");
    for (uint8_t i = 0; i < std::size(ACTIVE_BUSES); i++) {
      SerialBusManager::createBus(ACTIVE_BUSES[i]);
    }

    // start created Teensy buses
    Serial.println("Starting all buses...");
    SerialBusManager::startAllBuses(BAUD_RATE::SPEED_115K);

    // for each bus
    for (uint8_t i = 0; i < ACTIVE_BUSES.size() ; i++) {
      
      // define active bus ID
      uint8_t bus_id = ACTIVE_BUSES[i];

      // Convert bus number (1-8) to vector index (0-7)
      uint8_t bus_index = bus_id - 1;

      // for each motor belonging to the bus
      for (uint8_t j = 0; j < TEST_MOTORS[bus_index].size(); j++) {

        uint8_t motor_id = TEST_MOTORS[bus_index][j];
        MotorModel motor_type = TEST_MOTOR_TYPES[bus_index][j];

        // define a new motor using selected Motor IDs, Bus IDs, and Motor types
        HerkulexMotor* new_motor = new HerkulexMotor(motor_id, motor_type, bus_id);
        testMotors.push_back(new_motor);

        // Create motor refs for selected motors
        Serial.println("Creating motor references");
        for (HerkulexMotor* motor : testMotors) {
          testMotorRefs.push_back(motor->getMotorRef());
        };
        

        Serial.print("Created motor ID ");
        Serial.print(motor_id);
        Serial.print(" on bus ");  // Create MotorRefs for all selected motors
        Serial.println(bus_id);

      }
    }
    

    Serial.println("Initializing motors...");
    SerialBusManager::initAllMotors();

    Serial.println("Enabling torque...");
    SerialBusManager::torqueOnAllMotors();

  }

  else{
    Serial.println("ERROR: Please enter valid test type");
  }

  Serial.println("SETUP COMPLETE");
  Serial.println("Motor bus initialized and torque enabled");
  Serial.println("Type 'start' to enable motion or 'stop' to pause");

}

void loop() {


  if (TEST_TYPE == 1){

    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');
      command.trim();

      if (command.equalsIgnoreCase("start")) {
        motionEnabled = true;
        Serial.println("Motion enabled");
        delay(1000);
        
      } else if (command.equalsIgnoreCase("stop")) {
        motionEnabled = false;
        Serial.println("Motion disabled");
      }
    }


    if (!motionEnabled) {
      delay(20);
      return;
    }


    // move all motors in parallel
    if (moveNegative) {
      Serial.println("Moving all motors -30 degrees");

      // On the bus, queue motor movement for each motor
      // Each motor that is daisy chained along the bus will recieve this command after the previous one is done
      for (HerkulexMotor* motor : testMotors){
        float currentPos = motor->getPos();
        motor->queueMove(currentPos - 30.0f);
      }

      // Send movement requests to all motors in parallel
      SerialBusManager::actionAll(100);
      moveNegative = false;
    } 
    else {

      Serial.println("Moving all motors +30 degrees");

      for (HerkulexMotor* motor : testMotors){
        float currentPos = motor->getPos();
        motor->queueMove(currentPos + 30.0f);
      }

      // Send movement requests to all motors in parallel
      SerialBusManager::actionAll(100);
      moveNegative = true;

    }
    delay(2000);
  }
}
