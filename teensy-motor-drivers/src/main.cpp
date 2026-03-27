#include <HerkulexMotor.h>
#include <RPIComs.h>
#include <SerialBusManager.h>
#include <IMU.h>
#include <debug.h>

elapsedMillis imuTimer; 

// start at serial 2 because the raspberry pi is connected through serial 1
enum SERIAL_BUS {
    BUS_L_LEG = 2,
    BUS_R_LEG = 3,
    BUS_CHEST = 4,
    BUS_L_ARM = 5,
    BUS_R_ARM = 6,
    EXTRA_1 = 7,
    EXTRA_2 = 8
};

uint16_t startTime = 0;
uint16_t elapsedTime = 0;
boolean testSetPosBool = false;    // when runTest is true, the moving motor will run once upon restart and when the arduino is uploaded.
boolean testQueueBool = false;
boolean testRPi = true;
boolean latency_queuing = false;
boolean queuedMotors = false;

const uint8_t MOTOR_COUNT = 3;
const uint8_t PACKET_SIZE = 192;   // this depends on the number of motors used

uint16_t motorResults[MOTOR_COUNT] = {0};


HerkulexMotor motors[MOTOR_COUNT] = {
  HerkulexMotor(12, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG),
  HerkulexMotor(5, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG),
  HerkulexMotor(1, MotorModel::DRS_0601, SERIAL_BUS::BUS_L_LEG),
};

uint16_t rawPositions[MOTOR_COUNT] = {0};     // filled by collectAllPositions
MotorRef motorRefs[MOTOR_COUNT];

RPIComs rpi = RPIComs();

IMU imu1;  // uses sensorID=55 and address=0x28 automatically

void setup(){
  delay(2000);  // a delay to have time for serial monitor opening on platformio after uploading
  Serial.begin(9600);    // Open serial communications with computer
  Serial.println("Begin");

  Serial1.begin(9600); //begin serial communication with the raspberry pi
  delay(2000);

  if (!imu1.begin()) {
      Serial.println("ERROR: IMU not detected. Check wiring!");
      while (1);
  }

  SerialBusManager::createBus(SERIAL_BUS::BUS_L_LEG); // begin serial communications with motor, these are on Serial 2
  SerialBusManager::createBus(SERIAL_BUS::BUS_R_LEG);
  SerialBusManager::startAllBuses(115200);
  SerialBusManager::initAllMotors();

  // // MotorRef table — used by the parallelized position read.
  // // Each entry is {busId, servoId}. Order here determines order in rawPositions[], can mix and match serial buses
  // // Add or remove entries to match the motors needed
  for (int i = 0; i < MOTOR_COUNT; i++) motorRefs[i] = motors[i].getMotorRef();

  // // set the motor positions to 0 to initialize
  for (int i = 0; i < MOTOR_COUNT; i++) motors[i].setPos(20.0);
  delay(1000);
  
  uint32_t startTimeUs = micros();
  SerialBusManager::getAllPositionsParallel(motorRefs, rawPositions, MOTOR_COUNT);
  uint32_t elapsed = micros() - startTimeUs;

  Serial.print("elapsed read time:");
  Serial.println(elapsed);
  
  for (int i = 0; i < MOTOR_COUNT; i++){
    Serial.println("motor n: ");
    Serial.println(rawPositions[i] & 0x07FF);
  };

  Serial.println("at end");

  debug_motors(motors, MOTOR_COUNT);
  for (int i = 0; i < MOTOR_COUNT; i++) motors[i].setPos(0.0);
  // delay(500);
}




void loop(){
    if (imuTimer >= 10) {
        imuTimer = 0;

        // Phase 1 — time the write
        uint32_t t1 = micros();
        imu1.requestUpdate();
        uint32_t t2 = micros();

        // gap — in real firmware motor reads go here
        
        // Phase 2 — time the read
        uint32_t t3 = micros();
        imu1.collectUpdate();
        uint32_t t4 = micros();

        // print as CSV for easy reading
        Serial.print(t2 - t1);   // requestUpdate duration
        Serial.print(",");
        Serial.print(t4 - t3);   // collectUpdate duration
        Serial.print(",");
        Serial.println(t4 - t1); // total duration
    }


  // // this loops
  // // while(testRPi){
  // //   testingRPi();
  // //   testRPi = true;
  // // }

  // // Working on the control loop for moving the motors and everything
  // rpi.uartRead();
  //   // If a packet arrived, handle it
  //   const char* pkt = rpi.getPacket();
  //   if (pkt != nullptr) {
  //     // Serial.print("Received: ");
  //     // Serial.println(pkt);

  //     // copy packet to avoid buffer overwrite
  //     char buffer[PACKET_SIZE];        
  //     strncpy(buffer, pkt, sizeof(buffer));
  //     buffer[sizeof(buffer)-1] = '\0';

  //     float targets[MOTOR_COUNT];
  //     char* token = strtok(buffer, ",");
  //     for (int i = 0; i < MOTOR_COUNT && token != nullptr; i++) {
  //         targets[i] = atof(token);
  //         token = strtok(nullptr, ",");
  //     }

  //     // then queue all motors in a loop
  //     for (int i = 0; i < MOTOR_COUNT; i++) motors[i].queueMove(targets[i]);

  //     // Serial.print("queing motor 1 to: ");
  //     // Serial.print(pos1);
  //     // Serial.print(" and motor 2 to : ");
  //     // Serial.println(pos2);

  //     queuedMotors = true;      
  //     } 
  //   if (queuedMotors){
  //     // direct all of the motors to move, this signals to send the serial commands
  //     SerialBusManager::actionAll(10);
  //     queuedMotors = false;

  //     // Phase 1: fire RAMREAD requests to all motors without waiting, all motors start replying in parallel.
  //     SerialBusManager::requestAllPositions(motorRefs, MOTOR_COUNT);

  //     // we can do operations in the middle

  //     // Phase 2: collect all replies.By now most/all bytes are already in the RX buffers, so this is fast.
  //     SerialBusManager::collectAllPositions(motorRefs, rawPositions, MOTOR_COUNT);

  //     //send the motor positions back to the raspberry pi
  //     // float pos1 = myMotor.getPos();
  //     // float pos2 = myMotor2.getPos();

  //     char response[PACKET_SIZE];                        // this number will be dependent on how long the message will be (how many motors)
  //     int offset = 0;
  //     // sending the data like this makes it variable to the number of motors so that you don't have to update it everytime a new motor is added or removed
  //     for (int i = 0; i < MOTOR_COUNT; i++) {
  //         offset += snprintf(response + offset, sizeof(response) - offset,
  //                           i < MOTOR_COUNT - 1 ? "%.2f," : "%.2f",
  //                           motors[i].rawToDegs(rawPositions[i]));
  //     }

  //     rpi.enqueueTXPacket(response);
  //     rpi.uartSend();
  //   }
}



// Notes on how the different libraries work together (as is my undersanding):
// in main.cpp, all of the functions are quite abstracted (as simple as possible to understand)
// things with the raspberry pi deal with the functins in the RPIComs library, that deals with packet sending and receiving through serial with the pi
// the motors are initialized with a specific serial and motor type
// the functions are called on those motors instances, which use the HerkulexMotor class.
// in the HerkulexMotor library, the functions use the SerialBusManager library so that every command/function gests called on the proper instance
// each instance fo the SeiralBusManager will correspond to a bus, that way the same Herkulex functions can be used the same way


// Reading parallization
// Why this is faster:
    //   Old: send bus2 -> wait -> read bus2 -> send bus3 -> wait -> read bus3  (N × latency)
    //   New: send bus2, send bus3, send busN -> read bus2, read bus3, read busN (~1 × latency)
        
// In requestAllPositions, Each sendData() call queues bytes into the hardware TX FIFO and returns immediately. The motor starts composing its reply right away. 
// By the time we loop back around in collectAllPositions, most replies are already sitting in the corresponding RX buffers waiting to be read.