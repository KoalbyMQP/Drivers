#include <Herkulex.h>
#include <HerkulexMotor.h>
#include <RPIComs.h>
#include <SerialBusManager.h>

// start at serial 2 because the raspberry pi is connected through serial 1
typedef enum {
    BUS_L_LEG = 2,
    BUS_R_LEG = 3,
    BUS_CHEST = 4,
    BUS_L_ARM = 5,
    BUS_R_ARM = 6,
    EXTRA_1 = 7,
    EXTRA_2 = 8
} SERIAL_BUS;

uint16_t startTime = 0;
uint16_t elapsedTime = 0;
boolean testSetPosBool = false;    // when runTest is true, the moving motor will run once upon restart and when the arduino is uploaded.
boolean testQueueBool = false;
boolean testRPi = true;
boolean latency_queuing = false;
boolean queuedMotors = false;
const uint8_t PACKET_SIZE = 192;   // this depends on the number of motors used

HerkulexMotor motors[] = {
  HerkulexMotor(12, MotorModel::DRS_0601, SERIAL_BUS::BUS_L_LEG),
  HerkulexMotor(5, MotorModel::DRS_0601, SERIAL_BUS::BUS_L_LEG),
  HerkulexMotor(11, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG),
};
const uint8_t MOTOR_COUNT = 3;
uint16_t rawPositions[MOTOR_COUNT];     // filled by collectAllPositions
MotorRef motorRefs[MOTOR_COUNT];

RPIComs rpi = RPIComs();


void setup(){
  delay(2000);  // a delay to have time for serial monitor opening on platformio after uploading
  Serial.begin(9600);    // Open serial communications with computer
  Serial.println("Begin");

  Serial1.begin(9600); //begin serial communication with the raspberry pi

  SerialBusManager::createBus(BUS_L_LEG); // begin serial communications with motor, these are on Serial 2
  SerialBusManager::createBus(BUS_R_LEG);
  SerialBusManager::startAllBuses(115200);
  SerialBusManager::initAllMotors();

  delay(500);

  // MotorRef table — used by the parallelized position read.
  // Each entry is {busId, servoId}. Order here determines order in rawPositions[], can mix and match serial buses
  // Add or remove entries to match the motors needed
  for (int i = 0; i < MOTOR_COUNT; i++) motorRefs[i] = motors[i].getMotorRef();

  // set the motor positions to 0 to initialize
  for (int i = 0; i < MOTOR_COUNT; i++) motors[i].setPos(0.0);

  delay(500);
}


void testingRPi(){
    rpi.uartRead();
    if (latency_queuing){
      startTime = micros();
    }
    // If a packet arrived, handle it
    const char* pkt = rpi.getPacket();
    if (pkt != nullptr) {
      if (latency_queuing) {
        elapsedTime = micros() - startTime;
        Serial.print("It took ");
        Serial.print(elapsedTime);
        Serial.println(" microseconds between putting the message in the queue (right after uartRead) and reading it");
      }
      Serial.print("Received: ");
      Serial.println(pkt);
    }
}

void testingQueue(){
  startTime = micros();
  // QUEUE the movements
  motors[0].queueMove(-80.0);     // Queue motor 1
  motors[1].queueMove(158);     // Queue motor 2

  elapsedTime = micros() - startTime;

  Serial.print("time to queue 1 movement: ");
  Serial.print(elapsedTime);
  Serial.println(" microseconds");

  delay(1200);  

  startTime = micros();
  // execute all of the queued movements
  SerialBusManager::actionAll(10);
  elapsedTime = micros() - startTime;

  Serial.print("time to execute 1 movement: ");
  Serial.print(elapsedTime);
  Serial.println(" microseconds");
  Serial.println();

  delay(3000);                // Wait for movement to complete
  
  float pos1 = motors[0].getPos();
  float pos2 = motors[1].getPos();
  Serial.print("Motor 1 position: ");
  Serial.println(pos1);
  Serial.print("Motor 2 position: ");
  Serial.println(pos2);
}

void loop(){
  // this loops
  // while(testRPi){
  //   testingRPi();
  //   testRPi = true;
  // }

  // Working on the control loop for moving the motors and everything
  rpi.uartRead();
    // If a packet arrived, handle it
    const char* pkt = rpi.getPacket();
    if (pkt != nullptr) {
      // Serial.print("Received: ");
      // Serial.println(pkt);

      // copy packet to avoid buffer overwrite
      char buffer[PACKET_SIZE];        
      strncpy(buffer, pkt, sizeof(buffer));
      buffer[sizeof(buffer)-1] = '\0';

      float targets[MOTOR_COUNT];
      char* token = strtok(buffer, ",");
      for (int i = 0; i < MOTOR_COUNT && token != nullptr; i++) {
          targets[i] = atof(token);
          token = strtok(nullptr, ",");
      }

      // then queue all motors in a loop
      for (int i = 0; i < MOTOR_COUNT; i++) motors[i].queueMove(targets[i]);

      // Serial.print("queing motor 1 to: ");
      // Serial.print(pos1);
      // Serial.print(" and motor 2 to : ");
      // Serial.println(pos2);

      queuedMotors = true;      
      } 
    if (queuedMotors){
      // direct all of the motors to move, this signals to send the serial commands
      SerialBusManager::actionAll(10);
      queuedMotors = false;

      // Phase 1: fire RAMREAD requests to all motors without waiting, all motors start replying in parallel.
      SerialBusManager::requestAllPositions(motorRefs, MOTOR_COUNT);

      // we can do operations in the middle

      // Phase 2: collect all replies.By now most/all bytes are already in the RX buffers, so this is fast.
      SerialBusManager::collectAllPositions(motorRefs, rawPositions, MOTOR_COUNT);

      //send the motor positions back to the raspberry pi
      // float pos1 = myMotor.getPos();
      // float pos2 = myMotor2.getPos();

      char response[PACKET_SIZE];                        // this number will be dependent on how long the message will be (how many motors)
      int offset = 0;
      // sending the data like this makes it variable to the number of motors so that you don't have to update it everytime a new motor is added or removed
      for (int i = 0; i < MOTOR_COUNT; i++) {
          offset += snprintf(response + offset, sizeof(response) - offset,
                            i < MOTOR_COUNT - 1 ? "%.2f," : "%.2f",
                            motors[i].rawToDegs(rawPositions[i]));
      }

      rpi.enqueueTXPacket(response);
      rpi.uartSend();
    }
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