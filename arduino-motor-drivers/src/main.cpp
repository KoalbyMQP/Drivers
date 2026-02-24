#include <Herkulex.h>
#include <HerkulexMotor.h>
#include <RPIComs.h>


uint16_t startTime = 0;
uint16_t elapsedTime = 0;
boolean testSetPosBool = false;    // when runTest is true, the moving motor will run once upon restart and when the arduino is uploaded.
boolean testQueueBool = false;
boolean testRPi = true;
boolean latency_queuing = false;

HerkulexMotor myMotor = HerkulexMotor(12, MotorModel::DRS_0601);
HerkulexMotor myMotor2 = HerkulexMotor(7, MotorModel::DRS_0602);
RPIComs rpi = RPIComs();


void setup(){
  delay(2000);  // a delay to have time for serial monitor opening on platformio after uploading
  Serial.begin(9600);    // Open serial communications with computer
  Serial.println("Begin");

  Serial3.begin(9600); //begin serial communication with the raspberry pic

  HerkulexMotor::initSerialPorts(115200); // begin serial communications with motor

  myMotor.reboot();
  myMotor2.reboot();

  delay(500);

  HerkulexMotor::initialize(); // initialize all motors

  // set the motor positions to 0 to initialize
  myMotor.setPos(0.0);
  myMotor2.setPos(0.0);
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
  myMotor.queueMove(-80.0);     // Queue motor 1
  myMotor2.queueMove(158);     // Queue motor 2

  elapsedTime = micros() - startTime;

  Serial.print("time to queue 1 movement: ");
  Serial.print(elapsedTime);
  Serial.println(" microseconds");

  delay(1200);  

  startTime = micros();
  // execute all of the queued movements
  Herkulex.actionMoves(10);
  elapsedTime = micros() - startTime;

  Serial.print("time to execute 1 movement: ");
  Serial.print(elapsedTime);
  Serial.println(" microseconds");
  Serial.println();

  delay(3000);                // Wait for movement to complete
  
  float pos1 = myMotor.getPos();
  float pos2 = myMotor2.getPos();
  Serial.print("Motor 1 position: ");
  Serial.println(pos1);
  Serial.print("Motor 2 position: ");
  Serial.println(pos2);
}

void testingSetPos(){
  startTime = micros();
  myMotor.setPos(0.0);
  elapsedTime = micros() - startTime;


  Serial.print("time to send moveOne cmd: ");
  Serial.print(elapsedTime);
  Serial.println(" microseconds");

  delay(1200);

  startTime = micros();
  float myPos = myMotor.getPos();
  elapsedTime = micros() - startTime;

  Serial.print("position: ");
  Serial.print(myPos);
  Serial.println(" degrees");

  Serial.print("time to read one: ");
  Serial.print(elapsedTime);
  Serial.println(" microseconds");

  delay(1200);

  startTime = micros();
  myMotor.setPos(-90.0);
  elapsedTime = micros() - startTime;


  Serial.print("time to send moveOne cmd: ");
  Serial.print(elapsedTime);
  Serial.println(" microseconds");

  delay(1200);

  startTime = micros();
  myPos = myMotor.getPos();
  elapsedTime = micros() - startTime;

  Serial.print("position: ");
  Serial.print(myPos);
  Serial.println(" degrees");

  Serial.print("time to read one: ");
  Serial.print(elapsedTime);
  Serial.println(" microseconds");
}


void loop(){

  // this loops
  while(testRPi){
    testingRPi();
    testRPi = true;
  }


  while(testSetPosBool){
    testingSetPos();
    testSetPosBool = false;
  }

  while (testQueueBool){
    testingQueue();
    testQueueBool = false;
  }
  
}