#include <Herkulex.h>
#include <HerkulexMotor.h>
#include <RPIComs.h>

int n = 5; //motor ID - verify your ID !!!!
int n2 = 12;

uint16_t startTime = 0;
uint16_t elapsedTime = 0;
boolean testSetPosBool = false;    // when runTest is true, the moving motor will run once upon restart and when the arduino is uploaded.
boolean testQueueBool = true;

HerkulexMotor myMotor = HerkulexMotor(7, MotorModel::DRS_0602);
HerkulexMotor myMotor2 = HerkulexMotor(5, MotorModel::DRS_0601);

void setup(){
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(9600);    // Open serial communications
  Serial.println("Begin");
  Herkulex.beginSerial1(115200); //open serial port 1
  Herkulex.reboot(n); //reboot first motor
  Herkulex.reboot(n2);
  delay(500);
  Herkulex.initialize(); //initialize motors

  // set the motor positions to 0 to initialize
  myMotor.setPos(0.0);
  myMotor2.setPos(0.0);
  delay(500);
}

void testingQueue(){
  startTime = micros();
  // QUEUE the movements
  myMotor.queueMove(-80.0);     // Queue motor 1
  myMotor2.queueMove(50.0);     // Queue motor 2

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

  delay(1200);                // Wait for movement to complete
  
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
  myMotor.setPos(90.0);
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
  rpi.uartRead();

  // If a packet arrived, handle it
  const char* pkt = rpi.getPacket();
  if (pkt != nullptr) {
    Serial.print("Received: ");
    Serial.println(pkt);
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