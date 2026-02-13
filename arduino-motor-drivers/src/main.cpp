#include <Herkulex.h>
#include <HerkulexMotor.h>
int n = 5; //motor ID - verify your ID !!!!
int n2 = 12;

uint16_t startTime = 0;
uint16_t elapsedTime = 0;
boolean runTest = false;    // when runTest is true, the moving motor will run once upon restart and when the arduino is uploaded.
boolean testQueueBool = true;

HerkulexMotor myMotor = HerkulexMotor(5, MotorModel::DRS_0601);
HerkulexMotor myMotor2 = HerkulexMotor(12, MotorModel::DRS_0601);

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
  delay(1000);
}

void testingQueue(){
  delay(1000);
  // queue two different motors
  myMotor.queueMove(-20);
  myMotor2.setPos(-50);

  delay(500);
  Herkulex.actionMoves(100);
}


void loop(){
  while(runTest){
    startTime = micros();
    myMotor.setPos(0.0);
    elapsedTime = micros() - startTime;

    Serial.print("time to send moveOne cmd: ");
    Serial.print(elapsedTime);
    Serial.println(" microseconds");

    delay(300);

    startTime = micros();
    myMotor.setPos(20.0);
    elapsedTime = micros() - startTime;

    Serial.print("time to send moveOne cmd: ");
    Serial.print(elapsedTime);
    Serial.println(" microseconds");

    delay(300);

    startTime = micros();
    myMotor.setPos(50.0);
    elapsedTime = micros() - startTime;

    Serial.print("time to send moveOne cmd: ");
    Serial.print(elapsedTime);
    Serial.println(" microseconds");

    runTest = false;
  }

  while (testQueueBool){
    testingQueue();
    testQueueBool = false;
  }
  
}