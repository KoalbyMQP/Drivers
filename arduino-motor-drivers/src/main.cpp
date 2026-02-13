#include <Herkulex.h>
#include <HerkulexMotor.h>
int n=5; //motor ID - verify your ID !!!!

uint16_t startTime = 0;
uint16_t elapsedTime = 0;
float myPos;
boolean runTest = true;

HerkulexMotor myMotor = HerkulexMotor(3, MotorModel::DRS_0602);

void setup(){
  delay(2000);  //a delay to have time for serial monitor opening
  Serial.begin(9600);    // Open serial communications
  Serial.println("Begin");
  Herkulex.beginSerial1(115200); //open serial port 1
  Herkulex.reboot(n); //reboot first motor
  delay(500);
  Herkulex.initialize(); //initialize motors
  delay(200);


}

void loop(){
  while(runTest){
  // this if statement makes sure that this only runs once at the beginning
    startTime = micros();
    myMotor.setPos(0.0);
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

    runTest = false;
  }
  
}