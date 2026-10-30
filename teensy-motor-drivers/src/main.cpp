// LAPTOP › VS Code   file: teensy-motor-drivers/src/main.cpp   (branch pi-link)
// Ava firmware with the Raspberry Pi link: the Pi decides, this firmware moves the servos,
// keeps reading them, and answers the Pi (see pi_link.cpp). DRAFT: not yet compiled.
#include <Arduino.h>
#include <HerkulexMotor.h>
#include <SerialBusManager.h>
#include <motorDefs.h>

void piLinkSetup();   // in pi_link.cpp
void piLinkLoop();    // in pi_link.cpp

void setup() {
  Serial.begin(1000000);              // USB link to the Raspberry Pi: no debug prints on it

  createAvaMotorDef();                // fills allMotors[] from motorDefs.h

  SerialBusManager::createBus(SERIAL_BUS::BUS_L_LEG);
  SerialBusManager::createBus(SERIAL_BUS::BUS_R_LEG);
  SerialBusManager::createBus(SERIAL_BUS::BUS_CHEST);
  SerialBusManager::createBus(SERIAL_BUS::BUS_L_ARM);
  SerialBusManager::createBus(SERIAL_BUS::BUS_R_ARM);
  SerialBusManager::startAllBuses(BAUD_RATE::SPEED_115K);   // keep in step with the motor teammate

  // A missing servo must never freeze the loop. By default a read waits up to 1 s for bytes that
  // never come; with a 0 ms timeout it returns at once and the read is marked as failed.
  Serial1.setTimeout(0);
  Serial2.setTimeout(0);
  Serial3.setTimeout(0);
  Serial4.setTimeout(0);
  Serial5.setTimeout(0);

  delay(500);
  SerialBusManager::initAllMotors();  // reply policy + clear old servo errors
  // Torque stays OFF here. The Pi switches it on when it is ready.

  piLinkSetup();
}

void loop() {
  piLinkLoop();
}