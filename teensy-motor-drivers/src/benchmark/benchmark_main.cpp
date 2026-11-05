// Ava motor benchmark — Step 3: hello program.
// No motor code yet: this only proves the laptop can program the Teensy.
#include <Arduino.h>

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  while (!Serial && millis() < 4000) {}   // wait up to 4 s for the serial monitor
  Serial.println("Ava motor benchmark: hello from the Teensy");
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(900);
  Serial.print("alive, uptime ");
  Serial.print(millis() / 1000);
  Serial.println(" s");
}