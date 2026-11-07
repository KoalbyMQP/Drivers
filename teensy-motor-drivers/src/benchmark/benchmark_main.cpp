// =====================================================================
// Ava motor benchmark - Step 4: read-only ping of the leg motors
//
// Sends only STAT (status) and EEP-read (model number) requests.
// It never switches torque on, never writes a register, never moves a motor.
//
//   Left leg  = bus 1 = Teensy Serial1 (RX1 = pin 0, TX1 = pin 1)
//   Right leg = bus 2 = Teensy Serial2 (RX2 = pin 7, TX2 = pin 8)
// =====================================================================
#include <Arduino.h>
#include <SerialBusManager.h>

// ---------- what we expect to find on each leg bus ----------
struct ExpectedMotor {
  uint8_t     id;      // servo ID on its bus
  const char* joint;   // joint name (URDF)
  uint16_t    model;   // model register: 0x0601 = DRS-0601, 0x0602 = DRS-0602
};

// The same five motors, in chain order, on each leg
const ExpectedMotor LEG_MOTORS[] = {
  {1, "pelvis-lift",  0x0601},
  {2, "pelvis-twist", 0x0601},
  {3, "thigh",        0x0601},
  {4, "knee",         0x0602},
  {5, "ankle",        0x0601},
};
const uint8_t MOTORS_PER_LEG = sizeof(LEG_MOTORS) / sizeof(LEG_MOTORS[0]);

struct LegBus {
  uint8_t         busId;  // SerialBusManager bus number (bus N = Teensy SerialN)
  const char*     name;
  HardwareSerial* port;   // the same UART, used only to throw away stale bytes
  const char*     pins;
};

LegBus LEGS[] = {
  {1, "LEFT LEG",  &Serial1, "RX1 = pin 0, TX1 = pin 1"},
  {2, "RIGHT LEG", &Serial2, "RX2 = pin 7, TX2 = pin 8"},
};
const uint8_t LEG_COUNT = sizeof(LEGS) / sizeof(LEGS[0]);

const uint32_t SUPPORTED_BAUDS[] = {57600, 115200, 200000, 250000,
                                    400000, 500000, 666666, 1000000};
uint32_t busBaud = 115200;  // HerkuleX factory default; Ava's firmware also uses it

// ---------- helpers ----------
void flushInput(HardwareSerial* port) {
  while (port->available() > 0) port->read();
}

void modelName(uint16_t model, char* out, size_t len) {
  snprintf(out, len, "DRS-%04X", model);
}

void printErrorMeaning(uint8_t e) {
  if (e == 0x00) { Serial.print("no errors"); return; }
  if (e & 0x01) Serial.print("input-voltage ");
  if (e & 0x02) Serial.print("pot-limit ");
  if (e & 0x04) Serial.print("over-temperature ");
  if (e & 0x08) Serial.print("invalid-packet ");
  if (e & 0x10) Serial.print("overload ");
  if (e & 0x20) Serial.print("driver-fault ");
  if (e & 0x40) Serial.print("EEPROM-distorted ");
}

void printDetailProblems(uint8_t d) {
  if (d & 0x04) Serial.print("checksum-error ");
  if (d & 0x08) Serial.print("unknown-command ");
  if (d & 0x10) Serial.print("register-range ");
  if (d & 0x20) Serial.print("garbage-detected ");
}

void printHelp() {
  Serial.println();
  Serial.println("Commands (type one, then press Enter):");
  Serial.println("  ping         ask leg motors ID 1-5 for status and model (read-only)");
  Serial.println("  scan         ask every ID 0-253 on both leg buses (read-only, ~4 s)");
  Serial.println("  stress [n]   repeat STAT n times (default 100) on each leg, count replies");
  Serial.println("  led <id> <color>  set one ID's LED: green, blue, red or off (nothing stored)");
  Serial.println("  ledrange <a> <b> <color>  set the LED on IDs a..b (finds a motor that can't reply)");
  Serial.println("  id <bus> <old> <new>  change one motor's ID (stored in the motor)");
    Serial.println("  init                  all motors: reply policy 1 + clear errors (RAM only)");
  Serial.println("  baud <rate>  change the Teensy's bus speed for searching (motors unchanged)");
  Serial.println("  help         show this list");
  Serial.println();
}

// ---------- ping: IDs 1-5 on each leg ----------
uint8_t pingLeg(LegBus& leg) {
  HerkulexClass& bus = SerialBusManager::getBus(leg.busId);
  uint8_t answered = 0, ok = 0;

  Serial.printf("\n== %s  (bus %u, Teensy %s, %lu baud) ==\n",
                leg.name, leg.busId, leg.pins, (unsigned long)busBaud);
  Serial.println("  ID  joint         model     reply     error  detail  torque  meaning");

  for (uint8_t i = 0; i < MOTORS_PER_LEG; i++) {
    const ExpectedMotor& m = LEG_MOTORS[i];
    uint8_t err = 0, det = 0;
    uint16_t model = 0;

    flushInput(leg.port);
    bool statOk = bus.stat(m.id, &err, &det);
    delay(5);

    if (!statOk) {
      Serial.printf("  %2u  %-12s  --------  NO REPLY  --     --      --      no answer from this ID\n",
                    m.id, m.joint);
      continue;
    }
    answered++;

    flushInput(leg.port);
    bool modelOk = bus.getModel(m.id, &model);
    delay(5);

    char name[12];
    if (modelOk) modelName(model, name, sizeof(name));
    else         snprintf(name, sizeof(name), "unknown");

    const char* reply = "OK";
    if (!modelOk)                reply = "MODEL?";
    else if (model != m.model)   reply = "WRONG";
    else if (err != 0x00)        reply = "ERROR";

    Serial.printf("  %2u  %-12s  %-8s  %-8s  0x%02X   0x%02X    %-6s  ",
                  m.id, m.joint, name, reply, err, det, (det & 0x40) ? "on" : "off");
    if (!modelOk) {
      Serial.print("answered STAT but not the model read; ");
    } else if (model != m.model) {
      char want[12];
      modelName(m.model, want, sizeof(want));
      Serial.printf("expected %s; ", want);
    }
    printErrorMeaning(err);
    printDetailProblems(det);
    Serial.println();

    if (strcmp(reply, "OK") == 0) ok++;
  }

  Serial.printf("  %s: %u of %u motors answered, %u of %u OK\n",
                leg.name, answered, MOTORS_PER_LEG, ok, MOTORS_PER_LEG);
  return ok;
}

void ping() {
  uint8_t ok = 0;
  for (uint8_t i = 0; i < LEG_COUNT; i++) ok += pingLeg(LEGS[i]);
  uint8_t total = LEG_COUNT * MOTORS_PER_LEG;
  Serial.printf("\nPING RESULT: %s (%u of %u leg motors OK)\n\n",
                ok == total ? "PASS" : "FAIL", ok, total);
}

// ---------- scan: every ID 0-253 on each leg ----------
void scanLeg(LegBus& leg) {
  HerkulexClass& bus = SerialBusManager::getBus(leg.busId);
  uint8_t found = 0;

  Serial.printf("\n== %s: scanning IDs 0-253 at %lu baud ==\n",
                leg.name, (unsigned long)busBaud);

  for (uint16_t id = 0; id <= 253; id++) {
    uint8_t err = 0, det = 0;
    flushInput(leg.port);
    if (!bus.stat((uint8_t)id, &err, &det)) continue;

    uint16_t model = 0;
    delay(5);
    flushInput(leg.port);
    char name[12];
    if (bus.getModel((uint8_t)id, &model)) modelName(model, name, sizeof(name));
    else                                   snprintf(name, sizeof(name), "unknown");

    Serial.printf("  found ID %3u  model %-8s  error 0x%02X  detail 0x%02X\n",
                  id, name, err, det);
    found++;
    delay(5);
  }
  Serial.printf("  %s: %u motor(s) found\n", leg.name, found);
}

void scan() {
  for (uint8_t i = 0; i < LEG_COUNT; i++) scanLeg(LEGS[i]);
  Serial.println();
}

   // ---------- stress: repeat STAT many times, count replies per ID ----------
   void stressLeg(LegBus& leg, uint16_t rounds) {
     HerkulexClass& bus = SerialBusManager::getBus(leg.busId);
     uint16_t okCount[MOTORS_PER_LEG] = {0};

     Serial.printf("\n== %s: %u rounds of STAT to IDs 1-%u at %lu baud ==\n",
                   leg.name, rounds, MOTORS_PER_LEG, (unsigned long)busBaud);

     for (uint16_t r = 0; r < rounds; r++) {
       for (uint8_t i = 0; i < MOTORS_PER_LEG; i++) {
         uint8_t err = 0, det = 0;
         flushInput(leg.port);
         if (bus.stat(LEG_MOTORS[i].id, &err, &det)) okCount[i]++;
         delay(2);
       }
     }

     Serial.println("  ID  joint         replies   success");
     for (uint8_t i = 0; i < MOTORS_PER_LEG; i++) {
       Serial.printf("  %2u  %-12s  %3u/%-3u   %3u %%\n",
                     LEG_MOTORS[i].id, LEG_MOTORS[i].joint, okCount[i], rounds,
                     (unsigned)(100UL * okCount[i] / rounds));
     }
   }

   void stress(uint16_t rounds) {
     if (rounds == 0) rounds = 100;
     for (uint8_t i = 0; i < LEG_COUNT; i++) stressLeg(LEGS[i], rounds);
     Serial.println();
   }

   // ---------- led: set one ID's LED (RAM write only: nothing stored, no torque) ----------
   void setLedCmd(String args) {
     args.trim();                       // e.g. "3 green"
     int space = args.indexOf(' ');
     if (space < 0) { Serial.println("Usage: led <id> <green|blue|red|off>"); return; }
     int id = args.substring(0, space).toInt();
     String color = args.substring(space + 1);
     color.trim();

     LED_STATE c;
     if      (color == "green") c = LED_GREEN;
     else if (color == "blue")  c = LED_BLUE;
     else if (color == "red")   c = LED_RED;
     else if (color == "off")   c = LED_OFF;
     else { Serial.println("Color must be green, blue, red or off."); return; }

     for (uint8_t i = 0; i < LEG_COUNT; i++) {
       SerialBusManager::getBus(LEGS[i].busId).setLed((uint8_t)id, c);
     }
     Serial.printf("Sent LED %s to ID %d on both leg buses.\n", color.c_str(), id);
   }

      // ---------- id: change one motor's ID (EEPROM write + reboot) ----------
   // Usage: id <bus> <oldID> <newID>    e.g. "id 2 3 30"
   // Only run it when exactly ONE motor on that bus has <oldID>.
   void changeIdCmd(String args) {
     args.trim();
     int s1 = args.indexOf(' ');
     String rest = (s1 < 0) ? String("") : args.substring(s1 + 1);
     rest.trim();
     int s2 = rest.indexOf(' ');
     if (s1 < 0 || s2 < 0) { Serial.println("Usage: id <bus> <oldID> <newID>   e.g. id 2 3 30"); return; }

     int busId = args.substring(0, s1).toInt();
     int oldId = rest.substring(0, s2).toInt();
     int newId = rest.substring(s2 + 1).toInt();

     LegBus* leg = nullptr;
     for (uint8_t i = 0; i < LEG_COUNT; i++) if (LEGS[i].busId == busId) leg = &LEGS[i];
     if (leg == nullptr) { Serial.println("Bus must be 1 (left leg) or 2 (right leg). Nothing changed."); return; }
     if (oldId < 0 || oldId > 253 || newId < 0 || newId > 253 || oldId == newId) {
       Serial.println("IDs must be 0-253 and different. Nothing changed.");
       return;
     }

     HerkulexClass& bus = SerialBusManager::getBus(busId);
     uint8_t err = 0, det = 0;

     flushInput(leg->port);
     if (!bus.stat((uint8_t)oldId, &err, &det)) {
       Serial.printf("No motor answers at ID %d on bus %d. Nothing changed.\n", oldId, busId);
       return;
     }
     delay(5);
     flushInput(leg->port);
     if (bus.stat((uint8_t)newId, &err, &det)) {
       Serial.printf("ID %d is already taken on bus %d. Nothing changed.\n", newId, busId);
       return;
     }

     Serial.printf("Changing ID %d -> %d on bus %d (EEPROM write + reboot, about 2 s)...\n",
                   oldId, newId, busId);
     bus.setID((uint8_t)oldId, (uint8_t)newId);   // driver: EEP write, 1 s, reboot, 1 s
     delay(200);

     flushInput(leg->port);
     bool newAnswers = bus.stat((uint8_t)newId, &err, &det);
     delay(5);
     flushInput(leg->port);
     bool oldAnswers = bus.stat((uint8_t)oldId, &err, &det);

     Serial.printf("  ID %d answers now:    %s\n", newId, newAnswers ? "yes" : "NO");
     Serial.printf("  ID %d still answers:  %s\n", oldId, oldAnswers ? "YES" : "no");
     Serial.println((newAnswers && !oldAnswers) ? "ID CHANGE: DONE" : "ID CHANGE: CHECK FAILED - run scan");
   }


      // ---------- ledrange: set the LED on a range of IDs (finds a motor that hears but can't reply) ----------
   // Usage: ledrange <first> <last> <green|blue|red|off>    e.g. "ledrange 0 127 green"
   void ledRangeCmd(String args) {
     args.trim();
     int s1 = args.indexOf(' ');
     String rest = (s1 < 0) ? String("") : args.substring(s1 + 1);
     rest.trim();
     int s2 = rest.indexOf(' ');
     if (s1 < 0 || s2 < 0) { Serial.println("Usage: ledrange <first> <last> <green|blue|red|off>"); return; }

     int first = args.substring(0, s1).toInt();
     int last  = rest.substring(0, s2).toInt();
     String color = rest.substring(s2 + 1);
     color.trim();

     LED_STATE c;
     if      (color == "green") c = LED_GREEN;
     else if (color == "blue")  c = LED_BLUE;
     else if (color == "red")   c = LED_RED;
     else if (color == "off")   c = LED_OFF;
     else { Serial.println("Color must be green, blue, red or off."); return; }
     if (first < 0 || last > 253 || first > last) { Serial.println("IDs must be 0-253, first <= last."); return; }

     for (int id = first; id <= last; id++) {
       for (uint8_t i = 0; i < LEG_COUNT; i++) {
         SerialBusManager::getBus(LEGS[i].busId).setLed((uint8_t)id, c);
       }
       delay(2);
     }
     Serial.printf("Sent LED %s to IDs %d-%d on both leg buses.\n", color.c_str(), first, last);
   }

// ---------- baud: Teensy side only ----------
void setBaud(uint32_t rate) {
  bool supported = false;
  for (uint32_t b : SUPPORTED_BAUDS) if (b == rate) supported = true;
  if (!supported) {
    Serial.println("Not a HerkuleX baud rate. Use one of: 57600 115200 200000 250000 400000 500000 666666 1000000");
    return;
  }
  for (uint8_t i = 0; i < LEG_COUNT; i++) {
    SerialBusManager::getBus(LEGS[i].busId).updateSerialBaud(rate);
  }
  busBaud = rate;
  Serial.printf("Both leg buses on the Teensy now run at %lu baud (motor settings unchanged).\n",
                (unsigned long)rate);
}

// ---------- Arduino entry points ----------
void setup() {
  Serial.begin(115200);
  while (!Serial) {}  // wait here until the serial monitor is open

  for (uint8_t i = 0; i < LEG_COUNT; i++) SerialBusManager::createBus(LEGS[i].busId);
  SerialBusManager::startAllBuses(BAUD_RATE::SPEED_115K);

  Serial.println("Ava motor benchmark - Step 4: read-only leg ping");
  Serial.println("Nothing in this program moves a motor or switches torque on.");
  printHelp();
}

void loop() {
  if (Serial.available() == 0) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  if      (cmd == "ping")         ping();
  else if (cmd == "scan")         scan();
  else if (cmd.startsWith("stress")) stress((uint16_t)cmd.substring(6).toInt());
  else if (cmd.startsWith("ledrange")) ledRangeCmd(cmd.substring(8));
  else if (cmd.startsWith("led"))    setLedCmd(cmd.substring(3));
  else if (cmd.startsWith("id "))    changeIdCmd(cmd.substring(3));
    else if (cmd == "init") {
    SerialBusManager::initAllMotors();   // RAM writes to all IDs: reply policy = 1, clear errors
    Serial.println("Sent to every motor on both leg buses: reply policy = 1, errors cleared (RAM only, nothing stored).");
  }
  else if (cmd.startsWith("baud")) setBaud((uint32_t)cmd.substring(4).toInt());
  else if (cmd == "help")         printHelp();
  else if (cmd.length() > 0)      Serial.println("Unknown command. Type help.");
}