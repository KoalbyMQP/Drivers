#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>

#include <BoardConfig.h>
#include <HerkulexMotor.h>
#include <SerialBusManager.h>

namespace {

constexpr uint8_t MOTOR_BUS = 1;

constexpr MotorModel configuredMotorModel() {
  return BoardConfig::MOTOR_MODEL == BoardConfig::MotorVariant::DRS_0601
             ? MotorModel::DRS_0601
             : MotorModel::DRS_0602;
}

// The connection scan sends only HSTAT requests. Try the configured rate
// first, then every standard Herkulex baud rate.
constexpr uint32_t SUPPORTED_MOTOR_BAUDS[] = {
    115200,
    57600,
    200000,
    250000,
    400000,
    500000,
    666666,
    1000000,
};

MotorRef motor = HerkulexMotor(
                     BoardConfig::MOTOR_ID,
                     configuredMotorModel(),
                     MOTOR_BUS,
                     BoardConfig::MIN_ANGLE_DEG,
                     BoardConfig::MAX_ANGLE_DEG)
                     .getMotorRef();

bool motorOnline = false;
bool torqueEnabled = false;
uint32_t activeMotorBaud = BoardConfig::MOTOR_BAUD;
uint8_t lastStatusError = 0;
uint8_t lastStatusDetail = 0;

HerkulexClass& motorBus() {
  return SerialBusManager::getBus(MOTOR_BUS);
}

const char* configuredModelName() {
  switch (BoardConfig::MOTOR_MODEL) {
    case BoardConfig::MotorVariant::DRS_0601:
      return "DRS-0601";
    case BoardConfig::MotorVariant::DRS_0602:
      return "DRS-0602";
    default:
      return "unsupported model";
  }
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  position       Read the current angle");
  Serial.println("  move <degrees> Move within the configured safety limits");
  Serial.println("  torque on      Enable motor torque");
  Serial.println("  torque off     Release motor torque");
  Serial.println("  status         Read the motor status bytes");
  Serial.println("  rescan         Send STAT across all baud rates and IDs");
  Serial.println("  zeroall        At 115200 baud, command IDs 0-253 to 0 degrees");
  Serial.println("  help           Show this list");
  Serial.println();
}

void moveAllIdsToZero() {
  constexpr uint32_t TEST_BAUD = 115200;
  const uint16_t ZERO_RAW =
      ModelInfo[static_cast<int>(configuredMotorModel())].zeroSteps;
  const uint8_t playTime =
      static_cast<uint8_t>(BoardConfig::MOVE_TIME_MS / CONVERT_PLAYTIME_TO_MS);

  // This diagnostic intentionally does not depend on a successful STAT scan.
  // Reset the UART to the requested rate and enable every attached servo first.
  motorBus().updateSerialBaud(TEST_BAUD);
  delay(20);
  motorBus().clearError(PACKET_CONSTS::ALL_SERVOS);
  delay(20);
  motorBus().torqueON(PACKET_CONSTS::ALL_SERVOS);
  delay(20);

  // The hardware ID is unknown, so address each valid Herkulex ID explicitly.
  // The raw zero differs by model: 1024 for DRS-0601 and 16384 for DRS-0602.
  for (uint16_t id = 0; id < PACKET_CONSTS::ALL_SERVOS; ++id) {
    const motorMoveInfo move = {
        ZERO_RAW,
        LED_BLUE,
        static_cast<uint8_t>(id),
        playTime,
    };
    motorBus().moveOne(move);
  }

  activeMotorBaud = TEST_BAUD;
  torqueEnabled = true;
  Serial.print("Sent ");
  Serial.print(configuredModelName());
  Serial.print(" zero target (raw ");
  Serial.print(ZERO_RAW);
  Serial.println(") to IDs 0-253");
  Serial.println("at 115200 baud with torque enabled and 1000 ms play time.");
}

void printPosition() {
  if (!motorOnline) {
    Serial.println("Motor is offline.");
    return;
  }

  const uint16_t raw = motorBus().getPositionBlocking(motor.servoId);
  if (raw == 0xFFFF) {
    Serial.println("Position read failed (timeout or checksum error).");
    return;
  }

  Serial.print("Position: ");
  Serial.print(HerkulexMotor::motorRefRawToDegs(motor, raw), 2);
  Serial.print(" deg (raw ");
  Serial.print(raw & ModelInfo[static_cast<int>(motor.type)].posBitMask);
  Serial.println(")");
}

void printStatus() {
  uint8_t error = 0;
  uint8_t detail = 0;
  if (!motorBus().stat(motor.servoId, &error, &detail)) {
    motorOnline = false;
    Serial.println("No status response from the motor.");
    return;
  }

  motorOnline = true;
  Serial.print("Status error: 0x");
  Serial.print(error, HEX);
  Serial.print("  detail: 0x");
  Serial.println(detail, HEX);
}

bool parseAngle(const String& text, float& angle) {
  const char* start = text.c_str();
  char* end = nullptr;
  angle = strtof(start, &end);

  if (end == start) return false;
  while (*end != '\0' && isspace(static_cast<unsigned char>(*end))) ++end;
  return *end == '\0';
}

void moveMotor(float angle) {
  if (!motorOnline) {
    Serial.println("Motor is offline; run 'status' after checking the wiring.");
    return;
  }
  if (!torqueEnabled) {
    Serial.println("Torque is off; run 'torque on' first.");
    return;
  }
  if (angle < BoardConfig::MIN_ANGLE_DEG || angle > BoardConfig::MAX_ANGLE_DEG) {
    Serial.print("Rejected. Allowed range is ");
    Serial.print(BoardConfig::MIN_ANGLE_DEG, 1);
    Serial.print(" to ");
    Serial.print(BoardConfig::MAX_ANGLE_DEG, 1);
    Serial.println(" degrees.");
    return;
  }

  HerkulexMotor::motorRefQueueMove(motor, angle);
  SerialBusManager::actionAll(BoardConfig::MOVE_TIME_MS);
  Serial.print("Moving to ");
  Serial.print(angle, 2);
  Serial.println(" degrees.");
}

bool findMotorAtCurrentBaud(uint32_t baud) {
  uint8_t error = 0;
  uint8_t detail = 0;

  Serial.print("Sending STAT at ");
  Serial.print(baud);
  Serial.println(" baud...");

  if (motorBus().stat(motor.servoId, &error, &detail)) {
    activeMotorBaud = baud;
    lastStatusError = error;
    lastStatusDetail = detail;
    return true;
  }

  Serial.print("No response from configured ID ");
  Serial.print(motor.servoId);
  Serial.println(". Scanning IDs 0-253...");

  for (uint16_t id = 0; id < 0xFE; ++id) {
    if (id == motor.servoId) continue;
    if (motorBus().stat(static_cast<uint8_t>(id), &error, &detail)) {
      motor.servoId = static_cast<uint8_t>(id);
      activeMotorBaud = baud;
      lastStatusError = error;
      lastStatusDetail = detail;
      return true;
    }
  }
  return false;
}

bool scanBaud(uint32_t baud) {
  motorBus().updateSerialBaud(baud);
  delay(20);
  return findMotorAtCurrentBaud(baud);
}

bool findMotor() {
  if (scanBaud(BoardConfig::MOTOR_BAUD)) return true;

  for (const uint32_t baud : SUPPORTED_MOTOR_BAUDS) {
    if (baud == BoardConfig::MOTOR_BAUD) continue;
    if (scanBaud(baud)) return true;
  }

  return false;
}

void reportScanResult() {
  if (!motorOnline) {
    Serial.println("No STAT response at any supported baud rate or ID.");
    return;
  }

  Serial.print("STAT response from motor ID ");
  Serial.print(motor.servoId);
  Serial.print(" at ");
  Serial.print(activeMotorBaud);
  Serial.println(" baud.");
  Serial.print("Status error: 0x");
  Serial.print(lastStatusError, HEX);
  Serial.print("  detail: 0x");
  Serial.println(lastStatusDetail, HEX);
}

void rescanMotor() {
  torqueEnabled = false;
  motorOnline = findMotor();
  reportScanResult();
}

void handleCommand(String command) {
  command.trim();
  command.toLowerCase();

  if (command == "help") {
    printHelp();
  } else if (command == "position" || command == "pos") {
    printPosition();
  } else if (command == "status") {
    printStatus();
  } else if (command == "rescan") {
    rescanMotor();
  } else if (command == "zeroall" || command == "zero all") {
    moveAllIdsToZero();
  } else if (command == "torque on") {
    if (!motorOnline) {
      Serial.println("Motor is offline.");
      return;
    }
    motorBus().clearError(motor.servoId);
    motorBus().torqueON(motor.servoId);
    torqueEnabled = true;
    Serial.println("Torque enabled.");
  } else if (command == "torque off") {
    motorBus().torqueFree(motor.servoId);
    torqueEnabled = false;
    Serial.println("Torque released.");
  } else if (command.startsWith("move ")) {
    float angle = 0.0f;
    if (!parseAngle(command.substring(5), angle)) {
      Serial.println("Invalid angle. Example: move 10.5");
      return;
    }
    moveMotor(angle);
  } else if (command.length() != 0) {
    Serial.println("Unknown command. Enter 'help'.");
  }
}

}  // namespace

void setup() {
  Serial.begin(BoardConfig::DEBUG_BAUD);
  delay(1500);

  Serial.print("Single ");
  Serial.print(configuredModelName());
  Serial.println(" controller starting...");
  SerialBusManager::createBus(MOTOR_BUS);
  rescanMotor();

  if (!motorOnline) {
    Serial.println("Check crossed RX/TX, motor power, common ground, and logic levels.");
  }

  printHelp();
}

void loop() {
  if (Serial.available() > 0) {
    handleCommand(Serial.readStringUntil('\n'));
  }
}
