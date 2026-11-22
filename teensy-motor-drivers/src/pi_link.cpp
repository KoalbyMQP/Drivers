// LAPTOP › VS Code   file: teensy-motor-drivers/src/pi_link.cpp   (branch pi-link)
// Raspberry Pi <-> Teensy link for Ava. Message format: sim/robot/protocol.py.
// LEGS-ONLY VERSION: the Pi moves and reads the 10 leg motors. All 15 upper-body motors (chest, arms,
// neck) are switched on and held at UPPER_HOLD_DEG from the first "torque on", and never follow the Pi.
// The message code (CRC, frames, watchdog) is copied from sim/robot/teensy/pi_link_reference.ino.
// DRAFT: written from the project files, not yet compiled.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <HerkulexMotor.h>
#include <SerialBusManager.h>
#include <motorDefs.h>

#define USE_IMU 0   // 0 = IMU disconnected: never touch it (Pi config: interface: none). 1 = read the BNO055.

// Message types (MSG_ prefix so they cannot clash with names in Herkulex.h)
static const uint8_t MSG_PING = 0x01, MSG_TORQUE = 0x02, MSG_SET_TARGETS = 0x03, MSG_GET_STATE = 0x04;
static const uint8_t MSG_PONG = 0x81, MSG_ACK = 0x82, MSG_STATE = 0x84;
static const uint16_t FW_VERSION = 3;
static const uint32_t WATCHDOG_MS = 200;    // must match teensy.watchdog_ms in config.yaml

// ===================== WHICH MOTORS THE PI CONTROLS =============================================
// Global ID = bus number x 10 + servo ID on that bus.
// LEG_IDS: the 10 leg motors, which are allMotors[0..9]. The Pi moves them; the firmware reads them.
static const uint8_t LEG_IDS[] = {11, 12, 13, 14, 15,    // left leg:  pelvis-lift, pelvis-twist, thigh, calf, ankle
                                  21, 22, 23, 24, 25};   // right leg: same order (checked by the benchmark)
// UPPER_REPORT_IDS: one ID per upper-body URDF joint (11 of them). The firmware tells the Pi they sit at
// UPPER_HOLD_DEG and ignores Pi targets for them. They must match the upper-body ids in config.yaml.
static const uint8_t UPPER_REPORT_IDS[] = {31, 32, 33, 34, 35, 42, 43, 44, 52, 53, 54};
// UPPER_HOLD_DEG: the angle every upper-body motor (allMotors[10..24], neck and wrist pitch too) holds.
static const float UPPER_HOLD_DEG = 0.0f;
// ================================================================================================

static const uint8_t N_LEGS = sizeof(LEG_IDS);            // 10 (legs are the first 10 entries of allMotors)
static const uint8_t N_UPPER = sizeof(UPPER_REPORT_IDS);  // 11
static const uint8_t N_SERVOS = N_LEGS + N_UPPER;         // 21 joints: what the Pi sees

static uint16_t rawPos[TOTAL_COUNT];      // raw positions, written by the background poller
static float lastGoodRad[TOTAL_COUNT];    // last valid angle of every leg motor, radians
static uint8_t badReads[TOTAL_COUNT];     // failed reads in a row
static uint8_t statErr[TOTAL_COUNT];      // last HerkuleX status error byte
static uint8_t statDetail[TOTAL_COUNT];   // last HerkuleX status detail byte
static uint8_t statNext = 0;              // which leg motor gets a status check next
static uint8_t torqueMode = 0;            // legs: 0 free, 1 brake, 2 on
static bool upperHeld = false;            // upper body switched on and sent to UPPER_HOLD_DEG
static uint32_t lastCommandMs = 0;

static uint8_t reportedId(uint8_t i) {    // the i-th joint the Pi sees: legs first, then upper body
  return i < N_LEGS ? LEG_IDS[i] : UPPER_REPORT_IDS[i - N_LEGS];
}

// ---- IMU side: BNO055 board wired to the Teensy's I2C pins (only used when USE_IMU is 1) ---------
static Adafruit_BNO055 bno(55, BNO055_ADDRESS_A, &Wire);
static bool imuOk = false;                // true once the BNO055 answered at start-up
static uint16_t imuCount = 0;             // goes up by one for every new reading
static float imuQuat[4] = {1, 0, 0, 0};   // orientation w, x, y, z
static float imuGyro[3] = {0, 0, 0};      // angular velocity, rad/s, in the IMU's own axes
static uint8_t imuCal = 0;                // calibration: sys << 6 | gyro << 4 | accel << 2 | mag
static uint32_t lastImuMs = 0;

static void pollImu() {                   // read the BNO055 every 10 ms (its fusion output rate)
  if (!imuOk || millis() - lastImuMs < 10) return;
  lastImuMs = millis();
  imu::Quaternion q = bno.getQuat();
  imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);   // degrees per second
  imuQuat[0] = q.w(); imuQuat[1] = q.x(); imuQuat[2] = q.y(); imuQuat[3] = q.z();
  imuGyro[0] = g.x() * DEG_TO_RAD; imuGyro[1] = g.y() * DEG_TO_RAD; imuGyro[2] = g.z() * DEG_TO_RAD;
  uint8_t s = 0, gy = 0, a = 0, m = 0;
  bno.getCalibration(&s, &gy, &a, &m);
  imuCal = (s << 6) | (gy << 4) | (a << 2) | m;
  imuCount++;
}

// ---- servo side ---------------------------------------------------------------------------------
static int motorIndex(uint8_t gid) {      // global ID -> index in allMotors[], -1 if unknown
  for (int i = 0; i < TOTAL_COUNT; i++) {
    if (allMotors[i].busId == gid / 10 && allMotors[i].servoId == gid % 10) return i;
  }
  return -1;
}

static bool isBadRaw(uint16_t r) { return r == 0xFFFF || r == 0xF0F0 || r == 0xF1F1; }

static void drainBuses() {                // throw away late or stray reply bytes
  while (Serial1.available()) Serial1.read();
  while (Serial2.available()) Serial2.read();
  while (Serial3.available()) Serial3.read();
  while (Serial4.available()) Serial4.read();
  while (Serial5.available()) Serial5.read();
}

// Keeps reading the 10 leg motors (both leg buses in parallel). Between two full sweeps the buses are
// idle, so that is when results are stored and one leg motor's status is checked.
static void pollServos() {
  if (!SerialBusManager::isDoneCollecting()) {
    SerialBusManager::tick(allMotors, rawPos, N_LEGS);
    return;
  }
  for (int i = 0; i < N_LEGS; i++) {
    if (isBadRaw(rawPos[i])) {
      if (badReads[i] < 255) badReads[i]++;
    } else {
      lastGoodRad[i] = HerkulexMotor::motorRefRawToDegs(allMotors[i], rawPos[i]) * DEG_TO_RAD;
      badReads[i] = 0;
    }
    rawPos[i] = 0xFFFF;                   // not read yet in the next sweep
  }
  drainBuses();
  uint8_t e = 0, d = 0;                   // status check of one leg motor (a few ms)
  if (SerialBusManager::getBus(allMotors[statNext].busId).stat(allMotors[statNext].servoId, &e, &d)) {
    statErr[statNext] = e;
    statDetail[statNext] = d;
  }
  statNext = (statNext + 1) % N_LEGS;
  drainBuses();
  SerialBusManager::requestAllPositions(allMotors, rawPos, N_LEGS);
}

static void setTargets(const uint8_t *ids, const float *rad, uint8_t n, uint16_t playtime_ms) {
  if (torqueMode != 2) return;            // only move when the Pi has switched torque on
  for (uint8_t i = 0; i < n; i++) {
    int m = motorIndex(ids[i]);
    if (m >= 0 && m < N_LEGS) HerkulexMotor::motorRefQueueMove(allMotors[m], rad[i] * RAD_TO_DEG);
  }                                       // upper-body targets from the Pi are ignored
  SerialBusManager::actionAll(playtime_ms);   // one move packet per leg bus, both start together
}

static void holdCurrentPose(int playtimeMs) {   // aim every leg motor at where it is now
  for (int i = 0; i < N_LEGS; i++) {
    if (badReads[i] == 0) HerkulexMotor::motorRefQueueMove(allMotors[i], lastGoodRad[i] * RAD_TO_DEG);
  }
  SerialBusManager::actionAll(playtimeMs);
}

static void holdUpperBody() {             // chest + arm buses: torque on, all 15 motors to UPPER_HOLD_DEG in 2 s
  for (uint8_t b = 3; b <= 5; b++) SerialBusManager::getBus(b).torqueON(PACKET_CONSTS::ALL_SERVOS);
  for (int i = N_LEGS; i < TOTAL_COUNT; i++) HerkulexMotor::motorRefQueueMove(allMotors[i], UPPER_HOLD_DEG);
  SerialBusManager::actionAll(2000);
  upperHeld = true;
}

static void setTorque(uint8_t mode) {     // the Pi's torque commands act on the two leg buses only
  if (mode == 2) holdCurrentPose(100);    // no jump when torque comes on
  for (uint8_t b = 1; b <= 2; b++) {
    HerkulexClass &bus = SerialBusManager::getBus(b);
    if (mode == 2) bus.torqueON(PACKET_CONSTS::ALL_SERVOS);
    else if (mode == 1) bus.torqueBrake(PACKET_CONSTS::ALL_SERVOS);
    else bus.torqueFree(PACKET_CONSTS::ALL_SERVOS);
  }
  torqueMode = mode;
  if (mode == 2) {
    holdCurrentPose(100);                 // again, now that torque is on
    if (!upperHeld) holdUpperBody();      // first torque on: the upper body goes to zero and stays there
  }
}

static void readServo(uint8_t gid, float &rad, int16_t &load, uint8_t &err, uint8_t &detail) {
  load = 0;                               // not measured yet
  for (uint8_t j = 0; j < N_UPPER; j++) { // upper body: reported as held, never read
    if (UPPER_REPORT_IDS[j] == gid) { rad = UPPER_HOLD_DEG * DEG_TO_RAD; err = 0; detail = 0; return; }
  }
  int m = motorIndex(gid);
  if (m < 0 || m >= N_LEGS) { rad = 0.0f; err = 0x08; detail = 0; return; }
  rad = lastGoodRad[m];
  err = statErr[m];
  detail = statDetail[m];
  if (badReads[m] >= 3) err |= 0x08;      // no valid reading 3 sweeps in a row: the Pi stops
}

static float supplyVoltage() { return NAN; }   // not measured yet: the Pi skips its voltage check

// ---- message side (from pi_link_reference.ino) ---------------------------------------------------
static uint16_t crc16(const uint8_t *d, size_t n) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= (uint16_t)d[i] << 8;
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

static void sendFrame(uint8_t type, const uint8_t *payload, uint16_t len) {
  static uint8_t buf[512];
  buf[0] = 0xAA; buf[1] = 0x55; buf[2] = type; buf[3] = len & 0xFF; buf[4] = len >> 8;
  memcpy(buf + 5, payload, len);
  uint16_t crc = crc16(buf + 2, 3 + len);
  buf[5 + len] = crc & 0xFF; buf[6 + len] = crc >> 8;
  Serial.write(buf, 7 + len);
}

static void sendState(uint16_t seq) {
  static uint8_t p[16 + 9 * 32];
  uint32_t t = micros();
  float v = supplyVoltage();
  memcpy(p, &seq, 2); memcpy(p + 2, &t, 4); memcpy(p + 6, &v, 4); p[10] = N_SERVOS;
  uint16_t off = 11;
  for (uint8_t i = 0; i < N_SERVOS; i++) {
    uint8_t id = reportedId(i);
    float rad; int16_t load; uint8_t err, det;
    readServo(id, rad, load, err, det);
    p[off] = id; memcpy(p + off + 1, &rad, 4); memcpy(p + off + 5, &load, 2);
    p[off + 7] = err; p[off + 8] = det; off += 9;
  }
  // IMU block, read on the Pi by protocol.parse_imu (Appendix D1), 32 bytes:
  // valid u8, count u16, quaternion w,x,y,z f32, gyro x,y,z f32 (rad/s), calibration u8
  p[off] = imuOk ? 1 : 0; memcpy(p + off + 1, &imuCount, 2);
  memcpy(p + off + 3, imuQuat, 16); memcpy(p + off + 19, imuGyro, 12); p[off + 31] = imuCal;
  off += 32;
  sendFrame(MSG_STATE, p, off);
}

static void handleFrame(uint8_t type, const uint8_t *p, uint16_t len) {
  lastCommandMs = millis();
  switch (type) {
    case MSG_PING: {
      uint8_t r[3 + 32];
      memcpy(r, &FW_VERSION, 2); r[2] = N_SERVOS;
      for (uint8_t i = 0; i < N_SERVOS; i++) r[3 + i] = reportedId(i);
      sendFrame(MSG_PONG, r, 3 + N_SERVOS);
      break;
    }
    case MSG_TORQUE:
      if (len >= 1) { setTorque(p[0]); sendFrame(MSG_ACK, p, 1); }
      break;
    case MSG_SET_TARGETS: {
      if (len < 5) break;
      uint16_t seq, playtime; memcpy(&seq, p, 2); memcpy(&playtime, p + 2, 2);
      uint8_t n = p[4];
      if (n > 32) n = 32;
      if (len < 5 + 5 * n) break;
      uint8_t ids[32]; float rad[32];
      for (uint8_t i = 0; i < n; i++) { ids[i] = p[5 + 5 * i]; memcpy(&rad[i], p + 6 + 5 * i, 4); }
      setTargets(ids, rad, n, playtime);
      sendState(seq);
      break;
    }
    case MSG_GET_STATE: {
      uint16_t seq = 0;
      if (len >= 2) memcpy(&seq, p, 2);
      sendState(seq);
      break;
    }
  }
}

static void pollPi() {                    // byte-by-byte frame parser (never blocks)
  static uint8_t buf[600]; static uint16_t n = 0;
  while (Serial.available()) {
    uint8_t b = Serial.read();
    if (n == 0 && b != 0xAA) continue;
    if (n == 1 && b != 0x55) { n = (b == 0xAA) ? 1 : 0; continue; }
    buf[n++] = b;
    if (n >= 5) {
      uint16_t len = buf[3] | (buf[4] << 8);
      if (len > sizeof(buf) - 7) { n = 0; continue; }
      if (n == 7 + len) {
        uint16_t crc = buf[5 + len] | (buf[6 + len] << 8);
        if (crc == crc16(buf + 2, 3 + len)) handleFrame(buf[2], buf + 5, len);
        n = 0;
      }
    }
  }
}

// ---- called from main.cpp --------------------------------------------------------------------------
void piLinkSetup() {
  for (int i = 0; i < TOTAL_COUNT; i++) {
    rawPos[i] = 0xFFFF; lastGoodRad[i] = 0.0f; badReads[i] = 0; statErr[i] = 0; statDetail[i] = 0;
  }
#if USE_IMU
  Wire.setClock(400000);
  imuOk = bno.begin(OPERATION_MODE_IMUPLUS);   // gyro + accelerometer fusion, no magnetometer
#endif
  lastCommandMs = millis();
}

void piLinkLoop() {
  pollPi();
  pollServos();
  pollImu();
  if (torqueMode == 2 && millis() - lastCommandMs > WATCHDOG_MS) setTorque(1);   // Pi silent: brake the legs
}