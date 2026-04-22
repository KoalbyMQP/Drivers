#include "Arduino.h"
#include "IMU.h"

IMU::IMU(int32_t sensorID, uint8_t address)
    : _bno(sensorID, address, &Wire),
      _data{},
      _address(address),
      _requested(false) {}

// ---------------------------------------------------------------------------
// begin — initialize I2C and start NDOF fusion mode
// Adafruit's begin() defaults to NDOF (0x0C), so no manual mode write needed.
// ---------------------------------------------------------------------------
bool IMU::begin() {
    Wire.setClock(400000);      // 400kHz — stable for fusion mode polling
    if (!_bno.begin()) {
        return false;
    }

    return true;
}

// ---------------------------------------------------------------------------
// writeRegister — helper to write a single byte to a BNO055 register
// ---------------------------------------------------------------------------
void IMU::writeRegister(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_address);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

// ---------------------------------------------------------------------------
// requestUpdate — Phase 1
// Points the BNO055 register cursor at GYR_DATA_X_LSB (0x14), the start of
// the gyroscope output registers.
// endTransmission(false) holds the bus open (no STOP) so the follow-up
// requestFrom in collectUpdate can issue a repeated START.
// ---------------------------------------------------------------------------
void IMU::requestUpdate() {
    Wire.beginTransmission(_address);
    Wire.write(BURST_START_REG);        // 0x14 — GYR_DATA_X_LSB
    Wire.endTransmission(false);        // no STOP — keeps bus held
    _requested = true;
}

// ---------------------------------------------------------------------------
// collectUpdate — Phase 2
// Issues a single 26-byte burst read from 0x14 (GYR_DATA_X_LSB) through
// 0x2D (LIA_DATA_Z_MSB), covering gyro and linear acceleration in one I2C
// transaction. Bytes in between (Euler, Quaternion) are captured but ignored.
//
// Burst layout (datasheet register map, Table 4-2):
//   Offset  0– 5 : GYR_DATA X/Y/Z  (0x14–0x19)   — angular velocity
//   Offset  6–11 : EUL_DATA H/R/P  (0x1A–0x1F)   — (unused, Euler angles)
//   Offset 12–19 : QUA_DATA W/X/Y/Z(0x20–0x27)   — (unused, Quaternion)
//   Offset 20–25 : LIA_DATA X/Y/Z  (0x28–0x2D)   — linear acceleration
//
// Also reads the calibration status register separately (0x35).
// ---------------------------------------------------------------------------
bool IMU::collectUpdate() {
    if (!_requested) return false;

    uint8_t count = Wire.requestFrom(_address, BURST_READ_LEN, (uint8_t)1);
    if (count < BURST_READ_LEN) {
        _requested = false;
        return false;
    }

    uint8_t buf[BURST_READ_LEN];
    Wire.readBytes(buf, BURST_READ_LEN);
    parseBuffer(buf);

    // Read all four calibration fields from CALIB_STAT register (0x35)
    Wire.beginTransmission(_address);
    Wire.write(CALIB_STAT_REG);
    Wire.endTransmission(false);
    Wire.requestFrom(_address, (uint8_t)1, (uint8_t)1);
    if (Wire.available()) {
        uint8_t cal = Wire.read();
        _data.cal_sys   = (cal >> 6) & 0x03;  // bits [7:6]
        _data.cal_gyro  = (cal >> 4) & 0x03;  // bits [5:4]
        _data.cal_accel = (cal >> 2) & 0x03;  // bits [3:2]
        _data.cal_mag   =  cal       & 0x03;  // bits [1:0]
    }

    _requested = false;
    return true;
}

// ---------------------------------------------------------------------------
// parseBuffer — convert raw 26-byte burst into float values
//
// Angular velocity (gyro) — bytes 0–5, offset GYRO_OFFSET_IN_BURST:
//   Scale: 16 LSB per °/s  (BNO055 datasheet Table 3-22, default ±2000 °/s range)
//   1 LSB = 1/16 °/s = 0.0625 °/s
//
// Linear acceleration (accel) — bytes 20–25, offset LIA_OFFSET_IN_BURST:
//   Scale: 100 LSB per m/s²  (datasheet Table 3-17)
//   1 LSB = 1/100 m/s² = 0.01 m/s²
//
// Both use little-endian signed 16-bit format: LSB at lower address.
// ---------------------------------------------------------------------------
void IMU::parseBuffer(uint8_t* buf) {
    // --- Angular velocity (°/s) ---
    // BNO055 default range is ±2000 °/s → sensitivity = 16 LSB/°/s (Table 3-22)
    const float GYRO_SCALE = 1.0f / 16.0f;
    const uint8_t g = GYRO_OFFSET_IN_BURST;
    _data.gyro[0] = (int16_t)((buf[g+1] << 8) | buf[g+0]) * GYRO_SCALE;  // X
    _data.gyro[1] = (int16_t)((buf[g+3] << 8) | buf[g+2]) * GYRO_SCALE;  // Y
    _data.gyro[2] = (int16_t)((buf[g+5] << 8) | buf[g+4]) * GYRO_SCALE;  // Z

    // --- Linear acceleration (m/s², gravity removed) ---
    // Scale: 100 LSB/m/s²  (datasheet Table 3-17)
    const float ACCEL_SCALE = 1.0f / 100.0f;
    const uint8_t a = LIA_OFFSET_IN_BURST;
    _data.accel[0] = (int16_t)((buf[a+1] << 8) | buf[a+0]) * ACCEL_SCALE;  // X
    _data.accel[1] = (int16_t)((buf[a+3] << 8) | buf[a+2]) * ACCEL_SCALE;  // Y
    _data.accel[2] = (int16_t)((buf[a+5] << 8) | buf[a+4]) * ACCEL_SCALE;  // Z
}

// ---------------------------------------------------------------------------
// Public accessors
// ---------------------------------------------------------------------------
IMUData IMU::getData() const { return _data; }

void IMU::getAccelArray(float out[3]) const {
    out[0] = _data.accel[0];
    out[1] = _data.accel[1];
    out[2] = _data.accel[2];
}

void IMU::getGyroArray(float out[3]) const {
    out[0] = _data.gyro[0];
    out[1] = _data.gyro[1];
    out[2] = _data.gyro[2];
}

// Formats fused data as CSV: "aX,aY,aZ,gX,gY,gZ"
// Linear acceleration in m/s², angular velocity in °/s
int IMU::formatPacket(char* buf, size_t bufSize) const {
    return snprintf(buf, bufSize,
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
        _data.gyro[0],  _data.gyro[1],  _data.gyro[2],
        _data.accel[0], _data.accel[1], _data.accel[2]);
}
