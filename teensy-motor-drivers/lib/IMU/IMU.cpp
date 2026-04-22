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
// Writes the LIA start register address to the BNO055.
// endTransmission(false) holds the bus open (no STOP) so the follow-up
// requestFrom in collectUpdate can issue a repeated START.
// ---------------------------------------------------------------------------
void IMU::requestUpdate() {
    Wire.beginTransmission(_address);
    Wire.write(ACCEL_START_REG);
    Wire.endTransmission(false);    // no STOP — keeps bus held
    _requested = true;
}

// ---------------------------------------------------------------------------
// collectUpdate — Phase 2
// Issues the read and pulls all 6 fused linear accel bytes from the RX buffer.
// Also reads the calibration status register separately.
// ---------------------------------------------------------------------------
bool IMU::collectUpdate() {
    if (!_requested) return false;

    // Read 6 LIA bytes (X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB)
    uint8_t count = Wire.requestFrom(_address, ACCEL_READ_LEN, (uint8_t)1);
    if (count < ACCEL_READ_LEN) {
        _requested = false;
        return false;
    }

    uint8_t buf[ACCEL_READ_LEN];
    Wire.readBytes(buf, ACCEL_READ_LEN);
    parseBuffer(buf);

    // Read all four calibration fields from CALIB_STAT register
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
// parseBuffer — convert raw 6 bytes into float m/s² values
//
// BNO055 LIA register map from 0x28:
//   Bytes 0-1: X (LSB, MSB) — raw units: 1 LSB = 1 m/s² / 100
//   Bytes 2-3: Y (LSB, MSB)
//   Bytes 4-5: Z (LSB, MSB)
//
// Scale factor: 1/100 m/s² per LSB (same as raw accel, 100 LSB/m/s²)
// ---------------------------------------------------------------------------
void IMU::parseBuffer(uint8_t* buf) {
    const float ACCEL_SCALE = 1.0f / 100.0f;   // 100 LSB per m/s²
    _data.accel[0] = (int16_t)((buf[1] << 8) | buf[0]) * ACCEL_SCALE;  // X
    _data.accel[1] = (int16_t)((buf[3] << 8) | buf[2]) * ACCEL_SCALE;  // Y
    _data.accel[2] = (int16_t)((buf[5] << 8) | buf[4]) * ACCEL_SCALE;  // Z
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

// Formats as CSV: "X,Y,Z"
int IMU::formatPacket(char* buf, size_t bufSize) const {
    return snprintf(buf, bufSize,
        "%.4f,%.4f,%.4f",
        _data.accel[0], _data.accel[1], _data.accel[2]);
}