#include "Arduino.h"
#include "IMU.h"

IMU::IMU(int32_t sensorID, uint8_t address)
    : _bno(sensorID, address, &Wire),
      _data{},
      _address(address),
      _requested(false) {}

bool IMU::begin() {
    Wire.setClock(1000000);
    if (!_bno.begin()) {
        return false;
    }
    _bno.setExtCrystalUse(true);
    return true;
}

// Phase 1 — write the starting register address to the BNO055.
// endTransmission(false) sends the address byte but holds the bus open
// (no STOP condition). This primes the BNO055 to serve bytes from
// START_REG onward when the read comes in Phase 2.
void IMU::requestUpdate() {
    Wire.beginTransmission(_address);
    Wire.write(START_REG);
    Wire.endTransmission(false);  // false = no STOP, keeps bus held
    _requested = true;
}

// Phase 2 — issue the read request and pull all bytes from the RX buffer.
// requestFrom() sends a repeated START, reads READ_LEN bytes into the
// RX buffer, then issues a STOP. readBytes() then drains the buffer
// in one memcpy rather than byte-by-byte.
bool IMU::collectUpdate() {
    if (!_requested) return false;

    uint8_t count = Wire.requestFrom(_address, READ_LEN, (uint8_t)1);
    if (count < READ_LEN) {
        _requested = false;
        return false;   // sensor did not return expected number of bytes
    }

    uint8_t buf[READ_LEN];
    Wire.readBytes(buf, READ_LEN);  // drains entire RX buffer in one call

    parseBuffer(buf);
    _requested = false;
    return true;
}

// Parse raw bytes from the RX buffer into _data.
// BNO055 register map from START_REG (0x1A):
// Bytes 0–1:   Euler Heading  (LSB, MSB) — units: 1/16 degree
// Bytes 2–3:   Euler Roll     (LSB, MSB)
// Bytes 4–5:   Euler Pitch    (LSB, MSB)
// Bytes 6–7:   Quaternion W   (LSB, MSB) — units: 1/(2^14)
// Bytes 8–9:   Quaternion X   (LSB, MSB)
// Bytes 10–11: Quaternion Y   (LSB, MSB)
// Bytes 12–13: Quaternion Z   (LSB, MSB)
// Byte  14:    Calibration status register (packed nibbles)
void IMU::parseBuffer(uint8_t* buf) {
    const float EULER_SCALE = 1.0f / 16.0f;
    _data.heading = (int16_t)((buf[1] << 8) | buf[0]) * EULER_SCALE;
    _data.roll    = (int16_t)((buf[3] << 8) | buf[2]) * EULER_SCALE;
    _data.pitch   = (int16_t)((buf[5] << 8) | buf[4]) * EULER_SCALE;

    const float QUAT_SCALE = 1.0f / (1 << 14);
    _data.qw = (int16_t)((buf[7]  << 8) | buf[6])  * QUAT_SCALE;
    _data.qx = (int16_t)((buf[9]  << 8) | buf[8])  * QUAT_SCALE;
    _data.qy = (int16_t)((buf[11] << 8) | buf[10]) * QUAT_SCALE;
    _data.qz = (int16_t)((buf[13] << 8) | buf[12]) * QUAT_SCALE;

    // Calibration packed into one register — two bits per sensor
    uint8_t cal = buf[14];
    _data.cal_mag   = (cal >> 0) & 0x03;
    _data.cal_accel = (cal >> 2) & 0x03;
    _data.cal_gyro  = (cal >> 4) & 0x03;
    _data.cal_sys   = (cal >> 6) & 0x03;
}

IMUData IMU::getData() const { return _data; }

int IMU::formatPacket(char* buf, size_t bufSize) const {
    return snprintf(buf, bufSize,
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%u,%u,%u",
        _data.heading, _data.roll,  _data.pitch,
        _data.qw,      _data.qx,   _data.qy,   _data.qz,
        _data.cal_sys, _data.cal_gyro, _data.cal_accel, _data.cal_mag);
}

bool IMU::isCalibrated() const {
    return (_data.cal_sys   == 3 &&
            _data.cal_gyro  == 3 &&
            _data.cal_accel == 3 &&
            _data.cal_mag   == 3);
}