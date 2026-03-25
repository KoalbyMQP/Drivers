#include "Arduino.h"
#include "IMU.h"

// Constructor — store sensorID and I2C address, pass them to the BNO055 driver
IMU::IMU(int32_t sensorID, uint8_t address)
    : _bno(sensorID, address, &Wire), _data{} {}


// Initialize the BNO055 over I2C
// Call once inside setup(). Returns false if the sensor is not detected.
bool IMU::begin() {
    if (!_bno.begin()) {
        return false;
    }

    // Use the external 32.768 kHz crystal for better long-term timing accuracy
    _bno.setExtCrystalUse(true);

    return true;
}


// Read the latest Euler angles, quaternion, and calibration status from the sensor
// Call this once per control loop iteration, alongside motor position reads
void IMU::update() {
    // Euler angles
    sensors_event_t orientationData;
    _bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    _data.heading = orientationData.orientation.x;
    _data.roll    = orientationData.orientation.y;
    _data.pitch   = orientationData.orientation.z;

    // Quaternion
    imu::Quaternion quat = _bno.getQuat();
    _data.qw = quat.w();
    _data.qx = quat.x();
    _data.qy = quat.y();
    _data.qz = quat.z();

    // Calibration status
    _bno.getCalibration(&_data.cal_sys, &_data.cal_gyro, &_data.cal_accel, &_data.cal_mag);
}


// Return a copy of the most recently read IMUData struct
IMUData IMU::getData() const {
    return _data;
}


// Format the current IMU state into a CSV string ready to append to the serial packet
// Output: heading,roll,pitch,qw,qx,qy,qz,cal_sys,cal_gyro,cal_accel,cal_mag
// This mirrors the motor position CSV format used in RPIComs so both can be concatenated
// into a single packet before calling rpi.enqueueTXPacket()
int IMU::formatPacket(char* buf, size_t bufSize) const {
    return snprintf(buf, bufSize,
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u,%u,%u,%u",
        _data.heading, _data.roll,  _data.pitch,
        _data.qw,      _data.qx,   _data.qy,   _data.qz,
        _data.cal_sys, _data.cal_gyro, _data.cal_accel, _data.cal_mag
    );
}


// Returns true when all four calibration values have reached 3 (fully calibrated)
bool IMU::isCalibrated() const {
    return (_data.cal_sys   == 3 &&
            _data.cal_gyro  == 3 &&
            _data.cal_accel == 3 &&
            _data.cal_mag   == 3);
}