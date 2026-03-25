#ifndef IMU_h
#define IMU_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Data structure holding all orientation output from the BNO055
struct IMUData {
    // Euler angles (degrees)
    float heading;      // rotation about Z axis (yaw),   0–360°
    float roll;         // rotation about X axis,         -180–180°
    float pitch;        // rotation about Y axis,         -90–90°

    // Quaternion components (unit quaternion)
    float qw;
    float qx;
    float qy;
    float qz;

    // Calibration status (0 = uncalibrated, 3 = fully calibrated)
    uint8_t cal_sys;
    uint8_t cal_gyro;
    uint8_t cal_accel;
    uint8_t cal_mag;
};

class IMU {
    public:
        // Constructor — sensorID is an arbitrary identifier, address is the I2C address (0x28 or 0x29)
        IMU(int32_t sensorID = 55, uint8_t address = BNO055_ADDRESS_A);

        // Initialize the sensor; returns true on success
        bool begin();

        // Read latest data from the sensor into internal state
        void update();

        // Return the most recently read data
        IMUData getData() const;

        // Format the IMU data as a CSV string for serial transmission
        // Output format: heading,roll,pitch,qw,qx,qy,qz,cal_sys,cal_gyro,cal_accel,cal_mag
        // buf must be at least bufSize bytes; returns number of characters written
        int formatPacket(char* buf, size_t bufSize) const;

        // Returns true only once all four calibration values reach 3
        bool isCalibrated() const;

        // Getters
        float getHeading() const { return _data.heading; }
        float getRoll()    const { return _data.roll;    }
        float getPitch()   const { return _data.pitch;   }

    private:
        Adafruit_BNO055 _bno;
        IMUData _data;
};

#endif