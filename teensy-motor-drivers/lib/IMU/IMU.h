#ifndef IMU_h
#define IMU_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// IMUData holds raw accelerometer readings in m/s²
// Index mapping:
//   accel[0] = X  (remapped: physical -Y)
//   accel[1] = Y  (remapped: physical +X)
//   accel[2] = Z  (remapped: physical +Z)
struct IMUData {
    float accel[3];         // [0]=X, [1]=Y, [2]=Z  in m/s²
    uint8_t cal_accel;      // 0-3, 3 = fully calibrated
};

class IMU {
    public:
        IMU(int32_t sensorID = 55, uint8_t address = BNO055_ADDRESS_A);

        // Initializes I2C, applies axis remap, starts ACCONLY mode.
        // Returns false if sensor not detected.
        bool begin();

        // Phase 1: prime the BNO055 to serve accel bytes.
        // endTransmission(false) holds bus open for follow-up read.
        void requestUpdate();

        // Phase 2: read accel bytes from RX buffer.
        // Call after other work has been done in the loop.
        bool collectUpdate();

        // Returns full IMUData struct
        IMUData getData() const;

        // Returns pointer to internal accel array [X, Y, Z] in m/s²
        // Usage: const float* a = imu.getAccel();  → a[0], a[1], a[2]
        const float* getAccel() const { return _data.accel; }

        // Individual axis accessors
        float getAccelX() const { return _data.accel[0]; }
        float getAccelY() const { return _data.accel[1]; }
        float getAccelZ() const { return _data.accel[2]; }

        // Copies accel values into a user-provided array[3]
        // Usage: float buf[3]; imu.getAccelArray(buf);
        void getAccelArray(float out[3]) const;

        // Accel calibration status (0-3)
        uint8_t getCalAccel() const { return _data.cal_accel; }

        // Formats accel as CSV: "X,Y,Z,cal_accel"
        int formatPacket(char* buf, size_t bufSize) const;

        bool isDoneCollecting() const { return !_requested; }

    private:
        Adafruit_BNO055 _bno;
        IMUData _data;
        uint8_t _address;
        bool _requested = false;

        // Accelerometer registers
        // BNO055 register 0x08: ACC_DATA_X_LSB — 6 bytes total (X, Y, Z)
        // Calibration status register: 0x35
        static constexpr uint8_t ACCEL_START_REG = 0x08;  // ACC_DATA_X_LSB
        static constexpr uint8_t ACCEL_READ_LEN  = 6;     // X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
        static constexpr uint8_t CALIB_STAT_REG  = 0x35;  // calibration status register

        // Axis remap registers
        static constexpr uint8_t BNO055_OPR_MODE_ADDR  = 0x3D;
        static constexpr uint8_t BNO055_AXIS_MAP_CONFIG = 0x41;
        static constexpr uint8_t BNO055_AXIS_MAP_SIGN   = 0x42;
        static constexpr uint8_t MODE_CONFIG            = 0x00;
        static constexpr uint8_t MODE_ACCONLY           = 0x01;  // accel only, no gyro/mag

        void writeRegister(uint8_t reg, uint8_t val);
        void parseBuffer(uint8_t* buf);
};

#endif
