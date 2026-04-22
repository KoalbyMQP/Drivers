#ifndef IMU_h
#define IMU_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// IMUData holds fused sensor readings from the BNO055 in NDOF fusion mode.
//
// Linear acceleration (accel):
//   Gravity-compensated acceleration from the LIA fusion output registers (0x28–0x2D).
//   Scale: 100 LSB per m/s²  →  1 LSB = 0.01 m/s²
//   Index mapping (after axis remap):
//     accel[0] = X  (remapped: physical -Y)
//     accel[1] = Y  (remapped: physical +X)
//     accel[2] = Z  (remapped: physical +Z)
//
// Angular velocity (gyro):
//   Fusion-compensated gyroscope output from registers 0x14–0x19.
//   Scale: 16 LSB per °/s  →  1 LSB = 0.0625 °/s  (default ±2000 °/s range, Table 3-22)
//   Index mapping mirrors the accel axis remap above.
struct IMUData {
    float accel[3];         // [0]=X, [1]=Y, [2]=Z  in m/s²   (fused linear accel, gravity removed)
    float gyro[3];          // [0]=X, [1]=Y, [2]=Z  in °/s    (fused angular velocity)
    uint8_t cal_sys;        // 0–3, bits [7:6] — overall fusion system calibration
    uint8_t cal_gyro;       // 0–3, bits [5:4]
    uint8_t cal_accel;      // 0–3, bits [3:2]
    uint8_t cal_mag;        // 0–3, bits [1:0]
};

class IMU {
    public:
        IMU(int32_t sensorID = 55, uint8_t address = BNO055_ADDRESS_A);

        // Initializes I2C and starts NDOF fusion mode (9-DOF, absolute orientation).
        // Returns false if sensor not detected.
        bool begin();

        // Phase 1: prime the BNO055 to serve fused data bytes.
        // Points the register cursor at GYR_DATA_X_LSB (0x14).
        // endTransmission(false) holds bus open for follow-up read.
        void requestUpdate();

        // Phase 2: read fused gyro + linear accel bytes from RX buffer.
        // Single burst: 0x14–0x2D (26 bytes) — gyro X/Y/Z then LIA X/Y/Z.
        // Call after other work has been done in the loop.
        bool collectUpdate();

        // Returns full IMUData struct
        IMUData getData() const;

        // --- Linear acceleration accessors (m/s², gravity removed) ---
        const float* getAccel() const { return _data.accel; }
        float getAccelX() const { return _data.accel[0]; }
        float getAccelY() const { return _data.accel[1]; }
        float getAccelZ() const { return _data.accel[2]; }
        void getAccelArray(float out[3]) const;

        // --- Angular velocity accessors (°/s) ---
        const float* getGyro() const { return _data.gyro; }
        float getGyroX() const { return _data.gyro[0]; }
        float getGyroY() const { return _data.gyro[1]; }
        float getGyroZ() const { return _data.gyro[2]; }
        void getGyroArray(float out[3]) const;

        // --- Calibration status accessors (0–3, 3 = fully calibrated) ---
        uint8_t getCalSys()   const { return _data.cal_sys; }
        uint8_t getCalGyro()  const { return _data.cal_gyro; }
        uint8_t getCalAccel() const { return _data.cal_accel; }
        uint8_t getCalMag()   const { return _data.cal_mag; }

        // Formats fused data as CSV: "aX,aY,aZ,gX,gY,gZ"
        int formatPacket(char* buf, size_t bufSize) const;

        bool isDoneCollecting() const { return !_requested; }

    private:
        Adafruit_BNO055 _bno;
        IMUData _data;
        uint8_t _address;
        bool _requested = false;

        // Burst read window: GYR_DATA_X_LSB (0x14) → LIA_DATA_Z_MSB (0x2D)
        // Layout of the 26 bytes read:
        //   [0– 5]  GYR_DATA  X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB  (0x14–0x19)
        //   [6–11]  MAG_DATA  (skipped)                                    (0x1A–0x1F)  ← Euler angles
        //   [12–19] QUA_DATA  (skipped)                                    (0x20–0x27)  ← Quaternion (8 bytes)
        //   [20–25] LIA_DATA  X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB  (0x28–0x2D)
        static constexpr uint8_t BURST_START_REG = 0x14;  // GYR_DATA_X_LSB
        static constexpr uint8_t BURST_READ_LEN  = 26;    // through LIA_DATA_Z_MSB (0x2D)

        static constexpr uint8_t GYRO_OFFSET_IN_BURST = 0;   // bytes 0–5
        static constexpr uint8_t LIA_OFFSET_IN_BURST  = 20;  // bytes 20–25

        static constexpr uint8_t CALIB_STAT_REG  = 0x35;

        // Axis remap registers
        static constexpr uint8_t BNO055_OPR_MODE_ADDR   = 0x3D;
        static constexpr uint8_t BNO055_AXIS_MAP_CONFIG  = 0x41;
        static constexpr uint8_t BNO055_AXIS_MAP_SIGN    = 0x42;
        static constexpr uint8_t MODE_CONFIG             = 0x00;
        static constexpr uint8_t MODE_NDOF               = 0x0C;

        void writeRegister(uint8_t reg, uint8_t val);
        void parseBuffer(uint8_t* buf);
};

#endif
