#ifndef SINGLE_HERKULEX_BOARD_CONFIG_H
#define SINGLE_HERKULEX_BOARD_CONFIG_H

#include <Arduino.h>

#if !defined(ARDUINO_ARCH_ESP32)
#error "This project requires an ESP32 Arduino target."
#endif

namespace BoardConfig {

constexpr uint32_t DEBUG_BAUD = 115200;
constexpr uint32_t MOTOR_BAUD = 115200;

// UART2 pins on a common ESP32 Dev Module. RX/TX are from the ESP32's view.
constexpr int8_t MOTOR_RX_PIN = 16;
constexpr int8_t MOTOR_TX_PIN = 17;

constexpr uint8_t MOTOR_ID = 1;

enum class MotorVariant : uint8_t {
  DRS_0601,
  DRS_0602,
};

// Select the connected motor model here.
constexpr MotorVariant MOTOR_MODEL = MotorVariant::DRS_0602;

// Conservative first-test limits. Increase only after checking the mechanism.
constexpr float MIN_ANGLE_DEG = -30.0f;
constexpr float MAX_ANGLE_DEG = 30.0f;
constexpr uint16_t MOVE_TIME_MS = 1000;

}  // namespace BoardConfig

#endif
