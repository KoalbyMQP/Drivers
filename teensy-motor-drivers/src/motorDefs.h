#ifndef motorDefs_h
#define motorDefs_h

#include <HerkulexMotor.h>

enum SERIAL_BUS_COUNTS {
  BUS_L_LEG_COUNT = 5,
  BUS_R_LEG_COUNT = 5,
  BUS_CHEST_COUNT = 5,
  BUS_L_ARM_COUNT = 5,
  BUS_R_ARM_COUNT = 5,
  TOTAL_COUNT = BUS_L_LEG_COUNT + BUS_R_LEG_COUNT + BUS_CHEST_COUNT + BUS_L_ARM_COUNT + BUS_R_ARM_COUNT,
};

enum SERIAL_BUS {
    BUS_L_LEG = 1,
    BUS_R_LEG = 2,
    BUS_CHEST = 3,
    BUS_L_ARM = 4,
    BUS_R_ARM = 5,
    EXTRA_1 = 6,
    EXTRA_2 = 7,
};

inline MotorRef rightLegMotors[BUS_R_LEG_COUNT] = {
  (HerkulexMotor(1, MotorModel::DRS_0201, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(2, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(3, MotorModel::DRS_0602_GEARBOX, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(4, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(5, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
};

inline MotorRef leftLegMotors[BUS_L_LEG_COUNT] = {
  (HerkulexMotor(1, MotorModel::DRS_0201, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(2, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(3, MotorModel::DRS_0602_GEARBOX, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(4, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(5, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
};

inline MotorRef chestMotors[BUS_CHEST_COUNT] = {
  (HerkulexMotor(1, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(2, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(3, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(4, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(5, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
};

inline MotorRef leftArmMotors[BUS_L_ARM_COUNT] = {
  (HerkulexMotor(1, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(2, MotorModel::DRS_0602_GEARBOX, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(3, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(4, MotorModel::DRS_0201, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(5, MotorModel::DRS_0201, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
};

inline MotorRef rightArmMotors[BUS_R_ARM_COUNT] = {
  (HerkulexMotor(1, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(2, MotorModel::DRS_0602_GEARBOX, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(3, MotorModel::DRS_0601, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(4, MotorModel::DRS_0201, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
  (HerkulexMotor(5, MotorModel::DRS_0201, SERIAL_BUS::BUS_R_LEG)).getMotorRef(),
};

inline MotorRef allMotors[TOTAL_COUNT];

inline void createAvaMotorDef() {
    size_t offset = 0;
    memcpy(allMotors + offset, rightLegMotors, sizeof(rightLegMotors)); offset += BUS_R_LEG_COUNT;
    memcpy(allMotors + offset, leftLegMotors,  sizeof(leftLegMotors));  offset += BUS_L_LEG_COUNT;
    memcpy(allMotors + offset, chestMotors,    sizeof(chestMotors));    offset += BUS_CHEST_COUNT;
    memcpy(allMotors + offset, leftArmMotors,  sizeof(leftArmMotors));  offset += BUS_L_ARM_COUNT;
    memcpy(allMotors + offset, rightArmMotors, sizeof(rightArmMotors));
}

#endif