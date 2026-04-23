#ifndef motorDefs_h
#define motorDefs_h

// FUNCTION MOVE TO ZERO IN MAIN IS RELIANT ON THIS. IF YOU UPDATE MOTORDEFS ORDER, UPDATE MOVETOZERO!

#include <HerkulexMotor.h>

enum SERIAL_BUS_COUNTS {
  BUS_L_LEG_COUNT = 5,
  BUS_R_LEG_COUNT = 5,
  BUS_CHEST_COUNT = 5,
  BUS_L_ARM_COUNT = 3,
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

inline MotorRef leftLegMotors[BUS_L_LEG_COUNT] = {
  (HerkulexMotor(1, MotorModel::DRS_0601,         SERIAL_BUS::BUS_L_LEG, -3.0, 19.0)).getMotorRef(),
  (HerkulexMotor(2, MotorModel::DRS_0601,         SERIAL_BUS::BUS_L_LEG, -16.0, 215.0)).getMotorRef(), // but this motor technically, if no other motor moves, could be moved indefinitely
  (HerkulexMotor(3, MotorModel::DRS_0601,         SERIAL_BUS::BUS_L_LEG, -120.0, 33.0)).getMotorRef(),
  (HerkulexMotor(4, MotorModel::DRS_0602_GEARBOX, SERIAL_BUS::BUS_L_LEG, -74.0, 69.0)).getMotorRef(),
  (HerkulexMotor(5, MotorModel::DRS_0201,         SERIAL_BUS::BUS_L_LEG, -45.0, 42.0)).getMotorRef(),
};

inline MotorRef rightLegMotors[BUS_R_LEG_COUNT] = {
  (HerkulexMotor(1, MotorModel::DRS_0601,         SERIAL_BUS::BUS_R_LEG, -19.0, 3.0)).getMotorRef(), // to show how we would to by adding the bounds for all of the motors
  (HerkulexMotor(2, MotorModel::DRS_0601,         SERIAL_BUS::BUS_R_LEG, -215.0, 16.0)).getMotorRef(),
  (HerkulexMotor(3, MotorModel::DRS_0601,         SERIAL_BUS::BUS_R_LEG, -33.0, 120.0)).getMotorRef(),
  (HerkulexMotor(4, MotorModel::DRS_0602_GEARBOX, SERIAL_BUS::BUS_R_LEG, -69.0, 74.0)).getMotorRef(),
  (HerkulexMotor(5, MotorModel::DRS_0201,         SERIAL_BUS::BUS_R_LEG, -42.0, 45.0)).getMotorRef(),
};

inline MotorRef chestMotors[BUS_CHEST_COUNT] = {
  (HerkulexMotor(1, MotorModel::DRS_0602, SERIAL_BUS::BUS_CHEST, -180.0, 180.0)).getMotorRef(),
  (HerkulexMotor(2, MotorModel::DRS_0602, SERIAL_BUS::BUS_CHEST, -180.0, 180.0)).getMotorRef(),
  (HerkulexMotor(3, MotorModel::DRS_0601, SERIAL_BUS::BUS_CHEST, -45.0, 45.0)).getMotorRef(),  // this motor can also go 360 degrees but no way we need that
  (HerkulexMotor(4, MotorModel::DRS_0601, SERIAL_BUS::BUS_CHEST, -45.0, 45.0)).getMotorRef(),  //this one could do much more, but but this is a safe bet, there's no world in which we want more range
  (HerkulexMotor(5, MotorModel::DRS_0601, SERIAL_BUS::BUS_CHEST, -40.0, 25.0)).getMotorRef(),
};

inline MotorRef leftArmMotors[BUS_L_ARM_COUNT] = {
  (HerkulexMotor(1, MotorModel::DRS_0201,         SERIAL_BUS::BUS_L_ARM, -10.0, 10.0)).getMotorRef(),
  (HerkulexMotor(2, MotorModel::DRS_0602_GEARBOX, SERIAL_BUS::BUS_L_ARM, -180.0, 40.0)).getMotorRef(),
  (HerkulexMotor(3, MotorModel::DRS_0601,         SERIAL_BUS::BUS_L_ARM, -99.0, 80.0)).getMotorRef(),
  // (HerkulexMotor(4, MotorModel::DRS_0201,         SERIAL_BUS::BUS_L_ARM, -180.0, 180.0)).getMotorRef(),
  // (HerkulexMotor(5, MotorModel::DRS_0201,         SERIAL_BUS::BUS_L_ARM, -110.0, 110.0)).getMotorRef(),
};

inline MotorRef rightArmMotors[BUS_R_ARM_COUNT] = {
  (HerkulexMotor(1, MotorModel::DRS_0201,         SERIAL_BUS::BUS_R_ARM, -45.0, 45.0)).getMotorRef(), //technically this motor (bottom neck) should be able to do 360, but no need
  (HerkulexMotor(2, MotorModel::DRS_0602_GEARBOX, SERIAL_BUS::BUS_R_ARM, -40.0, 180.0)).getMotorRef(),
  (HerkulexMotor(3, MotorModel::DRS_0601,         SERIAL_BUS::BUS_R_ARM, -80.0 ,99.0)).getMotorRef(),
  (HerkulexMotor(4, MotorModel::DRS_0201,         SERIAL_BUS::BUS_R_ARM, -180.0, 180.0)).getMotorRef(),
  (HerkulexMotor(5, MotorModel::DRS_0201,         SERIAL_BUS::BUS_R_ARM, -110.0, 110.0)).getMotorRef(),
};

inline MotorRef allMotors[TOTAL_COUNT];

inline void createAvaMotorDef() {
    size_t offset = 0;
    memcpy(allMotors + offset, leftLegMotors,   sizeof(leftLegMotors));  offset += BUS_L_LEG_COUNT;
    memcpy(allMotors + offset, rightLegMotors,  sizeof(rightLegMotors)); offset += BUS_R_LEG_COUNT;
    memcpy(allMotors + offset, chestMotors,     sizeof(chestMotors));    offset += BUS_CHEST_COUNT;
    memcpy(allMotors + offset, leftArmMotors,   sizeof(leftArmMotors));  offset += BUS_L_ARM_COUNT;
    memcpy(allMotors + offset, rightArmMotors,  sizeof(rightArmMotors));
}

#endif