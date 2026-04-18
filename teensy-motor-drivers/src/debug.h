#ifndef debug_h
#define debug_h
#include <HerkulexMotor.h>

void debug_motors(const MotorRef* motors, int motors_size);
int find_all_motors_on_bus(HerkulexClass& SerialBus);
void test_motor_latency(HerkulexMotor* motors, int motors_size);

#endif