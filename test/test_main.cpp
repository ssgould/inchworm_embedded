#include <Arduino.h>
#include <unity.h>

#include "JointMotor2.h"
#include "pins.h"

void test_motor_init() {
    JointMotor2 JointMotor = JointMotor2(0, 1, 2, 0xee, 20, 0.3, 20, 30, 0.35, 20, 27.81, true, 0);

    TEST_ASSERT_EQUAL(JointMotor.pwmForward, 0);
}