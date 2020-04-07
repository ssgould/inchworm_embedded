//*****************************************************************************
//
// pins.h - Centralizes where pins are set and added. Readability improved
//
//*****************************************************************************

#ifndef GLOBAL_PINS
#define GLOBAL_PINS

//LED PINS
#define POWER_LED 13
// #define STATUS_LED 4

//HALL EFFECT SENSORS
#define HALL_EFFECT_A_LINK_1 0
#define HALL_EFFECT_D_LINK_1 0

//END EFFECTOR GRIPPER MOTORS
#define GRIPPER_MOTOR_1 0           //red
#define GRIPPER_MOTOR_2 0           //yellow
#define GRIPPER_MOTOR_3 0           //a_link allen key
#define GRIPPER_MOTOR_4 0           //d_link allen key
#define GRIPPER_ROTATION_BUTTON_A_LINK 0 //rotation button
#define GRIPPER_ROTATION_BUTTON_D_LINK 0 //rotation button

//3-DOF INCHWORM JOINT MOTORS
#define JOINT_MOTOR1_FWD 2
#define JOINT_MOTOR1_REV 3
#define JOINT_MOTOR1_EN 4
#define JOINT_MOTOR1_ADR 0x40

#define JOINT_MOTOR2_FWD 5
#define JOINT_MOTOR2_REV 6
#define JOINT_MOTOR2_EN 7
#define JOINT_MOTOR2_ADR 0x41

#define JOINT_MOTOR3_FWD 8
#define JOINT_MOTOR3_REV 9
#define JOINT_MOTOR3_EN 10
#define JOINT_MOTOR3_ADR 0x42

// #define JOINT_MOTOR4_FWD 8
// #define JOINT_MOTOR4_REV 9
// #define JOINT_MOTOR4_EN 10
// #define JOINT_MOTOR4_ADR 0x42

// #define JOINT_MOTOR5_FWD 8
// #define JOINT_MOTOR5_REV 9
// #define JOINT_MOTOR5_EN 10
// #define JOINT_MOTOR5_ADR 0x42

#endif
