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
#define GRIPPER_MOTOR_1 6           //a_link gripper motor
#define GRIPPER_MOTOR_2 7           //d_link gripper motor
#define GRIPPER_MOTOR_3 0           //a_link allen key motor
#define GRIPPER_MOTOR_4 0           //d_link allen key motor
#define GRIPPER_ROTATION_BUTTON_A_LINK 0 //rotation button
#define GRIPPER_ROTATION_BUTTON_D_LINK 0 //rotation button
#define ALLEN_KEY_BUTTON_A_LINK 0 //limit switch for allen key
#define ALLEN_KEY_BUTTON_D_LINK 0 //limit switch for allen key

//TEENSY CLOCK PINS
// FTM0
// FTM1
// FTM3

//3-DOF INCHWORM JOINT MOTORS
#define JOINT_MOTOR1_FWD 36
#define JOINT_MOTOR1_REV 35
#define JOINT_MOTOR1_EN 0
#define JOINT_MOTOR1_ADR 0x40

#define JOINT_MOTOR2_FWD 2
#define JOINT_MOTOR2_REV 3
#define JOINT_MOTOR2_EN 0
#define JOINT_MOTOR2_ADR 0x41

#define JOINT_MOTOR3_FWD 5
#define JOINT_MOTOR3_REV 4
#define JOINT_MOTOR3_EN 0
#define JOINT_MOTOR3_ADR 0x42

// TODO: need to be changed
// #define JOINT_MOTOR4_FWD 8
// #define JOINT_MOTOR4_REV 9
// #define JOINT_MOTOR4_EN 10
// #define JOINT_MOTOR4_ADR 0x43

// #define JOINT_MOTOR5_FWD 8
// #define JOINT_MOTOR5_REV 9
// #define JOINT_MOTOR5_EN 10
// #define JOINT_MOTOR5_ADR 0x44

// DEBUG PIN TO BE USED FOR ANYTHING
#define DEBUG_PIN 23

#endif
