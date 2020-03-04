//*****************************************************************************
//
// pins.h - Centralizes where pins are set and added. Readability improved
//
//*****************************************************************************

#ifndef GLOBAL_PINS
#define GLOBAL_PINS

//END EFFECTOR GRIPPER MOTORS
#define GRIPPER_MOTOR_1 1 //red
#define GRIPPER_MOTOR_2 2 //yellow
#define GRIPPER_MOTOR_3 3 //a_link allen key
#define GRIPPER_MOTOR_4 4 //d_link allen key
//STORAGE MECHANISM
// #define STRAGE_MOTOR_LC 5

// //END EFFECTOR JOINT MOTORS
// #define END_EFFECTOR_JOINT_1 9
// #define END_EFFECTOR_JOINT_2 2

//3-DOF INCHWORM JOINT MOTORS
#define JOINT_MOTOR1_FWD 3
#define JOINT_MOTOR1_REV 5
#define JOINT_MOTOR1_EN 4
#define JOINT_MOTOR1_ADR 0x40

#define JOINT_MOTOR2_FWD 6
#define JOINT_MOTOR2_REV 9
#define JOINT_MOTOR2_EN 7
#define JOINT_MOTOR2_ADR 0x41

#define JOINT_MOTOR3_FWD 11
#define JOINT_MOTOR3_REV 10
#define JOINT_MOTOR3_EN 12
#define JOINT_MOTOR3_ADR 0x42

//CONTROLLER

//IMU

//LED pins
// #define LED_TRANSMITTING 7
// #define LED_RECIEVING 4

#endif
