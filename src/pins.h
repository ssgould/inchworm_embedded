//*****************************************************************************
//
// pins.h - Centralizes where pins are set and added. Readability improved
//
//*****************************************************************************

#ifndef GLOBAL_PINS
#define GLOBAL_PINS

//END EFFECTOR GRIPPER MOTORS
#define GRIPPER_MOTOR_1 5  //red
#define GRIPPER_MOTOR_2 10 //yellow

//STORAGE MECHANISM
#define STORAGE_MOTOR_LC 6

// //END EFFECTOR JOINT MOTORS
// #define END_EFFECTOR_JOINT_1 9
// #define END_EFFECTOR_JOINT_2 2

//3-DOF INCHWORM JOINT MOTORS
#define JOINT_MOTOR1_1 2
#define JOINT_MOTOR1_2 3
#define JOINT_MOTOR1_PWM 4
#define JOINT_MOTOR1_ADR 0x40

#define JOINT_MOTOR2_1 7
#define JOINT_MOTOR2_2 8
#define JOINT_MOTOR2_PWM 9
#define JOINT_MOTOR2_ADR 0x41

#define JOINT_MOTOR3_1 12
#define JOINT_MOTOR3_2 11
#define JOINT_MOTOR3_PWM 13
#define JOINT_MOTOR3_ADR 0x42

//CONTROLLER

//IMU

//LED pins
// #define LED_TRANSMITTING 7
// #define LED_RECIEVING 4

#endif
