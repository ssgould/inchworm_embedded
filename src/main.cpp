#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "JointMotor.h"
#include "global_pins.h"
#include "gripper.h"

//Variables
JointMotor motor1;
JointMotor motor2;
JointMotor motor3;
Servo servo;

//Function definitions
void setDirection(int direction);

void setup() {
    Serial.begin(9600); //Debug Serial
    Wire.begin(); //begin I2C

    motor1 = JointMotor(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 150, 0.1, 125);
    motor2 = JointMotor(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 150, 0.1, 125);
    motor3 = JointMotor(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 150, 0.1, 125);
    /*
    * TODO
    * add i2c error case to getAngle() => set to last measured angle
    */
    motor1.setAngle(45);
    motor2.setAngle(-90);
    motor3.setAngle(5);
}

void loop() {
    motor1.updateSpeed();
    motor2.updateSpeed();
    motor3.updateSpeed();
}
