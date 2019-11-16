#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "joint_motor.h"
#include "global_pins.h"

//Variables
JointMotor motor1;
Servo servo;

//Function definitions
void setDirection(int direction);

void setup() {
    Serial.begin(9600); //Debug Serial
    Wire.begin(); //begin I2C

    motor1 = JointMotor(INCHWORM_JOINT_MOTOR1_1, INCHWORM_JOINT_MOTOR1_2, INCHWORM_JOINT_MOTOR1_PWM);
    motor1.setAngle(45);

    /* Debug */
    // pinMode(6, OUTPUT);
    // pinMode(7, OUTPUT);
    // //pinMode(3, OUTPUT);
    // servo.attach(3);
}

void loop() {
    // motor1.setSpeed(127);
    // delay(500);
    // motor1.setSpeed(-127);
    // delay(500);
    // analogWrite(3, 200);
    // digitalWrite(6, HIGH);
    // digitalWrite(7, LOW);
    // delay(500);
    // digitalWrite(6, LOW);
    // digitalWrite(7, HIGH);
    // delay(500);

    motor1.updateSpeed();
    //delay(1000);

    // digitalWrite(6, HIGH);
    // digitalWrite(7, LOW);
    // servo.write(90);
    // delay(500);
    // digitalWrite(6, LOW);
    // digitalWrite(7, HIGH);
    // delay(500);
}
