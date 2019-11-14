#include <Arduino.h>
#include <Wire.h>
#include "JointMotor.h"
#include <Servo.h>

#define MOTOR1_1  6
#define MOTOR1_2  4
#define MOTOR1_PWM  3

JointMotor motor1;
Servo servo;

void setDirection(int direction);

void setup() {
    Serial.begin(9600); //Debug Serial
    Wire.begin(); //begin i2c
    
    motor1 = JointMotor(MOTOR1_1, MOTOR1_2, MOTOR1_PWM);
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