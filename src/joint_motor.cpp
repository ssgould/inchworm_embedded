#include <Arduino.h>
#include "joint_motor.h"

JointMotor::JointMotor() {
}

JointMotor::JointMotor(int pinDirectionA1, int pinDirectionB1, int pinPWM1) {
    //Pin Configuration
    pinDirectionA = pinDirectionA1;
    pinDirectionB =  pinDirectionB1;
    pinPWM = pinPWM1;
    pinMode(pinDirectionA, OUTPUT);
    pinMode(pinDirectionB, OUTPUT);
    pinMode(pinPWM, OUTPUT);
    //Encoder Setup
    encoder     .begin(); //Encoder Constructor
    encoder.setZeroReg(); //Zero Encoders
    //PID
    kP = 125;
    kI = 0.0;
    kD = 0;
}
/*
* Takes speed -255 - 255 and moves motor
*/
void JointMotor::setSpeed(int speed) {
    if (speed < -255) { speed = -255; }
    else if (speed > 255) { speed = 255; }
    changeDirection(speed);
    analogWrite(pinPWM, abs(speed));
    return;
}
/*
* Changes motor direction pin states based on sign of speed
*/
void JointMotor::changeDirection(int speed) {
    if (speed < 0) {
        digitalWrite(pinDirectionA, HIGH);
        digitalWrite(pinDirectionA + 1, HIGH);
        digitalWrite(pinDirectionB, LOW);
        digitalWrite(pinDirectionB + 1, LOW);
    }
    else {
        digitalWrite(pinDirectionA, LOW);
        digitalWrite(pinDirectionA + 1, LOW);
        digitalWrite(pinDirectionB, HIGH);
        digitalWrite(pinDirectionB + 1, HIGH);
    }
    return;
}
/*
* Gets encoder angle in degrees
*/
double JointMotor::getAngleDegrees() {
    double angle = encoder.angleR(U_DEG, true);
    Serial.println(angle);
    return angle;
}

void JointMotor::setAngle(double angle) {
    desiredAngle = angle;
    lastError = 0;
    sumError = 0;
    return;
}
/*
* Update motor speed for PID
*/
void JointMotor::updateSpeed() {
    double currentAngle = getAngleDegrees();

    double error = desiredAngle - currentAngle;
    sumError = sumError + error;
    double changeError = error - lastError;

    int speed = (kP * error) + (kI * sumError) + (kD * changeError);
    setSpeed(speed);


    // Serial.println(error);
    // Serial.println(sumError);
    // Serial.println(changeError);
    // Serial.println(kP);
    // Serial.println(kI);
    // Serial.println(kD);
    // Serial.println(speed);

    lastError = error;
    return;
}
