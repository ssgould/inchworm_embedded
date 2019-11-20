#include <Arduino.h>
#include "JointMotor.h"

JointMotor::JointMotor() {
}

JointMotor::JointMotor(int pinDirectionA1, int pinDirectionB1, int pinPWM1, int encoderAddress, double kp, double ki, double kd) {
    //Pin Configuration
    pinDirectionA = pinDirectionA1;
    pinDirectionB =  pinDirectionB1;
    pinPWM = pinPWM1;
    pinMode(pinDirectionA, OUTPUT);
    pinMode(pinDirectionB, OUTPUT);
    pinMode(pinPWM, OUTPUT);
    //Encoder Setup
    encoder = AMS_AS5048B(encoderAddress);
    encoder.begin(); //Encoder Constructor
    encoder.setZeroReg(); //Zero Encoders
    //PID
    kP = kp;
    kI = ki;
    kD = kd;

    debug = false;
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
        digitalWrite(pinDirectionB, LOW);
    }
    else {
        digitalWrite(pinDirectionA, LOW);
        digitalWrite(pinDirectionB, HIGH);
    }
    return;
}
/*
* Gets encoder angle in degrees
*/
double JointMotor::getAngleDegrees() {
    double angle = encoder.angleR(U_DEG, true);
    if (debug) { Serial.print("angle: "); Serial.println(angle); }

    if (angle >= 0 && angle <= 365) { //don't return "I2C Error" as angle
        lastAngle = angle; 
        return angle;
        }
    else {
        return lastAngle;
    }
}
/*
* Set desired joint angle
*/
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
    if (abs(int(error)) > 180) { error = error + 360;}

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