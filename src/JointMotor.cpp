#include <Arduino.h>
#include "JointMotor.h"

JointMotor::JointMotor() {
}

JointMotor::JointMotor(int pinDirectionA1, int pinDirectionB1, int pinPWM1, int encoderAddress, double kp, double ki, double kd, double ang_offset) {
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

    kP2 = kp;
    kI2 = ki;
    kD2 = kd;
    lastPubAng=0;

    angle_offset = ang_offset;

    debug = false;
}
JointMotor::JointMotor(int pinDirectionA1, int pinDirectionB1, int pinPWM1, int encoderAddress, double kp, double ki, double kd, double kp2, double ki2, double kd2, double ang_offset) {
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

    kP2 = kp2;
    kI2 = ki2;
    kD2 = kd2;
    angle_offset = ang_offset;

    debug = false;
}
/*
* Takes speed -255 - 255 and moves motor
*/
void JointMotor::setSpeed(int speed) {
    double maxPercent = 0.9;
    if (speed < -255 * maxPercent) { speed = -255 * maxPercent; }
    else if (speed > 255  * maxPercent) { speed = 255  * maxPercent; }
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
     if (debug) {
         if (millis()-lastPubAng>2000)
         {
           Serial.print("angle: "); Serial.println(angle);
           lastPubAng=millis(); 
         }
          
          
         }

     if (angle >= 0 && angle <= 360) { //don't return "I2C Error" as angle
         lastAngle = angle; 
         return angle;
         }
     else {
         return lastAngle;
     }
 }
// double JointMotor::getAngleDegrees() {
// 	double angle = encoder.angleR(U_DEG, true);
// 	double calibrated_angle = 0;

// 	if (debug) { Serial.print("angle: "); Serial.println(angle); }
    

// 	if (angle >= 0 && angle <= 360) { //don't return "I2C Error" as angle
// 		calibrated_angle = angle + angle_offset;
// 		if (calibrated_angle > 360) {
// 			calibrated_angle = calibrated_angle - 360;
// 		}
// 		last_calibrated_angle = calibrated_angle;
        
// 		return calibrated_angle;
// 	}
// 	else {
// 		return last_calibrated_angle;
// 	}
// }
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
* Switch PID values for which joint is fixed
*/
void JointMotor::switchPID(){
    double tempkP = kP;
    double tempkI = kI;
    double tempkD = kD;

    kP = kP2;
    kI = kI2;
    kD = kD2;

    kP2 = tempkP;
    kI2 = tempkI;
    kD2 = tempkD;
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

    lastError = error;
    return;
}