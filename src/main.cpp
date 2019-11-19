#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "Button.h"
#include "JointMotor.h"
#include "pins.h"
#include "Gripper.h"

//Variables
JointMotor motor1;
JointMotor motor2;
JointMotor motor3;
Servo servo;

Gripper gripper[2];
gripperState gState = idle;

Button buttonUp = Button(A0,PULLUP);
Button buttonDown = Button(A1,PULLUP);
int count = 0;


//Function definitions
void setDirection(int direction);

void setup() {
    Serial.begin(9600); //Debug Serial
    Wire.begin(); //begin I2C
    pinMode(13,OUTPUT);

    gripper[0] = Gripper(GRIPPER_MOTOR_1);
    gripper[1] = Gripper(GRIPPER_MOTOR_2);

    //gripper[0].setGripper(idle, 0);

    // motor1 = JointMotor(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 150, 0.1, 125);
    // motor2 = JointMotor(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 150, 0.1, 125);
    // motor3 = JointMotor(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 150, 0.1, 125);
    //
    // motor1.setAngle(45);
    // motor2.setAngle(-90);
    // motor3.setAngle(5);
}

void loop() {
    // motor1.updateSpeed();
    // motor2.updateSpeed();
    // motor3.updateSpeed();
    // if(buttonUp.isPressed() && buttonUp.stateChanged()){
    //   count++;
    //   digitalWrite(13,HIGH);
    //
    // }
    //
    // if(buttonDown.isPressed() && buttonDown.stateChanged()){
    //   digitalWrite(13,LOW);
    //   gripper[0].setGripper(idle, 1000);
    //   count--;
    // }
    //
    // if(count < 0){
    //   count = 0;
    // }else if (count > 2){
    //   count = 2;
    // }
    //
    // Serial.println(count);
    // if(count == 0){
    //   gripper[0].setGripper(idle,10000)
    // }else if(count == 1){
    //   gripper[0].setGripper(engage, 10000);
    // }else if(count == 2){
    //   gripper[0].setGripper(disengage, 10000);
    // }


    if(buttonUp.isPressed() && buttonUp.stateChanged()){
      gripper[0].setGripper(engage, 10000);
    }

}
