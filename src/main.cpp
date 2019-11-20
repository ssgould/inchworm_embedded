#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "JointMotor.h"
#include "pins.h"
#include "gripper.h"

//Variables
JointMotor jointMotor[3];
// JointMotor motor0;
// JointMotor motor1;
// JointMotor motor2;

//Serial Buffer
const int len = 9;
char serialBuffer[len];
char temp[int(len/3)];

//Function definitions
void setDirection(int direction);

void setup() {
    Serial.begin(9600); //Debug Serial
    Wire.begin(); //begin I2C

    

    jointMotor[0] = JointMotor(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 180, 0.1, 90 );
    jointMotor[1] = JointMotor(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 150, 0.1, 70);
    jointMotor[2] = JointMotor(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 12, 0.1, 6);
    // motor0 = JointMotor(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 150, 0.1, 125);
    // motor1 = JointMotor(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 150, 0.1, 125);
    // motor2 = JointMotor(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 150, 0.1, 125);

    /* DEBUG */
    jointMotor[2].debug = true;
    jointMotor[0].setAngle(0);
    jointMotor[1].setAngle(5);
    jointMotor[2].setAngle(-55);
}

void loop() {
    // while (Serial.available() > 0) {
    //     //PacketFormat: "joint (1 byte) angle (2 bytes)," or "joint (1 byte) angle (2 bytes)\n"
    //     int joint = -1;
    //     double angle;
    //     const byte recievedByte = Serial.read();

    //     if ()

    // }

    if (Serial.available() > 0){
        Serial.println("Message received");
        Serial.readBytesUntil('\n', serialBuffer, len);
        int tempIndex = 0;
        int jointIndex = 0;

        for(int i = 0; i < len; i++){
        temp[tempIndex] = serialBuffer[i];

        if(tempIndex < 2){
            tempIndex++;
        }
        else{
            jointMotor[jointIndex].setAngle(atoi(temp));
            jointIndex++;
            tempIndex = 0;
            }
        }
    }

    jointMotor[0].updateSpeed();
    jointMotor[1].updateSpeed();
    jointMotor[2].updateSpeed();
}
