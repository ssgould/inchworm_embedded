#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "JointMotor.h"
#include "pins.h"
#include "Gripper.h"
#include "Button.h"

//Variables
JointMotor jointMotor[3];
// JointMotor motor0;
// JointMotor motor1;
// JointMotor motor2;

//Serial Buffer
const int len = 9;
char serialBuffer[len];
char temp[int(len/3)];

// Buttons have to be pull up
// Pull up: one terminal on GND and the other
//          attached to the analog pin.
Button buttonGrip_1 = Button(A0, PULLUP);
bool buttonState = true;
bool triggerGrip = true;

// FUNCTION DEFINITIONS
// to controls grippers with buttons. Remember to set grippers current state.
void gripperButtonTest(gripperState currentState, Gripper grip, Button buttonGripper);


Gripper gripper[2];

void setup() {
    Serial.begin(9600); //Debug Serial
    Wire.begin(); //begin I2C
    //pinMode(13,OUTPUT);
    

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

    gripper[0] = Gripper(GRIPPER_MOTOR_1, true);
    gripper[1] = Gripper(GRIPPER_MOTOR_2, false);
}

void loop() {

    if(triggerGrip){
        triggerGrip = !gripper[0].setGripper(engage, 21000);
    }
    //USED: when want ot enagage and diangage with button
    //gripperButtonTest(engage, gripper[0], buttonGrip_1);


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


/*
* Enables to interface (engage and disengage) the grippers using buttons.
* Buttons should be pulgged into the Analog pins.
* IMPORTANT: The gripper set function uses a blocking delay.
*/
void gripperButtonTest(gripperState currentState, Gripper grip, Button buttonGripper){

    if(buttonGripper.isPressed() && buttonGripper.stateChanged() && buttonState){
      buttonState = false;
      if(currentState == engage){
        Serial.println("disengage");
        grip.setGripper(disengage, 21000);
      }else{
        Serial.println("engage");
        grip.setGripper(engage, 21000);
      }
    }
    if(buttonGripper.isPressed() && buttonGripper.stateChanged() && !buttonState){
      buttonState = true;
      if(currentState == engage){
        Serial.println("engage");
        grip.setGripper(engage, 21000);
      }else{
        Serial.println("disengage");
        grip.setGripper(disengage, 21000);
      }
    }
}
