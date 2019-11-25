#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include "JointMotor.h"
#include "pins.h"
#include "Gripper.h"
#include "Button.h"
#include "TimerOne.h"

//Variables
JointMotor jointMotor[3];

//Serial Buffer
const int len = 12;
char serialBuffer[len];
char temp[int(len/3)];

/*DEBUG*/
// Buttons have to be pull up
// Pull up: one terminal on GND and the other
//          attached to the analog pin.
Button buttonGrip_1 = Button(A0, PULLUP);
bool buttonState = true;
bool triggerGrip = true;
bool gripperFinished = false;
int gripperStatus = 0;
int previousGripperState;

// FUNCTION DEFINITIONS
// to controls grippers with buttons. Remember to set grippers current state.
void gripperButtonTest(gripperState currentState, Gripper grip, Button buttonGripper);
void updateSpeeds();

Gripper gripper[2];

void setup() {
    Serial.begin(9600); //Debug Serial
    Wire.begin(); //begin I2C

    jointMotor[0] = JointMotor(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 170, 0.1, 85, 12, 0.1, 6);
    jointMotor[1] = JointMotor(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 150, 0.1, 70);
    jointMotor[2] = JointMotor(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 12, 0.1, 6, 170, 0.1, 85);

    /* DEBUG */
    jointMotor[0].setAngle(20);
    jointMotor[1].setAngle(20);
    jointMotor[2].setAngle(-55);

    gripper[0] = Gripper(GRIPPER_MOTOR_1, true);
    gripper[1] = Gripper(GRIPPER_MOTOR_2, false);

	//Timer1 Interupt
	// Timer1.initialize(500000);
	// Timer1.attachInterrupt(updateSpeeds);
	// interrupts();
}

void loop() {

    // if(triggerGrip){
    //     triggerGrip = !gripper[0].setGripper(1, 21000);
    // }
    //USED: when want ot enagage and disengage with button
    //gripperButtonTest(engage, gripper[0], buttonGrip_1);


      if (Serial.available() > 0){
        Serial.println("Message received");
        Serial.readBytesUntil('\n', serialBuffer, len);
        int tempIndex = 0;
        int jointIndex = 0;
        int gripperIndex = 0;


        for(int i = 0; i < len; i++){
          temp[tempIndex] = serialBuffer[i];

        	if(tempIndex < 2){
              	tempIndex++;
          }else{
              if (jointIndex > 2) { //Gripper

                //TODO: code that checks for previous state. have to do it for both
                // gripperStatus = atoi(temp[gripperIndex]);
                // if(gripperStatus != previousGripperState){
                //   gripper[gripperIndex].setGripper(gripperStatus,2100);
                //   previousGripperState = gripperStatus;
                // }

                if(gripperIndex != 0){ //Engage or disengage gripper
                  gripper[gripperIndex].setGripper(atoi(temp[gripperIndex]),2100);
                }else{ //the first char of the gripper message can be used here
                }
                gripperIndex++;
            } else { //Joint angles
                	jointMotor[jointIndex].setAngle(atoi(temp));
              		jointIndex++;
              		tempIndex = 0;
            }
        	}
        }
   }

	updateSpeeds();

    // jointMotor[0].updateSpeed();
    // jointMotor[1].updateSpeed();
    // jointMotor[2].updateSpeed();

}

/*
* Update Speed of all joint motor for PWM
*/
void updateSpeeds() {
	int numMotors = 3;
	for (int i = 0; i < numMotors; i++) {
		jointMotor[i].updateSpeed();
	}
}

/*
* Enables to interface (engage and disengage) the grippers using buttons.
* Buttons should be pulgged into the Analog pins.
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
