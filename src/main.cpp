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
bool gripperFinished1 = true;
bool gripperFinished2 = true;
int gripperStatus = 0;
int previousGripperState1;
int previousGripperState2;
int gripperStatusSerial1;
int gripperStatusSerial2;

// FUNCTION DEFINITIONS
// to controls grippers with buttons. Remember to set grippers current state.
void gripperButtonTest(gripperState currentState, Gripper grip, Button buttonGripper);
void updateSpeeds();

Gripper gripper[2];

void setup() {
    Serial.begin(9600); //Debug Serial
    Wire.begin(); //begin I2C

    Serial.println("Robot intializing....");
    jointMotor[0] = JointMotor(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 145, 0.1, 72.5, 10, 0.1, 5);
    jointMotor[1] = JointMotor(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 135, 0.1, 65.5);
    jointMotor[2] = JointMotor(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 10, 0.1, 5, 145, 0.1, 72.5);

    /* DEBUG */
    jointMotor[0].setAngle(0);
    jointMotor[1].setAngle(0);
    jointMotor[2].setAngle(0);

    gripper[0] = Gripper(GRIPPER_MOTOR_1, true);
    gripper[1] = Gripper(GRIPPER_MOTOR_2, false);
    Serial.println("Done");

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

        for(int i = 0; i < len; i++){
          temp[tempIndex] = serialBuffer[i];

        	if(tempIndex < 2){
              	tempIndex++;
          }
          else{
              if (jointIndex > 2) { //Gripper
                if((temp[1] - '0') != previousGripperState1){
                  gripperStatusSerial1 = temp[1] - '0';
                  previousGripperState1 = gripperStatusSerial1;
                  gripperFinished1 = false;
                }

                if((temp[2] - '0') != previousGripperState2){
                  gripperStatusSerial2 = temp[2] - '0';
                  previousGripperState2 = gripperStatusSerial2;
                  gripperFinished2 = false;
                }

                                  // Serial.print("Gripper 1 (Red): ");
                                  // Serial.println(gripperStatusSerial1);
                                  // Serial.print("Gripper 2 (Yellow): ");
                                  // Serial.println(gripperStatusSerial2);
               }
               else { //Joint angles
                  int sign = 1;
                  if (temp[0] == '-') {
                    sign = -1;
                  }
                  temp[0] = '0';
                	jointMotor[jointIndex].setAngle(sign*atoi(temp));
              		jointIndex++;
              		tempIndex = 0;
            }
        	}
        }
   }

  if(!gripperFinished1){
     gripperFinished1 = gripper[0].setGripper(gripperStatusSerial1, 21000);
   }

   if(!gripperFinished2){
     gripperFinished2 = gripper[1].setGripper(gripperStatusSerial2, 23000);
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
* Buttons should be plugged into the Analog pins.
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
