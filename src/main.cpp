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
const int len = 16;
char serialBuffer[len];
char temp[int(len/4)+1];

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
int previousGripperState1 = -1;
int previousGripperState2 = -1;
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
		temp[int(len/4)] = '\n'; //you need this
		jointMotor[0] = JointMotor(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 100, 0.1, 50, 10, 0.1, 5);
		jointMotor[1] = JointMotor(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 120, 0.1, 60);
		jointMotor[2] = JointMotor(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 10, 0.1, 5, 100, 0.1, 60);

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

		if (serialBuffer[0] == '-' || serialBuffer[0] == '0') {
			// Serial.println("____________Robot GO____________");

			for(int i = 0; i < len; i++){
				// Serial.print("Iteration: ");
				// Serial.println(i);

				temp[tempIndex] = serialBuffer[i];

				// Serial.print("temp properties: ");
				// Serial.print(tempIndex);
				// Serial.print(" ");
				// Serial.print(temp[tempIndex]);
				// Serial.print(" ");
				// Serial.println(atoi(temp));

				if(tempIndex < 3){
					tempIndex++;
				}
				else {

					if (jointIndex > 2) { //Gripper
						// Serial.println("Moving Gripper");
				
						if((temp[2] - '0') != previousGripperState1){ //'0'm = engage
							gripperStatusSerial1 = temp[2] - '0';
							previousGripperState1 = gripperStatusSerial1;
							gripperFinished1 = false;
						}

						if((temp[3] - '0') != previousGripperState2){
							gripperStatusSerial2 = temp[3] - '0';
							previousGripperState2 = gripperStatusSerial2;
							gripperFinished2 = false;
						}
						if (temp[2] != temp[3]) {
							jointMotor[0].switchPID();
							jointMotor[2].switchPID();
						}
					}
					else { //Joint angles
						int sign = 1;
						if (temp[0] == '-') {
							sign = -1;
						}
						
						temp[0] = '0';

						// Serial.print("joint ");
						// Serial.print(jointIndex);

						signed int angle = sign*atoi(temp);
						jointMotor[jointIndex].setAngle(angle);
						Serial.print(" angle: ");
						Serial.println(angle);
						jointIndex++;
					}
					tempIndex = 0;
				}
			}
		}
		else {
			//Serial.println("____________Why I am here____________");
			for (int i = 0; i < len; i ++) {
				Serial.read();
			}
		}
	}

	if(!gripperFinished1){
		 gripperFinished1 = gripper[0].setGripper(gripperStatusSerial1, 21000);
		 //Serial.println("Gripper red moving");
	 }

	if(!gripperFinished2){
		 gripperFinished2 = gripper[1].setGripper(gripperStatusSerial2, 23000);
		//Serial.println("Gripper yellow moving");
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
