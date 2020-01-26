#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
//#include "JointMotor.h"
#include "JointMotor2.h"
#include "pins.h"
#include "Gripper.h"
#include "Button.h"
#include "scLookUp.h"
//#include "TimerOne.h"

//Variables
// JointMotor jointMotor[3];
JointMotor2 jointMotor[3];
int theta[3];

float m1 = 0.09;
float m2 = 0.09;
float m3 = 0.15;

float L1 = 0.1633;
float L2 = 0.1633;
float L3 = 0.1048;

float g = 9.81;

float k1 = -150;
float k2 = -300;
float k3 = -125;

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

int gripperSelect = 0; //idle (No gripper selected)
int gripperState = 0; //idle (No gripper action)
int gripperEngagedSelect = 0;

// FUNCTION DEFINITIONS
// to controls grippers with buttons. Remember to set grippers current state.
void gripperButtonTest(int currentState, Gripper grip, Button buttonGripper);
void updateSpeeds();

Gripper gripper[2];

void setup() {
		Serial.begin(9600); //Debug Serial
		Wire.begin(); //begin I2C

		Serial.println("Robot intializing....");
		temp[int(len/4)] = '\n'; //you need this
		// jointMotor[0] = JointMotor(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 50, .12, 10, 10, 0.1, 20, 10, 1, 0.8);
		// jointMotor[1] = JointMotor(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 50, .12, 70, 50, .1, 50, 10, 2, .8);
		// jointMotor[2] = JointMotor(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 10, .12, 50, 60, 0.12, 60, 10, 3, 0.8);

		jointMotor[0] = JointMotor2(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 12, 0.1, 40, 10, 0.1, 5, 27.81, true,0);
		jointMotor[1] = JointMotor2(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 35, 0.2, 25, 124.38, true,1);
		jointMotor[2] = JointMotor2(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 15, 0.2, 5, 100, 0.1, 60, 27.81, false,2);
		
		/* DEBUG */
		jointMotor[0].setAngle(27.81);
		jointMotor[1].setAngle(124.38);
		jointMotor[2].setAngle(27.81);

		jointMotor[0].debug = true;
		jointMotor[1].debug = true;
		jointMotor[2].debug = true;

		gripper[0] = Gripper(GRIPPER_MOTOR_1, false, false); //yellow gripper
		gripper[1] = Gripper(GRIPPER_MOTOR_2, true, false);  //red gripper
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
						if((temp[2]-'0' == 1) || (temp[2]-'0' == 2)){
							gripperSelect = (temp[2] - '0');
							gripperState = (temp[3] - '0');

							if(gripperSelect == 1){
								gripperFinished1 = false;	
							}else if(gripperSelect == 2){
								gripperFinished2 = false;
							}
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

	//Note: last byte red gripper
	if(!gripperFinished1 && gripperSelect == 1){ //yellow gripper
		 gripperFinished1 = gripper[gripperSelect-1].setGripper(gripperState);
		 //Serial.println("Gripper red moving");
	 }

	if(!gripperFinished2 && gripperSelect == 2){
		gripperFinished2 = gripper[gripperSelect-1].setGripper(gripperState);
		//Serial.println("Gripper yellow moving");
	 }


	updateSpeeds();

		// jointMotor[0].updateSpeed();
		// jointMotor[1].updateSpeed();
		// jointMotor[2].updateSpeed();

}

/*
*	Calculate Gravity Compensation
*/
int gravityCompensation(JointMotor2 i, int th[]){

	
	if(i.id == 0){
		// Serial.print("Theta values ");
		// Serial.print(i.id);
		// Serial.print(": ");
		// Serial.println(th[0]);
		// Serial.print(", sin value: ");
		// Serial.print(sinLut[th[0]]);
		// Serial.print(", decimal: ");
		// Serial.println(sinLut[th[0]]*0.001);

		return k1*(g*m3*(L1*sinLut[th[0]]*0.001+L2*sinLut[th[1]]*0.001+L3*sinLut[th[2]]*0.001)+g*m2*(L1*sinLut[th[0]]*0.001+L2*sinLut[th[1]]*0.001)+g*L1*m1*sinLut[th[0]]*0.001);
	}else if(i.id == 1){
		return k2*(g*m3*(L2*sinLut[th[1]]*0.001+L3*sinLut[th[2]]*0.001)+g*L2*m2*sinLut[th[1]]*0.001);
	}else if(i.id == 2){
		Serial.print("Theta values ");
		Serial.print(i.id);
		Serial.print(": ");
		Serial.println(th[2]);
		return k3*(g*L3*m3*sinLut[th[2]]*0.001);
	}else{
		Serial.print("NO JOINT ID AVAILABLE FOR GRAVITY COMPENSATION");
		return 0;
	}
}

/*
* Update Speed of all joint motor for PWM
*/
void updateSpeeds() {
	if(gripper[0].isE){
		gripperEngagedSelect = 1;
	}else if(gripper[1].isE){
		gripperEngagedSelect = 2;
	}else{
		gripperEngagedSelect = 0;
	}

	int numMotors = 3;
	int gc = 0;
	for (int i = 0; i < numMotors; i++) {
		theta[i] = jointMotor[i].getAngleDegrees();
		gc = gravityCompensation(jointMotor[i], theta);
		jointMotor[i].updateSpeed(gc);
		//jointMotor[i].switchPID(gripperEngagedSelect);
	}
}

/*
* Enables to interface (engage and disengage) the grippers using buttons.
* Buttons should be plugged into the Analog pins.
*/
void gripperButtonTest(int currentState, Gripper grip, Button buttonGripper){

		if(buttonGripper.isPressed() && buttonGripper.stateChanged() && buttonState){
			buttonState = false;
			if(currentState == 1){
				Serial.println("disengage");
				grip.setGripper(2);
			}else{
				Serial.println("engage");
				grip.setGripper(1);
			}
		}
		if(buttonGripper.isPressed() && buttonGripper.stateChanged() && !buttonState){
			buttonState = true;
			if(currentState == 1){
				Serial.println("engage");
				grip.setGripper(1);
			}else{
				Serial.println("disengage");
				grip.setGripper(2);
			}
		}
}
