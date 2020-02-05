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
float m2 = 0.350; // with storing mechanism (with block 0.297 kg)
float m3 = 0.201; // mass with new screwing mechanism

float L1 = 0.1633;
float L2 = 0.1633;
float L3 = 0.1048;

float g = 9.81;

float k1 = -130;
float k2 = -170;
float k3 = -200;

//Serial Buffer
const int len = 16;
char serialBuffer[len];
char temp[int(len/4)+1];

/*DEBUG*/
// Buttons have to be pull up
// Pull up: one terminal on GND and the other
//          attached to the analog pin.

//Setup buttons for PID tunning, button gripper is used sometimes for gripper debug
//          A0-A3 are avaibale analog pins
//Button buttonGrip_1 = Button(A0, PULLUP);
int jointSelectTune = A0;
int pValue = A1;
int iValue = A2;
int dValue = A3;

bool buttonState = true;
bool triggerGrip = true;
bool gripperFinished1 = true;
bool gripperFinished2 = true;
int gripperStatus = 0;
int previousGripperState1 = -1;
int previousGripperState2 = -1;
int gripperStatusSerial1;
int gripperStatusSerial2;

double lastPubAng = 0;

int gripperSelect = 0; //idle (No gripper selected)
int gripperState = 0; //idle (No gripper action)
int gripperEngagedSelect = 0;

// FUNCTION DEFINITIONS
// to controls grippers with buttons. Remember to set grippers current state.
void gripperButtonTest(int currentState, Gripper grip, Button buttonGripper);
void updateSpeeds();
void pidTunning(int jointSelect, int potP, int potI, int potD);

Gripper gripper[2];

void setup() {
		Serial.begin(9600); //Debug Serial
		Wire.begin(); //begin I2C

		Serial.println("Robot intializing....");
		temp[int(len/4)] = '\n'; //you need this
		// jointMotor[0] = JointMotor(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 50, .12, 10, 10, 0.1, 20, 10, 1, 0.8);
		// jointMotor[1] = JointMotor(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 50, .12, 70, 50, .1, 50, 10, 2, .8);
		// jointMotor[2] = JointMotor(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 10, .12, 50, 60, 0.12, 60, 10, 3, 0.8);

		jointMotor[0] = JointMotor2(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 8.42, 0, 0.5, 10, 0.1, 5, 27.81, true,0);
		jointMotor[1] = JointMotor2(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 20, 0, 1, 124.38, true,1);
		jointMotor[2] = JointMotor2(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 8.1, 0, 0.7, 0, 0, 0, 27.81, false,2);//works
		
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
						// jointMotor[jointIndex].sumError = 0.0;
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

	 	// 		if (millis()-lastPubAng>2000)
        //  {
        //    Serial.print("gripper 0  is Engaged: "); Serial.print(gripper[0].isE);
		// 	Serial.print("; Gripper 1: "); Serial.println(gripper[1].isE);
        //    lastPubAng=millis(); 
        //  }


	updateSpeeds();

		// jointMotor[0].updateSpeed();
		// jointMotor[1].updateSpeed();
		// jointMotor[2].updateSpeed();
	
	//pidTunning(jointSelectTune, pValue, iValue, dValue);

}

/*
* Print variable every 2000 millis
*/
void debugPrint(char jName[3], char pName[3],char iName[3],char dName[3], double pInput,double iInput,double dInput){
	if (millis()-lastPubAng>2000)
    {
		Serial.print("--------"); Serial.println(jName);
		Serial.print(pName); Serial.print(": "); Serial.println(pInput);
		Serial.print(iName); Serial.print(": "); Serial.println(iInput);
    	Serial.print(dName); Serial.print(": "); Serial.println(dInput);
        lastPubAng=millis(); 
    }
}

/*
*	Calculate Gravity Compensation
*/
int gravityCompensation(JointMotor2 i, int th[], bool select){
	int theta0 = th[0];
	int theta1 = th[0]+th[1];
	int theta2 = th[0]+th[1]+th[2];


	//Wrap around
	if(theta0>=360){
		theta0 = theta0-360;
	}
	if(theta1>=360){
		theta1 = theta1-360;
	}
	if(theta2>=360){
		theta2 = theta2-360;
	}

	if(theta0<=-360){
		theta0 = theta0+360;
	}
	if(theta1<=-360){
		theta1 = theta1+360;
	}
	if(theta2<=-360){
		theta2 = theta2+360;
	}



	if(i.id == 0){
		// Serial.print("Theta values ");
		// Serial.print(i.id);
		// Serial.print(": ");
		// Serial.println(th[0]);
		// Serial.print(", sin value: ");
		// Serial.print(sinLut[th[0]]);
		// Serial.print(", decimal: ");
		// Serial.println(sinLut[th[0]]*0.001);

		return k1*(g*m3*(L1*sinLut[theta0]*0.001+L2*sinLut[theta1]*0.001+L3*sinLut[theta2]*0.001)+g*m2*(L1*sinLut[theta0]*0.001+L2*sinLut[theta1]*0.001)+g*L1*m1*sinLut[theta0]*0.001);
	}else if(i.id == 1){
		
		return k2*(g*m3*(L2*sinLut[theta1]*0.001+L3*sinLut[theta2]*0.001)+g*L2*m2*sinLut[theta1]*0.001);
	}else if(i.id == 2){
		// Serial.print("Theta values ");
		// Serial.print(i.id);
		// Serial.print(": ");
		// Serial.print(th[0]);
		// Serial.print(',');
		// Serial.print(th[1]);
		// Serial.print(',');
		// Serial.println(th[2]);
		

		// Serial.print("wrap around (sum of all Angles): ");
		// Serial.println(theta1);
		return k3*(g*L3*m3*sinLut[theta2]*0.001);
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

		//TODO: Implement the switching of the PID
		// if(jointMotor[i].switchPID(gripperEngagedSelect)){

		// }else{

		// }
		theta[i] = jointMotor[i].getAngleDegrees();
		gc = gravityCompensation(jointMotor[i], theta,true);
		jointMotor[i].updateSpeed(gc);
	}
}

/*
* Enables to interface (engage and disengage) the grippers using buttons.
* Buttons should be plugged into the Analog pins.
*/
// void gripperButtonTest(int currentState, Gripper grip, Button buttonGripper){

// 		if(buttonGripper.isPressed() && buttonGripper.stateChanged() && buttonState){
// 			buttonState = false;
// 			if(currentState == 1){
// 				Serial.println("disengage");
// 				grip.setGripper(2);
// 			}else{
// 				Serial.println("engage");
// 				grip.setGripper(1);
// 			}
// 		}
// 		if(buttonGripper.isPressed() && buttonGripper.stateChanged() && !buttonState){
// 			buttonState = true;
// 			if(currentState == 1){
// 				Serial.println("engage");
// 				grip.setGripper(1);
// 			}else{
// 				Serial.println("disengage");
// 				grip.setGripper(2);
// 			}
// 		}
// }

/*
* For tunnning of PID values on the three diffrent joints using one button and 3 potentiometers
*/
void pidTunning(int jointSelect, int potP, int potI, int potD){
	int js = analogRead(jointSelect);
	int offset = 70;
	double ratio = 0.01056;

	if(js < 341){
		if(js >170){
			jointMotor[0].kP = (analogRead(potP)-offset)*ratio;
			jointMotor[0].kI = (analogRead(potI)-offset)*ratio;
			jointMotor[0].kD = (analogRead(potD)-offset)*ratio;
		}
		debugPrint("J0", "KP", "KI","KD", jointMotor[0].kP,jointMotor[0].kI,jointMotor[0].kD);
	}else if(js > 341 && js < 682){
		if(js>511){
			jointMotor[1].kP = (analogRead(potP)-offset)*ratio;
			jointMotor[1].kI = (analogRead(potI)-offset)*ratio;
			jointMotor[1].kD = (analogRead(potD)-offset)*ratio;
		}
		debugPrint("J1", "KP", "KI","KD", jointMotor[1].kP,jointMotor[1].kI,jointMotor[1].kD);
	}else if( js > 682){
		if(js > 852){
			jointMotor[2].kP = (analogRead(potP)-offset)*ratio;
			jointMotor[2].kI = (analogRead(potI)-offset)*ratio;
			jointMotor[2].kD = (analogRead(potD)-offset)*ratio;
		}
		debugPrint("J2", "KP", "KI","KD", jointMotor[2].kP,jointMotor[2].kI,jointMotor[2].kD);
	}
	// Serial.print("Kp: ");
	// Serial.println(jointMotor[0].kP);
}

