#include <Arduino.h>
#include <Wire.h>
// #include <Servo.h>
//#include "JointMotor.h"
#include "JointMotor2.h"
#include "pins.h"
//#include "Gripper.h"
//#include "Button.h"
#include "scLookUp.h"
// #include "Storage.h"
//#include "TimerOne.h"

//Variables
// JointMotor jointMotor[3];
const int NUM_MOTORS = 3;
JointMotor2 jointMotor[NUM_MOTORS];
// Storage storage;
// int sMotor = 1;
int theta[3];

float m1 = 0.281; // 0.09  CAD value: 0.183
float m2 = 0.495; // with storing mechanism (with block 0.297 kg) Old: 0.357
float m3 = 0.207; // mass with new screwing mechanism // Old: 0.201

float L1 = 0.1633; // Old: 0.1633
float L2 = 0.1633; // Old: 0.1633
float L3 = 0.1048; // Old: 0.1048
float Lblock = 0.145;
float mblock = 0; //0.365

float LCoM1 = 0.055;
float LCoM2 = 0.082;
float LCoM3 = 0.064;

float g = 9.81;

float k1_a = -0.116; // -130 //-0.089 new
float k2_a = -0.129; //-200 //-0.13 new
float k3_a = -0.15;  //-200 //-0.1 new

float k1_d = -0.089; //a link
float k2_d = -0.1325;
float k3_d = -0.037;

// TODO: reenable gravity compensation
float gc_complimentary_filter = 1.0;
//Serial Buffer
const int MOTOR_PKT_LEN = 8;   // motor packet example: "-123.32_" (ending in space)
const int CONTROL_PKT_LEN = 4; // control packet example: "0131"
const int len = MOTOR_PKT_LEN * 3 + CONTROL_PKT_LEN;
char serialBuffer[len];
const int PARSE_PKT_LEN = 4;
char temp[PARSE_PKT_LEN];

unsigned long start_time;

// double test_angles[14][3] = {
// 	{5.21, 116.44, 58.3},
// 	{10.65444444, 109.16666667, 60.12888889},
// 	{16.09888889, 101.89333333, 61.95777778},
// 	{21.54333333, 94.62, 63.78666667},
// 	{26.98777778, 87.34666667, 65.61555556},
// 	{31.40444444, 84.41777778, 64.12777778},
// 	{34.79333333, 85.83333333, 59.32333333},
// 	{38.18222222, 87.24888889, 54.51888889},
// 	{41.57111111, 88.66444444, 49.71444444},
// 	{44.96, 90.08, 44.91},
// 	{36.12666667, 95.93777778, 47.88555556},
// 	{27.29333333, 101.79555556, 50.86111111},
// 	{18.46, 107.65333333, 53.83666667},
// 	{9.62666667, 113.51111111, 56.81222222}};

int test_angles_idx = 0;
unsigned long test_angle_time = 0;

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
int gripperState = 0;  //idle (No gripper action)
int gripperEngagedSelect = 0;

int useGravityComp = 1;
// FUNCTION DEFINITIONS
// to controls grippers with buttons. Remember to set grippers current state.
//void gripperButtonTest(int currentState, Gripper grip, Button buttonGripper);
void updateSpeeds();
// void pidTunning(int jointSelect, int potP, int potI, int potD);

//Gripper gripper[2];
double previous_time;

void setup()
{
	Serial.begin(9600); //Debug Serial
	Wire.begin();		//begin I2C

	Serial.println("Robot fintializing....");
	temp[int(len / 4)] = '\n'; //you need this
	// jointMotor[0] = JointMotor2(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 9.2, 0.034, 0.5, 8.5, 0.01, 0.5, 27.81, true, 0);
	// jointMotor[1] = JointMotor2(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 27, 0.05, 0, 27, 0.05, 0, 124.38, true, 1);
	// jointMotor[2] = JointMotor2(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 8.85, 0.07, 0.7, 8.85, 0.07, 0, 27.81, false, 2); //works

	// jointMotor[0] = JointMotor2(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 10, 0.15, 0, 8.4, 0.1, 2.4, 27.81, true, 0);
	// jointMotor[1] = JointMotor2(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 10, 0.14, 0, 8.4, 0.1, 3.2, 124.38, true, 1);
	// jointMotor[2] = JointMotor2(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 8, 0.02, 0, 8, 0.1, 2.6, 27.81, false, 2); //works original as of Feb 25

	jointMotor[0] = JointMotor2(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 17, 0.15, 5, 8.4, 0.1, 2.4, 27.81, true, 0);
	jointMotor[1] = JointMotor2(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 15, 0.15, 5, 8.4, 0.1, 3.2, 124.38, true, 1);
	jointMotor[2] = JointMotor2(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 6, 0.02, 1, 8, 0.1, 2.6, 27.81, false, 2);

	// jointMotor[0] = JointMotor2(JOINT_MOTOR1_1, JOINT_MOTOR1_2, JOINT_MOTOR1_PWM, JOINT_MOTOR1_ADR, 10, 0, 0, 0, 0, 0, 27.81, true, 0);
	// jointMotor[1] = JointMotor2(JOINT_MOTOR2_1, JOINT_MOTOR2_2, JOINT_MOTOR2_PWM, JOINT_MOTOR2_ADR, 6, 0, 0, 0, 0, 0, 124.38, true, 1);
	// jointMotor[2] = JointMotor2(JOINT_MOTOR3_1, JOINT_MOTOR3_2, JOINT_MOTOR3_PWM, JOINT_MOTOR3_ADR, 10, 0, 0, 0, 0, 2.6, 27.81, false, 2); //works

	// 0030.00 0084.00 0067.00 0100
	// 0045.00 0090.00 0045.00 0100

	// 0005.21 0116.44 0058.30 0100
	// 0029.71 0083.71 0066.53 0100
	// 0044.96 0090.08 0044.91 0100

	//D link fixed
	// 0067.00 0084.00 0030.00 0100
	// storage = Storage(STORAGE_MOTOR_LC);

	/* DEBUG */
	jointMotor[0].setAngle(27.81);
	jointMotor[1].setAngle(124.38);
	jointMotor[2].setAngle(27.81);

	jointMotor[0].debug = true;
	jointMotor[1].debug = true;
	jointMotor[2].debug = true;

	// gripper[0] = Gripper(GRIPPER_MOTOR_1, false, false); //yellow gripper
	// gripper[1] = Gripper(GRIPPER_MOTOR_2, true, false);  //red gripper
	Serial.println("Done");

	//Timer1 Interupt
	// Timer1.initialize(500000);
	// Timer1.attachInterrupt(updateSpeeds);
	// interrupts();

	previous_time = millis();
	// TCCR2B = TCCR2B & B11111000 | B00000100; // for PWM frequency of 490.20 Hz for pins 3, 11
	// TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency of 62500.00 Hz for Pin 6
}

void loop()
{

	// if(triggerGrip){
	//     triggerGrip = !gripper[0].setGripper(1, 21000);
	// }
	//USED: when want ot engage and disengage with button
	//gripperButtonTest(engage, gripper[0], buttonGrip_1);

	if (Serial.available() > 0)
	{
		// start_time = millis();
		double current_time = millis();
		Serial.println("Message received");
		Serial.println((current_time - previous_time) / 1000);
		previous_time = current_time;
		// digitalWrite(13, HIGH);
		Serial.readBytesUntil('\n', serialBuffer, len);
		int tempIndex = 0;
		int jointIndex = 0;
		float tempAngle = 0;
		boolean motorPktCompleted = true;

		if (serialBuffer[0] == '-' || serialBuffer[0] == '0')
		{
			// Serial.println("____________Robot GO____________");

			for (int i = 0; i < len; i++)
			{
				temp[tempIndex] = serialBuffer[i];
				if (tempIndex < 3)
				{
					tempIndex++;
				}
				else
				{
					tempIndex = 0;
					if (jointIndex > 2)
					{   //Gripper
						// if ((temp[2] - '0' == 1) || (temp[2] - '0' == 2) || (temp[2] - '0' == 3))
						// {
						// 	gripperSelect = (temp[2] - '0');
						// 	gripperState = (temp[3] - '0');

						// 	if (gripperSelect == 1)
						// 	{
						// 		gripperFinished1 = false;
						// 	}
						// 	else if (gripperSelect == 2)
						// 	{
						// 		gripperFinished2 = false;
						// 	}
						// 	else
						// 	{ // both gripper selected
						// 		gripperFinished1 = false;
						// 		gripperFinished2 = false;
						// 	}
						// }
						// sMotor = temp[1] - '0';
					}
					else
					{ //Joint angles
						if (motorPktCompleted)
						{							   // receiving first half of an angle value
							motorPktCompleted = false; // flip the flag to false
							if (temp[0] == '-')
							{
								temp[0] = '0';
								tempAngle = -1 * atoi(temp);
							}
							else
							{
								tempAngle = atoi(temp);
							}
						}
						else
						{							  // receiving second half of an angle value
							motorPktCompleted = true; // flip the flag to true
							temp[0] = '0';
							temp[PARSE_PKT_LEN - 1] = '0';
							tempAngle += (atof(temp) / 1000.0); // divide by 1000 to compensate for the extra 0
							useGravityComp = 1;
							jointMotor[jointIndex].setAngle(tempAngle);
							// jointMotor[jointIndex].sumError = 0.0;
							Serial.print("Setting angle[");
							Serial.print(jointIndex + 1);
							Serial.print("]: ");
							Serial.println(tempAngle);
							jointIndex++;
						}
					}
				}
			}
		}
		else
		{
			//Serial.println("____________Why I am here____________");
			for (int i = 0; i < len; i++)
			{
				Serial.read();
			}
		}
		// digitalWrite(13, LOW);
	}

	//Block storage control
	// if (sMotor == 1)
	// {
	// 	storage.restPosition();
	// }
	// else if (sMotor == 2)
	// {
	// 	storage.loadPosition();
	// }

	//Gripper Actuation
	// if (!gripperFinished1 && gripperSelect == 1)
	// {
	// 	gripperFinished1 = gripper[gripperSelect - 1].setGripper(gripperState);
	// }

	// if (!gripperFinished2 && gripperSelect == 2)
	// {
	// 	gripperFinished2 = gripper[gripperSelect - 1].setGripper(gripperState);
	// }

	//
	if (gripperFinished1 && gripperFinished2)
	{
		useGravityComp = 1;
	}

	// Switching PID values for joint motors
	// if (gripper[0].isEngaged) // Gripper 1 just engaged
	// {
	// 	if (gripperEngagedSelect != 1)
	// 	{
	// 		gripperEngagedSelect = 1;
	// 		for (int i = 0; i < NUM_MOTORS; i++) // Switches PID values for joint motors
	// 		{
	// 			jointMotor[i].switchPID(gripperEngagedSelect);
	// 			useGravityComp = 0;
	// 		}
	// 	}
	// }
	// else if (gripper[1].isEngaged) // Gripper 2 just engaged
	// {
	// 	if (gripperEngagedSelect != 2)
	// 	{
	// 		gripperEngagedSelect = 2;
	// 		for (int i = 0; i < NUM_MOTORS; i++) // Switches PID values for joint motors
	// 		{
	// 			jointMotor[i].switchPID(gripperEngagedSelect);
	// 			useGravityComp = 0;
	// 		}
	// 	}
	// }
	// else
	// {
	// 	gripperEngagedSelect = 0;
	// 	useGravityComp = 1;
	// }

	// pending support for controlling both grippers
	// if (!gripperFinished1 && !gripperFinished2 && gripperSelect == 3)
	// {
	// 	gripperFinished1 = gripper[gripperSelect - 1].setGripper(gripperState);
	// 	gripperFinished2 = gripper[gripperSelect - 1].setGripper(gripperState);
	// }

	// 		if (millis()-lastPubAng>2000)
	//  {
	//    Serial.print("gripper 0  is Engaged: "); Serial.print(gripper[0].isE);
	// 	Serial.print("; Gripper 1: "); Serial.println(gripper[1].isE);
	//    lastPubAng=millis();
	//  }

	// if (millis() - test_angle_time > 2000)
	// {
	// 	jointMotor[0].setAngle(test_angles[test_angles_idx % 14][0]);
	// 	jointMotor[1].setAngle(test_angles[test_angles_idx % 14][1]);
	// 	jointMotor[2].setAngle(test_angles[test_angles_idx % 14][2]);
	// 	test_angle_time = millis();
	// 	test_angles_idx++;
	// 	Serial.println("Changing angles");
	// 	// jointMotor[0].sumError = 0;
	// 	// jointMotor[1].sumError = 0;
	// 	// jointMotor[2].sumError = 0;
	// }

	updateSpeeds();

	// delayMicroseconds(500);
	// delay(1000);
	// unsigned long final_time = millis();

	// Serial.print("Elapsed time of loop: ");
	// Serial.println(final_time - start_time);

	// jointMotor[0].updateSpeed();
	// jointMotor[1].updateSpeed();
	// jointMotor[2].updateSpeed();

	//pidTunning(jointSelectTune, pValue, iValue, dValue);
}

/*
* Print variable every 2000 millis
*/
void debugPrint(char jName[3], char pName[3], char iName[3], char dName[3], double pInput, double iInput, double dInput)
{
	if (millis() - lastPubAng > 2000)
	{
		Serial.print("--------");
		Serial.println(jName);
		Serial.print(pName);
		Serial.print(": ");
		Serial.println(pInput);
		Serial.print(iName);
		Serial.print(": ");
		Serial.println(iInput);
		Serial.print(dName);
		Serial.print(": ");
		Serial.println(dInput);
		lastPubAng = millis();
	}
}

/*
*	Calculate Gravity Compensation
*/
int gravityCompensation(JointMotor2 i, int th[], bool select)
{
	int theta0 = th[0];
	int theta1 = th[1];
	int theta2 = th[2];

	//Wrap around
	if (theta0 >= 360)
	{
		theta0 %= 360;
	}
	if (theta1 >= 360)
	{
		theta1 %= 360;
	}
	if (theta2 >= 360)
	{
		theta2 %= 360;
	}

	// if(theta0<=-360){
	// 	theta0 = theta0+360;
	// }
	// if(theta1<=-360){
	// 	theta1 = theta1+360;
	// }
	// if(theta2<=-360){
	// 	theta2 = theta2+360;
	// }

	if (gripperEngagedSelect == 2) // TODO change back to 2
	{							   // D link gripper engaged (block on current link)
		if (i.id == 0)
		{
			return k1_d * (g * m3 * (L3 - LCoM3) * sinLut[theta2 + theta1 + theta0]);
		}
		else if (i.id == 1)
		{
			return k2_d * (g * m3 * (L2 * sinLut[theta0 + theta1] + (L3 - LCoM3) * sinLut[theta0 + theta1 + theta2]) + g * (L2 - LCoM2) * m2 * sinLut[theta1 + theta0] + g * mblock * Lblock * sinLut[theta1 + theta0]);
		}
		else if (i.id == 2)
		{
			return k3_d * (g * m3 * (L1 * sinLut[theta0] + L2 * sinLut[theta0 + theta1] + (L3 - LCoM3) * sinLut[theta0 + theta1 + theta2]) + g * m2 * (L1 * sinLut[theta0] + (L2 - (L2 - LCoM2)) * sinLut[theta0 + theta1]) + g * (L1 - LCoM1) * m1 * sinLut[theta0] + g * mblock * (L1 * sinLut[theta0] + Lblock * sinLut[theta0 + theta1]));
		}
		else
		{
			Serial.print("NO JOINT ID AVAILABLE FOR GRAVITY COMPENSATION");
			return 0;
		}
	}
	else
	{ // A link gripper engaged (block on opposite link)
		if (i.id == 0)
		{
			return k1_a * (g * m3 * (L1 * sinLut[theta0] + L2 * sinLut[theta0 + theta1] + LCoM3 * sinLut[theta0 + theta1 + theta2]) + g * m2 * (L1 * sinLut[theta0] + LCoM2 * sinLut[theta0 + theta1]) + g * LCoM1 * m1 * sinLut[theta0] + g * mblock * (L1 * sinLut[theta0] + Lblock * sinLut[theta0 + theta1]));
		}
		else if (i.id == 1)
		{
			return k2_a * (g * m3 * (L2 * sinLut[theta0 + theta1] + LCoM3 * sinLut[theta0 + theta1 + theta2]) + g * LCoM2 * m2 * sinLut[theta1 + theta0] + g * mblock * Lblock * sinLut[theta1 + theta0]);
		}
		else if (i.id == 2)
		{
			return k3_a * (g * m3 * LCoM3 * sinLut[theta2 + theta1 + theta0]);
		}
		else
		{
			Serial.print("NO JOINT ID AVAILABLE FOR GRAVITY COMPENSATION");
			return 0;
		}
	}
}

/*
* Update Speed of all joint motor for PWM
*/
void updateSpeeds()
{
	int gc = 0;
	double speeds[NUM_MOTORS] = {0, 0, 0};
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		theta[i] = jointMotor[i].getAngleDegrees();
		// gc = gravityCompensation(jointMotor[i], theta, true) * gc_complimentary_filter;
		gc = 0; //TODO: Uncomment line above to add gravity comp back in
		speeds[i] = jointMotor[i].calcSpeed(gc, useGravityComp);
		// Serial.print("\nGravity Comp:");
		// Serial.print(gc);
		// Serial.print("\nPID: ");
		// Serial.println(speeds[i] - gc);
	}
	// jointMotor speed should be updated after all gcs are calculated to
	// minimize delay between each joint movement
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		jointMotor[i].setSpeed(speeds[i]);
		// Serial.println("Setting speed of joint");
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
// void pidTunning(int jointSelect, int potP, int potI, int potD)
// {
// 	int js = analogRead(jointSelect);
// 	int offset = 70;
// 	double ratio = 0.01056;

// 	if (js < 341)
// 	{
// 		if (js > 170)
// 		{
// 			jointMotor[0].kP = (analogRead(potP) - offset) * ratio;
// 			jointMotor[0].kI = (analogRead(potI) - offset) * ratio;
// 			jointMotor[0].kD = (analogRead(potD) - offset) * ratio;
// 		}
// 		debugPrint("J0", "KP", "KI", "KD", jointMotor[0].kP, jointMotor[0].kI, jointMotor[0].kD);
// 	}
// 	else if (js > 341 && js < 682)
// 	{
// 		if (js > 511)
// 		{
// 			jointMotor[1].kP = (analogRead(potP) - offset) * ratio;
// 			jointMotor[1].kI = (analogRead(potI) - offset) * ratio;
// 			jointMotor[1].kD = (analogRead(potD) - offset) * ratio;
// 		}
// 		debugPrint("J1", "KP", "KI", "KD", jointMotor[1].kP, jointMotor[1].kI, jointMotor[1].kD);
// 	}
// 	else if (js > 682)
// 	{
// 		if (js > 852)
// 		{
// 			jointMotor[2].kP = (analogRead(potP) - offset) * ratio;
// 			jointMotor[2].kI = (analogRead(potI) - offset) * ratio;
// 			jointMotor[2].kD = (analogRead(potD) - offset) * ratio;
// 		}
// 		debugPrint("J2", "KP", "KI", "KD", jointMotor[2].kP, jointMotor[2].kI, jointMotor[2].kD);
// 	}
// 	// Serial.print("Kp: ");
// 	// Serial.println(jointMotor[0].kP);
// }
