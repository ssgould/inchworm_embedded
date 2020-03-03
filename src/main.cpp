#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "pins.h"
#include "JointMotor2.h"
const int NUM_MOTORS = 3;
JointMotor2 jointMotor[NUM_MOTORS];

// JointMotor2 jointMotor[3];
STATE state = ST_HOLDING;

// //Serial Buffer
// const int len = 16;
// char serialBuffer[len];
String inputBuffer; //String isn't the most efficient, but easier for I/O

void UpdateMotors(void);
void SetNewVias(void);
void StartMove(bool);

double theta[3];
// double via_point_theta[3];
float m1 = 0.281; // 0.09  CAD value: 0.183
float m2 = 0.297; // with storing mechanism (with block 0.297 kg) Old: 0.357
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

float k1_a = -0.1;  //-0.116
float k2_a = -0.12; //-0.129
float k3_a = -0.05; //-0.15

float k1_d = -0.089; //a link
float k2_d = -0.1325;
float k3_d = -0.037;

float gc_complimentary_filter = 0.;
int useGravityComp = 0;

//Serial Buffer
const int MOTOR_PKT_LEN = 8;   // motor packet example: "-123.32_" (ending in space)
const int CONTROL_PKT_LEN = 4; // control packet example: "0131"
const int len = MOTOR_PKT_LEN * 3 + CONTROL_PKT_LEN;
char serialBuffer[len];
const int PARSE_PKT_LEN = 5;
char temp[PARSE_PKT_LEN];

unsigned long start_time;

int test_angles_idx = 0;
unsigned long test_angle_time = 0;

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

int velocity_term_scale = 0;
unsigned long dead_ban_test_delay = 1000;
unsigned long dead_ban_test_time = 0;
double testSpeed = 0;

// void pidTunning(int jointSelect, int potP, int potI, int potD);

// Gripper gripper[2];

double previous_time;

void setup()
{
	Serial.begin(115200); //Debug Serial
	Wire.begin();		  //begin I2C

	Serial.println("Robot intializing....");
	temp[PARSE_PKT_LEN - 1] = '\n'; //you need this

	// jointMotor[0] = JointMotor2(JOINT_MOTOR1_FWD, JOINT_MOTOR1_REV, JOINT_MOTOR1_EN,
	// 							JOINT_MOTOR1_ADR, 40, 0.3, 20, 27.81, true, 0);
	// jointMotor[1] = JointMotor2(JOINT_MOTOR2_FWD, JOINT_MOTOR2_REV, JOINT_MOTOR2_EN,
	// 							JOINT_MOTOR2_ADR, 40, 0.3, 20, 124.38, true, 1);
	// jointMotor[2] = JointMotor2(JOINT_MOTOR3_FWD, JOINT_MOTOR3_REV, JOINT_MOTOR3_EN,
	// 							JOINT_MOTOR3_ADR, 40, 0.3, 20, 27.81, false, 2);

	jointMotor[0] = JointMotor2(JOINT_MOTOR1_FWD, JOINT_MOTOR1_REV, JOINT_MOTOR1_EN,
								JOINT_MOTOR1_ADR, 20, 0.3, 20, 27.81, true, 0);
	jointMotor[1] = JointMotor2(JOINT_MOTOR2_FWD, JOINT_MOTOR2_REV, JOINT_MOTOR2_EN,
								JOINT_MOTOR2_ADR, 20, 0.3, 20, 124.38, true, 1);
	jointMotor[2] = JointMotor2(JOINT_MOTOR3_FWD, JOINT_MOTOR3_REV, JOINT_MOTOR3_EN,
								JOINT_MOTOR3_ADR, 10, 0.3, 20, 27.81, false, 2);

	jointMotor[0].SetTarget(27.81);
	jointMotor[1].SetTarget(124.38);
	jointMotor[2].SetTarget(27.81);

	// inputBuffer.reserve(24);

	Serial.println("Done");
	previous_time = millis();
}

uint32_t lastViaUpdate = 0;
uint32_t startMoveTime = 0;

// 0005.21 0116.44 0058.30 0100 Trevor use this one [FIRST WAYPOINT]
// 0030.00 0084.00 0067.00 0100 //Try this after [SECOND WAYPOINT]
// 0045.00 0090.00 0045.00 0100 [THIRD WAYPOINT]

//D link fixed
// 0067.00 0084.00 0030.00 0100
// storage = Storage(STORAGE_MOTOR_LC);

// 0030.00 0130.00 0030.00 0100
// 0028.00 0124.00 0028.00 0100
// 0028.00 000 0124.00 000 0028.00 000 0100

void loop()
{
	// if (Serial.available())
	// {
	// 	Serial.println("Message received");

	// 	char c = Serial.read();
	// 	inputBuffer += c;

	// 	if (c == '\n')
	// 	{
	// 		if (inputBuffer[0] == 'M')
	// 		{
	// 			StartMove(0);
	// 			state = ST_MOVING;
	// 		}

	// 		if (inputBuffer[0] == 'N')
	// 		{
	// 			StartMove(1);
	// 			state = ST_MOVING;
	// 		}

	// 		if (inputBuffer[0] == 'P')
	// 		{
	// 			float k = inputBuffer.substring(1).toFloat();
	// 			for (int i = 0; i < MOTOR_COUNT; i++)
	// 			{
	// 				jointMotor[i].SetKp(k);
	// 			}
	// 		}

	// 		if (inputBuffer[0] == 'I')
	// 		{
	// 			float k = inputBuffer.substring(1).toFloat();
	// 			for (int i = 0; i < MOTOR_COUNT; i++)
	// 			{
	// 				jointMotor[i].SetKi(k);
	// 			}
	// 		}

	// 		if (inputBuffer[0] == 'D')
	// 		{
	// 			float k = inputBuffer.substring(1).toFloat();
	// 			for (int i = 0; i < MOTOR_COUNT; i++)
	// 			{
	// 				jointMotor[i].SetKd(k);
	// 			}
	// 		}

	// 		inputBuffer = "";
	// 	}
	// }
	if (Serial.available() > 0)
	{
		// start_time = millis();
		double current_time = millis();
		Serial.println("Message received");
		// Serial.println((current_time - previous_time) / 1000);
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
			// Serial.print("serialBuffer ");
			// Serial.println(serialBuffer);
			for (int i = 0; i < len; i++)
			{
				temp[tempIndex] = serialBuffer[i];
				// Serial.print("tempIndex ");
				// Serial.println(tempIndex);
				if (tempIndex < 3)
				{
					tempIndex++;
				}
				else
				{
					temp[PARSE_PKT_LEN - 1] = '\n'; //you need this
					// Serial.print("temp ");
					// Serial.println(temp);
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
						// // sMotor = temp[1] - '0';

						// // Parsing velocity term
						// // if (temp[0] == '0')
						// // {
						// // 	velocity_term_enable = false;
						// // }
						// // velocity_term = temp[0] - '0';

						// // Code for testing motor PWM dead ban
						// // testSpeed = (double)((temp[1] - '0') * 100 + (temp[2] - '0') * 10 + (temp[3] - '0'));
						// // if (temp[0] == '-')
						// // {
						// // 	testSpeed = -testSpeed;
						// // }

						// if (temp[0] - '0' >= 0 && temp[0] - '0' <= 9)
						// {
						// 	velocity_term_scale = temp[0] - '0';
						// }
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
							tempAngle += (atof(temp) * 0.01); // divide by 1000 to compensate for the extra 0
							useGravityComp = 1;
							jointMotor[jointIndex].SetTarget(tempAngle);
							// jointMotor[jointIndex].sumError = 0.0;
							Serial.print("Set angle[");
							Serial.print(jointIndex + 1);
							Serial.print("]: ");
							Serial.println(tempAngle);
							jointIndex++;
							// state = ST_MOVING;
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

	static uint32_t lastUpdateTime = millis();
	uint32_t currTime = millis();
	if (currTime - lastUpdateTime >= UPDATE_INTERVAL)
	{
		if (currTime - lastUpdateTime > UPDATE_INTERVAL)
			Serial.println("Missed update schedule.");

		lastUpdateTime += UPDATE_INTERVAL;

		if (state == ST_HOLDING || ST_MOVING)
		{
			UpdateMotors();
		}

		//Serial.println(currTime);
	}

	if (currTime - lastViaUpdate >= VIA_INTERVAL)
	{
		if (state == ST_MOVING)
		{
			if (currTime - startMoveTime >= VIA_COUNT * VIA_INTERVAL)
			{
				state = ST_HOLDING;
			}

			else
				SetNewVias();
		}
	}
}

/*
* Update motor efforts
*/
void UpdateMotors()
{
	int speeds[MOTOR_COUNT];
	for (int i = 0; i < MOTOR_COUNT; i++)
	{
		speeds[i] = jointMotor[i].CalcEffort();
	}

	// jointMotor speed should be updated after all gcs are calculated to
	// minimize delay between each joint movement
	for (int i = 0; i < MOTOR_COUNT; i++)
	{
		jointMotor[i].SendPWM(speeds[i]);
	}
}

float startAngles[MOTOR_COUNT];
float targetAngles[MOTOR_COUNT];

void StartMove(bool dir)
{
	for (int i = 0; i < MOTOR_COUNT; i++)
	{
		startAngles[i] = jointMotor[i].getAngleDegrees();
		if (dir)
			targetAngles[i] = startAngles[i] - 5;
		else
			targetAngles[i] = startAngles[i] + 5;
	}

	startMoveTime = millis();
	lastViaUpdate = startMoveTime;

	state = ST_MOVING;
}

void SetNewVias(void)
{
	uint32_t currTime = millis();
	float fraction = (currTime - startMoveTime) / (float)(VIA_COUNT * VIA_INTERVAL);
	if (fraction < 0)
		fraction = 0;
	if (fraction > 1)
		fraction = 1;

	for (int i = 0; i < MOTOR_COUNT; i++)
	{
		float viaAngle = startAngles[i] + (targetAngles[i] - startAngles[i]) * fraction;
		jointMotor[i].SetTarget(viaAngle);
	}
}