#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "pins.h"
#include "JointMotor2.h"
// #include "Gripper.h"

////////////////////////////////////////////////////////////////
// TUNABLE PARAMETERS
////////////////////////////////////////////////////////////////
const bool DEBUG = false;
const bool TUNING = false;
const bool USE_GRIPPERS = true;

////////////////////////////////////////////////////////////////
// CONTROLLER CONSTANTS
////////////////////////////////////////////////////////////////
const int NUM_MOTORS = 3;
JointMotor2 jointMotor[NUM_MOTORS];
STATE state = ST_HOLDING;
double previous_time;
uint32_t lastViaUpdate = 0;
uint32_t startMoveTime = 0;

////////////////////////////////////////////////////////////////
// SERIAL BUFFER
////////////////////////////////////////////////////////////////
const int MOTOR_PKT_LEN = 8;   // motor packet example: "-123.32_" (ending in space)
const int CONTROL_PKT_LEN = 4; // control packet example: "0131"
const int len = MOTOR_PKT_LEN * 3 + CONTROL_PKT_LEN;
char serialBuffer[len];
const int PARSE_PKT_LEN = 5;
char temp[PARSE_PKT_LEN];
String inputBuffer; //String isn't the most efficient, but easier for I/O

////////////////////////////////////////////////////////////////
// GRIPPER CONTROL
////////////////////////////////////////////////////////////////
// Gripper gripper[4];
bool gripperFinished1 = true;
bool gripperFinished2 = true;
bool allenKeyFinished = true;
int gripperStatus = 0;
int gripperSelect = 0; //idle (No gripper selected)
int gripperState = 0;  //idle (No gripper action)
int gripperEngagedSelect = 0;
uint8_t switchGrippers = 0;
int allenKeySelect = 0;
int allenKeyState = 0;
bool switchedPid_2 = false;

////////////////////////////////////////////////////////////////
// TEST ANGLES
////////////////////////////////////////////////////////////////
// 0005.21 0116.44 0058.30 0100 Trevor use this one [FIRST WAYPOINT]
// 0030.00 0084.00 0067.00 0100 //Try this after [SECOND WAYPOINT]
// 0045.00 0090.00 0045.00 0100 [THIRD WAYPOINT]

//D link fixed
// 0067.00 0084.00 0030.00 0100
// storage = Storage(STORAGE_MOTOR_LC);
// 0058.30 0116.44 0005.21 0100
// 0067.00 0084.00 0030.00 0100 //Try this after [SECOND WAYPOINT]
// 0045.00 0090.00 0045.00 0100 [THIRD WAYPOINT]

// 0030.00 0130.00 0030.00 0100
// 0028.00 0124.00 0028.00 0100
// 0028.00 000 0124.00 000 0028.00 000 0100

////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES
////////////////////////////////////////////////////////////////
void UpdateMotors(void);
void SetNewVias(void);
void StartMove(bool);
void ReadAngleInputs(void);
void RunPidTuningDebug(void);
void ActuateGrippers(void);

////////////////////////////////////////////////////////////////
// SETUP METHOD
////////////////////////////////////////////////////////////////
void setup()
{
	Wire.begin();		  //begin I2C
	Serial.begin(115200); //Debug Serial

	pinMode(POWER_LED, OUTPUT);     // Power LED
	digitalWrite(POWER_LED, HIGH);
	temp[PARSE_PKT_LEN - 1] = '\n'; // Buffer for serial message (important)
	Serial.println("Robot intializing....");

	// jointMotor[0] = JointMotor2(JOINT_MOTOR1_FWD, JOINT_MOTOR1_REV, JOINT_MOTOR1_EN,
	// 							JOINT_MOTOR1_ADR, 40, 0.3, 20, 27.81, true, 0);
	// jointMotor[1] = JointMotor2(JOINT_MOTOR2_FWD, JOINT_MOTOR2_REV, JOINT_MOTOR2_EN,
	// 							JOINT_MOTOR2_ADR, 40, 0.3, 20, 124.38, true, 1);
	// jointMotor[2] = JointMotor2(JOINT_MOTOR3_FWD, JOINT_MOTOR3_REV, JOINT_MOTOR3_EN,
	// 							JOINT_MOTOR3_ADR, 40, 0.3, 20, 27.81, false, 2);

	// jointMotor[0] = JointMotor2(JOINT_MOTOR1_FWD, JOINT_MOTOR1_REV, JOINT_MOTOR1_EN,
	// 							JOINT_MOTOR1_ADR, 20, 0.3, 20, 27.81, true, 0);
	// jointMotor[1] = JointMotor2(JOINT_MOTOR2_FWD, JOINT_MOTOR2_REV, JOINT_MOTOR2_EN,
	// 							JOINT_MOTOR2_ADR, 20, 0.3, 20, 124.38, true, 1);
	// jointMotor[2] = JointMotor2(JOINT_MOTOR3_FWD, JOINT_MOTOR3_REV, JOINT_MOTOR3_EN,
	// 							JOINT_MOTOR3_ADR, 10, 0.3, 20, 27.81, false, 2);

	// jointMotor[0] = JointMotor2(JOINT_MOTOR1_FWD, JOINT_MOTOR1_REV, JOINT_MOTOR1_EN,
	// 							JOINT_MOTOR1_ADR, 20, 0.3, 20, 5, 0.15, 15, 27.81, true, 0);
	// jointMotor[1] = JointMotor2(JOINT_MOTOR2_FWD, JOINT_MOTOR2_REV, JOINT_MOTOR2_EN,
	// 							JOINT_MOTOR2_ADR, 20, 0.3, 20, 7, 0.25, 15, 124.38, true, 1);
	// jointMotor[2] = JointMotor2(JOINT_MOTOR3_FWD, JOINT_MOTOR3_REV, JOINT_MOTOR3_EN,
	// 							JOINT_MOTOR3_ADR, 10, 0.3, 20, 15, 0.25, 15, 27.81, false, 2);

	jointMotor[0] = JointMotor2(JOINT_MOTOR1_FWD, JOINT_MOTOR1_REV, JOINT_MOTOR1_EN,
								JOINT_MOTOR1_ADR, 20, 0.3, 20, 30, 0.35, 20, 27.81, true, 0);
	// jointMotor[1] = JointMotor2(JOINT_MOTOR2_FWD, JOINT_MOTOR2_REV, JOINT_MOTOR2_EN,
	// 							JOINT_MOTOR2_ADR, 20, 0.3, 20, 15, 0.01, 0.8, 124.38, true, 1);
	// jointMotor[2] = JointMotor2(JOINT_MOTOR3_FWD, JOINT_MOTOR3_REV, JOINT_MOTOR3_EN,
	// 							JOINT_MOTOR3_ADR, 10, 0.3, 20, 17, 0.25, 8, 27.81, false, 2);

	/**
	 * Went into the hole on switched pid (d link engaged)
	 */
	// jointMotor[0] = JointMotor2(JOINT_MOTOR1_FWD, JOINT_MOTOR1_REV, JOINT_MOTOR1_EN,
	// 							JOINT_MOTOR1_ADR, 20, 0.3, 20, 30, 0.35, 20, 27.81, true, 0);
	// jointMotor[1] = JointMotor2(JOINT_MOTOR2_FWD, JOINT_MOTOR2_REV, JOINT_MOTOR2_EN,
	// 							JOINT_MOTOR2_ADR, 20, 0.3, 20, 4, 0.05, 1, 124.38, true, 1);
	// jointMotor[2] = JointMotor2(JOINT_MOTOR3_FWD, JOINT_MOTOR3_REV, JOINT_MOTOR3_EN,
	// 							JOINT_MOTOR3_ADR, 10, 0.3, 20, 17, 0.25, 8, 27.81, false, 2);

	jointMotor[0].SetTarget(27.81);
	// jointMotor[1].SetTarget(124.38);
	// jointMotor[2].SetTarget(27.81);

	inputBuffer.reserve(24);

	if (USE_GRIPPERS)
	{
		// gripper[0] = Gripper(GRIPPER_MOTOR_1, false, false); //yellow gripper
		// gripper[1] = Gripper(GRIPPER_MOTOR_2, true, false);  //red gripper
		// gripper[2] = Gripper(GRIPPER_MOTOR_3, false, false); //yellow gripper
		// gripper[3] = Gripper(GRIPPER_MOTOR_4, true, false);  //red gripper
		// gripperSelect = jointMotor[0].fixed_link == jointMotor[0].a_link_engaged ? 1 : 2;
		// gripperState = gripper[0].engage;
	}

	Serial.println("Done");
	previous_time = millis();
}

////////////////////////////////////////////////////////////////
// MAIN LOOP
////////////////////////////////////////////////////////////////
void loop()
{
	// Update flag to tune PID value on Robot or Run Robot Normally
	if (TUNING) 
	{
		RunPidTuningDebug();
	}
	else
	{
		ReadAngleInputs();
	}

	// Update flag to use grippers
	if (USE_GRIPPERS)
	{
		ActuateGrippers();
	}


	// Move joint motors
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
	}

	// Set vias between waypoints
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

////////////////////////////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////////////////////////////

/**
 * Tunes PID values quickly and tests small trajectories
 *  
 * This method allows for the use of the Serial Terminal to tune 
 * PID values separatly for when D or A link is fixed with the 
 * Serial inputs shown below. 
 * 
 * 			> P0.1 //Controller term and then value
 *  		> A    //Prints all PID values
 * 
 * Important: Once the PID values are tuned they can be printed 
 * to Serial as they are volatile. 
 * 
 * To make tunning easier this method allows for the robot to be
 * controlled by moving it down and up with two simple commands 
 * shown below. This is a simple way of checking if the values 
 * for the PID controller are tunned properly.
 * 
 * 			> M	// Move robot up
 *    		> N // Move robot down
 *  
 */
void RunPidTuningDebug()
{
	if (Serial.available())
	{
		Serial.println("Message received");

		char c = Serial.read();
		inputBuffer += c;

		if (c == '\n')
		{
			// Move up 
			if (inputBuffer[0] == 'M')
			{
				StartMove(0);
				state = ST_MOVING;
			}

			// Move down
			if (inputBuffer[0] == 'N')
			{
				StartMove(1);
				state = ST_MOVING;
			}

			// Tune PID values
			if (inputBuffer[0] == 'P')
			{
				float k = inputBuffer.substring(1).toFloat();
				for (int i = 0; i < MOTOR_COUNT; i++)
				{
					jointMotor[i].SetKp(k);
				}
			}

			if (inputBuffer[0] == 'I')
			{
				float k = inputBuffer.substring(1).toFloat();
				for (int i = 0; i < MOTOR_COUNT; i++)
				{
					jointMotor[i].SetKi(k);
				}
			}

			if (inputBuffer[0] == 'D')
			{
				float k = inputBuffer.substring(1).toFloat();
				for (int i = 0; i < MOTOR_COUNT; i++)
				{
					jointMotor[i].SetKd(k);
				}
			}

			// Print PID values
			if (inputBuffer[0] == 'A')
			{
				for (int i = 0; i < MOTOR_COUNT; i++)
				{
					jointMotor[i].printPID();
				}
			}

			// if (inputBuffer[0] == 'S')
			// {
			// 	switchGrippers = switchGrippers == 1 ? 2 : 1;
			// 	Serial.print("Switching PIDs");

			// 	for (int i = 0; i < MOTOR_COUNT; i++)
			// 	{
			// 		jointMotor[i].SwitchPID(switchGrippers);
			// 	}
			// }

			inputBuffer = ""; // Reset buffer
		}
	}
}

/**
 * Updates all motors on robot 
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
	if (switchedPid_2)
	{
		for (int i = MOTOR_COUNT - 1; i >= 0; i--)
		{
			switchedPid_2 = false;
			jointMotor[i].SendPWM(speeds[i]);
		}
	}
	else
	{
		for (int i = 0; i < MOTOR_COUNT; i++)
		{
			jointMotor[i].SendPWM(speeds[i]);
		}
	}
}


/**
 * Helper method to start robot movement when tunning PID values 
 */

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

/**
 * Configures vias between waypoints for robot to move
 */
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

/**
 * Main method that reads control message form high level code
 * 
 * This method parses and organizes the serial control message
 * to angle for each joint, gripper configuration, and allen 
 * key actuation. 
 */
void ReadAngleInputs()
{
	if (Serial.available() > 0)
	{
		double current_time = millis();
		Serial.println("Message received");
		previous_time = current_time;
		Serial.readBytesUntil('\n', serialBuffer, len);
		int tempIndex = 0;
		int jointIndex = 0;
		float tempAngle = 0;
		boolean motorPktCompleted = true;

		if (serialBuffer[0] == '-' || serialBuffer[0] == '0')
		{
			for (int i = 0; i < len; i++)
			{
				temp[tempIndex] = serialBuffer[i];
				if (tempIndex < 3)
				{
					tempIndex++;
				}
				else
				{
					temp[PARSE_PKT_LEN - 1] = '\n'; //you need this
					tempIndex = 0;
					if (jointIndex > 2)
					{   //Gripper
						if (USE_GRIPPERS)
						{
							// Allen Key Control
							if ((temp[0] - '0' == 1) || (temp[0] - '0' == 2) || (temp[0] - '0' == 3))
							{
								allenKeySelect = (temp[0] - '0');
								allenKeyState = (temp[1] - '0');

								allenKeyFinished = false;
							}

							// Gripper Control
							if ((temp[2] - '0' == 1) || (temp[2] - '0' == 2) || (temp[2] - '0' == 3))
							{
								gripperSelect = (temp[2] - '0');
								gripperState = (temp[3] - '0');

								if (gripperSelect == 1)
								{
									// Serial.println("Gripper select set to 1");
									gripperFinished1 = false;
								}
								else if (gripperSelect == 2)
								{
									// Serial.println("Gripper select set to 2");
									gripperFinished2 = false;
								}
								else
								{ // both gripper selected
									// Serial.println(gripperSelect);
									gripperFinished1 = false;
									gripperFinished2 = false;
								}
							}
						}
					}
					else
					{   //Joint angles
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
						{	// receiving second half of an angle value
							motorPktCompleted = true; // flip the flag to true
							temp[0] = '0';
							temp[PARSE_PKT_LEN - 1] = '0';
							tempAngle += (atof(temp) * 0.01); // divide by 1000 to compensate for the extra 0
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
			for (int i = 0; i < len; i++)
			{
				Serial.read();
			}
		}
	}
}


/**
 * Helper method to actuate grippers and allen key
 */
void ActuateGrippers()
{

	// Allen Key Control Code
	// if (!allenKeyFinished && (allenKeySelect == 1 || allenKeySelect == 2)
	// {
	// 	allenKeyFinished = gripper[allenKeySelect - 1 + 2].setGripper(allenKeyState);
	// }

	// Gripper Control Code
	if (!gripperFinished1 && gripperSelect == 1)
	{
		// gripperFinished1 = gripper[gripperSelect - 1].setGripper(gripperState);
		gripperFinished1 = true;
	}

	if (!gripperFinished2 && gripperSelect == 2)
	{
		// gripperFinished2 = gripper[gripperSelect - 1].setGripper(gripperState);
		gripperFinished2 = true;
	}

	// Switching PID values for joint motors
	// Serial.print("Gripper Finished 1: ");
	// Serial.print(gripperFinished1);

	// Serial.print("\tGripper Finished 2: ");
	// Serial.print(gripperFinished1);

	// Serial.print("\nGripper Select ");
	if (gripperFinished1 && gripperSelect == 1 && gripperState == 1) // A link gripper just engaged
	{

		gripperEngagedSelect = jointMotor[0].a_link_engaged;
		for (int i = 0; i < NUM_MOTORS; i++) // Switches PID values for joint motors
		{
			jointMotor[i].SwitchPID(gripperEngagedSelect);
		}
		gripperSelect = 0;
	}
	else if (gripperFinished2 && gripperSelect == 2 && gripperState == 1) // D link gripper just engaged
	{

		gripperEngagedSelect = jointMotor[0].d_link_engaged;
		for (int i = 0; i < NUM_MOTORS; i++) // Switches PID values for joint motors
		{
			switchedPid_2 = true;
			jointMotor[i].SwitchPID(gripperEngagedSelect);
		}
		gripperSelect = 0;
	}
	else
	{
		gripperEngagedSelect = jointMotor[0].neither_gripper_engaged;
	}
}