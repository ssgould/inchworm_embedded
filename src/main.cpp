// #ifndef UNIT_TEST //
#include <string>

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "pins.h"
#include "jointMotor2.h"
#include "Gripper.h"
#include "test1Step.h"
#include "SystemStateConstants.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <inchworm_hw_interface/MagnetState.h>

////////////////////////////////////////////////////////////////
// TUNABLE PARAMETERS
////////////////////////////////////////////////////////////////
const bool CHANGE_JOINTMOTORS_FREQUENCY = false; // Be careful when enabling this constant (check the frequency of the pins to be changed)
const bool USE_MOTORS = true;
const bool USE_GRIPPERS = true;
const bool USE_MAGNETS = true;
const bool USE_DEBUG_BUTTON = true;

////////////////////////////////////////////////////////////////
// CONTROLLER CONSTANTS
////////////////////////////////////////////////////////////////
const int NUM_MOTORS = 5;
JointMotor2 jointMotor[NUM_MOTORS];
STATE state = ST_HOLDING;
double previous_time;
uint32_t startMoveTime = 0;
int numHb = 0;

////////////////////////////////////////////////////////////////
// SERIAL BUFFER
////////////////////////////////////////////////////////////////
const int MOTOR_PKT_LEN = 8;   // motor packet example: "-123.32_" (ending in space)
const int CONTROL_PKT_LEN = 3; // gripper and allen key control packet example: "0 1"
const int TOTAL_PACKET_LEN = 249; //MOTOR_PKT_LEN * NUM_MOTORS + CONTROL_PKT_LEN + 1;
char serialBuffer[TOTAL_PACKET_LEN];
char tempSerialBuffer[TOTAL_PACKET_LEN]; // Temporary Serial Buffer
const int PARSE_PKT_LEN = 5;
char temp[PARSE_PKT_LEN];
String inputBuffer; //String isn't the most efficient, but easier for I/O
String angleInputs;
////////////////////////////////////////////////////////////////
// GRIPPER CONTROL
////////////////////////////////////////////////////////////////
bool switchedPid_2 = false;
unsigned long incrementTime[4];
unsigned long lastIncrementTime[4];
bool new_command = true;

////////////////////////////////////////////////////////////////
// MAGNET STUFF
////////////////////////////////////////////////////////////////
enum MagnetState {magnetsOn, magnet1Off, magnet2Off};
MagnetState magState = magnetsOn;

////////////////////////////////////////////////////////////////
// TEST ANGLES
////////////////////////////////////////////////////////////////
// 0000.00 0005.21 0116.44 0058.30 0000.00 0011 [FIRST WAYPOINT]
// 0000.00 0030.00 0084.00 0067.00 0000.00 0100  [SECOND WAYPOINT]
// 0000.00 0045.00 0090.00 0045.00 0000.00 0100 [THIRD WAYPOINT]
// 0000.00 0027.81 0124.38 0027.81 0000.00 0000 [HOME POSITION]

////////////////////////////////////////////////////////////////
// SYTEM CONSTANTS
////////////////////////////////////////////////////////////////
int testState = ROBOT_NORMAL;
//int testState = TEST_MOTORS;

ros::NodeHandle nh;
std_msgs::String debug_msg;
std_msgs::String fault_msg;
inchworm_hw_interface::MagnetState mag_msg;


////////////////////////////////////////////////////////////////
// PUBLISHERS
////////////////////////////////////////////////////////////////

ros::Publisher debugPub("hw_interface/debug", &debug_msg);
ros::Publisher faultPub("hw_interface/fault", &fault_msg);
ros::Publisher magPub("hw_interface/magnet_state", &mag_msg);

////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES
////////////////////////////////////////////////////////////////
void UpdateMotors(void);

//serial stuff

void printDebug(String theString);
void printFault(String theString);
void printMagnets(void);


////////////////////////////////////////////////////////////////
// SETUP METHOD
////////////////////////////////////////////////////////////////
void setup()
{
	Wire.begin();		  	// Begin I2C
	Serial.begin(57600); 	
	//Serial.setTimeout(20);
	// Serial.setTimeout(0);

	//Serial.println("Robot intializing....");

	// Power LED
	pinMode(POWER_LED, OUTPUT);
	digitalWrite(POWER_LED, HIGH);

	if(testState == TEST_ALL || testState == TEST_ENCODERS || testState == TEST_MOTORS){
		// No intialization is needed for testing
	}else if(testState == ROBOT_TUNNING || testState == ROBOT_NORMAL){
		/**
		* Change frequency pins for MotorControllers
		*/
		if(CHANGE_JOINTMOTORS_FREQUENCY)
		{
			analogWriteFrequency(5,FREQUENCY_JOINT_MOTORS);
			analogWriteFrequency(3,FREQUENCY_JOINT_MOTORS);
			analogWriteFrequency(2,FREQUENCY_JOINT_MOTORS);
		}

		temp[PARSE_PKT_LEN - 1] = '\n'; // Buffer for serial message (important)

		/**
		 * Intialize Joint Motors (PINs, Dynamic PID values, Encoder I2C address, direction, ID)
		 */
		if(USE_MOTORS){
			// proper home
			/*
			jointMotor[0] = JointMotor2(JOINT_MOTOR1_FWD, JOINT_MOTOR1_REV, JOINT_MOTOR1_EN, // A-LINK WRIST
										JOINT_MOTOR1_ADR, 0, 0, 0, 0, 0, 0, 0.0, false, 1);
			jointMotor[1] = JointMotor2(JOINT_MOTOR2_FWD, JOINT_MOTOR2_REV, JOINT_MOTOR2_EN, // AB-LINK JOINT
										JOINT_MOTOR2_ADR, 22, .5, 0, 8, .3, 0, 27.81, false, 2);
			jointMotor[2] = JointMotor2(JOINT_MOTOR3_FWD, JOINT_MOTOR3_REV, JOINT_MOTOR3_EN, // BC-LINK JOINT
										JOINT_MOTOR3_ADR, 22, 0.5, 0, 23, 0.4, 0, 124.38, true, 3);
			jointMotor[3] = JointMotor2(JOINT_MOTOR4_FWD, JOINT_MOTOR4_REV, JOINT_MOTOR4_EN, // CD-LINK JOINT
										JOINT_MOTOR4_ADR, 8, .3, 0, 23, .4, 0, 27.8, true, 4);
			jointMotor[4] = JointMotor2(JOINT_MOTOR5_FWD, JOINT_MOTOR5_REV, JOINT_MOTOR5_EN, // D-LINK WRIST
										JOINT_MOTOR5_ADR, 0, 0, 0, 0, 0, 0, 0.0, false, 5);
			*/
			jointMotor[0] = JointMotor2(JOINT_MOTOR1_FWD, JOINT_MOTOR1_REV, JOINT_MOTOR1_EN, // A-LINK WRIST
										JOINT_MOTOR1_ADR, 10, 0, 0, 10, 0, 0, 0.0, false, 1);
			jointMotor[1] = JointMotor2(JOINT_MOTOR2_FWD, JOINT_MOTOR2_REV, JOINT_MOTOR2_EN, // AB-LINK JOINT
										JOINT_MOTOR2_ADR, 10, 0, 0, 10, 0, 0, 0.0, false, 2);
			jointMotor[2] = JointMotor2(JOINT_MOTOR3_FWD, JOINT_MOTOR3_REV, JOINT_MOTOR3_EN, // BC-LINK JOINT
										JOINT_MOTOR3_ADR, 22, 0.5, 0, 23, 0.4, 0, 0.0, true, 3);
			jointMotor[3] = JointMotor2(JOINT_MOTOR4_FWD, JOINT_MOTOR4_REV, JOINT_MOTOR4_EN, // CD-LINK JOINT
										JOINT_MOTOR4_ADR, 8, .3, 0, 23, .4, 0, 0.0, true, 4);
			jointMotor[4] = JointMotor2(JOINT_MOTOR5_FWD, JOINT_MOTOR5_REV, JOINT_MOTOR5_EN, // D-LINK WRIST
										JOINT_MOTOR5_ADR, 10, 0, 0, 10, 0, 0, 0.0, false, 5);
			
			// proper home
			/*
			jointMotor[0].SetTarget(0.0);
			jointMotor[1].SetTarget(27.81);
			jointMotor[2].SetTarget(124.38);
			jointMotor[3].SetTarget(27.81);
			jointMotor[4].SetTarget(0.0);
			*/
			jointMotor[0].SetTarget(0.0);
			jointMotor[1].SetTarget(0.0);
			jointMotor[2].SetTarget(0.0);
			jointMotor[3].SetTarget(0.0);
			jointMotor[4].SetTarget(0.0);
		}

		inputBuffer.reserve(24);

		/*
		* Initialize Magnets
		*/
		pinMode(MAGNET_1, OUTPUT);
		pinMode(MAGNET_2, OUTPUT);

		digitalWrite(MAGNET_1, HIGH);
		digitalWrite(MAGNET_2, HIGH);
		
		if(USE_DEBUG_BUTTON)
		{
			pinMode(DEBUG_PIN, INPUT);
		}

		//Serial.println("Done");
		previous_time = millis();
	}
	nh.initNode();
	nh.advertise(debugPub);
	nh.advertise(faultPub);
	nh.advertise(magPub);
}

////////////////////////////////////////////////////////////////
// MAIN LOOP
////////////////////////////////////////////////////////////////

void loop()
{
	if(testState == ROBOT_TUNNING || testState == ROBOT_NORMAL){
		// Move joint motors
		static uint32_t lastUpdateTime = millis();
		uint32_t currTime = millis();
		if (currTime - lastUpdateTime >= UPDATE_INTERVAL)
		{
			if (currTime - lastUpdateTime > UPDATE_INTERVAL)
				//Serial.println("Missed update schedule.");

			lastUpdateTime += UPDATE_INTERVAL;

			if (state == ST_HOLDING || ST_MOVING)
			{
				UpdateMotors();
			}
		}
	}

	printMagnets();

	nh.spinOnce();
	delay(10);

}
////////////////////////////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////////////////////////////

/**
 * Updates all motors on robot
 */
void UpdateMotors()
{
	int speeds[MOTOR_COUNT];
	for (int i = 0; i < NUM_MOTORS; i++)
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
		for (int i = 0; i < NUM_MOTORS; i++)
		{
			jointMotor[i].SendPWM(speeds[i]);
		}
	}
	//Serial.printf("E %3.2f %3.2f %3.2f %3.2f %3.2f\n", speeds[0], speeds[1], speeds[2], speeds[3],  speeds[4]);
}

void printDebug(String theString){	
	debug_msg.data = theString.c_str();
	debugPub.publish(&debug_msg);
}

void printFault(String theString){
	fault_msg.data = theString.c_str();
	faultPub.publish(&fault_msg);
}

void printMagnets(){
	
	if (magState == magnetsOn)
	{
		mag_msg.magnet1 = 1;
		mag_msg.magnet2 = 1;
	} else if (magState == magnet1Off) {
		mag_msg.magnet1 = 0;
		mag_msg.magnet2 = 1;
	} else if (magState == magnet2Off) {
		mag_msg.magnet1 = 1;
		mag_msg.magnet2 = 0;
	} 
	
	magPub.publish(&mag_msg);
}