// #ifndef UNIT_TEST //
#include <string>

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "pins.h"
#include "jointMotor2.h"
#include "nfc.h"
#include "Gripper.h"
#include "test1Step.h"
#include "SystemStateConstants.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <inchworm_hw_interface/MagnetState.h>
#include <inchworm_hw_interface/PID.h>
#include <inchworm_hw_interface/PIDConsts.h>
#include <inchworm_hw_interface/ReadNFCBlock.h>
#include <inchworm_hw_interface/WriteNFCBlock.h>
#include <std_msgs/Int32.h>

////////////////////////////////////////////////////////////////
// TUNABLE PARAMETERS
////////////////////////////////////////////////////////////////
const bool CHANGE_JOINTMOTORS_FREQUENCY = true; // Be careful when enabling this constant (check the frequency of the pins to be changed)
const bool USE_MOTORS = true;
const bool USE_GRIPPERS = true;
const bool USE_MAGNETS = true;
const bool USE_DEBUG_BUTTON = true;

const bool PRINT_DEBUG = false;

////////////////////////////////////////////////////////////////
// CONTROLLER CONSTANTS
////////////////////////////////////////////////////////////////
const int NUM_MOTORS = 5;
JointMotor2 jointMotor[NUM_MOTORS];
STATE state = ST_HOLDING;
uint32_t previous_time;
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
// NFC STUFF
////////////////////////////////////////////////////////////////
NFC* nfc_0;
NFC* nfc_1;

uint8_t NFC_0_PIN = 29;
uint8_t NFC_1_PIN = 28;

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

ros::NodeHandle nh;
std_msgs::Int32 heartbeat_msg;
std_msgs::String debug_msg;
std_msgs::String fault_msg;
sensor_msgs::JointState joint_msg;
sensor_msgs::JointState goal_msg;
inchworm_hw_interface::MagnetState mag_msg;
inchworm_hw_interface::PIDConsts consts_msg;

////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES
////////////////////////////////////////////////////////////////

void UpdateMotors(void);

// ROS Serial publishing functions

void printDebug(const String& theString, const String& id);
void printFault(const String& theString, const String& id);
void printJointState(void);
void printJointGoal(void);
void printMagnets(void);
void printPID(void);
// ROS Serial callbacks

void heartbeatCB(const std_msgs::Int32 &msg);
void magnetCB(const inchworm_hw_interface::MagnetState &msg);
void goalCB(const sensor_msgs::JointState &msg);
void pidCB(const inchworm_hw_interface::PIDConsts &msg);

void handleNFCRead(const inchworm_hw_interface::ReadNFCBlockRequest &req, inchworm_hw_interface::ReadNFCBlockResponse &res);
void handleNFCWrite(const inchworm_hw_interface::WriteNFCBlockRequest &req, inchworm_hw_interface::WriteNFCBlockResponse &res);

////////////////////////////////////////////////////////////////
// PUBLISHERS, SUBSCRIBERS, SERVICES
////////////////////////////////////////////////////////////////

ros::Publisher heartbeatPub("inchworm/heartbeat_res", &heartbeat_msg);
ros::Publisher debugPub("inchworm/debug", &debug_msg);
ros::Publisher faultPub("inchworm/fault", &fault_msg);
ros::Publisher jointPub("inchworm/joint_states", &joint_msg);
ros::Publisher goalPub("inchworm/joint_goal", &goal_msg);
ros::Publisher magPub("inchworm/magnet_states", &mag_msg);
ros::Publisher pidPub("inchworm/pid_consts", &consts_msg);

ros::Subscriber<std_msgs::Int32> heartbeatSub("inchworm/heartbeat_req", &heartbeatCB);
ros::Subscriber<inchworm_hw_interface::MagnetState> magnetSub("inchworm/set_magnet_state", &magnetCB);
ros::Subscriber<sensor_msgs::JointState> goalSub("inchworm/set_joint_goal", &goalCB);
ros::Subscriber<inchworm_hw_interface::PIDConsts> pidSub("inchworm/set_pid_consts", &pidCB);

ros::ServiceServer<inchworm_hw_interface::ReadNFCBlockRequest, inchworm_hw_interface::ReadNFCBlockResponse> nfcReadServer("/inchworm/nfc_read", &handleNFCRead);
ros::ServiceServer<inchworm_hw_interface::WriteNFCBlockRequest, inchworm_hw_interface::WriteNFCBlockResponse> nfcWriteServer("/inchworm/nfc_write", &handleNFCWrite);

////////////////////////////////////////////////////////////////
// SETUP METHOD
////////////////////////////////////////////////////////////////
void setup()
{
	pinMode(18, INPUT_PULLUP);
	pinMode(19, INPUT_PULLUP);

	// setup timers
	Wire.begin();
	Serial.begin(115200);

	nh.initNode();
	nh.advertise(debugPub);
	nh.advertise(faultPub);
	nh.advertise(jointPub);
	nh.advertise(goalPub);
	nh.advertise(magPub);
	nh.advertise(pidPub);

	nh.advertiseService(nfcReadServer);
	nh.advertiseService(nfcWriteServer);

	nh.subscribe(heartbeatSub);
	nh.subscribe(pidSub);
	nh.subscribe(goalSub);
	nh.subscribe(magnetSub);

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
			analogWriteFrequency(JOINT_MOTOR1_FWD,FREQUENCY_JOINT_MOTORS);
			analogWriteFrequency(JOINT_MOTOR1_REV,FREQUENCY_JOINT_MOTORS);
			analogWriteFrequency(JOINT_MOTOR2_FWD,FREQUENCY_JOINT_MOTORS);
			analogWriteFrequency(JOINT_MOTOR2_REV,FREQUENCY_JOINT_MOTORS);
			analogWriteFrequency(JOINT_MOTOR3_FWD,FREQUENCY_JOINT_MOTORS);
			analogWriteFrequency(JOINT_MOTOR3_REV,FREQUENCY_JOINT_MOTORS);
			analogWriteFrequency(JOINT_MOTOR4_FWD,FREQUENCY_JOINT_MOTORS);
			analogWriteFrequency(JOINT_MOTOR4_REV,FREQUENCY_JOINT_MOTORS);
			analogWriteFrequency(JOINT_MOTOR5_FWD,FREQUENCY_JOINT_MOTORS);
			analogWriteFrequency(JOINT_MOTOR5_REV,FREQUENCY_JOINT_MOTORS);
		}

		temp[PARSE_PKT_LEN - 1] = '\n'; // Buffer for serial message (important)

		/**
		 * Intialize Joint Motors (PINs, Dynamic PID values, Encoder I2C address, direction, ID)
		 */
		if(USE_MOTORS){
			// proper home			
			
			jointMotor[0] = JointMotor2(JOINT_MOTOR1_FWD, JOINT_MOTOR1_REV, JOINT_MOTOR1_EN, // A-LINK WRIST
										JOINT_MOTOR1_ADR, 10, 0, 0, 0, 10, 0, 0, 0, 0.0, -180, 180, true, 1, &printDebug, &printFault);
			jointMotor[1] = JointMotor2(JOINT_MOTOR2_FWD, JOINT_MOTOR2_REV, JOINT_MOTOR2_EN, // AB-LINK JOINT
										JOINT_MOTOR2_ADR, 40, 0, 10, -115, 10, 0, 0, 0, 19.655, -10, 90, false, 2, &printDebug, &printFault);
			jointMotor[2] = JointMotor2(JOINT_MOTOR3_FWD, JOINT_MOTOR3_REV, JOINT_MOTOR3_EN, // BC-LINK JOINT
										JOINT_MOTOR3_ADR, 30, 0, 6.25, -60, 23, 0, 0, 0, 140.689, -10, 140, true, 3, &printDebug, &printFault);
			jointMotor[3] = JointMotor2(JOINT_MOTOR4_FWD, JOINT_MOTOR4_REV, JOINT_MOTOR4_EN, // CD-LINK JOINT
										JOINT_MOTOR4_ADR, 25, 0, 0, 0, 23, 0, 0, 0, 19.655, -10, 90, true, 4, &printDebug, &printFault);
			jointMotor[4] = JointMotor2(JOINT_MOTOR5_FWD, JOINT_MOTOR5_REV, JOINT_MOTOR5_EN, // D-LINK WRIST
										JOINT_MOTOR5_ADR, 10, 0, 0, 10, 0, 0, 0, 0, 0.0, -180.0, 180.0, false, 5, &printDebug, &printFault);
			
			// proper home
			jointMotor[0].SetTarget(0);
			jointMotor[1].SetTarget(19.655);
			jointMotor[2].SetTarget(140.689);
			jointMotor[3].SetTarget(19.655);
			jointMotor[4].SetTarget(0);
		}

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

		previous_time = millis();
	}

	/*
	 * Start NFC
	 */
	nfc_0 = new NFC(NFC_0_PIN, &printDebug, &printFault);

	for (int i = 0; i < NUM_MOTORS; i++){
		jointMotor[i].set_vel_startTime(millis());
		jointMotor[i].set_vel_posStart(jointMotor[i].getAngleDegrees()* 2*(3.14159) / 360);
	}
}

////////////////////////////////////////////////////////////////
// MAIN LOOP
////////////////////////////////////////////////////////////////

void loop()
{
	printDebug("Beginning of loop", "loop");
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
				printDebug("Updating motors", "");
				UpdateMotors();
			}
		}
	}

	printDebug("Sending data over serial", "");
	printMagnets();
	printPID();
	printJointState();
	printJointGoal();

	printDebug("Spinning and delay", "");
	nh.spinOnce();
	delay(3);
}
////////////////////////////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////////////////////////////

/**
 * Updates all motors on robot
 */
void UpdateMotors()
{
	int speeds[NUM_MOTORS];
	printDebug("Calculating effort", "");
	for (int i = 1; i < NUM_MOTORS; i++)
	{
		speeds[i] = jointMotor[i].CalcEffort();
	}

	printDebug("Sending PWM", "");
	// jointMotor speed should be updated after all gcs are calculated to
	// minimize delay between each joint movement
	if (switchedPid_2)
	{
		for (int i = NUM_MOTORS - 1; i >= 1; i--)
		{
			switchedPid_2 = false;
			jointMotor[i].SendPWM(speeds[i]);
		}
	}
	else
	{
		for (int i = 1; i < NUM_MOTORS; i++)
		{
			jointMotor[i].SendPWM(speeds[i]);
		}
	}
}

void printDebug(const String& theString, const String& id)
{
	if(PRINT_DEBUG) {
		debug_msg.data = (id + String("\t") + theString).c_str();
		debugPub.publish(&debug_msg);
	}
}

void printFault(const String& theString, const String& id)
{
	fault_msg.data = (id + String("\t") + theString).c_str();
	faultPub.publish(&fault_msg);
}

void printJointState()
{
	joint_msg.header.stamp = nh.now();

	char *name[] = {"iw_ankle_foot_bottom", "iw_beam_ankle_bottom", "iw_mid_joint", "iw_beam_ankle_top", "iw_ankle_foot_top"};
	float pos[5];
	float vel[5];
	float eff[5];

	for(int i = 0; i < NUM_MOTORS; i++)
	{
		// Convert to radians
		pos[i] = jointMotor[i].getAngleDegrees() * 2*(3.14159) / 360;
		vel[i] = jointMotor[i].get_velocity(millis());
		eff[i] = jointMotor[i].CalcEffort();

		jointMotor[i].set_vel_posStart(pos[i]);
		jointMotor[i].set_vel_startTime(millis());
	}

	joint_msg.name = name;
	joint_msg.position = pos;
	joint_msg.velocity = vel;
	joint_msg.effort = eff;

	joint_msg.name_length = 5;
	joint_msg.position_length = 5;
	joint_msg.velocity_length = 5;
	joint_msg.effort_length = 5;

	jointPub.publish(&joint_msg);
}

void printJointGoal()
{
	goal_msg.header.stamp = nh.now();

	char *name[] = {"iw_ankle_foot_bottom", "iw_beam_ankle_bottom", "iw_mid_joint", "iw_beam_ankle_top", "iw_ankle_foot_top"};
	float pos[5];

	for(int i = 0; i < NUM_MOTORS; i++)
	{
		// Convert to radians
		pos[i] = jointMotor[i].GetTarget() * 2*(3.14159) / 360;
	}

	goal_msg.name = name;
	goal_msg.position = pos;

	goal_msg.name_length = 5;
	goal_msg.position_length = 5;

	goalPub.publish(&goal_msg);
}

void printMagnets()
{
	
	if (magState == magnetsOn)
	{
		mag_msg.magnet1 = 1;
		mag_msg.magnet2 = 1;
	}
	else if (magState == magnet1Off)
	{
		mag_msg.magnet1 = 0;
		mag_msg.magnet2 = 1;
	} 
	else if (magState == magnet2Off)
	{
		mag_msg.magnet1 = 1;
		mag_msg.magnet2 = 0;
	}
	
	magPub.publish(&mag_msg);
}

void printPID()
{
	inchworm_hw_interface::PID forward[5];
	inchworm_hw_interface::PID backward[5];

	double pid[3];

	for(int i = 0; i < NUM_MOTORS; i++)
	{
		jointMotor[i].getPIDF(pid);

		forward[i].p = pid[0];
		forward[i].i = pid[1];
		forward[i].d = pid[2];

		jointMotor[i].getPIDB(pid);

		backward[i].p = pid[0];
		backward[i].i = pid[1];
		backward[i].d = pid[2];
	}

	consts_msg.forward = forward;
	consts_msg.backward = backward;

	consts_msg.forward_length = 5;
	consts_msg.backward_length = 5;

	pidPub.publish(&consts_msg);
}

void heartbeatCB(const std_msgs::Int32 &msg)
{
	printDebug("Running heartbeat CB", "");
	heartbeat_msg.data = msg.data;
	printDebug("Set data", "");

	//delay(250);

	heartbeatPub.publish(&heartbeat_msg);
	printDebug("Responded to heartbeat", "");
}

void magnetCB(const inchworm_hw_interface::MagnetState &msg)
{
	if(msg.magnet1 == 1 && msg.magnet2 == 1)
	{
		magState = magnetsOn;
	}
	else if(msg.magnet1 == 0 && msg.magnet2 == 1)
	{
		magState = magnet1Off;
	}
	else if(msg.magnet1 == 1 && msg.magnet2 == 0)
	{
		magState = magnet2Off;
	}
	else
	{
		printFault("Requested magnet state is invalid!", "");
		magState = magnetsOn;
	}

	switch(magState)
	{
		case magnetsOn:
			digitalWrite(MAGNET_1, HIGH);
			digitalWrite(MAGNET_2, HIGH);
			break;
		case magnet1Off:
			digitalWrite(MAGNET_1, LOW);
			digitalWrite(MAGNET_2, HIGH);
			break;
		case magnet2Off:
			digitalWrite(MAGNET_1, HIGH);
			digitalWrite(MAGNET_2, LOW);
			break;
	}
}

void goalCB(const sensor_msgs::JointState &msg)
{
	for(int i = 0; i < NUM_MOTORS; i++)
	{
		// Convert from radians to degrees
		jointMotor[i].SetTarget(msg.position[i] * 180/PI);
	}
}

void pidCB(const inchworm_hw_interface::PIDConsts &msg)
{
	for(int i = 0; i < NUM_MOTORS; i++)
	{
		float forward[3] = {msg.forward[i].p, msg.forward[i].i, msg.forward[i].d};
		float backward[3] = {msg.backward[i].p, msg.backward[i].i, msg.backward[i].d};
		jointMotor[i].set_PID(forward, backward);
	}
}

void handleNFCRead(const inchworm_hw_interface::ReadNFCBlockRequest &req, inchworm_hw_interface::ReadNFCBlockResponse &res) {
	uint8_t data[16];

	NFC* board;

	if(req.bottom_foot) {
		printDebug("handleNFCRead from board 0", "");
		board = nfc_0;

		//if(!nfc_1->isPowered())
			//nfc_1->powerOff();
	} else {
		printDebug("handleNFCRead from board 1", "");
		board = nfc_1;

		//if(nfc_0->isPowered())
			//nfc_0->powerOff();
	}

	//if(!board->isPowered())
		//board->powerOn();

	bool success = board->readBlock(req.block, data);

	printDebug("readBlock() called", "");

	if(success) {
		res.data = data;
		res.data_length = 16;
	} else {
		printFault("Failed to read NFC data", "");
	}
}

void handleNFCWrite(const inchworm_hw_interface::WriteNFCBlockRequest &req, inchworm_hw_interface::WriteNFCBlockResponse &res) {
	if(req.data_length != 16) {
		printFault("NFC write service called with incorrect data array length", "");
		return;
	}

	NFC* board;

	if(req.bottom_foot) {
		printDebug("handleNFCWrite to board 0", "");
		board = nfc_0;
		//board->powerOn();
		//nfc_1->powerOff();
	} else {
		printDebug("handleNFCWrite to board 1", "");
		board = nfc_1;
		//board->powerOn();
		//nfc_0->powerOff();
	}

	printDebug("Writing block", "");

	bool success = board->writeBlock(req.block, req.data);

	if(!success) {
		printFault("Failed to read NFC data", "");
	}
}