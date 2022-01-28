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
#include "messageStructs.h"

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

////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES
////////////////////////////////////////////////////////////////
void UpdateMotors(void);
void StartMove(bool);
void ReadAngleInputs(void);
void ReadAngleString(void);
void RunPidTuningDebug(void);
bool buttonPressed(void);
void testJointMotor(void);
void testEncoders(void);
void testMotors(void);

//serial stuff
void readSerial(void);
void readJoints(poseGoalPacket_t message);
void readPID(PID_Packet message);
void readMagnets(magnetPacket_t message);
void printTarget(void);
void printDebug(String theString);
void printFault(String theString);
void printMagnets(void);
void printPID(void);
void printJointState(void);
void readHeartbeat(heartPacket_t heartBeat);
void printHeartbeat(void);

//magnet switching
void setMagnetState(int mag1, int mag2);
void updateMagnets(void);
void testMagnets(void);
void test(void);

////////////////////////////////////////////////////////////////
// SETUP METHOD
////////////////////////////////////////////////////////////////
void setup()
{
	Wire.begin();		  	// Begin I2C
	Serial.begin(115200); 	
	Serial.setTimeout(20);
	// Serial.setTimeout(0);

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
										JOINT_MOTOR1_ADR, 10, 0, 0, 10, 0, 0, 0.0, false, 1, 30.0);
			jointMotor[1] = JointMotor2(JOINT_MOTOR2_FWD, JOINT_MOTOR2_REV, JOINT_MOTOR2_EN, // AB-LINK JOINT
										JOINT_MOTOR2_ADR, 10, 0, 0, 10, 0, 0, 0.0, false, 2, 60.0);
			jointMotor[2] = JointMotor2(JOINT_MOTOR3_FWD, JOINT_MOTOR3_REV, JOINT_MOTOR3_EN, // BC-LINK JOINT
										JOINT_MOTOR3_ADR, 22, 0.5, 0, 23, 0.4, 0, 0.0, true, 3, 90.0);
			jointMotor[3] = JointMotor2(JOINT_MOTOR4_FWD, JOINT_MOTOR4_REV, JOINT_MOTOR4_EN, // CD-LINK JOINT
										JOINT_MOTOR4_ADR, 8, .3, 0, 23, .4, 0, 0.0, true, 4, 60.0);
			jointMotor[4] = JointMotor2(JOINT_MOTOR5_FWD, JOINT_MOTOR5_REV, JOINT_MOTOR5_EN, // D-LINK WRIST
										JOINT_MOTOR5_ADR, 10, 0, 0, 10, 0, 0, 0.0, false, 5, 30.0);
			
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

	bool readyToStart = false;
	while (!readyToStart) {
		//wait for heartbeat
		if (Serial.available() > 0) {
			Serial.readBytesUntil('\n', serialBuffer, TOTAL_PACKET_LEN);
			if (serialBuffer[0] == 'h') {
				Serial.print("hhhhh");
				readyToStart = true;
				printPID();
			}
		}

	}
	
}

////////////////////////////////////////////////////////////////
// MAIN LOOP
////////////////////////////////////////////////////////////////

void loop()
{
	printJointState();
	
	if(testState == TEST_ALL){
		testEncoders();
		testMotors();
	}else if(testState == TEST_ENCODERS){
		testEncoders();
	}else if(testState == TEST_MOTORS){
		testMotors();
	}else if(testState == ROBOT_TUNNING){
		//not this
		printDebug("this is where we used to tune pid");
	}else if(testState == ROBOT_NORMAL){
		readSerial();
	}	

	if(testState == ROBOT_TUNNING || testState == ROBOT_NORMAL){
		// turn the magnets on/off
		if (USE_MAGNETS)
		{
			updateMagnets();
		}

		// Move joint motors
		static uint32_t lastUpdateTime = millis();
		uint32_t currTime = millis();
		if (currTime - lastUpdateTime >= UPDATE_INTERVAL)
		{
			if (currTime - lastUpdateTime > UPDATE_INTERVAL)
				printDebug("Missed update schedule");

			lastUpdateTime += UPDATE_INTERVAL;

			if (state == ST_HOLDING || ST_MOVING)
			{
				UpdateMotors();
			}
		}
	}
	
}

/**
 * Tests inchworm taking a step multiple times (back and forth) 
 */
void test1Robot(int numberSteps){

	for(int i = 0; i < numberSteps; i++){ // Number of total step
		for(int p = 0; p < NUM_ANGLES_STEP_1; p = p + NUM_MOTORS){ // Every 5 angles
			for(int j = 0; j < NUM_MOTORS; j++){
				jointMotor[j].SetTarget(step1[p+j]);
			}			
		}
	}
}


void test() {
	Serial.println("motor 1");
	analogWrite(JOINT_MOTOR1_FWD, 255);
	analogWrite(JOINT_MOTOR1_REV, 0);
	delay(1000);
	analogWrite(JOINT_MOTOR1_FWD, 0);
	analogWrite(JOINT_MOTOR1_REV, 0);
	Serial.println("motor 2");
	analogWrite(JOINT_MOTOR2_FWD, 255);
	analogWrite(JOINT_MOTOR2_REV, 0);
	delay(1000);
	analogWrite(JOINT_MOTOR2_FWD, 0);
	analogWrite(JOINT_MOTOR2_REV, 0);
	Serial.println("motor 3");
	analogWrite(JOINT_MOTOR3_FWD, 255);
	analogWrite(JOINT_MOTOR3_REV, 0);
	delay(1000);
	analogWrite(JOINT_MOTOR3_FWD, 0);
	analogWrite(JOINT_MOTOR3_REV, 0);
	Serial.println("motor 4");
	analogWrite(JOINT_MOTOR4_FWD, 255);
	analogWrite(JOINT_MOTOR4_REV, 0);
	delay(1000);
	analogWrite(JOINT_MOTOR4_FWD, 0);
	analogWrite(JOINT_MOTOR4_REV, 0);
	Serial.println("motor 5");
	analogWrite(JOINT_MOTOR5_FWD, 255);
	analogWrite(JOINT_MOTOR5_REV, 0);
	delay(1000);
	analogWrite(JOINT_MOTOR5_FWD, 0);
	analogWrite(JOINT_MOTOR5_REV, 0);
}


void testMagnets(){

	Serial.print("------ MAGNET TEST -----\n");
	Serial.print("magnetic\n");
	digitalWrite(MAGNET_1, HIGH);
	digitalWrite(MAGNET_2, HIGH);
	delay(1000);
	Serial.print("magnet 1 off\n");
	digitalWrite(MAGNET_1, LOW);
	digitalWrite(MAGNET_2, HIGH);
	delay(1000);
	Serial.print("magnetic\n");
	digitalWrite(MAGNET_1, HIGH);
	digitalWrite(MAGNET_2, HIGH);
	delay(1000);
	Serial.print("magnet 2 off\n");
	digitalWrite(MAGNET_1, HIGH);
	digitalWrite(MAGNET_2, LOW);
	delay(1000);
	Serial.print("magnetic\n");
	digitalWrite(MAGNET_1, HIGH);
	digitalWrite(MAGNET_2, HIGH);
	Serial.println("\nFinishing Test ...\n");
	delay(1000);
	//testState = ROBOT_TUNNING;
}
////////////////////////////////////////////////////////////////
// FUNCTIONS
////////////////////////////////////////////////////////////////

void printDebug(String theString){
	int counter = 0;
	char temp[100];
	while (theString[counter] != '\0' && counter < 100) 
	{
		temp[counter] = theString[counter];
		counter++;
	}
	temp[counter] = '\n';
	DebugPacket_t debug;
	debug.message.type = 'd';
	memcpy(debug.message.string, temp, sizeof(temp));
	Serial.write(debug.BytePacket, sizeof(debug.BytePacket));
}

void printFault(String theString){
	int counter = 0;
	char temp[100];
	while (theString[counter] != '\0' && counter < 100) 
	{
		temp[counter] = theString[counter];
		counter++;
	}
	temp[counter] = '\n';
	DebugPacket_t debug;
	debug.message.type = 'f';
	memcpy(debug.message.string, temp, sizeof(temp));
	Serial.write(debug.BytePacket, sizeof(debug.BytePacket));
}

void readHeartbeat(heartPacket_t heartBeat){
	numHb = heartBeat.hb.counter;
}

void printHeartbeat(){
	heartPacket_t hb;
	hb.hb.type = 'h';
	hb.hb.counter = numHb;
	Serial.write(hb.BytePacket, sizeof(hb.BytePacket));
}


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
			if (abs(jointMotor[i].getAngleDegrees()) > jointMotor[i].getLimit()) {
				String fault = "motor ";
				fault += i;
				fault += " at ";
				fault += jointMotor[i].getAngleDegrees();
				fault += " past the limit ";
				fault += jointMotor[i].getLimit();
				printFault(fault);
			} else {
				jointMotor[i].SendPWM(speeds[i]);
			}

			
		}
	}
	else
	{
		for (int i = 0; i < NUM_MOTORS; i++)
		{
			if (abs(jointMotor[i].getAngleDegrees()) > jointMotor[i].getLimit()) {
				String fault = "motor ";
				fault += i;
				fault += " at ";
				fault += jointMotor[i].getAngleDegrees();
				fault += " past the limit ";
				fault += jointMotor[i].getLimit();
			} else {
				jointMotor[i].SendPWM(speeds[i]);
			}
		}
	}
	//Serial.printf("E %3.2f %3.2f %3.2f %3.2f %3.2f\n", speeds[0], speeds[1], speeds[2], speeds[3],  speeds[4]);
}



/**
 * @brief read control message from ROS
 * 
 */
void readSerial() {
	
	// make a switch statement
	Serial.readBytesUntil('\n', serialBuffer, 249);
	printDebug("read a line");
	//byteMessage = serialBuffer;//from serial
	switch(serialBuffer[0])
	{
		case 'h':
			heartPacket_t hb;
			unsigned char tempHB[16];
			for (int i = 0; i < 16; i ++){
				tempHB[i] = serialBuffer[i];
			}
			memcpy(hb.BytePacket, tempHB, sizeof(hb.BytePacket));
			readHeartbeat(hb);
			//do the joint reading 
			printHeartbeat();
			printDebug("here");
			break;
		case 'g':
			poseGoalPacket_t pose;
			unsigned char tempPose[48];
			for (int i = 0; i < 48; i ++){
				tempPose[i] = serialBuffer[i];
			}
			memcpy(pose.BytePacket, tempPose, sizeof(pose.BytePacket));
			readJoints(pose);
			//do the joint reading 
			printTarget();
			break;
		case 'm':
			magnetPacket_t mag;
			unsigned char temp[16];
			for (int i = 0; i < 16; i ++)
			{
				temp[i] = serialBuffer[i];
			}
			memcpy(mag.BytePacket, temp, sizeof(mag.BytePacket));
			readMagnets(mag);
			printMagnets();
			break;
		case 'p':
			PID_Packet pid;
			unsigned char tempPid[248];
			for (int i = 0; i < 248; i ++)
			{
				tempPid[i] = serialBuffer[i];
			}
			memcpy(pid.BytePacket, tempPid, sizeof(pid.BytePacket));
			readPID(pid);
			printPID();
			break;
		default:
			//print fault
			break;
	}
}

void readJoints(poseGoalPacket_t message) {
	jointMotor[0].SetTarget(message.message.j0);
	jointMotor[1].SetTarget(message.message.j1);
	jointMotor[2].SetTarget(message.message.j2);
	jointMotor[3].SetTarget(message.message.j3);
	jointMotor[4].SetTarget(message.message.j4);
}

void readMagnets(magnetPacket_t message){
	setMagnetState(message.message.magnet1, message.message.magnet2);
}

void printMagnets(){
	magnetPacket_t mag;
	mag.message.type = 'm';
	
	if (magState == magnetsOn)
	{
		mag.message.magnet1 = 1;
		mag.message.magnet2 = 1;
	} else if (magState == magnet1Off) {
		mag.message.magnet1 = 0;
		mag.message.magnet2 = 1;
	} else if (magState == magnet2Off) {
		mag.message.magnet1 = 1;
		mag.message.magnet2 = 0;
	} 
	Serial.write(mag.BytePacket, sizeof(mag.BytePacket));
}

void readPID(PID_Packet message){
	jointMotor[0].set_PID(message.message.j0F, message.message.j0B);
	jointMotor[1].set_PID(message.message.j1F, message.message.j1B);
	jointMotor[2].set_PID(message.message.j2F, message.message.j2B);
	jointMotor[3].set_PID(message.message.j3F, message.message.j3B);
	jointMotor[4].set_PID(message.message.j4F, message.message.j4B);
}

void printPID(){
	PID_Packet pid;
	pid.message.type = 'p';
	double arr[3];
	arr[0] = 0.0;
	arr[1] = 0.0;
	arr[2] = 0.0;
	
	jointMotor[0].getPIDF(arr);
	memcpy(pid.message.j0F, arr, sizeof(pid.message.j0F));
	jointMotor[0].getPIDB(arr);
	memcpy(pid.message.j0B, arr, sizeof(pid.message.j0B));
	jointMotor[1].getPIDF(arr);
	memcpy(pid.message.j1F, arr, sizeof(pid.message.j1F));
	jointMotor[1].getPIDB(arr);
	memcpy(pid.message.j1B, arr, sizeof(pid.message.j1B));
	jointMotor[2].getPIDF(arr);
	memcpy(pid.message.j2F, arr, sizeof(pid.message.j2F));
	jointMotor[2].getPIDB(arr);
	memcpy(pid.message.j2B, arr, sizeof(pid.message.j2B));
	jointMotor[3].getPIDF(arr);
	memcpy(pid.message.j3F, arr, sizeof(pid.message.j3F));
	jointMotor[3].getPIDB(arr);
	memcpy(pid.message.j3B, arr, sizeof(pid.message.j3B));
	jointMotor[4].getPIDF(arr);
	memcpy(pid.message.j4F, arr, sizeof(pid.message.j4F));
	jointMotor[4].getPIDB(arr);
	memcpy(pid.message.j4B, arr, sizeof(pid.message.j4B));

	Serial.write(pid.BytePacket, sizeof(pid.BytePacket));
}

void printJointState(){
	posePacket_t pose;
	pose.message.type = 'j';

	pose.message.j0 = jointMotor[0].getAngleDegrees();
	pose.message.j1 = jointMotor[1].getAngleDegrees();
	pose.message.j2 = jointMotor[2].getAngleDegrees();
	pose.message.j3 = jointMotor[3].getAngleDegrees();
	pose.message.j4 = jointMotor[4].getAngleDegrees();

	Serial.write(pose.BytePacket, sizeof(pose.BytePacket));
}

/**
 * @brief print where the joints are going
 * 
 */
void printTarget() {
	poseGoalPacket_t pose;
	pose.message.type = 'g';
	
	pose.message.j0 = jointMotor[0].GetTarget();
	pose.message.j1 = jointMotor[1].GetTarget();
	pose.message.j2 = jointMotor[2].GetTarget();
	pose.message.j3 = jointMotor[3].GetTarget();
	pose.message.j4 = jointMotor[4].GetTarget();
	Serial.write(pose.BytePacket, sizeof(pose.BytePacket));
}

/**
 * Helper function to use for the debug button
 *
 * return: if button is pressed or not
 **/

bool buttonPressed(){
	bool pressed = false;

	if(analogRead(DEBUG_PIN) < 50)
	{
		pressed = true;
	}
	else
	{
		pressed = false;
	}
	return pressed;
}

void testJointMotor() {
	char motor[1];
    char direction[1];
    char speed[3];
	char duration[4];


    Serial.print("\nJoint Motor Number: ");
    while (Serial.available() == 0){
        continue;
    }
    Serial.readBytesUntil('\n', motor, TOTAL_PACKET_LEN); //put input into buffer

	Serial.print("\nDirection (0 or 1) (0 = -; 1 = +): ");
    while (Serial.available() == 0){
        continue;
    }
    Serial.readBytesUntil('\n', direction, TOTAL_PACKET_LEN); //put input into buffer

	Serial.print("\nSpeed (0 - 255)): ");
    while (Serial.available() == 0){
        continue;
    }
    Serial.readBytesUntil('\n', speed, TOTAL_PACKET_LEN); //put input into buffer

	Serial.print("\nDuration (0ms - 9999ms)): ");
    while (Serial.available() == 0){
        continue;
    }
    Serial.readBytesUntil('\n', duration, TOTAL_PACKET_LEN); //put input into buffer

	if (direction[0] == '0'){ jointMotor[atoi(motor)].SendPWM((-1*atoi(speed))); }
	else { jointMotor[atoi(motor)].SendPWM(atoi(speed)); }
	Serial.println("Moving motor");
	delay(atoi(duration));
	jointMotor[atoi(motor)].SendPWM(0);
}

/** 
 * Sets the magnet state off the enum
 * 
 **/
void setMagnetState(int mag1, int mag2) {
	
    if (mag1 == 1 && mag2 == 1) {
		magState = magnetsOn;
	}
	else if (mag1 == 0 && mag2 == 1) {
		magState = magnet1Off;
	}
	else if (mag1 == 1 && mag2 == 0) {
		magState = magnet2Off;
	}
    
}

/**
 * @brief Does the magnet signal
 * 
 */
void updateMagnets() {
	switch(magState) {
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

////////////////////////////////////////////////////////////////
// TEST FUNCTIONS
////////////////////////////////////////////////////////////////
void testEncoders(void){

	Serial.print("------ ENCODER TEST -----\n");
	
	// Scanning
	Serial.print("Scanning...\n");
	Serial.print("Scanned addresses: ");

	int error = 0;
	for(int add = 0; add < 127; add++){
		Wire.beginTransmission(add);
		error = Wire.endTransmission();

		if(error == 0){
			
			Serial.print(add, HEX);
			Serial.print(",");
		} else if(error == 4){
			Serial.print(add, HEX);
			Serial.print(" (Communicating with error 4),");
		}
		Serial.print("\n");
	}

	// Encoder values set in program 
	Serial.print("  The following addresses are set for each encoder: \n");

	AMS_AS5048B encoder;

	unsigned char addressE[NUM_MOTORS] = {JOINT_MOTOR1_ADR, JOINT_MOTOR2_ADR, JOINT_MOTOR3_ADR, JOINT_MOTOR4_ADR, JOINT_MOTOR5_ADR};
	for(int i = 0; i < NUM_MOTORS; i++){
		Serial.print(addressE[i], HEX);
		Serial.print(" | ");
	} 
	Serial.print("\n");
	
	// Communicate with each enoder
	Serial.print("Starting Individual Test ...");
	for(int i = 0; i < NUM_MOTORS; i++){
		Serial.printf("\n   Testing Encoder: %d Address: ", i);
		Serial.print(addressE[i], HEX);
		Serial.print(" | ");
		encoder = AMS_AS5048B(addressE[i]);
		encoder.begin();
		encoder.angleRegR();
	}
	Serial.println("\nFinishing Test ...\n");
	delay(4000);
}

void testMotors(void){
		Serial.print("------ MOTOR TEST -----\n");
	Serial.print("  The following pins are set for each motor (forward, reverse): \n");

	JointMotor2 motor;
	int totalPins = NUM_MOTORS*2;

	const int pinM[totalPins] = {JOINT_MOTOR1_FWD, JOINT_MOTOR1_REV, JOINT_MOTOR2_FWD, JOINT_MOTOR2_REV, JOINT_MOTOR3_FWD, JOINT_MOTOR3_REV, JOINT_MOTOR4_FWD, JOINT_MOTOR4_REV, JOINT_MOTOR5_FWD, JOINT_MOTOR5_REV};
	for(int i = 0; i < totalPins*2; i = i+2){
		Serial.printf("  (%d, %d) | ", pinM[i], pinM[i+1]);
	} 
	
	Serial.print("\nStating Test ...");
	for(int i = 0; i < totalPins; i = i + 2){
		Serial.printf("\n   Testing Motor: %d Pins: (%d, %d) ", i, pinM[i], pinM[i+1]);
		motor = JointMotor2(pinM[i], pinM[i+1]);
		Serial.printf("\n  forward");
		motor.SendPWM(10);
		delay(2000);
		Serial.printf("\n  stop");
		motor.SendPWM(0);
		delay(2000);
		Serial.printf("\n  backward");
		motor.SendPWM(-10);
		delay(2000);
		Serial.printf("\n  stop");
		motor.SendPWM(0);
		delay(2000);
	}
	Serial.println("\nFinishing Test ...\n");
	delay(4000);
	//testState = ROBOT_TUNNING;
}