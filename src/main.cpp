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
uint32_t lastViaUpdate = 0;
uint32_t startMoveTime = 0;

////////////////////////////////////////////////////////////////
// SERIAL BUFFER
////////////////////////////////////////////////////////////////
const int MOTOR_PKT_LEN = 8;   // motor packet example: "-123.32_" (ending in space)
const int CONTROL_PKT_LEN = 3; // gripper and allen key control packet example: "0 1"
const int TOTAL_PACKET_LEN = MOTOR_PKT_LEN * NUM_MOTORS + CONTROL_PKT_LEN + 1;
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

////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES
////////////////////////////////////////////////////////////////
void UpdateMotors(void);
void SetNewVias(void);
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
void printSerial(void);
void printFakeSerial(void);

//magnet switching
void setMagnetState(char, char);
void updateMagnets(void);

////////////////////////////////////////////////////////////////
// SETUP METHOD
////////////////////////////////////////////////////////////////
void setup()
{
	Wire.begin();		  	// Begin I2C
	Serial.begin(115200); 	
	// Serial.setTimeout(0);

	Serial.println("Robot intializing....");

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
			//jointMotor[0] = JointMotor2(JOINT_MOTOR1_FWD, JOINT_MOTOR1_REV, JOINT_MOTOR1_EN, // A-LINK WRIST
			//							JOINT_MOTOR1_ADR, 0, 0, 0, 0, 0, 0, 0.0, false, 1);
			//jointMotor[1] = JointMotor2(JOINT_MOTOR2_FWD, JOINT_MOTOR2_REV, JOINT_MOTOR2_EN, // AB-LINK JOINT
			//							JOINT_MOTOR2_ADR, 22, .5, 0, 8, .3, 0, 27.81, false, 2);
			//jointMotor[2] = JointMotor2(JOINT_MOTOR3_FWD, JOINT_MOTOR3_REV, JOINT_MOTOR3_EN, // BC-LINK JOINT
			//							JOINT_MOTOR3_ADR, 22, 0.5, 0, 23, 0.4, 0, 124.38, true, 3);
			jointMotor[0] = JointMotor2(JOINT_MOTOR4_FWD, JOINT_MOTOR4_REV, JOINT_MOTOR4_EN, // CD-LINK JOINT
										JOINT_MOTOR4_ADR, 8, .3, 0, 23, .4, 0, 27.8, true, 4);
			//jointMotor[4] = JointMotor2(JOINT_MOTOR5_FWD, JOINT_MOTOR5_REV, JOINT_MOTOR5_EN, // D-LINK WRIST
			//							JOINT_MOTOR5_ADR, 0, 0, 0, 0, 0, 0, 0.0, false, 5);
			// jointMotor[0] = JointMotor2(JOINT_MOTOR1_FWD, JOINT_MOTOR1_REV, JOINT_MOTOR1_EN, // A-LINK WRIST
			// 							JOINT_MOTOR1_ADR, 0, 0, 0, 0, 0, 0, 0.0, false, 1);
			// jointMotor[1] = JointMotor2(JOINT_MOTOR2_FWD, JOINT_MOTOR2_REV, JOINT_MOTOR2_EN, // AB-LINK JOINT
			// 							JOINT_MOTOR2_ADR, 0, 0, 0, 0, 0, 0, 27.81, false, 2);
			// jointMotor[2] = JointMotor2(JOINT_MOTOR3_FWD, JOINT_MOTOR3_REV, JOINT_MOTOR3_EN, // BC-LINK JOINT
			// 							JOINT_MOTOR3_ADR, 0, 0, 0, 0, 0, 0, 124.38, true, 3);
			// jointMotor[3] = JointMotor2(JOINT_MOTOR4_FWD, JOINT_MOTOR4_REV, JOINT_MOTOR4_EN, // CD-LINK JOINT
			// 							JOINT_MOTOR4_ADR, 0, 0, 0, 0, 0, 0, 27.8, true, 4);
			// jointMotor[0] = JointMotor2(JOINT_MOTOR5_FWD, JOINT_MOTOR5_REV, JOINT_MOTOR5_EN, // D-LINK WRIST
			//							JOINT_MOTOR5_ADR, 2, .3, 0, 0, 0, 0, 0.0, false, 5);

			jointMotor[0].SetTarget(0.0);
			//jointMotor[1].SetTarget(27.81);
			//jointMotor[2].SetTarget(124.38);
			//jointMotor[3].SetTarget(27.81);
			//jointMotor[4].SetTarget(0.0);
	
		}

		inputBuffer.reserve(24);

		/*
		* Initialize Magnets
		*/
		if (USE_MAGNETS)
		{
			pinMode(MAGNET_1, OUTPUT);
			pinMode(MAGNET_2, OUTPUT);
		}


		if(USE_DEBUG_BUTTON)
		{
			pinMode(DEBUG_PIN, INPUT);
		}

		Serial.println("Done");
		previous_time = millis();
	}
}

////////////////////////////////////////////////////////////////
// MAIN LOOP
////////////////////////////////////////////////////////////////

void loop()
{
    //readSerial();
    //printSerial();
    //printFakeSerial();;
	Serial.println("hi");
	Serial.println(jointMotor[0].getAngleDegrees());
	/*
	if(testState == TEST_ALL){
		testEncoders();
		testMotors();
	}else if(testState == TEST_ENCODERS){
		testEncoders();
	}else if(testState == TEST_MOTORS){
		testMotors();
	}else if(testState == ROBOT_TUNNING){
		RunPidTuningDebug();
	}else if(testState == ROBOT_NORMAL){
		ReadAngleInputs();
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
	*/
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

void ReadAngleString()
{

	if (Serial.available() > 0)
	{
		angleInputs = Serial.readString();

		int jointIndex = 0;
		int index;
		for(index = 0; index < (MOTOR_PKT_LEN*NUM_MOTORS); index+=MOTOR_PKT_LEN){
			float angle = angleInputs.substring(index, index+MOTOR_PKT_LEN).toFloat();
			//jointMotor[jointIndex].SetTarget(angle);
			Serial.printf("%d angle: %.2f\n", index+1, angle);
			jointIndex++;
		}
		//allenKey_A = angleInputs..

	}
}

/**
 * Main method that reads control message form high level code
 *
 * This method parses and organizes the serial control message
 * to angle for each joint, gripper configuration, and allen
 * key actuation.
 */
boolean toggleBuffer = true;
void ReadAngleInputs()
{
	if (Serial.available() > 0)
	{
		new_command=true;
		double current_time = millis();
		Serial.println("Message received");
		previous_time = current_time;
		Serial.readBytesUntil('\n', serialBuffer, TOTAL_PACKET_LEN);
		int tempIndex = 0;
		int jointIndex = 0;
		float tempAngle = 0;
		boolean motorPktCompleted = true;

		if(toggleBuffer){
			for(int i = 0; i < TOTAL_PACKET_LEN; i++){
				tempSerialBuffer[i] = serialBuffer[i];
			}
		}
		for(int i = 0; i < TOTAL_PACKET_LEN; i++){
			serialBuffer[i] = tempSerialBuffer[i];
		}

		if (serialBuffer[0] == '-' || serialBuffer[0] == '0')
		{
			for (int i = 0; i < TOTAL_PACKET_LEN; i++)
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
					if (jointIndex >= MOTOR_COUNT)
					{   
						//Gripper							
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
			for (int i = 0; i < TOTAL_PACKET_LEN; i++)
			{
				Serial.read();
			}
		}

		Serial.println(serialBuffer);
		Serial.println(tempSerialBuffer);
		toggleBuffer = !toggleBuffer;
	}
}

/**
 * @brief read control message from ROS
 * 
 */
void readSerial() {
	if (Serial.available() > 0)
	{
		new_command=true;
		double current_time = millis();
		previous_time = current_time;
		Serial.readBytesUntil('\n', serialBuffer, TOTAL_PACKET_LEN);


		//String outString = "Message received: ";
		//outString.concat(serialBuffer);
		//Serial.println(outString);
		int tempIndex = 0;
		int jointIndex = 0;

		if (serialBuffer[0] == '-' || serialBuffer[0] == ' ')
		{
			//read serial
			//split it up into the separate pieces
			tempIndex = 0;

			//cycle through joints and decode angles
			for (jointIndex = 0; jointIndex < NUM_MOTORS; jointIndex++)
			{
				tempIndex = jointIndex * MOTOR_PKT_LEN; //offsets start of motor packet
				char angleChars[MOTOR_PKT_LEN];
				for (int i = 0; i < MOTOR_PKT_LEN; i++)
				{
					angleChars[i] = serialBuffer[tempIndex + i];
				}
				std::string angleString = std::string(angleChars);
				float tempAngle = atof(angleChars);
				
				//check that it correctly decodes things
				//Serial.print("calc joint ");
				//Serial.print(jointIndex);
				//Serial.print(": ");
				//Serial.print(tempAngle);
				//Serial.print("\n");

				// send robot off
				jointMotor[jointIndex].SetTarget(tempAngle);
			}
			Serial.print(tempIndex+1);
			tempIndex = NUM_MOTORS * MOTOR_PKT_LEN + 1;
			Serial.print(tempIndex);
			//Serial.println(tempIndex);
			Serial.println(serialBuffer[TOTAL_PACKET_LEN - 4]);
			Serial.println((serialBuffer[TOTAL_PACKET_LEN - 2]));
			
			// get magnet state
			setMagnetState((serialBuffer[TOTAL_PACKET_LEN - 4]), (serialBuffer[TOTAL_PACKET_LEN - 42]));
			//Serial.println("got here");
			printFakeSerial();
		}
	    else
		{
			Serial.println("Invalid Input"); //didn't get the serial message
		}
	}
}

/**
 * @brief print where the joints currently are
 * 
 */
void printSerial() {
	String outputString = "";
	double tempAngle;
	char tempString[20];

	//turn the angles into strings
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		tempAngle = jointMotor[i].getAngleDegrees();
		
		dtostrf(tempAngle,5,2,tempString);
		
		if (tempAngle >= 0)
		{
			outputString.concat(' ');
		}
		else
		{
			outputString.concat('-');
		}
		tempAngle = abs(tempAngle);
		if (tempAngle < 100)
			tempString[0] = '0';
		if (tempAngle < 10)
			tempString[1] = '0';
		
		outputString.concat(tempString);
		outputString.concat(" ");
	}
	//add magnet state to string
	switch(magState)
	{
		case magnetsOn:
			outputString.concat("0 0");
			break;
		case magnet1Off:
			outputString.concat("1 0");
			break;
		case magnet2Off:
			outputString.concat("0 1");
			break;
	}
	//print string
	Serial.println(outputString);

}

/**
 * @brief print where the fake joints currently are
 * 
 */
void printFakeSerial() {
	//Serial.println("here");
	String outputString = "";
	double tempAngle;
	char tempString[20];

	//turn the angles into strings
	for (int i = 0; i < NUM_MOTORS; i++)
	{
		tempAngle = jointMotor[i].GetTarget();
		
		dtostrf(tempAngle,6,2,tempString);
		
		if (tempAngle >= 0)
		{
			outputString.concat(' ');
		}
		else
		{
			outputString.concat('-');
		}
		tempAngle = abs(tempAngle);
		if (tempAngle < 100)
			tempString[0] = '0';
		if (tempAngle < 10)
			tempString[1] = '0';
		
		outputString.concat(tempString);
		outputString.concat(" ");
	}
	//add magnet state to string
	switch(magState)
	{
		case magnetsOn:
			outputString.concat("0 0");
			break;
		case magnet1Off:
			outputString.concat("1 0");
			break;
		case magnet2Off:
			outputString.concat("0 1");
			break;
	}
	//print string
	Serial.println(outputString);

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
void setMagnetState(char mag1, char mag2) {
	Serial.println("made it");
	Serial.println(mag1);
	Serial.println(mag2);
	Serial.println(mag1 == '0');
	Serial.println(mag1 == '1');
	Serial.println(mag2 == '0');
	Serial.println(mag2 == '1');
	/*
    if (mag1 == '0' && mag2 == '0')
		magState = magnetsOn;
	else if (mag1 == '1' && mag2 == '0')
		magState = magnet1Off;
	else if (mag1 == '0' && mag2 == '1')
		magState = magnet2Off;
	else
		printf("Error - invalid magnet state\n");
    */
}

/**
 * @brief Does the magnet signal
 * 
 */
void updateMagnets() {
	switch(magState) {
		case magnetsOn:
			digitalWrite(MAGNET_1, LOW);
			digitalWrite(MAGNET_2, LOW);
			break;
		case magnet1Off:
			digitalWrite(MAGNET_1, HIGH);
			digitalWrite(MAGNET_2, LOW);
			break;
		case magnet2Off:
			digitalWrite(MAGNET_1, LOW);
			digitalWrite(MAGNET_2, HIGH);
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
		motor.SendPWM(10);
		delay(1000);
		motor.SendPWM(0);
		delay(1000);
	}
	Serial.println("\nFinishing Test ...\n");
	delay(4000);
}