#include "jointMotor2.h"
#include "config.h"


JointMotor2::JointMotor2(int pwmF, int pwmR)
{
	//Pin Configuration
	pwmForward = pwmF;
	pwmReverse = pwmR;
	pinMode(pwmForward, OUTPUT);
	pinMode(pwmReverse, OUTPUT);
}

JointMotor2::JointMotor2(int pwmF, int pwmR, int pinE,
						 uint8_t encoderAddress, double kp_a_link_fixed, double ki_a_link_fixed, double kd_a_link_fixed,
						 double kp_d_link_fixed, double ki_d_link_fixed, double kd_d_link_fixed,
						 double ang_offset, bool encoder_clockwise, uint8_t id_input)
{
	//Pin Configuration
	pwmForward = pwmF;
	pwmReverse = pwmR;
	pinEnable = pinE;
	pinMode(pwmForward, OUTPUT);
	pinMode(pwmReverse, OUTPUT);
	pinMode(pinEnable, OUTPUT);
	digitalWrite(pinEnable, HIGH);
	//Encoder Setup
	encoder = AMS_AS5048B(encoderAddress);
	encoder.begin();	  //Encoder Constructor
	encoder.setZeroReg(); //Zero Encoders

	//PID
	kP = kP1 = kp_a_link_fixed;
	kI = kI1 = ki_a_link_fixed;
	kD = kD1 = kd_a_link_fixed;

	kP2 = kp_d_link_fixed;
	kI2 = ki_d_link_fixed;
	kD2 = kd_d_link_fixed;

	angle_offset = ang_offset;
	enc_clockwise = encoder_clockwise;
	encoder.setClockWise(enc_clockwise);

	id = id_input;
}

const int maxDutyCycle = 230;
void JointMotor2::SendPWM(int speed)
{
	if (speed < 0)
	{
		if (speed < -maxDutyCycle)
		{
			speed = -maxDutyCycle;
		}
		analogWrite(pwmReverse, 0);
		analogWrite(pwmForward, -speed);
	}
	else
	{
		if (speed > maxDutyCycle)
		{
			speed = maxDutyCycle;
		}
		analogWrite(pwmForward, 0);
		analogWrite(pwmReverse, speed);
	}

	return;
}

/*
* Gets encoder angle in degrees
*/
double JointMotor2::getAngleDegrees()
{
	double angle = encoder.angleR(U_DEG, true);
	// Serial.println("Ang:");
	// Serial.print(angle);
	double calibrated_angle = angle + angle_offset;
	// Serial.println("CA: ");
	// Serial.print(calibrated_angle);
	// Serial.print("ER:")
	// Serial.print(angle-targetAngle);

	// if (calibrated_angle > 360)
	// {
	// 	calibrated_angle -= 360;
	// }
	// if (calibrated_angle < 0)
	// {
	// 	calibrated_angle += 360;
	// }

	if (calibrated_angle > 180)
	{
		calibrated_angle -= 360;
	}

	double delta = calibrated_angle - last_calibrated_angle;
	if (delta > 180)
		delta -= 360;
	if (delta < -180)
		delta += 360;

	if (fabs(delta) > ANGLE_ERROR_THRESHOLD)
	{
		Serial.println("Angle error!");
	}

	last_calibrated_angle = calibrated_angle;
	// double currentTime = millis();
	// if (currentTime - lastDebugUpdate >= 3000)
	// {
	// 	Serial.print("\nID: ");
	// 	Serial.print(id);
	// 	Serial.print("\tCALIBRATED ANGLE: ");
	// 	Serial.print(calibrated_angle);
	// 	Serial.print("\t ANGLE: ");
	// 	Serial.print(angle);
	// 	// Serial.print("\t TAngle: ");
	// 	// Serial.print(targetAngle);
	// 	// Serial.print("\t Error:");
	// 	// Serial.println(targetAngle-calibrated_angle);
	//
	// 	lastDebugUpdate = currentTime;
	// }
	return calibrated_angle;
}

/*
* Set desired joint angle
*/
void JointMotor2::SetTarget(double angle)
{
	targetAngle = angle;
	return;
}

/*
* Get desired joint angle
*/
double JointMotor2::GetTarget(void)
{
	return targetAngle;
}

/*
* TODO: Add moving average to see if the max change is greater than the one wanted 
  if it is then stop the motor from moving to that angle. There is two places where 
  this could work:1) in this calcEffor() function when I get the getAngleDegrees() 
  function to see what the change is 2) In the function getAngleDegrees() above to 
  get the actual angle and smooth out the noise.
TODO: Add movement restiriction on robot (variables for the min and max
  values that a joint can move to). This can be done with a simple implementation 
  of a if the angle reading is greater than the MAX then stop and print ANGLE OUT
  OF POSSIBLE PHYSICAL ROBOTS RANGE.
  */
int JointMotor2::CalcEffort(void)
{
	double currentAngle = getAngleDegrees();

	double error = targetAngle - currentAngle;

	if (error > 180)
		error -= 360;
	if (error < -180)
		error += 360;

	sumError += error;

	//Cap sum of the error
	if (sumError > SUM_THRESHOLD)
	{
		sumError = SUM_THRESHOLD;
	}
	if (sumError < -SUM_THRESHOLD)
	{
		sumError = -SUM_THRESHOLD;
	}

	double deltaError = error - lastError;

	double effort = (kP * error) + (kI * sumError) + (kD * deltaError);

	lastError = error;

	// //TODO:
	double currentTime = millis();
	if (currentTime - lastDebugUpdate >= 1000)
	{
		/*
		Serial.print("\nID: ");
		Serial.print(id);
		// Serial.print(" CA: ");
		// Serial.print(currentAngle);
		// Serial.print(" TA: ");
		// Serial.print(targetAngle);
		Serial.print(" ER:");
		Serial.print(error);
		Serial.print(" Effort:");
		Serial.print(effort);
		*/
		lastDebugUpdate = currentTime;
	}

	return effort;
}

/*
* Switch PID values for which joint is fixed
* Returns false if gripper 1 is engaged and true if gripper 2
*/
bool JointMotor2::SwitchPID(void)
{
	// Serial.println("Switching the PID values");
	angle_offset += targetAngle - getAngleDegrees();

	if (fixed_link == a_link_engaged) // TODO switch to a_link_engaged
	{
		kP = kP1;
		kI = kI1;
		kD = kD1;
		//Serial.printf("JOINT %d Switching to PID 1: %f, %f, %f\n", id, kP, kI, kD);
		return false;
	}
	else if (fixed_link == d_link_engaged) // TODO switch to d_link_engaged
	{
		kP = kP2;
		kI = kI2;
		kD = kD2;
		//Serial.printf("JOINT %d Switching to PID 2: %f, %f, %f\n", id, kP, kI, kD);
		return true;
	}
	else
	{
		return;
	}
}


bool JointMotor2::SwitchPID(uint8_t gripperEngagedSelect)
{
	// Serial.println("Switching the PID values");
	angle_offset += targetAngle - getAngleDegrees();

	if (fixed_link == d_link_engaged && gripperEngagedSelect == a_link_engaged) // TODO switch to a_link_engaged
	{
		kP = kP1;
		kI = kI1;
		kD = kD1;
		//Serial.println("Switching to PID 1");
		fixed_link = a_link_engaged;
		return false;
	}
	else if (fixed_link == a_link_engaged && gripperEngagedSelect == d_link_engaged) // TODO switch to d_link_engaged
	{
		kP = kP2;
		kI = kI2;
		kD = kD2;
		//Serial.println("Switching to PID 2");
		fixed_link = d_link_engaged;
		return true;
	}
	else
	{
		return;
	}
}

void JointMotor2::printPID()
{

	Serial.print("\nJoint ");Serial.print(id);Serial.print(": ");
	Serial.print("    ");Serial.print("P1: ");Serial.print(kP);
	Serial.print("    ");Serial.print("I1: ");Serial.print(kI);
	Serial.print("    ");Serial.print("D1: ");Serial.print(kD);

	// Serial.println("    ");Serial.print("P2: ");Serial.print(kP2);
	// Serial.println("    ");Serial.print("I2: ");Serial.print(kI2);
	// Serial.println("    ");Serial.print("D2: ");Serial.print(kD2);
}

////**********************GETTTERS**************************////

//Pin Configuration
int JointMotor2::get_pwmForwardPin() {
	return pwmForward;
}
int JointMotor2::get_pwmReversePin() {
	return pwmReverse;
}
int JointMotor2::get_pinEnable() {
	return pinEnable;
}
int JointMotor2::get_encoderAddress() {
	return encoder.get_chipAddress();
}
double JointMotor2::get_kP() {
	return kP;
}
double JointMotor2::get_kI() {
	return kI;
}
double JointMotor2::get_kD() {
	return kD;
}
double JointMotor2::get_kP1() {
	return kP1;
}
double JointMotor2::get_kI1() {
	return kI1;
}
double JointMotor2::get_kD1() {
	return kD1;
}
double JointMotor2::get_kP2() {
	return kP2;
}
double JointMotor2::get_kI2() {
	return kI2;
}
double JointMotor2::get_kD2() {
	return kD2;
}
double JointMotor2::get_angle_offset() {
	return angle_offset;
}
uint8_t JointMotor2::get_id() {
	return id;
}

void JointMotor2::getPIDF(double (&arr)[3]){
	arr[0] = get_kP1();
	arr[1] = get_kI1();
	arr[2] = get_kD1();
}
void JointMotor2::getPIDB(double (&arr)[3]){
	arr[0] = get_kP2();
	arr[1] = get_kI2();
	arr[2] = get_kD2();
}

/***********setters*************/
void JointMotor2::set_PID(float kF[], float kB[]) {
	
	kP1 = kF[0];
	kI1 = kF[1];
	kD1 = kF[2];
	kP2 = kB[0];
	kI2 = kB[1];
	kD2 = kB[2];

	// switching pid back and forth should correctly set the pid values
	SwitchPID();
	SwitchPID();
	
}