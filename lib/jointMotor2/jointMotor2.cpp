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
						 uint8_t encoderAddress,
						 double kp_a_link_fixed, double ki_a_link_fixed, double kd_a_link_fixed, double kf_a_link_fixed,
						 double kp_d_link_fixed, double ki_d_link_fixed, double kd_d_link_fixed, double kf_d_link_fixed,
						 double ang_offset, double min_angle, double max_angle, bool encoder_clockwise, uint8_t id_input,
						 void (*printDebug)(const String&, const String&), void (*printFault)(const String&, const String&))
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
	encoder = AMS_AS5048B(encoderAddress, printDebug, printFault);
	encoder.begin();	  //Encoder Constructor
	encoder.setZeroReg(); //Zero Encoders

	//PID
	kP = kP1 = kp_a_link_fixed;
	kI = kI1 = ki_a_link_fixed;
	kD = kD1 = kd_a_link_fixed;
	kF = kF1 = kf_a_link_fixed;

	kP2 = kp_d_link_fixed;
	kI2 = ki_d_link_fixed;
	kD2 = kd_d_link_fixed;
	kF2 = kf_d_link_fixed;

	angle_offset = ang_offset;
	enc_clockwise = encoder_clockwise;
	encoder.setClockWise(enc_clockwise);

	id = id_input;

	//min and max safety
	minAngle = min_angle;
	maxAngle = max_angle;

	//set up arrays
	for (int i = 0; i < BUFFER_SIZE; i++) {
		vel[i] = 0;
		integral[i] = 0;
	}
	vel_pos = 0;
	int_pos = 0;

	this->printDebug = printDebug;
	this->printFault = printFault;
}

void JointMotor2::SendPWM(int speed)
{
	if (speed < 0)
	{
		if (speed < -MAX_DUTY_CYCLE)
		{
			speed = -MAX_DUTY_CYCLE;
		}
		analogWrite(pwmReverse, 0);
		analogWrite(pwmForward, -speed);
	}
	else
	{
		if (speed > MAX_DUTY_CYCLE)
		{
			speed = MAX_DUTY_CYCLE;
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
		this->printFault("Angle error!", "");
	}

	last_calibrated_angle = calibrated_angle;
	
	return calibrated_angle;
}

/*
* Set desired joint angle
*/
void JointMotor2::SetTarget(double angle)
{
	if (angle > maxAngle)
		targetAngle = maxAngle;
	else if (angle < minAngle)
		targetAngle = minAngle;
	else
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

	// if (error > 180)
	// 	error -= 360;
	// if (error < -180)
	// 	error += 360;

	double deltaError = error - lastError;
	
	//put the integral term in the array, replacing oldest value
	integral[int_pos] = error;
	int_pos++;

	if (int_pos == BUFFER_SIZE)
		int_pos = 0;

	double currentIntegralErr = 0;

	for (int i = 0; i < BUFFER_SIZE; i++)
		currentIntegralErr += integral[i];

	//Cap sum of the error
	if (currentIntegralErr > SUM_THRESHOLD)
	{
		currentIntegralErr = SUM_THRESHOLD;
	}
	if (currentIntegralErr < -SUM_THRESHOLD)
	{
		currentIntegralErr = -SUM_THRESHOLD;
	}

	double i_amt = kI * (currentIntegralErr / BUFFER_SIZE);

	double I_LIMIT = MAX_DUTY_CYCLE / 2;

	if(i_amt > I_LIMIT) {
		i_amt = I_LIMIT;
	} else if(i_amt < -I_LIMIT) {
		i_amt = -I_LIMIT;
	}

	double effort = (kP * error) + i_amt + (kD * deltaError) + kF;

	lastError = error;

	// //TODO:
	uint32_t currentTime = millis();
	if (currentTime - lastDebugUpdate >= 1000)
	{
		lastDebugUpdate = currentTime;
	}
	if (effort > MAX_DUTY_CYCLE) {
		effort = MAX_DUTY_CYCLE;
	} else if (effort < -MAX_DUTY_CYCLE) {
		effort = -MAX_DUTY_CYCLE;
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
		kF = kF1;
		//Serial.printf("JOINT %d Switching to PID 1: %f, %f, %f\n", id, kP, kI, kD);
		return false;
	}
	else if (fixed_link == d_link_engaged) // TODO switch to d_link_engaged
	{
		kP = kP2;
		kI = kI2;
		kD = kD2;
		kF = kF2;
		//Serial.printf("JOINT %d Switching to PID 2: %f, %f, %f\n", id, kP, kI, kD);
		return true;
	}
	else
	{
		return false;
	}
}


bool JointMotor2::SwitchPID(uint8_t gripperEngagedSelect)
{
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
		return false;
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

uint32_t JointMotor2::get_vel_startTime(){
	return vel_startTime;
}

void JointMotor2::set_vel_startTime(uint32_t startTime){
	vel_startTime = startTime;
}

double JointMotor2::get_vel_posStart(){
	return vel_posStart;
}

void JointMotor2::set_vel_posStart(double posStart){
	vel_posStart = posStart;
}

double JointMotor2::get_velocity(uint32_t mil){
	double velocity, sum = 0;
	velocity = (get_vel_posStart() - getAngleDegrees())/((get_vel_startTime() - mil)/1000);
	
	//put the velocity term in the array, replacing oldest value
	vel[vel_pos] = velocity;
	vel_pos++;

	if(vel_pos == BUFFER_SIZE)
		vel_pos = 0;

	//sum the velocites
	for (int i = 0; i < BUFFER_SIZE; i++) {
		sum += vel[i];
	}
	
	return (sum / BUFFER_SIZE);
}