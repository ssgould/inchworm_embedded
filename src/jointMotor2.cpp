#include <Arduino.h>
#include "JointMotor2.h"

JointMotor2::JointMotor2()
{
}

JointMotor2::JointMotor2(int pinDirectionA1, int pinDirectionB1, int pinPWM1, int encoderAddress, double kp, double ki, double kd, double ang_offset, bool encoder_clockwise, int id_input, double velocity_term_init)
{
	//Pin Configuration
	pinDirectionA = pinDirectionA1;
	pinDirectionB = pinDirectionB1;
	pinPWM = pinPWM1;
	pinMode(pinDirectionA, OUTPUT);
	pinMode(pinDirectionB, OUTPUT);
	pinMode(pinPWM, OUTPUT);
	digitalWrite(pinPWM, HIGH);

	//Encoder Setup
	encoder = AMS_AS5048B(encoderAddress);
	encoder.begin();	  //Encoder Constructor
	encoder.setZeroReg(); //Zero Encoders
	//PID
	kP = kp;
	kI = ki;
	kD = kd;

	kP2 = kp;
	kI2 = ki;
	kD2 = kd;

	angle_offset = ang_offset;
	enc_clockwise = encoder_clockwise;
	encoder.setClockWise(enc_clockwise);
	lastPubAng = 0;
	id = id_input;

	debug = false;

	sumError = 0;
	velocity_term = velocity_term_init;
	// myRA.clear();
}
JointMotor2::JointMotor2(int pinDirectionA1, int pinDirectionB1, int pinPWM1, int encoderAddress, double kp, double ki, double kd, double kp2, double ki2, double kd2, double ang_offset, bool encoder_clockwise, int id_input, double velocity_term_init)
{
	//Pin Configuration

	pinDirectionA = pinDirectionA1;
	pinDirectionB = pinDirectionB1;
	pinPWM = pinPWM1;
	pinMode(pinDirectionA, OUTPUT);
	pinMode(pinDirectionB, OUTPUT);
	pinMode(pinPWM, OUTPUT);
	digitalWrite(pinPWM, HIGH);

	//Encoder Setup
	encoder = AMS_AS5048B(encoderAddress);

	encoder.begin(); //Encoder Constructor

	encoder.setZeroReg(); //Zero Encoders

	//Variables to store PID values
	kP1 = kp;
	kI1 = ki;
	kD1 = kd;

	kP2 = kp2;
	kI2 = ki2;
	kD2 = kd2;

	//Variable for PID values that is currently using
	kP = kp;
	kI = ki;
	kD = kd;

	angle_offset = ang_offset;
	enc_clockwise = encoder_clockwise;
	encoder.setClockWise(enc_clockwise);
	lastPubAng = 0;
	id = id_input;

	debug = false;

	// myRA.clear();
	// error_idx = 0;

	sumError = 0;

	velocity_term = velocity_term_init;
}

/*
* Print variable every 2000 millis
*/
void JointMotor2::debugPrint(char vName[3], double vInput)
{
	double currentTime = millis();
	if (currentTime - lastPubAng2 > 200)
	{
		Serial.print("joint id:");
		Serial.print(id);
		Serial.print(" --> ");
		Serial.print(vName);
		Serial.print(": ");
		Serial.println(vInput);
		lastPubAng2 = currentTime;
	}
}

void JointMotor2::debugPrint2(char vName[3], double vInput)
{
	double currentTime = millis();
	if (currentTime - lastPubAng3 > 200)
	{
		Serial.print("joint id:");
		Serial.print(id);
		Serial.print(" --> ");
		Serial.print(vName);
		Serial.print(": ");
		Serial.println(vInput);
		lastPubAng3 = currentTime;
	}
}

/*
* Takes speed -255 - 255 and moves motor
*/
void JointMotor2::setSpeed(double speed)
{
	// double maxPercent = 0.9;
	// if (speed < -255 * maxPercent)
	// {
	// 	// speed = -255 * maxPercent;
	// 	digitalWrite(pinPWM, HIGH);
	// }
	// else if (speed > 255 * maxPercent)
	// {
	// 	// speed = 255 * maxPercent;
	// 	digitalWrite(pinPWM, HIGH);
	// }
	// else
	// {
	// 	analogWrite(pinPWM, abs(speed));
	// }
	// changeDirection(speed);
	// int maxSpeed = 255;		 //Maximum PWM output
	// double maxPercent = 0.9; //Max PWM output (percent of maxSpeed)
	// bool digWrite = false;   //If true if abs(Speed) >= speed pin will be set HIGH
	// int deadMinPer = 8;		 //min deadband in percentage of max speed
	// int deadMaxPer = 12;	 //max deadband in percentage of max speed

	// int deadMin = int((maxSpeed / 100) * deadMinPer); //min deadband in percentage of max speed
	// int deadMax = int((maxSpeed / 100) * deadMaxPer); //max deadband in percentage of max speed

	changeDirection(speed);
	// int absSpeed = abs();

	// analogWrite(pinPWM,abs(speed));
	
	// if (absSpeed > maxSpeed * maxPercent)
	// {
	// 	if (digWrite)
	// 	{
	// 		digitalWrite(pinPWM, HIGH);
	// 	}
	// 	else
	// 	{
	// 		absSpeed = maxSpeed * maxPercent;
	// 		analogWrite(pinPWM, absSpeed);
	// 	}
	// }
	// else
	// {
	// 	//Deadband
	// 	if (absSpeed < deadMin)
	// 	{
	// 		absSpeed = 0;
	// 	}
	// 	else if (absSpeed >= deadMin && absSpeed < deadMax)
	// 	{
	// 		absSpeed = deadMax;
	// 	}
	// 	analogWrite(pinPWM, absSpeed);
	// }
	return;
}
/*
* Changes motor direction pin states based on sign of speed
*/
void JointMotor2::changeDirection(double speed)
{
	//CONFIG 1
	// if (speed < 0)
	// {
	// 	// digitalWrite(pinDirectionA, HIGH);
	// 	// digitalWrite(pinDirectionB, LOW);

	// 	//NEW 2 PWM
	// 	analogWrite(pinDirectionA, 0);
	// 	// digitalWrite(pinDirectionB, LOW);
	// 	analogWrite(pinDirectionB, abs(speed));
		
	// }
	// else
	// {
	// 	// digitalWrite(pinDirectionA, LOW);
	// 	// digitalWrite(pinDirectionB, HIGH);

	// 	//NEW 2 PWM
	// 	analogWrite(pinDirectionB, 0);
	// 	analogWrite(pinDirectionA, abs(speed));		
	// }

	//CONFIG 2
	if (speed < 0)
	{
		//NEW 2 PWM
		analogWrite(pinDirectionB, 0);
		// digitalWrite(pinDirectionB, LOW);
		analogWrite(pinDirectionA, abs(speed));
		
	}
	else
	{
		// digitalWrite(pinDirectionA, LOW);
		// digitalWrite(pinDirectionB, HIGH);

		//NEW 2 PWM
		analogWrite(pinDirectionA, 0);
		analogWrite(pinDirectionB, abs(speed));		
	}
	return;
}
/*
* Gets encoder angle in degrees
*/
double JointMotor2::getAngleDegrees()
{
	double angle = encoder.angleR(U_DEG, true);
	double calibrated_angle = 0;

	// if (debug) { Serial.print("angle: "); Serial.println(angle); }

	if (angle >= 0 && angle <= 360)
	{ //don't return "I2C Error" as angle
		calibrated_angle = angle + angle_offset;
		if (calibrated_angle > 180)
		{
			calibrated_angle = calibrated_angle - 360;
		}
		last_calibrated_angle = calibrated_angle;
		if (debug)
		{
			if (millis() - lastPubAng > 2000)
			{
				Serial.print("angle[");
				Serial.print(id);
				Serial.print("]: ");
				Serial.println(calibrated_angle);
				// Serial.print("Actual Angle");
				// Serial.println(angle);
				// Serial.print("Offset");
				// Serial.println(angle_offset);
				lastPubAng = millis();
			}
		}
		return calibrated_angle;
	}
	else
	{
		return last_calibrated_angle;
	}
}
/*
* Set desired joint angle
*/
void JointMotor2::setAngle(double angle)
{
	desiredAngle = angle;
	lastError = 0;
	// sumError = 0;
	return;
}
/*
* Switch PID values for which joint is fixed
* Returns false if gripper 1 is engaged and true if gripper 2
*/
bool JointMotor2::switchPID(int gripperEngagedSelect)
{

	//TODO: set error to zero, offset angles with current value, set flag to turn off motors

	// sumError = 0;
	// myRA.clear();
	angle_offset += desiredAngle - getAngleDegrees();

	if (gripperEngagedSelect == 1) // TODO switch back to 1
	{
		kP = kP1;
		kI = kI1;
		kD = kD1;
		Serial.println("Switching to PID 1");
		return false;
	}
	else if (gripperEngagedSelect == 2) // TODO switch back to 2
	{
		kP = kP2;
		kI = kI2;
		kD = kD2;
		Serial.println("Switching to PID 2");
		return true;
	}
}
/*
* calculate motor speed for PID
*/
double JointMotor2::calcSpeed(double currentAngle, int gc, int useGravityComp, int velocity_term_scale)
{

	/**
	 * TODO:
	 *if inHomePosition:
	 	setSpeed(0) 
	 *
	 */
	double speed = 0;
	// if (useGravityComp == 0)
	// {
	// 	setSpeed(0);
	// 	return 0;
	// }
	// else
	{
		// double currentAngle = getAngleDegrees();

		double error = desiredAngle - currentAngle;
		// if (abs(int(error)) > 180)
		// {
		// 	error = error + 360;
		// }

		// sumError = (sumError + error) * 0.5;
		// sumError = sumError * 0.8 + error;
		// if (error > 1 || error < -1)
		// {
		// 	// error_idx++;
		// 	// sumError = sumError + error - last_errors[(error_idx - 1) % num_last_errors];
		// 	// last_errors[error_idx % num_last_errors] = error;
		// 	sumError += error;
		// }

		// if (abs(error) < 8 && abs(error) > 1)
		// {
		sumError += error;
		// }
		// if (abs(error) < 2)
		// {
		// 	sumError;
		// }
		// else
		// {
		// 	myRA.addValue(error);
		// 	sumError = myRA.getAverage();
		// }

		// if (abs(error) >= 8)
		// {
		// 	myRA.addValue(error);
		// 	sumError = myRA.getAverage();
		// }
		// else if (abs(error) > 7 && abs(error) < 8)
		// {
		// 	// sumError += error;
		// 	myRA.clear();
		// 	sumError = 0;
		// }
		// else
		// {
		// 	sumError += error;
		// }

		//Wrap around if error is big
		// if (sumError > 1000)
		// {
		// 	sumError = 1000;
		// }
		// else if (sumError < -1000)
		// {
		// 	sumError = -1000;
		// }
		sumError = constrain(sumError, -4000, 4000);
		// sumError = constrain(sumError, -2000, 2000);

		double changeError = error - lastError;

		double pid_error = (kP * error) + (kI * sumError) + (kD * changeError); // change constants back to kP kI kD
		double deadbandScale = 5.0;

		if (pid_error < 0)
		{
			speed = pid_error - deadbandScale + (gc * useGravityComp);
		}
		else
		{
			speed = pid_error + deadbandScale + (gc * useGravityComp);
		}

		if (speed < 0)
		{
			speed -= (velocity_term * velocity_term_scale);
		}
		else
		{
			speed += (velocity_term * velocity_term_scale);
		}

		// speed = pid_error + (gc * useGravityComp);
		// if (id == 0)
		// {
		// 	double currentTime = millis();
		// 	if (currentTime - lastPubAng2 > 200)
		// 	{

		// 		// Serial.print(currentAngle);
		// 		// Serial.print('\t');
		// 		Serial.print(error);
		// 		Serial.print('\t');
		// 		Serial.print(sumError);
		// 		Serial.print('\t');
		// 		// Serial.print(changeError);
		// 		// Serial.print('\t');
		// 		// Serial.print(pid_error);
		// 		Serial.print('\n');
		// 		lastPubAng2 = currentTime;
		// 	}
		// }

		// if (speed < -5)
		// {
		// 	speed = constrain(speed, -180, -15);
		// }
		// else if (speed > 5)
		// {
		// 	speed = constrain(speed, 15, 180);
		// }
		// speed = pid_error;
		// debugPrint("Error", error);
		// debugPrint2("Sum", sumError);

		// if (speed > -50 && speed < -20)
		// {
		// 	speed += -10;
		// }
		// if (speed < 50 && speed > 20)
		// {
		// 	speed += 10;
		// }

		// speed = constrain(speed, -150, 150);
		// debugPrint("Error", pid_error);

		// Serial.print("ID: ");
		// Serial.println(id);
		// Serial.print("PID: ");
		// Serial.println(pid_error);
		// Serial.print("Speed: ");
		// Serial.println(speed);
		// Serial.println("\n");
		// double speed = gc;
		// Serial.print("speed of angle ");
		// Serial.print(id);
		// Serial.print(": ");
		// Serial.println(speed);

		// setSpeed(speed);

		// Serial.println(error);
		// Serial.println(sumError);
		// Serial.println(changeError);
		// Serial.println(kP);
		// Serial.println(kI);
		// Serial.println(kD);
		// Serial.println(speed);
		// speed *= 0.5;
		lastError = error;
		return speed;
	}
}
