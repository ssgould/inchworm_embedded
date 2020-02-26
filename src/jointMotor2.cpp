#include <Arduino.h>
#include "JointMotor2.h"


JointMotor2::JointMotor2()
{
}

JointMotor2::JointMotor2(int pinDirectionA1, int pinDirectionB1, int pinPWM1, int encoderAddress, double kp, double ki, double kd, double ang_offset, bool encoder_clockwise, int id_input)
{
	//Pin Configuration
	pinDirectionA = pinDirectionA1;
	pinDirectionB = pinDirectionB1;
	pinPWM = pinPWM1;
	pinMode(pinDirectionA, OUTPUT);
	pinMode(pinDirectionB, OUTPUT);
	pinMode(pinPWM, OUTPUT);
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

	moving_average_integral.clear();
}
JointMotor2::JointMotor2(int pinDirectionA1, int pinDirectionB1, int pinPWM1, int encoderAddress, double kp, double ki, double kd, double kp2, double ki2, double kd2, double ang_offset, bool encoder_clockwise, int id_input)
{
	//Pin Configuration

	pinDirectionA = pinDirectionA1;
	pinDirectionB = pinDirectionB1;
	pinPWM = pinPWM1;
	pinMode(pinDirectionA, OUTPUT);
	pinMode(pinDirectionB, OUTPUT);
	pinMode(pinPWM, OUTPUT);

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

	moving_average_integral.clear();
	// error_idx = 0;
}

/*
* Print variable every 2000 millis
*/
void JointMotor2::debugPrint(char vName[3], double vInput)
{
	double currentTime = millis();
	if (currentTime - lastPubAng2 > 2000)
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

/*
* Takes speed -255 - 255 and moves motor
*/
void JointMotor2::setSpeed(double speed)
{
	double maxPercent = 0.9;
	if (speed < -255 * maxPercent)
	{
		// speed = -255 * maxPercent;
		digitalWrite(pinPWM, HIGH);
	}
	else if (speed > 255 * maxPercent)
	{
		// speed = 255 * maxPercent;
		digitalWrite(pinPWM, HIGH);
	}
	else
	{
		analogWrite(pinPWM, abs(speed));
	}
	changeDirection(speed);
	return;
}
/*
* Changes motor direction pin states based on sign of speed
*/
void JointMotor2::changeDirection(double speed)
{
	if (speed < 0)
	{
		digitalWrite(pinDirectionA, HIGH);
		digitalWrite(pinDirectionB, LOW);
	}
	else
	{
		digitalWrite(pinDirectionA, LOW);
		digitalWrite(pinDirectionB, HIGH);
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
		if (calibrated_angle > 360)
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

	sumError = 0;
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
double JointMotor2::calcSpeed(int gc, int useGravityComp)
{

	/**
	 * TODO:
	 *if inHomePosition:
	 	setSpeed(0) 
	 *
	 */
	double speed = 0;
	if (useGravityComp == 0)
	{
		setSpeed(0);
		return 0;
	}
	else
	{
		double currentAngle = getAngleDegrees();

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

		moving_average_integral.addValue(error);
		sumError = moving_average_integral.getAverage();
		// debugPrint("sumError", sumError);

		//Wrap around if error is big
		// if (sumError > 1000)
		// {
		// 	sumError = 1000;
		// }
		// else if (sumError < -1000)
		// {
		// 	sumError = -1000;
		// }

		sumError = constrain(sumError, -1000, 1000);

		double changeError = error - lastError;

		double pid_error = (kP * error) + (kI * sumError) + (kD * changeError); // change constants back to kP kI kD
		speed = pid_error + (gc * useGravityComp);
		if (speed < -5)
		{
			speed = constrain(speed, -180, -15);
		}
		else if (speed > 5)
		{
			speed = constrain(speed, 15, 180);
		}
		// speed = pid_error;
		// debugPrint("GC", gc);

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
