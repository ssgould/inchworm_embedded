#include "JointMotor2.h"
#include "config.h"

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
	double calibrated_angle = angle + angle_offset;

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

	//TODO: REMOVE ME
	double currentTime = millis();
	if (currentTime - lastDebugUpdate >= 2000)
	{
		Serial.print("ID: ");
		Serial.print(id);
		Serial.print("\t Angle: ");
		Serial.println(error);
		lastDebugUpdate = currentTime;
	}

	return effort;
}

/*
* Switch PID values for which joint is fixed
* Returns false if gripper 1 is engaged and true if gripper 2
*/
bool JointMotor2::SwitchPID(uint8_t gripperEngagedSelect)
{
	// Serial.println("Switching the PID values");
	angle_offset += targetAngle - getAngleDegrees();

	if (fixed_link == d_link_engaged && gripperEngagedSelect == a_link_engaged) // TODO switch to a_link_engaged
	{
		kP = kP1;
		kI = kI1;
		kD = kD1;
		Serial.println("Switching to PID 1");
		fixed_link = a_link_engaged;
		return false;
	}
	else if (fixed_link == a_link_engaged && gripperEngagedSelect == d_link_engaged) // TODO switch to d_link_engaged
	{
		kP = kP2;
		kI = kI2;
		kD = kD2;
		Serial.println("Switching to PID 2");
		fixed_link = d_link_engaged;
		return true;
	}
	else
	{
		return;
	}
}