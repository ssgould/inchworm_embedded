// JointMotor2.h

#ifndef _JOINTMOTOR2_h
#define _JOINTMOTOR2_h

#include "ams_as5048b.h"

class JointMotor2
{
private:
	uint8_t id;

	uint8_t pwmForward, pwmReverse, pinEnable;
	AMS_AS5048B encoder;

	//PID
	double kP, kI, kD;
	double targetAngle;

	double last_calibrated_angle; //angle of joint
	double angle_offset;		  // offset of angle in calibration position
	bool enc_clockwise;			  //1 if switch direction

	double sumError = 0;
	double lastError = 0;

public:
	JointMotor2() {}
	JointMotor2(int pinDirectionA1, int pinDirectionB1, int pinPWM1,
				uint8_t encoderAddress, double kp, double ki, double kd,
				double ang_offset, bool encoder_clockwise, uint8_t id_input);

	void SendPWM(int speed);
	void SetTarget(double angle);
	int CalcEffort(void);
	double getAngleDegrees();

	void SetKp(float k) { kP = k; }
	void SetKi(float k) { kI = k; }
	void SetKd(float k) { kD = k; }
};

#endif
