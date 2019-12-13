// JointMotor2.h

#ifndef _JOINTMOTOR2_h
#define _JOINTMOTOR2_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "ams_as5048b.h"
//#include <Servor.h>

class JointMotor2 {
private:
	int pinDirectionA, pinDirectionB, pinPWM;
	AMS_AS5048B encoder;

	//PID
	double desiredAngle;
	double kP, kI, kD;
	double kP2, kI2, kD2;
	double sumError, lastError;

	double last_calibrated_angle; //angle of joint
	double angle_offset; // offset of angle in calibration position
	bool enc_clockwise; //1 if switch direction

	double lastPubAng;
	int id;


public:
	bool debug;

	JointMotor2();
	JointMotor2(int pinDirectionA1, int pinDirectionB1, int pinPWM1, int encoderAddress, double kp, double ki, double kd, double ang_offset, bool encoder_clockwise,int id_input);
	JointMotor2(int pinDirectionA1, int pinDirectionB1, int pinPWM1, int encoderAddress, double kp, double ki, double kd, double kp2, double ki2, double kd2, double ang_offset, bool encoder_clockwise, int id_input);

	void    setSpeed(int speed);
	void    changeDirection(int speed);
	void    setAngle(double angle);
	void    switchPID();
	void    updateSpeed();
	double  getAngleDegrees();
	// double getKP();
	// void setKP(double kpValue)

};

#endif
