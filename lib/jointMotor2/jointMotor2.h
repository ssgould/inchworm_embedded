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

	// PID
	double kP2, kI2, kD2; //For when D link is fixed
	double kP1, kI1, kD1; //For when A link is fixed
	double kP, kI, kD;	  //Params actually being used

	double targetAngle;
	double minAngle;
	double maxAngle;

	double last_calibrated_angle; //angle of joint
	double angle_offset;		  // offset of angle in calibration position
	bool enc_clockwise;			  //1 if switch direction

	double sumError = 0;
	double lastError = 0;

	double lastDebugUpdate = 0;

	double vel_startTime;  	// start time for delta time when calculating velocity
	double vel_posStart;	// start position for change in joint position for calculating velocity

public:
	JointMotor2() {}
	JointMotor2(int pwmF, int pwmR);
	JointMotor2(int pinDirectionA1, int pinDirectionB1, int pinPWM1,
				uint8_t encoderAddress, double kp_a_link_fixed, double ki_a_link_fixed, double kd_a_link_fixed,
				double kp_d_link_fixed, double ki_d_link_fixed, double kd_d_link_fixed,
				double ang_offset, double min_angle, double max_angle, bool encoder_clockwise, uint8_t id_input);

	void SendPWM(int speed);
	void SetTarget(double angle);
	double GetTarget(void);
	int CalcEffort(void);
	double getAngleDegrees();
	bool SwitchPID(uint8_t gripperEngagedSelect);
	bool SwitchPID(void);
	void SetKp(float k) { kP = k; }
	void SetKi(float k) { kI = k; }
	void SetKd(float k) { kD = k; }
	void printPID();

	//**********getters*************/
	int get_pwmForwardPin();
	int get_pwmReversePin();
	int get_pinEnable();
	int get_encoderAddress();
	double get_kP();
	double get_kI();
	double get_kD();
	double get_kP1();
	double get_kI1();
	double get_kD1();
	double get_kP2();
	double get_kI2();
	double get_kD2();
	void getPIDF(double (&arr)[3]);
	void getPIDB(double (&arr)[3]);
	double get_angle_offset();
	uint8_t get_id();
	double get_vel_startTime();
	double get_vel_posStart();

	/***********setters*************/
	void set_PID(float kF[], float kB[]);
	void set_vel_startTime(double startTime);
	void set_vel_posStart(double posStart);

	//*****************************

	typedef enum
	{							 // Used globaly as states for pid selection
		both_grippers_engaged,   //0
		a_link_engaged,			 //1
		d_link_engaged,			 //2
		neither_gripper_engaged, //3
	} gripperSelect;

	int fixed_link = a_link_engaged;

};

#endif
