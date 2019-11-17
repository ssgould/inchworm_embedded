#include "ams_as5048b.h"
//#include <Servor.h>

class JointMotor {
    private:
        int pinDirectionA, pinDirectionB, pinPWM;
        AMS_AS5048B encoder;

        //PID
        double desiredAngle;
        double kP, kI, kD;
        double sumError, lastError;

    public:
        JointMotor(); 
        JointMotor(int pinDirectionA1, int pinDirectionB1, int pinPWM1, int encoderAddress, double kp, double ki, double kd);

        void    setSpeed(int speed);
        void    changeDirection(int speed);
        void    setAngle(double angle);
        void    updateSpeed();
        double  getAngleDegrees();

};