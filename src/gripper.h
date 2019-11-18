//*****************************************************************************
//
// gripper.h - Prototypes for the gripper driver. Using the Vex 29 motorcontrollers
//
//*****************************************************************************

#include <Servo.h>
#include <Arduino.h>

class gripper {
    private:
      int pin, zeroPosition, threshold;
      int maxSpeedCCW, maxSpeedCW;
      int maxPulse, minPulse, medianPulse;

      typedef enum{
        engage,
        disengage,
        idle,
      }gripperState;

      Servo grip;
      
    public:
        Gripper(); //default constructor
        Gripper(int pin, int zeroPosition = 0, int threshold = 5); //contructor with values

        void write(int power);
        char setGripper(gripperState gState, int time);
};
