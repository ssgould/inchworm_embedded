//*****************************************************************************
//
// gripper.h - Prototypes for the gripper driver. Using the Vex 29 motorcontrollers
//
//*****************************************************************************

#ifndef GRIPPER_H
#define GRIPPER_H

#include <Servo.h>
#include <Arduino.h>


typedef enum{ // Used globaly as states for gripper functions
    engage,
    disengage,
    idle,
    }gripperState;

class Gripper {
    private:
        int pin, zeroPosition, threshold;
        int maxSpeedCCW, maxSpeedCW;
        int maxPulse, minPulse, medianPulse;
        bool directionCW;

        Servo grip;

    public:

        Gripper(); //default constructor
        Gripper(int pin, bool directionCW, int zeroPosition = 0, int threshold = 5); //contructor with values

        // Writes the pulsewidth specified to the motorcontroller and maps the values.
        void write(int power);
        // Sets gripper state to engage or disengage. Time is in milliseconds (1000 milliseconds = 1 sec)
        bool setGripper(gripperState gState, int time);
        //gripperState gripperButtonTest(gripperState currentState);

};

#endif
