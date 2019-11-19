//*****************************************************************************
//
// gripper.h - Prototypes for the gripper driver. Using the Vex 29 motorcontrollers
//
//*****************************************************************************

#ifndef GRIPPER_H
#define GRIPPER_H

#include <Servo.h>
#include <Arduino.h>
#include "Button.h"

typedef enum{
    engage,
    disengage,
    idle,
    }gripperState;

class Gripper {
    private:
        int pin, zeroPosition, threshold;
        int maxSpeedCCW, maxSpeedCW;
        int maxPulse, minPulse, medianPulse;

        Servo grip;

        // Buttons have to be pull up
        // Pull up: one terminal on GND and the other
        //          attached to the analog pin.
        int buttonUpPin = A0;
        Button buttonGripper = Button(buttonUpPin, PULLUP);
        bool buttonState = true;

    public:

        Gripper(); //default constructor
        Gripper(int pin, int zeroPosition = 0, int threshold = 5); //contructor with values

        void write(int power);
        bool setGripper(gripperState gState, int time);// time is in milliseconds (1000 milliseconds = 1 sec)
        gripperState gripperButtonTest(gripperState currentState);
};

#endif
