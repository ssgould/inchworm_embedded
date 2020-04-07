//*****************************************************************************
//
// Gripper.h - Prototypes for the gripper driver. Using the Vex 29 motorcontrollers
//
//*****************************************************************************

#ifndef STORAGE_H
#define STORAGE_H

#include <Servo.h>
#include <Arduino.h>

class Storage {
    private:
        bool isImplemented = false; // flag for unit test since this has not been implemented on robot
        Servo storageMotor;

    public:

        Storage(); //default constructor
        Storage(int pin);

        void restPosition(void);
        void loadPosition(void);

        void getImplementation(void){ return isImplemented; }
};

#endif
