//*****************************************************************************
//
// Gripper.h - Prototypes for the gripper driver. Using the Vex 29 motorcontrollers
//
//*****************************************************************************

#ifndef GRIPPER_H
#define GRIPPER_H

#include <Servo.h>
#include <Arduino.h>

class Gripper
{
private:
    int pin, zeroPosition, threshold;
    int maxSpeedCCW, maxSpeedCW;
    int maxPulse, minPulse, medianPulse;
    bool directionCW;
    bool isEngaged;

    int lastUpdate = 0; //increment for motor move interval
    int startTime = 0;
    bool resetTime = true;
    bool gripperFinished;

    int time = 30000; //Time is takes for gripper to engage/disengage

    Servo grip;

public:
    Gripper();                                                                                   //default constructor
    Gripper(int pin, bool directionCW, bool isEngaged, int zeroPosition = 0, int threshold = 5); //contructor with values

    void setEngaged(bool e);
    bool getEngaged(void);
    // Writes the pulsewidth specified to the motorcontroller and maps the values.
    void write(int power);

    bool isE; //Flag to know if gripper is enagaged or not

    typedef enum
    {              // Used globaly as states for gripper functions
        idle,      //0
        engage,    //1
        disengage, //2
    } gripperState;

    // Sets gripper state to engage or disengage. Time is in milliseconds (1000 milliseconds = 1 sec)
    bool setGripper(gripperState gState);
    bool setGripper(int gState);
};

#endif
