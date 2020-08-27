/*****************************************************************************
*
* Gripper.h - Prototypes for the gripper driver. Using the Vex 29 motorcontrollers
*
*****************************************************************************/

#ifndef GRIPPER_H
#define GRIPPER_H

#include <Servo.h>
#include <Arduino.h>


class Gripper
{
private:
    int gripperPin, buttonPin, threshold;
    int maxSpeedCCW, maxSpeedCW;
    int maxPulse, minPulse, medianPulse, pulseWidth;
    bool rotationDirection;
    bool isEngaged;
    

    int startTime = 0; // TODO: replace with the start turns 
    bool resetTime = true; // TODO: change the "resetTime" name to "resetRotationItterator"
    bool gripperFinished;

    int time = 30000; //Time is takes for gripper to engage/disengage
    long turns = 600;

    Servo grip;

public:

    typedef enum
    {  // Used globaly as states for gripper functions
        idle,      //0
        engage,    //1
        disengage, //2
    } gripperState;
    
    ////////////////////////////////////////////////////////////////
    // CONSTRUCTORS
    ////////////////////////////////////////////////////////////////
    Gripper(); //default constructor
    Gripper(int gripperPin, bool directionCW, bool isEngaged, int buttonPin, int threshold = 5); //contructor with values
    bool isE; //Flag to know if gripper is enagaged or not TODO: check if this is actauly useful
    long turnsItterator;

    ////////////////////////////////////////////////////////////////
    // FUNCTIONS
    ////////////////////////////////////////////////////////////////

    // Sets gripper state to engage or disengage (1000 milliseconds = 1 sec)
    void begin(void);
    bool setGripper(gripperState gState);
    bool setGripper(int gState);
    bool setGripperwTime(int gState);
    void incrementIterator(void);
    void resetItterator(void);
    void screwTurns(void);
    // Writes the pulsewidth specified to the motorcontroller and maps the values.
    void write(int power);
    void intService(void);

    

    ////////////////////////////////////////////////////////////////
    // SETTERS AND GETTERS
    ////////////////////////////////////////////////////////////////
    void setEngaged(bool e){ isEngaged = e;}

    int getGripperPin(void){ return gripperPin; } // make sure all global pins are as ectablished if not just show a warning
    int getPulseWidth(void){ return pulseWidth; } // check the correct pulse width is being sent to the grippers for max speed
    bool getRotationDirection(void){ return rotationDirection; } // check if the rotation direction is correct for each gripper
    bool getEngaged(void){ return isEngaged; } // check if the flag is engaged after commanding the grippers to the enagage state
    long getTurnsItterator(void){ return turnsItterator; } // check if the right amount of turns have been done to fully engage
    long getTurns(void);
    //void Gripper::intService(void);

};

#endif