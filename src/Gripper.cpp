/*******************************************************************
* Gripper library that allows to enagage and disengage the grippers
* of the robot. This code is designed for the Vex 29 motor controllers.
* This code is also adapted from the vexMotor Arduino Library found in
* the following link: https://github.com/sparkfun/vexMotor
*
*
* 22 Dec 2014 - Brian Huang, SparkFun Electronics
* Nov 2019 - Josue Contreras and Trevor Rizzo, Swarm Construction MQP
/******************************************************************/

#include "Gripper.h"

/*
* Gripper Constructors
*/
Gripper::Gripper(){
}

Gripper::Gripper(int pin, int zeroPosition = 0, int threshold = 5){

    pin = pin;
    zeroPosition = zeroPosition;
    threshold = threshold;
    maxSpeedCCW = -255;
    maxSpeedCW = 255;
    maxPulse = 2000;
    minPulse = 1000;
    medianPulse = 1500;

    grip.attach(pin);

}

/*
* Function adapted from the vexMotor Arduino Library. VEX Motor
* Controller spins CW for pulse widths less than 1500 uS and CCW
* for pulse widths greater than 1500. map() scales the power to a
* pulse width.
*/
void Gripper::write(int power){

    int pulseWidth;
    int direction = power - zeroPosition;

    // if it's greater than threshold -- then it spinning in the CW
    if (direction > threshold)
    {
        pulseWidth = map(power, zeroPosition, maxSpeedCW, medianPulse, minPulse);
        grip.writeMicroseconds(pulseWidth);
        }
    else if (direction < -threshold)
    {
        pulseWidth = map(power, maxSpeedCCW, zeroPosition, maxPulse, medianPulse);
        grip.writeMicroseconds(pulseWidth);
    }
    else
    {
        grip.writeMicroseconds(medianPulse);
    }
}

/*
* For gripper engagment and disengament
* TODO: when current sensing added to gripper, add that sensng instead of the time delay.
*/
bool Gripper::setGripper(gripperState gState, int time){

  switch(gState){
    case engage: //engage gripper
        write(255);
        delay(time);
        write (0);
        break;
    case disengage: //disengage gripper
        write(-255);
        delay(time);
        write (0);
        break;
    case idle: //nothing happens
        write(0);
        break;
    default: //if none are selected
        write(0);
        delay(time);
        break;
  }

  return true;
}

/*
* Chooses the gripper and action (enagage or disenagage) of the grippers
*/
