

#include <Servo.h>

int leftMotorPin = 4; //left motor pin
int rightMotorPin = 2; //left motor pin

Servo leftMotor; //Define variables for each motor
Servo rightMotor;

void setup() {
  // put your setup code here, to run once:
  pinMode(leftMotorPin,OUTPUT);  //Both motor pins are sending a signal out to the VEX motor controller
  pinMode(rightMotorPin,OUTPUT);
  pinMode(13,OUTPUT);

  leftMotor.attach(leftMotorPin); //These lines tell the Arduino to send signals to the pins identified above to control the motor variables
  rightMotor.attach(rightMotorPin);
}

void loop() {
  // put your main code here, to run repeatedly:

  //Send full forward to both motors
  leftMotorControl(100);
  rightMotorControl(100);
  digitalWrite(13,HIGH); //Turn the LED on pin 13 on
  delay(1000); //Wait 1000 ms, or one second

  //Send full backward to both motors
  leftMotorControl(-100);
  rightMotorControl(-100);
  digitalWrite(13,LOW); //Turn the LED on pin 13 off
  delay(1000);

}

//The functions below let you send a power value between -100 and 100 to the motors.
//The servo library uses numbers between 1000 and 2000 to control servos.

int leftMotorControl(int value){

leftMotor.write(map(value,-100,100,1000,2000));

}

int rightMotorControl(int value){

rightMotor.write(map(value,-100,100,1000,2000));

}
