# Smart Scaffolding Inchworm Embedded System

This repo is for the control of the robots motors on the joint and grippers. The control is done through PID and the encoders placed on each joint.

## Getting Started

These instructions will allow you to run this system on a robot like the inchworm.

### Prerequisites (libraries needed)

What things you need to install the software and how to install them

```
Give examples
```

### Debugging the System

Additionally, for debugging we added some functions that allow you to debug the system. Some of these functions allow you to connect a button to test various parts of the physical system separately. For example the in main the helper function:

```
void gripperButtonTest(gripperState currentState, Gripper grip, Button buttonGripper)
```
Enables to interface (engage and disengage) the grippers using one button. The button should be plugged into the analog pins in a pull up configuration.

NOTE: Pull Up Configuration
```
GND => Button Terminal 1
Analog Pin => Button Terminal 2
```

## Running the system

The system is simple to run. To move the joint angles and the grippers a message has to be sent through the serial terminal. The message is 12 characters long and has the following structure.

```
Joint_1 Angle  Joint_2 Angle  Joint_3 Angle  Gripper1&2_States  
   (0 0 0)         (0 0 0)       (0 0 0)           (0 0 0)
```

## Deployment

It is important to mention that this code in non-blocking.

## Built With

* [PlataformIO](https://platformio.org/) - Generation ecosystem for embedded development

## Contributing

Please read [CONTRIBUTING.md](https://gist.github.com/PurpleBooth/b24679402957c63ec426) for details on our code of conduct, and the process for submitting pull requests to us.

## Authors

* **Trevor Rizzo** - *PID* -
* **Josue Contreras** - *Gripper* -

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* Multi-tasking in arduino [article](https://learn.adafruit.com/multi-tasking-the-arduino-part-1/a-clean-sweep)
