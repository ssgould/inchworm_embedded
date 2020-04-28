#define SUM_THRESHOLD 5000
#define ANGLE_ERROR_THRESHOLD 10
#define UPDATE_INTERVAL 20
#define MOTOR_COUNT 5

#define VIA_INTERVAL 3
#define VIA_COUNT 50

#define FREQUENCY_JOINT_MOTORS 120000 // Adjust desired PWM Frequency at Pins (make sure to toggle TRUE in main file for use)

enum STATE
{
    ST_IDLE,
    ST_HOLDING,
    ST_MOVING
};
